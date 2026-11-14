#!/usr/bin/env python3
"""Benchmark: how do collision-sphere sets change planning success, time and path length?

Same random start/goal pairs and the same scenes for every (backend, sphere set); results are then
cross-checked against sphere models and a mesh-surface oracle that are independent of the planner.
Method, results and caveats: docs/spherization_benchmark.md.

Stages (each reads/writes files under --work; planners run in their own docker images):

  prepare      CPU, host python (numpy trimesh scipy pyyaml; see compare_spheres.py): scenes, valid
               start/goal pairs, probe states -> problems.json, vamp_problems.txt
  run-curobo   inside motion-planner:gpu: one cuRobo robot config -> raw results (one planner/process)
  run-vamp     host, calls docker with motion-planner:full: compiles vamp_bench.cpp against one generated
               header and runs it -> raw results
  sphere-urdf  CPU: a cuRobo yml (or a spherized URDF) -> spherized URDF for Cricket, optional buffer
  buffer-cfg   CPU: copy a cuRobo robot yml with collision_sphere_buffer set
  report       CPU: oracle cross-check of every raw result -> results.json + a markdown table

Run `bench_spheres.py <stage> -h` for the options. Frames: the robot base frame (URDF root), metres.
"""
import argparse
import json
import math
import pathlib
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
JOINT_LIMIT = math.pi          # start/goal sampling range, rad (planners allow more)
SELF_HOPS = 2                  # links up to this many joints apart are not self-checked (rigid_pairs_srdf --hops 2)
FAMILIES = ('empty', 'boxes', 'slab')


# ------------------------------------------------------------------------------------------------
# numpy robot model: FK + sphere sets + boxes. Independent of both planners, used for pair validity
# (common reference) and for the oracle.
# ------------------------------------------------------------------------------------------------
def _floats(s, default):
    return [float(v) for v in s.split()] if s else default


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _origin_tf(origin):
    t = np.eye(4)
    if origin is not None:
        t[:3, :3] = _rpy(*_floats(origin.get('rpy'), [0, 0, 0]))
        t[:3, 3] = _floats(origin.get('xyz'), [0, 0, 0])
    return t


class Arm:
    """Serial revolute arm from a URDF: batched FK to every link frame."""

    def __init__(self, urdf):
        self.urdf = pathlib.Path(urdf)
        root = ET.parse(self.urdf).getroot()
        self.joints = [j for j in root.findall('joint')]
        self.by_child = {j.find('child').get('link'): j for j in self.joints}
        self.dof_joints = [j for j in self.joints if j.get('type') == 'revolute']
        self.dof = len(self.dof_joints)
        self.joint_names = [j.get('name') for j in self.dof_joints]
        self.links = [l.get('name') for l in root.findall('link')]

    def link_tfs(self, q):
        """q: (n, dof) -> {link: (n, 4, 4)}, in the URDF root frame."""
        q = np.asarray(q, float).reshape(-1, self.dof)
        n = len(q)
        idx = {j.get('name'): i for i, j in enumerate(self.dof_joints)}
        cache = {}

        def pose(link):
            if link in cache:
                return cache[link]
            j = self.by_child.get(link)
            if j is None:
                out = np.tile(np.eye(4), (n, 1, 1))
            else:
                out = pose(j.find('parent').get('link')) @ _origin_tf(j.find('origin'))
                if j.get('type') == 'revolute':
                    a = np.array(_floats(j.find('axis').get('xyz'), [0, 0, 1]))
                    a /= np.linalg.norm(a)
                    th = q[:, idx[j.get('name')]]
                    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                    rot = np.tile(np.eye(4), (n, 1, 1))
                    rot[:, :3, :3] = (np.eye(3) + np.sin(th)[:, None, None] * k
                                      + (1 - np.cos(th))[:, None, None] * (k @ k))
                    out = out @ rot
            cache[link] = out
            return out
        for l in self.links:
            pose(l)
        return cache

    def chain_index(self, links):
        """Position of each link along the kinematic chain (number of parent hops to the root)."""
        def depth(l):
            d = 0
            while l in self.by_child:
                l = self.by_child[l].find('parent').get('link')
                d += 1
            return d
        return {l: depth(l) for l in links}


def load_spheres(path):
    """link -> (m, 4) array. A spherized URDF or a cuRobo/XRDF yaml, as in compare_spheres.py."""
    path = pathlib.Path(path)
    sys.path.insert(0, str(HERE))
    import compare_spheres as cs
    return cs.load_yaml_spheres(path) if path.suffix in ('.yml', '.yaml', '.xrdf') else cs.load_urdf_spheres(path)


def parse_sets(specs):
    """NAME=PATH[@BUFFER_M][#HOPS] -> {name: (spheres, buffer, hops)}. HOPS is the self-collision policy
    of that set: links up to HOPS joints apart are not checked (default 2, the rigid_pairs_srdf --hops 2
    rule used for FOAM spheres; cuRobo's own fit is checked with its matrix, which is HOPS=1)."""
    out = {}
    for spec in specs:
        name, path = spec.split('=', 1)
        buf, hops = 0.0, SELF_HOPS
        if '#' in path:
            path, h = path.rsplit('#', 1)
            hops = int(h)
        if '@' in path:
            path, b = path.rsplit('@', 1)
            buf = float(b)
        out[name] = (load_spheres(path), buf, hops)
    return out


class SphereModel:
    """A sphere set on an Arm: world / self collision of batches of states, numpy."""

    def __init__(self, arm, spheres, buffer=0.0, hops=SELF_HOPS, name=''):
        self.arm, self.name = arm, name
        self.buffer = buffer
        self.sph = {l: np.asarray(v, float) for l, v in spheres.items() if l in arm.links}
        self.links = sorted(self.sph, key=lambda l: arm.chain_index([l])[l])
        depth = arm.chain_index(self.links)
        self.pairs = [(a, b) for i, a in enumerate(self.links) for b in self.links[i + 1:]
                      if abs(depth[a] - depth[b]) > hops]
        self.n = sum(len(v) for v in self.sph.values())

    def world_spheres(self, tfs):
        """{link: (n, m, 4)} world centres and radii. `buffer` pads only the radius used against the
        world (cuRobo's collision_sphere_buffer; its self-collision padding is separate and 0)."""
        out = {}
        for l in self.links:
            c = self.sph[l]
            T = tfs[l]
            ctr = np.einsum('nij,mj->nmi', T[:, :3, :3], c[:, :3]) + T[:, None, :3, 3]
            rad = np.broadcast_to(c[:, 3], ctr.shape[:2])
            out[l] = np.concatenate([ctr, rad[..., None]], axis=2)
        return out

    def self_collision(self, ws):
        n = next(iter(ws.values())).shape[0]
        hit = np.zeros(n, bool)
        ball = {}
        for l, w in ws.items():          # per-state bounding sphere of each link: cheap broadphase
            c = w[..., :3].mean(axis=1)
            ball[l] = (c, (np.linalg.norm(w[..., :3] - c[:, None], axis=2) + w[..., 3]).max(axis=1))
        for a, b in self.pairs:
            near = (np.linalg.norm(ball[a][0] - ball[b][0], axis=1) < ball[a][1] + ball[b][1]) & ~hit
            if not near.any():
                continue
            wa, wb = ws[a][near], ws[b][near]
            d = np.linalg.norm(wa[:, :, None, :3] - wb[:, None, :, :3], axis=3)
            hit[np.flatnonzero(near)[(d < wa[:, :, None, 3] + wb[:, None, :, 3]).any(axis=(1, 2))]] = True
        return hit

    def world_collision(self, ws, boxes):
        n = next(iter(ws.values())).shape[0]
        hit = np.zeros(n, bool)
        for b in boxes:
            c, half, _ = box_frame(b)
            reach = np.linalg.norm(half) + self.buffer
            for l in self.links:
                w = ws[l]
                ctr = w[..., :3].mean(axis=1)       # broadphase: link bounding sphere vs box bounding sphere
                rho = (np.linalg.norm(w[..., :3] - ctr[:, None], axis=2) + w[..., 3]).max(axis=1)
                near = (np.linalg.norm(ctr - c, axis=1) < rho + reach) & ~hit
                if not near.any():
                    continue
                d = box_distance(w[near][..., :3], b)
                hit[np.flatnonzero(near)[(d < w[near][..., 3] + self.buffer).any(axis=1)]] = True
        return hit

    def in_collision(self, q, boxes, self_check=True):
        out = np.zeros(len(q), bool)
        for i in range(0, len(q), 400):
            tfs = self.arm.link_tfs(q[i:i + 400])
            ws = self.world_spheres(tfs)
            h = self.world_collision(ws, boxes)
            if self_check:
                h |= self.self_collision(ws)
            out[i:i + 400] = h
        return out


def box_frame(b):
    """(center, half extents, R) with R's columns the box axes; boxes only rotate about z (yaw)."""
    c, s = math.cos(b['yaw']), math.sin(b['yaw'])
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    return np.array(b['center'], float), np.array(b['size'], float) / 2, R


def box_distance(pts, b):
    """Signed distance of points (..., 3) to a box (negative inside)."""
    c, h, R = box_frame(b)
    local = (pts - c) @ R
    q = np.abs(local) - h
    return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(axis=-1), 0)


class MeshOracle:
    """World collision of the true link meshes, as seeded surface samples (about one per 1.5 cm^2) plus
    vertices, in the link frame. A point inside a box means the mesh touches it. Self-collision of the
    meshes is NOT checked (no mesh-mesh query without fcl)."""

    def __init__(self, arm, seed=0):
        import trimesh
        self.arm = arm
        self.pts, self.ball = {}, {}
        for link in ET.parse(arm.urdf).getroot().findall('link'):
            parts = []
            for col in link.findall('collision'):
                m = col.find('geometry/mesh')
                if m is None:
                    continue
                mesh = trimesh.load(arm.urdf.parent / m.get('filename'), force='mesh')
                mesh.apply_scale(_floats(m.get('scale'), [1, 1, 1]))
                mesh.apply_transform(_origin_tf(col.find('origin')))
                parts.append(mesh)
            if not parts:
                continue
            mesh = trimesh.util.concatenate(parts)
            n = max(500, int(mesh.area / 1.5e-4))
            p, _ = trimesh.sample.sample_surface(mesh, n, seed=seed)
            p = np.vstack([np.asarray(p), mesh.vertices])
            self.pts[link.get('name')] = p
            ctr = (p.max(0) + p.min(0)) / 2
            self.ball[link.get('name')] = (ctr, float(np.linalg.norm(p - ctr, axis=1).max()))

    def world_collision(self, q, boxes):
        q = np.asarray(q, float).reshape(-1, self.arm.dof)
        hit = np.zeros(len(q), bool)
        for i in range(0, len(q), 400):
            tfs = self.arm.link_tfs(q[i:i + 400])
            h = np.zeros(len(tfs[next(iter(self.pts))]), bool)
            for l, p in self.pts.items():
                T = tfs[l]
                ctr, rho = self.ball[l]
                wc = T[:, :3, :3] @ ctr + T[:, :3, 3]
                for b in boxes:
                    c, half, R = box_frame(b)
                    near = (np.linalg.norm(wc - c, axis=1) < rho + np.linalg.norm(half)) & ~h
                    if not near.any():
                        continue
                    wp = np.einsum('nij,pj->npi', T[near][:, :3, :3], p) + T[near][:, None, :3, 3]
                    inside = (np.abs((wp - c) @ R) <= half).all(axis=2).any(axis=1)
                    h[np.flatnonzero(near)[inside]] = True
            hit[i:i + 400] = h
        return hit


def densify(path, step):
    """Dense states along the polyline, largest joint step <= `step` rad (endpoints included)."""
    pts = np.asarray(path, float)
    if len(pts) < 2:
        return pts
    out = []
    for a, b in zip(pts, pts[1:]):
        k = max(1, int(math.ceil(np.abs(b - a).max() / step)))
        out.append(a + (b - a) * (np.arange(k)[:, None] / k))
    out.append(pts[-1:])
    return np.vstack(out)


def path_length(path):
    p = np.asarray(path, float)
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) if len(p) > 1 else 0.0


# ------------------------------------------------------------------------------------------------
# prepare
# ------------------------------------------------------------------------------------------------
def make_scenes(rng, n_box_scenes, n_slab_scenes):
    scenes = [{'family': 'empty', 'boxes': []}]
    for _ in range(n_box_scenes):
        boxes = []
        for _ in range(4):
            r, th, z = rng.uniform(0.35, 0.9), rng.uniform(0, 2 * math.pi), rng.uniform(0.1, 0.8)
            boxes.append({'center': [r * math.cos(th), r * math.sin(th), z],
                          'size': rng.uniform(0.10, 0.35, 3).tolist(), 'yaw': float(rng.uniform(0, math.pi))})
        scenes.append({'family': 'boxes', 'boxes': boxes})
    for _ in range(n_slab_scenes):
        phi, d = rng.uniform(0, 2 * math.pi), rng.uniform(0.45, 0.60)
        scenes.append({'family': 'slab', 'boxes': [{
            'center': [d * math.cos(phi), d * math.sin(phi), 0.45], 'size': [0.06, 1.2, 0.9], 'yaw': float(phi)}]})
    return scenes


def cmd_prepare(a):
    work = pathlib.Path(a.work)
    work.mkdir(parents=True, exist_ok=True)
    arm = Arm(a.urdf)
    rng = np.random.default_rng(a.seed)
    sets = parse_sets(a.set)
    ref = [SphereModel(arm, *sets[n], name=n) for n in a.reference]
    oracle = MeshOracle(arm, a.seed)
    print(f'reference for start/goal validity: free under sets {a.reference} (own radii, buffer and '
          f'self-collision hops) and under the true meshes (world only)')

    scenes = make_scenes(rng, a.box_scenes, a.slab_scenes)
    per = {'empty': a.n, 'boxes': a.n // a.box_scenes, 'slab': a.n // a.slab_scenes}
    pid = 0
    for si, sc in enumerate(scenes):
        sc['id'] = si
        want = per[sc['family']]
        states = []
        tries = 0
        while len(states) < 2 * want:
            q = rng.uniform(-JOINT_LIMIT, JOINT_LIMIT, (4000, arm.dof))
            ok = ~oracle.world_collision(q, sc['boxes'])
            for m in ref:
                ok &= ~m.in_collision(q, sc['boxes'])
            states += list(q[ok])
            tries += 4000
        sc['valid_fraction'] = round(len(states) / tries, 4)
        states = np.array(states[:2 * want])
        sc['problems'] = []
        for k in range(want):
            sc['problems'].append({'id': pid, 'start': states[2 * k].round(6).tolist(),
                                   'goal': states[2 * k + 1].round(6).tolist()})
            pid += 1
        sc['probe_states'] = rng.uniform(-JOINT_LIMIT, JOINT_LIMIT, (a.probes, arm.dof)).round(6).tolist()
        print(f'scene {si} ({sc["family"]}): {want} problems, {sc["valid_fraction"]:.3f} of random states valid')
    doc = {'robot': {'urdf': str(a.urdf), 'joint_names': arm.joint_names}, 'seed': a.seed, 'scenes': scenes,
           'reference_sets': a.reference}
    (work / 'problems.json').write_text(json.dumps(doc))
    # plain-text copy for the C++ side (no JSON parser needed there)
    lines = []
    for sc in scenes:
        lines.append(f"S {sc['id']} {len(sc['boxes'])} {len(sc['problems'])} {len(sc['probe_states'])}")
        for b in sc['boxes']:
            c, h, R = box_frame(b)
            lines.append('B ' + ' '.join(f'{v:.7g}' for v in [*c, *R[:, 0], *R[:, 1], *R[:, 2], *h]))
        for p in sc['problems']:
            lines.append(f"P {p['id']} " + ' '.join(f'{v:.7g}' for v in p['start'] + p['goal']))
        for q in sc['probe_states']:
            lines.append('Q ' + ' '.join(f'{v:.7g}' for v in q))
    (work / 'vamp_problems.txt').write_text('\n'.join(lines) + '\n')
    print(f'wrote {work}/problems.json and vamp_problems.txt ({pid} problems)')


# ------------------------------------------------------------------------------------------------
# run-curobo (inside the gpu image)
# ------------------------------------------------------------------------------------------------
def cmd_run_curobo(a):
    sys.path.insert(0, a.bridge)
    from curobo_bridge.planner import Box, CuroboPlanner
    doc = json.loads(pathlib.Path(a.problems).read_text())
    if a.limit:
        for sc in doc['scenes']:
            sc['problems'] = sc['problems'][:a.limit]
    t0 = time.perf_counter()
    planner = CuroboPlanner(robot=a.cfg, max_boxes=16, warmup=True)
    construct_s = time.perf_counter() - t0
    names = doc['robot']['joint_names']
    assert planner.joint_names == names, (planner.joint_names, names)
    rng = np.random.default_rng(0)
    out = {'backend': 'curobo', 'set': a.name, 'cfg': a.cfg, 'construct_warmup_s': construct_s, 'scenes': []}

    def world(sc):
        boxes = [Box(f'b{i}', tuple(b['center']), tuple(b['size']),
                     (math.cos(b['yaw'] / 2), 0.0, 0.0, math.sin(b['yaw'] / 2))) for i, b in enumerate(sc['boxes'])]
        planner.set_world(boxes)

    # first-call latency: the first plan_joint after warmup, on problem 0 (reported separately)
    sc0 = doc['scenes'][0]
    world(sc0)
    p0 = sc0['problems'][0]
    t = time.perf_counter()
    planner.plan_joint(p0['start'], p0['goal'])
    out['first_call_s'] = time.perf_counter() - t

    for sc in doc['scenes']:
        world(sc)
        rec = {'id': sc['id'], 'problems': []}
        for p in sc['problems']:
            valid = planner.check_states([p['start'], p['goal']])
            t = time.perf_counter()
            res = planner.plan_joint(p['start'], p['goal'])
            wall = time.perf_counter() - t
            rec['problems'].append({
                'id': p['id'], 'success': bool(res.success), 'wall_s': wall, 'solver_s': res.planning_time_s,
                'reason': res.failure_reason_code, 'endpoints_valid': [bool(v) for v in valid],
                'path': np.asarray(res.positions).round(5).tolist() if res.success else []})
        # micro timings: batched validity (1000 random states, GPU kernel + host sync) and one state
        q = rng.uniform(-JOINT_LIMIT, JOINT_LIMIT, (1000, planner.dimension)).tolist()
        planner.check_states(q)
        ts = []
        for _ in range(5):
            t = time.perf_counter()
            mask = planner.check_states(q)
            ts.append(time.perf_counter() - t)
        t1 = []
        for _ in range(20):
            t = time.perf_counter()
            planner.check_states(q[:1])
            t1.append(time.perf_counter() - t)
        rec['micro'] = {'batch1000_us_per_state': float(np.median(ts)) / 1000 * 1e6,
                        'single_state_us': float(np.median(t1)) * 1e6, 'valid_fraction': float(np.mean(mask))}
        out['scenes'].append(rec)
        print(f'scene {sc["id"]} {sc["family"]}: ' + f'{sum(p["success"] for p in rec["problems"])}/{len(rec["problems"])} ok',
              flush=True)
    planner.close()
    pathlib.Path(a.out).write_text(json.dumps(out))


# ------------------------------------------------------------------------------------------------
# run-vamp (host -> docker)
# ------------------------------------------------------------------------------------------------
def cmd_run_vamp(a):
    work = pathlib.Path(a.work).resolve()
    header = pathlib.Path(a.header).resolve()
    out = pathlib.Path(a.out).resolve()
    binary = f'/out/vamp_bench_{a.name}'
    compile_cmd = (f'g++ -w -O3 -std=c++17 -march=x86-64-v3 -mavx2 -DROBOT_HEADER=\'"/hdr/{header.name}"\' '
                   f'-DROBOT_STRUCT={a.struct} -I/opt/vamp/include -I/usr/include/eigen3 /tools/vamp_bench.cpp '
                   f'-L/opt/vamp/lib -lsimdxorshift -lcdd -o {binary}')
    run_cmd = f'{binary} /work/vamp_problems.txt /out/{out.name} {a.name}'
    steps = ([] if a.skip_compile else [compile_cmd]) + ([] if a.compile_only else [run_cmd])
    cmd = ['docker', 'run', '--rm', '--user', f'{__import__("os").getuid()}:{__import__("os").getgid()}',
           '-v', f'{work}:/work', '-v', f'{header.parent}:/hdr:ro', '-v', f'{HERE}:/tools:ro',
           '-v', f'{out.parent}:/out', a.image, 'bash', '-c', 'set -e; ' + ' && '.join(steps)]
    print(' '.join(cmd))
    subprocess.run(cmd, check=True)


# ------------------------------------------------------------------------------------------------
# helpers: sphere-urdf, buffer-cfg
# ------------------------------------------------------------------------------------------------
def cmd_sphere_urdf(a):
    spheres = load_spheres(a.spheres)
    if a.max_per_link:
        spheres = {l: v[np.argsort(-v[:, 3])[:a.max_per_link]] for l, v in spheres.items()}
    tree = ET.parse(a.base)
    for link in tree.getroot().findall('link'):
        for col in link.findall('collision'):
            link.remove(col)
        for c in spheres.get(link.get('name'), []):
            col = ET.SubElement(link, 'collision')
            ET.SubElement(col, 'origin', xyz=' '.join(f'{v:.6f}' for v in c[:3]), rpy='0 0 0')
            ET.SubElement(ET.SubElement(col, 'geometry'), 'sphere', radius=f'{c[3] + a.buffer:.6f}')
    tree.write(a.out)
    print(f'wrote {a.out}: {sum(len(v) for v in spheres.values())} spheres')


def cmd_buffer_cfg(a):
    import yaml
    doc = yaml.safe_load(pathlib.Path(a.cfg).read_text())
    doc['kinematics']['collision_sphere_buffer'] = a.buffer
    pathlib.Path(a.out).write_text(yaml.safe_dump(doc, sort_keys=False))
    print(f'wrote {a.out} (buffer {a.buffer} m)')


# ------------------------------------------------------------------------------------------------
# report
# ------------------------------------------------------------------------------------------------
def pct(v, p):
    return float(np.percentile(v, p)) if len(v) else float('nan')


def load_runs(files, doc):
    """Raw result files -> uniform run dicts. cuRobo writes one json; VAMP writes json lines."""
    runs = []
    sid = {p['id']: sc['id'] for sc in doc['scenes'] for p in sc['problems']}
    for f in files:
        text = pathlib.Path(f).read_text()
        if f.endswith('.jsonl'):
            run = {'backend': 'vamp', 'scenes': []}
            by_scene = {}
            for ln in text.splitlines():
                o = json.loads(ln)
                if o['type'] == 'header':
                    run['set'], run['robot'], run['n_spheres'] = o['set'], o['robot'], o['n_spheres']
                elif o['type'] == 'first_call':
                    run['first_call_s'] = o['us'] * 1e-6
                elif o['type'] == 'plan':
                    rs = by_scene.setdefault(o['scene'], {'id': o['scene'], 'problems': []})
                    o['wall_s'] = o['us'] * 1e-6
                    o['success'] = bool(o['success'])
                    o['endpoints_valid'] = [bool(v) for v in o['endpoints_valid']]
                    rs['problems'].append(o)
                elif o['type'] == 'probes':
                    by_scene.setdefault(o['scene'], {'id': o['scene'], 'problems': []})['probes'] = o['valid']
                elif o['type'] == 'micro':
                    by_scene.setdefault(o['scene'], {'id': o['scene'], 'problems': []})['micro'] = o
            run['scenes'] = [by_scene[k] for k in sorted(by_scene)]
        else:
            run = json.loads(text)
        run['file'] = f
        runs.append(run)
    return runs


def cmd_report(a):
    work = pathlib.Path(a.work)
    doc = json.loads((work / 'problems.json').read_text())
    arm = Arm(a.urdf)
    models = {n: SphereModel(arm, s, b, h, n) for n, (s, b, h) in parse_sets(a.set).items()}
    oracle = MeshOracle(arm, doc['seed'])
    scenes = {sc['id']: sc for sc in doc['scenes']}
    prob_scene = {p['id']: sc for sc in doc['scenes'] for p in sc['problems']}
    prob = {p['id']: p for sc in doc['scenes'] for p in sc['problems']}
    if a.first_k:
        for sc in doc['scenes']:
            sc['problems'] = sc['problems'][:a.first_k]
        prob_scene = {p['id']: sc for sc in doc['scenes'] for p in sc['problems']}
        prob = {p['id']: p for sc in doc['scenes'] for p in sc['problems']}
    runs = load_runs(a.results, doc)
    if a.first_k:
        for r in runs:
            for rs in r['scenes']:
                rs['problems'] = [p for p in rs['problems'] if p['id'] in prob]
    step = a.step

    def judge(q_path, sc):
        """collision flags of a dense path: mesh world, and each model (world+self, own radii)."""
        dense = densify(q_path, step)
        flags = {'mesh': bool(oracle.world_collision(dense, sc['boxes']).any())}
        for n, m in models.items():
            flags[n] = bool(m.in_collision(dense, sc['boxes']).any())
        return flags

    # candidate witnesses per problem: every returned path + the straight line
    witness = {}
    for r in runs:
        for rs in r['scenes']:
            for pr in rs['problems']:
                if pr['success']:
                    pr['flags'] = judge(pr['path'], prob_scene[pr['id']])
                    pr['length'] = path_length(pr['path'])
                    witness.setdefault(pr['id'], []).append(pr['flags'])
    for pid, p in prob.items():
        fl = judge([p['start'], p['goal']], prob_scene[pid])
        witness.setdefault(pid, []).append(fl)
    feasible = {pid for pid, w in witness.items() if any(not f['mesh'] for f in w)}

    agreement = {}
    for r in runs:
        if r['backend'] != 'vamp' or r['set'] not in models:
            continue
        agree = tot = 0
        for rs in r['scenes']:
            sc = scenes[rs['id']]
            q = np.array(sc['probe_states'])
            mine = ~models[r['set']].in_collision(q, sc['boxes'])
            theirs = np.array(rs['probes'], bool)
            agree += int((mine == theirs).sum())
            tot += len(q)
        agreement[r['set']] = [agree, tot]
    rows = []
    for r in runs:
        label = (r['backend'], r['set'])
        for fam in FAMILIES:
            ps = [pr for rs in r['scenes'] if scenes[rs['id']]['family'] == fam for pr in rs['problems']]
            if not ps:
                continue
            ok = [p for p in ps if p['success']]
            wall = [p['wall_s'] * 1e3 for p in ok]
            allwall = [p['wall_s'] * 1e3 for p in ps]
            fail = [p for p in ps if not p['success']]
            mesh_bad = [p for p in ok if p['flags']['mesh']]
            other = {n: sum(p['flags'][n] for p in ok) for n in models}
            own = other.get(r['set'], 0)
            # failed although a mesh-clean path exists (from another config or the straight line):
            # blocked by this set's own spheres (every witness collides under them) vs a planner miss
            fail_feasible = [p for p in fail if p['id'] in feasible]
            blocked_by_set = 0
            if r['set'] in models:
                for p in fail_feasible:
                    clean = [f for f in witness[p['id']] if not f['mesh']]
                    blocked_by_set += all(f[r['set']] for f in clean)
            rows.append({
                'backend': r['backend'], 'set': r['set'], 'family': fam, 'n': len(ps), 'ok': len(ok),
                'success': len(ok) / len(ps), 'time_ms_median': pct(wall, 50), 'time_ms_p95': pct(wall, 95),
                'time_ms_median_all': pct(allwall, 50),
                'len_median': pct([p['length'] for p in ok], 50),
                'mesh_collide_paths': len(mesh_bad), 'collide_under': other,
                'fail_feasible': len(fail_feasible), 'fail_blocked_by_own_spheres': blocked_by_set,
                'first_call_s': r.get('first_call_s'),
                'endpoints_invalid': sum(not all(p['endpoints_valid']) for p in ps)})
    runinfo = []
    for r in runs:
        ms = [rs['micro'] for rs in r['scenes'] if rs.get('micro')]
        info = {'backend': r['backend'], 'set': r['set'], 'first_call_s': r.get('first_call_s'),
                'construct_warmup_s': r.get('construct_warmup_s')}
        for k in ('batch1000_us_per_state', 'single_state_us', 'state_ns', 'motion_us'):
            if ms and k in ms[0]:
                info[k] = float(np.median([m[k] for m in ms]))
        runinfo.append(info)
    (work / 'results.json').write_text(json.dumps({'runs': runinfo, 'rows': rows, 'n_feasible_problems': len(feasible),
                                                   'n_problems': len(prob), 'step_rad': step,
                                                   'vamp_probe_agreement': agreement}, indent=1))
    print('VAMP probe-state validity vs numpy model of the same set [agree, total]:', agreement)
    print(f'{len(feasible)}/{len(prob)} problems have at least one mesh-clean witness path (any config or straight line)')
    for info in runinfo:
        print('run', {k: (round(v, 4) if isinstance(v, float) else v) for k, v in info.items()})
    names = list(models)
    head = ['backend', 'set', 'scene', 'ok/n', 'success', 'med ms', 'p95 ms', 'len rad', 'mesh-hit'] + \
           [f'hit {n}' for n in names] + ['fail,feasible', 'of which blocked by set', 'endpoint invalid']
    print('| ' + ' | '.join(head) + ' |')
    print('|' + '---|' * len(head))
    for r in rows:
        print(f"| {r['backend']} | {r['set']} | {r['family']} | {r['ok']}/{r['n']} | {r['success']*100:.0f}% | "
              f"{r['time_ms_median']:.1f} | {r['time_ms_p95']:.1f} | {r['len_median']:.2f} | {r['mesh_collide_paths']} | "
              + ' | '.join(str(r['collide_under'][n]) for n in names)
              + f" | {r['fail_feasible']} | {r['fail_blocked_by_own_spheres']} | {r['endpoints_invalid']} |")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='stage', required=True)

    p = sub.add_parser('prepare')
    p.add_argument('--work', required=True)
    p.add_argument('--urdf', required=True, help='meshed UR10e URDF (collision meshes next to it)')
    p.add_argument('--set', action='append', required=True, metavar='NAME=PATH',
                   help='sphere sets (spherized URDF or cuRobo/XRDF yaml, optional @buffer_m), repeatable')
    p.add_argument('--reference', nargs='+', required=True, help='set names a start/goal must be free under')
    p.add_argument('--n', type=int, default=100, help='problems per scene family')
    p.add_argument('--box-scenes', type=int, default=5)
    p.add_argument('--slab-scenes', type=int, default=5)
    p.add_argument('--probes', type=int, default=300, help='random probe states per scene (VAMP agreement check)')
    p.add_argument('--seed', type=int, default=0)
    p.set_defaults(fn=cmd_prepare)

    p = sub.add_parser('run-curobo')
    p.add_argument('--problems', required=True)
    p.add_argument('--cfg', required=True, help='cuRobo robot yml (as seen inside the container)')
    p.add_argument('--name', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--limit', type=int, default=0, help='only the first N problems of each scene')
    p.add_argument('--bridge', default='/bridge', help='dir holding the curobo_bridge package')
    p.set_defaults(fn=cmd_run_curobo)

    p = sub.add_parser('run-vamp')
    p.add_argument('--work', required=True)
    p.add_argument('--header', required=True, help='Cricket-generated robot header')
    p.add_argument('--struct', required=True, help='struct name in the header')
    p.add_argument('--name', required=True, help='sphere-set label')
    p.add_argument('--out', required=True)
    p.add_argument('--image', default='motion-planner:full')
    p.add_argument('--compile-only', action='store_true', help='build /out/vamp_bench_<name> and stop')
    p.add_argument('--skip-compile', action='store_true', help='run the binary a previous call built')
    p.set_defaults(fn=cmd_run_vamp)

    p = sub.add_parser('sphere-urdf')
    p.add_argument('--base', required=True, help='spherized URDF whose frames/joints are reused')
    p.add_argument('--spheres', required=True, help='cuRobo yml or spherized URDF to take spheres from')
    p.add_argument('--out', required=True)
    p.add_argument('--buffer', type=float, default=0.0)
    p.add_argument('--max-per-link', type=int, default=0, help='keep only the N largest spheres per link')
    p.set_defaults(fn=cmd_sphere_urdf)

    p = sub.add_parser('buffer-cfg')
    p.add_argument('--cfg', required=True)
    p.add_argument('--buffer', type=float, required=True)
    p.add_argument('--out', required=True)
    p.set_defaults(fn=cmd_buffer_cfg)

    p = sub.add_parser('report')
    p.add_argument('--work', required=True)
    p.add_argument('--urdf', required=True)
    p.add_argument('--set', action='append', required=True, metavar='NAME=PATH')
    p.add_argument('--results', nargs='+', required=True)
    p.add_argument('--first-k', type=int, default=0, help='restrict every run to the first K problems per scene')
    p.add_argument('--step', type=float, default=0.02, help='oracle path sampling, largest joint step in rad')
    p.set_defaults(fn=cmd_report)

    a = ap.parse_args()
    a.fn(a)


if __name__ == '__main__':
    main()
