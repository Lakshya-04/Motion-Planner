#!/usr/bin/env python3
"""Compare collision-sphere models of the same robot geometry, per link.

Sphere sets (each optional except the meshed URDF):
  A  FOAM output       --foam-urdf  spherized.urdf   (collision <sphere> elements, link frame)
  B  cuRobo v2 fit     --curobo-cfg robot.yml        (output of generate_curobo_robot.py)
  C  reference set     --ref-foam-urdf  <spherized urdf>   (previous production model, hand-adjusted FOAM)
  D  reference set     --ref-curobo-xrdf <xrdf or cuRobo yml>   (previous production model, cuRobo side)
Sets are matched to the true geometry by link name. Links a set carries that have no mesh in the
meshed URDF (and links with a mesh but no spheres) are reported, not forced.

The true geometry is each link's <collision> meshes of --urdf (mesh paths relative to the URDF, as
produced by primitives_to_meshes.py / FOAM's input), placed with the collision origin, in the link
frame. Metrics per link (CPU, numpy/trimesh/scipy; seeded):
  count        spheres on the link
  coverage     fraction of points sampled inside the mesh that lie inside the sphere union
  over_cm3     volume of the sphere union outside the mesh (sampling), cm^3
  over_ratio   over_cm3 / mesh volume
  gap_mean/max mesh surface -> spheres: distance from surface samples to the sphere union (0 where
               covered), mm. Large = geometry sticking out of the spheres.
  bulge_mean/max spheres -> mesh: distance from sphere-surface samples (outside other spheres) to the
               mesh surface, mm. Large = spheres sticking out of the mesh.
  query_us     CPU time to test one point against the link's spheres (numpy, batched), a proxy for
               collision-query cost, which scales with sphere count.

    compare_spheres.py --urdf meshed/robot.urdf --foam-urdf spherized.urdf --curobo-cfg robot.yml \
        --ref-foam-urdf PATH --ref-curobo-xrdf PATH --json out.json
Needs numpy, scipy, trimesh, pyyaml, rtree and embreex (embreex makes the inside tests ~60x faster; without it the
ray tests take minutes per link): python3 -m venv v && v/bin/pip install numpy scipy trimesh pyyaml rtree embreex.
"""
import argparse
import json
import pathlib
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import yaml
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def _floats(s, default):
    return [float(v) for v in s.split()] if s else default


def _tf(origin):
    T = np.eye(4)
    if origin is not None:
        T[:3, 3] = _floats(origin.get('xyz'), [0, 0, 0])
        T[:3, :3] = Rotation.from_euler('xyz', _floats(origin.get('rpy'), [0, 0, 0])).as_matrix()
    return T


def load_link_meshes(urdf):
    """link name -> one trimesh in the link frame (collision meshes, merged)."""
    urdf = pathlib.Path(urdf)
    out = {}
    for link in ET.parse(urdf).getroot().iter('link'):
        parts = []
        for col in link.findall('collision'):
            m = col.find('geometry/mesh')
            if m is None:
                continue
            mesh = trimesh.load(urdf.parent / m.get('filename'), force='mesh')
            mesh.apply_scale(_floats(m.get('scale'), [1, 1, 1]))
            mesh.apply_transform(_tf(col.find('origin')))
            parts.append(mesh)
        if parts:
            out[link.get('name')] = trimesh.util.concatenate(parts)
    return out


def load_urdf_spheres(urdf):
    """link name -> (n, 4) array of x, y, z, r from collision <sphere> elements."""
    out = {}
    for link in ET.parse(urdf).getroot().iter('link'):
        rows = []
        for col in link.findall('collision'):
            s = col.find('geometry/sphere')
            if s is not None:
                c = _tf(col.find('origin'))[:3, 3]
                rows.append([*c, float(s.get('radius'))])
        if rows:
            out[link.get('name')] = np.array(rows)
    return out


def _find_sphere_dict(node):
    """Find the {link: [{center, radius}, ...]} mapping in a cuRobo yml or an XRDF, wherever it sits."""
    if isinstance(node, dict):
        for key in ('collision_spheres', 'spheres'):
            v = node.get(key)
            if isinstance(v, dict) and v and all(isinstance(x, list) for x in v.values()):
                return v
        for v in node.values():
            r = _find_sphere_dict(v)
            if r is not None:
                return r
    elif isinstance(node, list):
        for v in node:
            r = _find_sphere_dict(v)
            if r is not None:
                return r
    return None


def load_yaml_spheres(path):
    path = pathlib.Path(path)
    doc = yaml.safe_load(path.read_text())
    spheres = _find_sphere_dict(doc)
    if spheres is None:  # cuRobo ymls may reference a separate spheres file
        ref = doc.get('robot_cfg', doc).get('kinematics', {}).get('collision_spheres') if isinstance(doc, dict) else None
        if isinstance(ref, str):
            spheres = _find_sphere_dict(yaml.safe_load((path.parent / ref).read_text()))
    if spheres is None:
        raise SystemExit(f'{path}: no collision sphere table found')
    out = {}
    for link, items in spheres.items():
        rows = [[*map(float, (s['center'] if 'center' in s else s['position'])), float(s['radius'])] for s in items]
        if rows:
            out[link] = np.array(rows)
    return out


def run_foam(urdf, out, image):
    import subprocess
    d = pathlib.Path(urdf).resolve().parent
    subprocess.run(['docker', 'run', '--rm', '--user', f'{__import__("os").getuid()}:{__import__("os").getgid()}',
                    '-v', f'{d}:/work', '-w', '/work', image, '--filename', pathlib.Path(urdf).name,
                    '--output', out, '--database', 'sphere_database.json'], check=True)
    return d / out


def in_union(pts, sph, chunk=20000):
    res = np.zeros(len(pts), bool)
    for i in range(0, len(pts), chunk):
        p = pts[i:i + chunk]
        d2 = ((p[:, None, :] - sph[None, :, :3]) ** 2).sum(-1) - sph[None, :, 3] ** 2
        res[i:i + chunk] = (d2 <= 0).any(1)
    return res


def dist_to_union(pts, sph, chunk=20000):
    """Distance from points to the sphere union (0 inside)."""
    res = np.empty(len(pts))
    for i in range(0, len(pts), chunk):
        p = pts[i:i + chunk]
        d = np.sqrt(((p[:, None, :] - sph[None, :, :3]) ** 2).sum(-1)) - sph[None, :, 3]
        res[i:i + chunk] = np.maximum(d.min(1), 0.0)
    return res


def sphere_surface_samples(sph, n_per, rng):
    v = rng.normal(size=(len(sph), n_per, 3))
    v /= np.linalg.norm(v, axis=2, keepdims=True)
    pts = sph[:, None, :3] + v * sph[:, None, 3:4]
    return pts.reshape(-1, 3)


def contains(mesh, pts):
    """mesh.contains with a bounding-box prefilter (the ray test is the slow part)."""
    res = np.zeros(len(pts), bool)
    near = ((pts >= mesh.bounds[0]) & (pts <= mesh.bounds[1])).all(1)
    if near.any():
        res[near] = mesh.contains(pts[near])
    return res


def metrics(mesh, sph, rng, n_vol=200000, n_surf=30000):
    out = {'count': int(len(sph))}
    lo = np.minimum(mesh.bounds[0], (sph[:, :3] - sph[:, 3:]).min(0))
    hi = np.maximum(mesh.bounds[1], (sph[:, :3] + sph[:, 3:]).max(0))
    pts = rng.uniform(lo, hi, size=(n_vol, 3))
    cov_mesh = contains(mesh, pts)
    cov_sph = in_union(pts, sph)
    box = float(np.prod(hi - lo))
    mesh_vol = cov_mesh.mean() * box  # sampled, so it is consistent with the other volumes
    out['mesh_cm3'] = round(mesh_vol * 1e6, 1)
    out['coverage'] = float((cov_mesh & cov_sph).sum() / max(cov_mesh.sum(), 1))
    over = float((cov_sph & ~cov_mesh).mean() * box)
    out['over_cm3'] = round(over * 1e6, 1)
    out['over_ratio'] = over / max(mesh_vol, 1e-12)

    surf, _ = trimesh.sample.sample_surface(mesh, n_surf, seed=int(rng.integers(1 << 31)))
    gap = dist_to_union(np.asarray(surf), sph)
    out['gap_mean_mm'] = float(gap.mean() * 1e3)
    out['gap_max_mm'] = float(gap.max() * 1e3)

    ss = sphere_surface_samples(sph, max(n_surf // len(sph), 200), rng)
    # keep only points on the union's outer boundary (not buried inside a neighbouring sphere)
    d_other = np.empty(len(ss))
    for i in range(0, len(ss), 20000):
        p = ss[i:i + 20000]
        d = np.sqrt(((p[:, None, :] - sph[None, :, :3]) ** 2).sum(-1)) - sph[None, :, 3]
        d_other[i:i + 20000] = d.min(1)
    ss = ss[d_other > -1e-6]
    dense, _ = trimesh.sample.sample_surface(mesh, 200000, seed=int(rng.integers(1 << 31)))
    d_mesh = cKDTree(np.asarray(dense)).query(ss)[0]
    d_mesh = np.where(contains(mesh, ss), 0.0, d_mesh)  # boundary points inside the mesh do not bulge
    out['bulge_mean_mm'] = float(d_mesh.mean() * 1e3) if len(d_mesh) else 0.0
    out['bulge_max_mm'] = float(d_mesh.max() * 1e3) if len(d_mesh) else 0.0

    q = rng.uniform(lo, hi, size=(20000, 3))
    t = time.perf_counter()
    for _ in range(5):
        in_union(q, sph)
    out['query_us'] = (time.perf_counter() - t) / 5 / len(q) * 1e6
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--urdf', required=True, help='meshed URDF (collision meshes, relative paths)')
    ap.add_argument('--foam-urdf', help='A: FOAM spherized URDF')
    ap.add_argument('--run-foam', metavar='IMAGE', help='run this FOAM image on --urdf to make set A if --foam-urdf is absent')
    ap.add_argument('--curobo-cfg', help='B: cuRobo robot yml from generate_curobo_robot.py')
    ap.add_argument('--ref-foam-urdf', help='C: reference set, spherized URDF (previous production model)')
    ap.add_argument('--ref-curobo-xrdf', help='D: reference set, XRDF or cuRobo yml (previous production model)')
    ap.add_argument('--links', help='comma-separated links to evaluate (default: every link with a mesh)')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--json', help='write results here')
    args = ap.parse_args()

    meshes = load_link_meshes(args.urdf)
    if args.links:
        meshes = {k: v for k, v in meshes.items() if k in args.links.split(',')}
    foam = args.foam_urdf
    if not foam and args.run_foam:
        foam = run_foam(args.urdf, 'spherized.urdf', args.run_foam)
    sets = {}
    if foam:
        sets['A'] = load_urdf_spheres(foam)
    if args.curobo_cfg:
        sets['B'] = load_yaml_spheres(args.curobo_cfg)
    if args.ref_foam_urdf:
        sets['C'] = load_urdf_spheres(args.ref_foam_urdf)
    if args.ref_curobo_xrdf:
        sets['D'] = load_yaml_spheres(args.ref_curobo_xrdf)

    results = {}
    print(f'{"set":3} {"link":16} {"n":>4} {"cover":>6} {"over_cm3":>9} {"over/vol":>8} {"gap mean/max mm":>16} '
          f'{"bulge mean/max mm":>18} {"q us":>6}')
    for name, table in sets.items():
        missing = sorted(set(meshes) - set(table))
        extra = sorted(set(table) - set(meshes))
        results[name] = {'links': {}, 'no_spheres': missing, 'no_mesh': {k: int(len(table[k])) for k in extra}}
        for link, mesh in meshes.items():
            if link not in table:
                continue
            m = metrics(mesh, table[link], np.random.default_rng(args.seed))
            results[name]['links'][link] = m
            print(f'{name:3} {link:16} {m["count"]:4d} {m["coverage"]:6.3f} {m["over_cm3"]:9.1f} {m["over_ratio"]:8.2f} '
                  f'{m["gap_mean_mm"]:7.2f}/{m["gap_max_mm"]:7.2f} {m["bulge_mean_mm"]:8.2f}/{m["bulge_max_mm"]:8.2f} '
                  f'{m["query_us"]:6.2f}')
        L = results[name]['links']
        if L:
            vol = sum(m['mesh_cm3'] for m in L.values())
            results[name]['total'] = {
                'count': sum(m['count'] for m in L.values()),
                'coverage_vol_weighted': sum(m['coverage'] * m['mesh_cm3'] for m in L.values()) / vol,
                'over_ratio': sum(m['over_cm3'] for m in L.values()) / vol,
                'query_us_all_links': sum(m['query_us'] for m in L.values()),
            }
            t = results[name]['total']
            print(f'{name:3} {"TOTAL":16} {t["count"]:4d} {t["coverage_vol_weighted"]:6.3f} {"":9} {t["over_ratio"]:8.2f}')
        if missing:
            print(f'{name}: links with a mesh but no spheres: {", ".join(missing)}')
        if extra:
            print(f'{name}: sphere links with no mesh in this URDF (skipped): '
                  + ', '.join(f'{k}({len(table[k])})' for k in extra))
    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(results, indent=1))


if __name__ == '__main__':
    sys.exit(main())
