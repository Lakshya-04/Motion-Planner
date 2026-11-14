#!/usr/bin/env python3
"""URDF -> MuJoCo loader, plus the dependency-light helpers the cross-check shares.

The URDF is the fixed-height arm model expanded from `mobile_manipulator_description`
(`xacro ... arm:=ur10e lift_fixed_height:=0.5`, then `resolve_package_uris.py`, then
`primitives_to_meshes.py` for the generic arm). MuJoCo's URDF importer is told, through an injected
`<mujoco><compiler/></mujoco>` block, to keep fixed-joint bodies (so `tcp_link` exists as a body) and to
use collision geometry only.

Everything here except `load_mujoco` / `mujoco_*` is pure numpy (no mujoco, no GPU), so the tests run
anywhere.

    mujoco_model.py robot.urdf         # prints the compiled model summary
"""
import itertools
import math
import pathlib
import sys
import xml.etree.ElementTree as ET

import numpy as np

COMPILER = {'fusestatic': 'false', 'discardvisual': 'true', 'strippath': 'false', 'balanceinertia': 'true'}


# ---------------------------------------------------------------- URDF kinematics (independent FK)
def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _vec(text, default='0 0 0'):
    return np.array([float(v) for v in (text or default).split()])


def origin_transform(elem):
    t = np.eye(4)
    if elem is not None:
        t[:3, :3] = rpy_matrix(*_vec(elem.get('rpy')))
        t[:3, 3] = _vec(elem.get('xyz'))
    return t


def axis_angle_matrix(axis, theta):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(theta) * k + (1 - math.cos(theta)) * k @ k


class UrdfKinematics:
    """Minimal revolute/fixed URDF forward kinematics, written from the URDF spec only."""

    def __init__(self, urdf):
        root = ET.parse(str(urdf)).getroot()
        self.joints = list(root.iter('joint'))
        self.by_child = {j.find('child').get('link'): j for j in self.joints}
        self.links = [l.get('name') for l in root.iter('link')]
        self.dof = [j.get('name') for j in self.joints if j.get('type') in ('revolute', 'continuous')]
        self.limits = {}
        for j in self.joints:
            if j.get('type') == 'revolute':
                lim = j.find('limit')
                self.limits[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
            elif j.get('type') == 'continuous':
                self.limits[j.get('name')] = (-math.pi, math.pi)
        unsupported = [j.get('name') for j in self.joints if j.get('type') not in ('fixed', 'revolute', 'continuous')]
        if unsupported:
            raise ValueError(f'only fixed/revolute joints supported (pin the lift): {unsupported}')

    def link_pose(self, link, q):
        """4x4 pose of `link` in the URDF root frame; `q` maps joint name -> angle."""
        j = self.by_child.get(link)
        if j is None:
            return np.eye(4)
        t = self.link_pose(j.find('parent').get('link'), q) @ origin_transform(j.find('origin'))
        if j.get('type') in ('revolute', 'continuous'):
            r = np.eye(4)
            r[:3, :3] = axis_angle_matrix(_vec(j.find('axis').get('xyz'), '1 0 0'), q[j.get('name')])
            t = t @ r
        return t

    def all_poses(self, q):
        cache = {}

        def pose(link):
            if link not in cache:
                j = self.by_child.get(link)
                if j is None:
                    cache[link] = np.eye(4)
                else:
                    t = pose(j.find('parent').get('link')) @ origin_transform(j.find('origin'))
                    if j.get('type') in ('revolute', 'continuous'):
                        r = np.eye(4)
                        r[:3, :3] = axis_angle_matrix(_vec(j.find('axis').get('xyz'), '1 0 0'), q[j.get('name')])
                        t = t @ r
                    cache[link] = t
            return cache[link]
        return pose

    def parent_link(self, link):
        j = self.by_child.get(link)
        return None if j is None else j.find('parent').get('link')

    def is_fixed(self, link):
        j = self.by_child.get(link)
        return j is not None and j.get('type') == 'fixed'


# ---------------------------------------------------------------- analytic UR10e FK
UR10E_DH = dict(d=[0.1807, 0, 0, 0.17415, 0.11985, 0.11655], a=[0, -0.6127, -0.57155, 0, 0, 0],
                alpha=[math.pi / 2, 0, 0, math.pi / 2, -math.pi / 2, 0])


def dh_matrix(theta, d, a, alpha):
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st],
                     [0, sa, ca, d], [0, 0, 0, 1]])


def ur10e_dh_fk(q, dh=UR10E_DH):
    """Standard Universal Robots DH flange pose in the DH base frame (z up, x = -x of URDF base_link)."""
    t = np.eye(4)
    for i in range(6):
        t = t @ dh_matrix(q[i], dh['d'][i], dh['a'][i], dh['alpha'][i])
    return t


def ur10e_flange_pose(q):
    """Analytic arm_flange_link pose in arm_base_link (URDF base is the DH base turned by pi about z)."""
    return np.diag([-1.0, -1.0, 1.0, 1.0]) @ ur10e_dh_fk(q)


# ---------------------------------------------------------------- pair filtering shared by all checkers
def allowed_pairs(kin, hops=2):
    """Link pairs ignored in every self-collision verdict.

    Links joined by fixed joints form one rigid group. Two groups are ignored when they are at most
    `hops` revolute joints apart (1 = directly adjacent, MuJoCo's own default; 2 also skips the
    neighbour-of-neighbour pairs a sphere model bulges into). Links of the same group are ignored.
    """
    group = {}

    def root_of(link):
        while kin.is_fixed(link):
            link = kin.parent_link(link)
        return link
    for l in kin.links:
        group[l] = root_of(l)
    parent_group = {}
    for g in set(group.values()):
        p = kin.parent_link(g)
        parent_group[g] = None if p is None else group[p]

    def ancestors(g):
        out = [g]
        while parent_group.get(out[-1]) is not None:
            out.append(parent_group[out[-1]])
        return out

    def tree_distance(a, b):
        aa, bb = ancestors(a), ancestors(b)
        common = next(x for x in aa if x in bb)
        return aa.index(common) + bb.index(common)
    out = set()
    for a, b in itertools.combinations(kin.links, 2):
        if group[a] == group[b] or tree_distance(group[a], group[b]) <= hops:
            out.add(frozenset((a, b)))
    return out


# ---------------------------------------------------------------- sphere models
def load_foam_spheres(urdf):
    """link -> list of (centre xyz in link frame, radius) from a FOAM-spherized URDF."""
    spheres = {}
    for link in ET.parse(str(urdf)).getroot().iter('link'):
        for c in link.findall('collision'):
            s = c.find('.//sphere')
            if s is not None:
                spheres.setdefault(link.get('name'), []).append((_vec(c.find('origin').get('xyz')), float(s.get('radius'))))
    return spheres


def load_curobo_spheres(cfg):
    """link -> list of (centre, radius) from a cuRobo robot yml (collision_spheres: link: [{center, radius}])."""
    import yaml
    data = yaml.safe_load(open(cfg))
    node = data
    while 'collision_spheres' not in node:
        node = next(v for v in node.values() if isinstance(v, dict))
    spheres = {}
    for link, items in node['collision_spheres'].items():
        spheres[link] = [(np.array(i['center'], float), float(i['radius'])) for i in items]
    return spheres


def sphere_world(kin, spheres, q):
    """link -> (centres n x 3 in the URDF root frame, radii n) for configuration `q` (joint -> angle)."""
    pose = kin.all_poses(q)
    world = {}
    for link, items in spheres.items():
        t = pose(link)
        centres = np.array([c for c, _ in items])
        world[link] = (centres @ t[:3, :3].T + t[:3, 3], np.array([r for _, r in items]))
    return world


def overlapping_pairs(world, allowed, margin=0.0):
    """Non-allowed link pairs of `sphere_world` output with any overlapping sphere pair."""
    bad = []
    for a, b in itertools.combinations(sorted(world), 2):
        if frozenset((a, b)) in allowed:
            continue
        (pa, ra), (pb, rb) = world[a], world[b]
        d = np.linalg.norm(pa[:, None, :] - pb[None, :, :], axis=2)
        if np.any(d < ra[:, None] + rb[None, :] + margin):
            bad.append((a, b))
    return bad


def sphere_colliding_pairs(kin, spheres, q, allowed, margin=0.0):
    """Non-allowed link pairs whose spheres overlap in configuration `q` (dict joint -> angle)."""
    return overlapping_pairs(sphere_world(kin, spheres, q), allowed, margin)


# ---------------------------------------------------------------- metrics
def verdict_rates(truth, other):
    """Agreement and error rates of boolean 'in collision' verdicts `other` against `truth`.

    false_free: truth collides, other says free (unsafe).  false_blocked: truth free, other collides.
    Rates are over all samples; the conditional rates divide by the truth class size.
    """
    truth, other = np.asarray(truth, bool), np.asarray(other, bool)
    n = len(truth)
    ff = int(np.sum(truth & ~other))
    fb = int(np.sum(~truth & other))
    n_col, n_free = int(truth.sum()), int((~truth).sum())
    return {'n': n, 'agree': float(np.mean(truth == other)), 'false_free': ff / n, 'false_blocked': fb / n,
            'false_free_given_collision': ff / n_col if n_col else float('nan'),
            'false_blocked_given_free': fb / n_free if n_free else float('nan'),
            'truth_collision_rate': n_col / n, 'other_collision_rate': float(other.mean())}


def sample_configs(kin, n, seed, joints=None):
    """n x dof uniform samples within the URDF joint limits (seeded)."""
    joints = joints or kin.dof
    rng = np.random.default_rng(seed)
    lo = np.array([kin.limits[j][0] for j in joints])
    hi = np.array([kin.limits[j][1] for j in joints])
    return rng.uniform(lo, hi, size=(n, len(joints)))


def pose_error(ta, tb):
    """(position error m, rotation error rad) between two 4x4 poses."""
    dp = float(np.linalg.norm(ta[:3, 3] - tb[:3, 3]))
    c = (np.trace(ta[:3, :3].T @ tb[:3, :3]) - 1) / 2
    return dp, float(math.acos(max(-1.0, min(1.0, c))))


# ---------------------------------------------------------------- MuJoCo side
def patch_urdf(src, dst=None):
    """Write a copy of the URDF with the MuJoCo compiler block injected; returns its path.

    The copy must sit in the same directory as the original so relative mesh paths still resolve.
    """
    src = pathlib.Path(src)
    dst = pathlib.Path(dst) if dst else src.with_name(src.stem + '.mujoco.urdf')
    tree = ET.parse(str(src))
    root = tree.getroot()
    for old in root.findall('mujoco'):
        root.remove(old)
    mj = ET.Element('mujoco')
    ET.SubElement(mj, 'compiler', COMPILER)
    root.insert(0, mj)
    tree.write(dst, xml_declaration=True, encoding='utf-8')
    return dst


def load_mujoco(urdf):
    import mujoco
    patched = patch_urdf(urdf)
    try:
        return mujoco.MjModel.from_xml_path(str(patched))
    finally:
        patched.unlink(missing_ok=True)


def hull_vertices(points, max_vertices):
    """Vertices of the convex hull of `points`, thinned by farthest-point sampling to <= max_vertices.

    The thinned hull is inscribed in the original one (slightly smaller), which is the price of making
    mesh-mesh collision cheap enough for MJX.
    """
    from scipy.spatial import ConvexHull
    pts = np.asarray(points, float)
    v = pts[ConvexHull(pts).vertices]
    if len(v) <= max_vertices:
        return v
    chosen = [int(np.argmax(np.linalg.norm(v - v.mean(0), axis=1)))]
    dist = np.linalg.norm(v - v[chosen[0]], axis=1)
    while len(chosen) < max_vertices:
        chosen.append(int(np.argmax(dist)))
        dist = np.minimum(dist, np.linalg.norm(v - v[chosen[-1]], axis=1))
    return v[chosen]


def simplify_meshes(urdf, out_dir, max_vertices=32):
    """Copy a URDF into `out_dir` with every collision mesh replaced by a thinned convex hull STL.

    Mesh files are written to `out_dir/hulls/` and referenced by absolute path. Returns the new URDF path.
    """
    import trimesh
    urdf, out = pathlib.Path(urdf), pathlib.Path(out_dir)
    (out / 'hulls').mkdir(parents=True, exist_ok=True)
    tree = ET.parse(str(urdf))
    for i, mesh in enumerate(tree.getroot().iter('mesh')):
        src = (urdf.parent / mesh.get('filename')).resolve()
        loaded = trimesh.load(src, force='mesh')
        scale = _vec(mesh.get('scale'), '1 1 1')
        pts = hull_vertices(loaded.vertices * scale, max_vertices)
        dst = (out / 'hulls' / f'{i}_{src.stem}.stl').resolve()
        trimesh.convex.convex_hull(pts).export(dst)
        mesh.set('filename', str(dst))
        mesh.attrib.pop('scale', None)
    path = out / 'robot_hulls.urdf'
    tree.write(path, xml_declaration=True, encoding='utf-8')
    return path


def mujoco_qpos_addresses(model, joints):
    import mujoco
    return [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in joints]


def mujoco_joint_ranges(model, joints):
    import mujoco
    return {j: tuple(model.jnt_range[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)]) for j in joints}


def mujoco_pose(model, data, body):
    import mujoco
    b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
    t = np.eye(4)
    t[:3, :3] = data.xmat[b].reshape(3, 3)
    t[:3, 3] = data.xpos[b]
    return t


def mujoco_colliding_pairs(model, data, allowed):
    """Non-allowed body-name pairs with a penetrating contact (dist < 0) after mj_forward."""
    import mujoco
    bad = set()
    for c in data.contact[:data.ncon]:
        if c.dist >= 0:
            continue
        a = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[c.geom1])
        b = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[c.geom2])
        if frozenset((a, b)) not in allowed:
            bad.add(tuple(sorted((a, b))))
    return sorted(bad)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    import mujoco
    m = load_mujoco(sys.argv[1])
    kin = UrdfKinematics(sys.argv[1])
    print(f'mujoco {mujoco.__version__}: nbody={m.nbody} njnt={m.njnt} ngeom={m.ngeom} nmesh={m.nmesh}')
    print('joint ranges (mujoco):', {k: tuple(round(x, 3) for x in v) for k, v in mujoco_joint_ranges(m, kin.dof).items()})
