"""Home-pose check for every profile: the profile's `home` / ready pose must be inside the joint
limits of the rendered URDF and must not self-collide.

Self-collision model (host-runnable, no FCL / MoveIt): every collision shape of the arm and tool
(everything below `arm_base_link`) becomes a capsule (a cylinder by its own axis and radius,
inscribed so the flat ends are optimistic; a sphere as a point capsule). The lift column's shapes
are obstacles for the arm (boxes exactly, as oriented boxes). Forward kinematics are computed from
the URDF (mimic joints follow their master). Pairs of bodies up to `HOPS` moving joints apart are
not checked (fixed joints weld two links into one body), the same rule the sphere tools use for the
structural pairs (`docs/robot_assets.md`).

This is a planning-grade check on the primitive geometry the repository ships (cylinders along the
kinematic chain), not a mesh check: a pass says the primitive model is free at home, not that the
real arm is. A profile that fails here fails honestly (the test reports the colliding pair and the
penetration depth); it is not skipped or loosened. Profiles that need `ur_description` are skipped
when it is missing, as in the other description tests.
"""
import itertools
import os
import pathlib
import tempfile

import numpy as np
import pytest
import yaml

xacro = pytest.importorskip('xacro')
urdf_parser = pytest.importorskip('urdf_parser_py.urdf')
URDF = urdf_parser.URDF

PKG = pathlib.Path(__file__).resolve().parents[1]
NAME = 'mobile_manipulator_description'
PROFILES = sorted((PKG / 'config' / 'profiles').glob('*.yaml'))
HOPS = 2
SAMPLES = 24      # points along a capsule axis when it is tested against a box
LIMIT_TOL = 1e-6


@pytest.fixture(scope='module', autouse=True)
def ament_prefix():
    tmp = tempfile.TemporaryDirectory()
    root = pathlib.Path(tmp.name)
    (root / 'share').mkdir()
    (root / 'share' / NAME).symlink_to(PKG)
    idx = root / 'share' / 'ament_index' / 'resource_index' / 'packages'
    idx.mkdir(parents=True)
    (idx / NAME).touch()
    old = os.environ.get('AMENT_PREFIX_PATH', '')
    os.environ['AMENT_PREFIX_PATH'] = f'{root}:{old}' if old else str(root)
    yield
    os.environ['AMENT_PREFIX_PATH'] = old
    tmp.cleanup()


def _has_ur_description():
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('ur_description')
        return True
    except Exception:  # noqa: BLE001
        return False


def _args(profile):
    import importlib.util
    spec = importlib.util.spec_from_file_location('profile_xacro_args',
                                                  PKG / 'scripts' / 'profile_xacro_args.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.xacro_args(profile)


# ---------------------------------------------------------------- kinematics

def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _transform(origin):
    t = np.eye(4)
    if origin is not None:
        t[:3, :3] = _rpy(*origin.rpy)
        t[:3, 3] = origin.xyz
    return t


def _axis_rotation(axis, angle):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)


def joint_values(robot, positions):
    """Positions of the actuated joints plus every mimic joint resolved from its master."""
    values = dict(positions)
    for j in robot.joints:
        if j.mimic is not None:
            values[j.name] = j.mimic.multiplier * values[j.mimic.joint] + j.mimic.offset
    return values


def link_poses(robot, values):
    """4x4 pose of every link in the URDF root frame."""
    poses = {robot.get_root(): np.eye(4)}
    children = {}
    for j in robot.joints:
        children.setdefault(j.parent, []).append(j)
    stack = [robot.get_root()]
    while stack:
        parent = stack.pop()
        for j in children.get(parent, []):
            motion = np.eye(4)
            q = values.get(j.name, 0.0)
            if j.type in ('revolute', 'continuous'):
                motion[:3, :3] = _axis_rotation(j.axis, q)
            elif j.type == 'prismatic':
                motion[:3, 3] = np.asarray(j.axis, float) / np.linalg.norm(j.axis) * q
            poses[j.child] = poses[parent] @ _transform(j.origin) @ motion
            stack.append(j.child)
    return poses


def bodies(robot):
    """Union-find over fixed joints: link name -> body id; and the tree of moving joints."""
    parent = {l.name: l.name for l in robot.links}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for j in robot.joints:
        if j.type == 'fixed':
            parent[find(j.child)] = find(j.parent)
    graph = {}
    for j in robot.joints:
        if j.type != 'fixed':
            a, b = find(j.parent), find(j.child)
            graph.setdefault(a, set()).add(b)
            graph.setdefault(b, set()).add(a)
    return find, graph


def hop_distance(graph, a, b, limit):
    if a == b:
        return 0
    frontier, seen = {a}, {a}
    for d in range(1, limit + 1):
        frontier = {n for f in frontier for n in graph.get(f, ())} - seen
        if b in frontier:
            return d
        seen |= frontier
    return limit + 1


# ---------------------------------------------------------------- geometry

def descendants(robot, root):
    kids = {}
    for j in robot.joints:
        kids.setdefault(j.parent, []).append(j.child)
    out, stack = set(), [root]
    while stack:
        n = stack.pop()
        out.add(n)
        stack.extend(kids.get(n, []))
    return out


def shapes(robot, poses, names):
    """(link, kind, data) for every collision shape of `names`: capsule = (p0, p1, r),
    box = (centre, rotation, half extents). Arm shapes are never boxes in the shipped arms."""
    out = []
    for name in sorted(names):
        link = robot.link_map[name]
        cols = link.collisions if hasattr(link, 'collisions') and link.collisions else (
            [link.collision] if link.collision is not None else [])
        for col in cols:
            t = poses[name] @ _transform(col.origin)
            c, rot = t[:3, 3], t[:3, :3]
            g = col.geometry
            if hasattr(g, 'length') and hasattr(g, 'radius'):          # cylinder, axis = local z
                # inscribed capsule: the axis is shortened by the radius at each end, so the
                # hemispherical caps do not poke out of the flat ends (which would flag every
                # cylinder that butts against the next one, e.g. the lift tube under the arm)
                h = rot[:, 2] * max(g.length / 2.0 - g.radius, 0.0)
                out.append((name, 'capsule', (c - h, c + h, g.radius)))
            elif hasattr(g, 'size'):                                   # box
                out.append((name, 'box', (c, rot, np.asarray(g.size, float) / 2.0)))
            elif hasattr(g, 'radius'):                                 # sphere
                out.append((name, 'capsule', (c, c, g.radius)))
    return out


def segment_distance(p1, q1, p2, q2):
    """Smallest distance between segments p1-q1 and p2-q2 (Ericson, Real-Time Collision Detection)."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    eps = 1e-12
    if a <= eps and e <= eps:
        return float(np.linalg.norm(r))
    if a <= eps:
        s, t = 0.0, np.clip(f / e, 0, 1)
    else:
        c = d1 @ r
        if e <= eps:
            t, s = 0.0, np.clip(-c / a, 0, 1)
        else:
            b = d1 @ d2
            denom = a * e - b * b
            s = np.clip((b * f - c * e) / denom, 0, 1) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0:
                t, s = 0.0, np.clip(-c / a, 0, 1)
            elif t > 1:
                t, s = 1.0, np.clip((b - c) / a, 0, 1)
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


def box_distance(point, centre, rot, half):
    """Distance from a point to an oriented box (0 inside)."""
    local = rot.T @ (point - centre)
    return float(np.linalg.norm(np.maximum(np.abs(local) - half, 0.0)))


def penetration(a, b):
    """Positive depth (m) when two shapes overlap, else <= 0."""
    (_, ka, da), (_, kb, db) = a, b
    if ka == 'box' and kb == 'box':
        return -1.0                                  # only arm-vs-lift is checked, never box-box
    if ka == 'box':
        a, b, ka, kb, da, db = b, a, kb, ka, db, da
    p0, p1, r = da
    if kb == 'capsule':
        return r + db[2] - segment_distance(p0, p1, db[0], db[1])
    centre, rot, half = db
    pts = [p0 + (p1 - p0) * s for s in np.linspace(0.0, 1.0, SAMPLES)]
    return r - min(box_distance(p, centre, rot, half) for p in pts)


def self_collisions(robot, arm_root, positions):
    """Overlapping (link_a, link_b, depth) pairs of the arm subtree and the lift column."""
    values = joint_values(robot, positions)
    poses = link_poses(robot, values)
    arm_links = descendants(robot, arm_root)
    lift_links = descendants(robot, 'lift_base_link') - arm_links
    arm_shapes = shapes(robot, poses, arm_links)
    lift_shapes = shapes(robot, poses, lift_links)
    find, graph = bodies(robot)
    found = {}
    pairs = list(itertools.combinations(arm_shapes, 2)) + list(itertools.product(arm_shapes, lift_shapes))
    for a, b in pairs:
        if hop_distance(graph, find(a[0]), find(b[0]), HOPS) <= HOPS:
            continue
        depth = penetration(a, b)
        if depth > 1e-6:
            key = tuple(sorted((a[0], b[0])))
            found[key] = max(found.get(key, 0.0), depth)
    return sorted((k[0], k[1], d) for k, d in found.items())


# ---------------------------------------------------------------- tests

def _render(profile):
    mappings = dict(_args(profile))
    mappings['profile'] = profile['_name']
    return URDF.from_xml_string(xacro.process_file(
        str(PKG / 'urdf' / 'mobile_manipulator.urdf.xacro'), mappings=mappings).toxml())


def _load(path):
    profile = yaml.safe_load(path.read_text())
    profile['_name'] = path.stem
    if profile['robot']['xacro_args']['arm'].startswith('ur') and not _has_ur_description():
        pytest.skip('ur_description not installed (UR arms render through it)')
    return profile


@pytest.mark.parametrize('path', PROFILES, ids=lambda p: p.stem)
def test_home_pose_is_inside_joint_limits(path):
    profile = _load(path)
    robot = _render(profile)
    home = profile['home']
    expected = ([profile['joints']['lift']] if profile['joints']['lift'] else []) + profile['joints']['arm']
    assert sorted(home) == sorted(expected), 'home must list exactly the actuated joints'
    for name, q in home.items():
        joint = robot.joint_map[name]
        if joint.type == 'continuous':
            continue
        assert joint.limit.lower - LIMIT_TOL <= q <= joint.limit.upper + LIMIT_TOL, (
            f'{path.stem}: home {name}={q} outside [{joint.limit.lower}, {joint.limit.upper}]')


@pytest.mark.parametrize('path', PROFILES, ids=lambda p: p.stem)
def test_home_pose_is_not_self_colliding(path):
    profile = _load(path)
    robot = _render(profile)
    hits = self_collisions(robot, profile['frames']['arm_base'], profile['home'])
    assert not hits, f'{path.stem}: home pose self-collides (link, link, depth m): ' + ', '.join(
        f'{a}/{b} {d:.3f}' for a, b, d in hits)


def test_the_model_detects_a_folded_arm():
    """The check is not vacuous: a pose that folds the elbow back through the upper arm collides."""
    profile = _load(PKG / 'config' / 'profiles' / 'generic_6dof_telescopic.yaml')
    robot = _render(profile)
    folded = dict(profile['home'], arm_joint2=-3.0, arm_joint3=3.0)
    assert self_collisions(robot, profile['frames']['arm_base'], folded)


def test_segment_distance_and_box_distance():
    z = np.zeros(3)
    assert segment_distance(z, np.array([1., 0, 0]), np.array([0., 1, 0]), np.array([1., 1, 0])) == pytest.approx(1.0)
    assert segment_distance(z, np.array([1., 0, 0]), np.array([.5, 0, 0]), np.array([.5, 2, 0])) == pytest.approx(0.0)
    assert box_distance(np.array([2., 0, 0]), z, np.eye(3), np.array([.5, .5, .5])) == pytest.approx(1.5)
    assert box_distance(z, z, np.eye(3), np.array([.5, .5, .5])) == 0.0
