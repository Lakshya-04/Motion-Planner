"""Gazebo (SDF) world generator for wall scenes.

Inputs are the files the coverage planner already reads:
  * wall_scene.json   (walls with plane basis and extents, obstacles protruding from a wall)
  * SceneModel YAML   (`objects:` list with box / sphere / cylinder, role obstacle or
                       contact_surface; see docs/scene_model.md)

wall_scene.json frame convention (matches coverage_planner's wall_scene_loader): a wall has an
origin on the plane, in-plane unit vectors basis_u and basis_v, and the outward normal
n = u x v pointing at the robot. The wall becomes a thin box behind the plane, every obstacle a
box on the front face (uv_on_wall x protrusion_max_m).

The scene frame is arbitrary (the as-mapped frame). `align=True` yaws the scene about z so the
wall's horizontal normal points back at the robot (-x), puts the plane `standoff` metres in
front of the robot base and the wall's lowest edge on the floor, so the arm can reach it.
"""
import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

WALL_THICKNESS = 0.10


# ---------------------------------------------------------------- small math
def _quat_from_matrix(R):
    """Rotation matrix to (x, y, z, w)."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0) * 2.0
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def _matrix_from_quat(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _rpy_from_matrix(R):
    sy = math.hypot(R[0, 0], R[1, 0])
    if sy > 1e-9:
        return (math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], sy), math.atan2(R[1, 0], R[0, 0]))
    return (math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], sy), 0.0)


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# ---------------------------------------------------------------- scene -> primitives
class Shape:
    """A static primitive: kind box (size) | sphere (radius) | cylinder (radius, length)."""

    def __init__(self, name, kind, dims, position, rotation, role='obstacle'):
        self.name = name
        self.kind = kind
        self.dims = dims
        self.position = np.asarray(position, dtype=float)
        self.rotation = np.asarray(rotation, dtype=float)
        self.role = role

    def transformed(self, R, t):
        return Shape(self.name, self.kind, self.dims, R @ self.position + t, R @ self.rotation, self.role)


def _unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def wall_scene_shapes(scene, wall_id=0, crop_u=None, crop_v=None):
    """Shapes for one wall of a wall_scene.json dict. crop_* are (min, max) in wall uv metres."""
    wall = next((w for w in scene['walls'] if int(w['id']) == int(wall_id)), None)
    if wall is None:
        raise ValueError(f'no wall with id {wall_id} (have {[w["id"] for w in scene["walls"]]})')
    o = np.array(wall['origin_xyz'], dtype=float)
    u = _unit(wall['basis_u'])
    v = _unit(wall['basis_v'])
    n = _unit(wall['normal_outward'])
    R = np.column_stack([u, v, n])
    u0, u1 = crop_u if crop_u else wall['u_range_m']
    v0, v1 = crop_v if crop_v else wall['v_range_m']
    shapes = [Shape(f'wall_{wall_id}', 'box', (u1 - u0, v1 - v0, WALL_THICKNESS),
                    o + u * (u0 + u1) / 2 + v * (v0 + v1) / 2 - n * WALL_THICKNESS / 2, R, 'wall')]
    for ob in scene.get('obstacles', []):
        if int(ob['parent_wall']) != int(wall_id):
            continue
        a0, a1 = ob['uv_on_wall']['u']
        b0, b1 = ob['uv_on_wall']['v']
        if a1 < u0 or a0 > u1 or b1 < v0 or b0 > v1:
            continue
        h = float(ob['protrusion_max_m'])
        shapes.append(Shape(f'obstacle_{ob["id"]}', 'box', (a1 - a0, b1 - b0, h),
                            o + u * (a0 + a1) / 2 + v * (b0 + b1) / 2 + n * h / 2, R))
    return shapes


def scene_model_shapes(doc):
    """Shapes for a SceneModel YAML dict (native `objects:` form: box, sphere, cylinder)."""
    shapes = []
    for obj in doc.get('objects', []):
        pose = obj.get('pose', {})
        pos = pose.get('position', [0, 0, 0])
        R = _matrix_from_quat(pose['orientation_xyzw']) if 'orientation_xyzw' in pose else np.eye(3)
        kind = obj['type']
        role = 'wall' if obj.get('role') == 'contact_surface' else 'obstacle'
        if kind == 'box':
            shapes.append(Shape(obj['id'], 'box', tuple(obj['size']), pos, R, role))
        elif kind == 'sphere':
            shapes.append(Shape(obj['id'], 'sphere', (float(obj['radius']),), pos, R, role))
        elif kind == 'cylinder':
            shapes.append(Shape(obj['id'], 'cylinder', (float(obj['radius']), float(obj['height'])), pos, R, role))
        else:
            raise ValueError(f"object {obj['id']!r}: type {kind!r} is not supported in the sim world "
                             "(box, sphere, cylinder only)")
    return shapes


def align_to_robot(shapes, wall_corners, standoff):
    """Yaw + translate the scene so the wall faces the robot at +x. wall_corners: 4x3 (scene frame)
    of the wall front face, normal derived from them."""
    c = np.asarray(wall_corners, dtype=float)
    n = np.cross(c[1] - c[0], c[3] - c[0])
    n = n / np.linalg.norm(n)
    yaw = math.pi - math.atan2(n[1], n[0])
    R = _rot_z(yaw)
    centre = R @ c.mean(axis=0)
    t = np.array([standoff - centre[0], -centre[1], 0.0])
    z_min = min((R @ p + t)[2] for p in c)
    t[2] = -z_min
    return [s.transformed(R, t) for s in shapes], R, t


def _wall_front_corners(wall, crop_u=None, crop_v=None):
    o = np.array(wall['origin_xyz'], dtype=float)
    u, v = _unit(wall['basis_u']), _unit(wall['basis_v'])
    u0, u1 = crop_u if crop_u else wall['u_range_m']
    v0, v1 = crop_v if crop_v else wall['v_range_m']
    # counter-clockwise seen from the front (along +n = u x v)
    return [o + u * u0 + v * v0, o + u * u1 + v * v0, o + u * u1 + v * v1, o + u * u0 + v * v1]


# ---------------------------------------------------------------- SDF
def _sub(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, **attrs)
    if text is not None:
        e.text = text
    return e


def _fmt(vals):
    return ' '.join(f'{float(x):.6f}' for x in vals)


def _geometry(parent, shape):
    g = _sub(parent, 'geometry')
    if shape.kind == 'box':
        _sub(_sub(g, 'box'), 'size', _fmt(shape.dims))
    elif shape.kind == 'sphere':
        _sub(_sub(g, 'sphere'), 'radius', _fmt(shape.dims))
    else:
        c = _sub(g, 'cylinder')
        _sub(c, 'radius', _fmt(shape.dims[:1]))
        _sub(c, 'length', _fmt(shape.dims[1:2]))


def _color(shape):
    return '0.8 0.78 0.72 1' if shape.role == 'wall' else '0.85 0.35 0.2 1'


def build_world_sdf(shapes, world_name='wall_world', friction=0.8, step_size=0.001):
    sdf = ET.Element('sdf', version='1.9')
    world = _sub(sdf, 'world', name=world_name)
    phys = _sub(world, 'physics', name='1ms', type='ignored')
    _sub(phys, 'max_step_size', f'{step_size:g}')
    _sub(phys, 'real_time_factor', '1.0')
    for fname, plugin in (('gz-sim-physics-system', 'gz::sim::systems::Physics'),
                          ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
                          ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster'),
                          ('gz-sim-forcetorque-system', 'gz::sim::systems::ForceTorque'),
                          ('gz-sim-contact-system', 'gz::sim::systems::Contact')):
        _sub(world, 'plugin', filename=fname, name=plugin)
    _sub(world, 'gravity', '0 0 -9.81')
    light = _sub(world, 'light', type='directional', name='sun')
    _sub(light, 'cast_shadows', 'false')
    _sub(light, 'pose', '0 0 10 0 0 0')
    _sub(light, 'direction', '-0.5 0.1 -0.9')
    _sub(light, 'diffuse', '0.9 0.9 0.9 1')

    ground = _sub(world, 'model', name='ground')
    _sub(ground, 'static', 'true')
    link = _sub(ground, 'link', name='link')
    for sect in ('collision', 'visual'):
        e = _sub(link, sect, name=sect)
        g = _sub(e, 'geometry')
        p = _sub(g, 'plane')
        _sub(p, 'normal', '0 0 1')
        _sub(p, 'size', '40 40')
        if sect == 'collision':
            _mu(e, friction)
        else:
            _sub(_sub(e, 'material'), 'ambient', '0.5 0.5 0.5 1')

    for s in shapes:
        m = _sub(world, 'model', name=s.name)
        _sub(m, 'static', 'true')
        rpy = _rpy_from_matrix(s.rotation)
        _sub(m, 'pose', _fmt(list(s.position) + list(rpy)))
        l = _sub(m, 'link', name='link')
        col = _sub(l, 'collision', name='collision')
        _geometry(col, s)
        _mu(col, friction)
        vis = _sub(l, 'visual', name='visual')
        _geometry(vis, s)
        mat = _sub(vis, 'material')
        _sub(mat, 'ambient', _color(s))
        _sub(mat, 'diffuse', _color(s))
    ET.indent(sdf)
    return '<?xml version="1.0"?>\n' + ET.tostring(sdf, encoding='unicode') + '\n'


def _mu(collision, mu):
    ode = _sub(_sub(_sub(collision, 'surface'), 'friction'), 'ode')
    _sub(ode, 'mu', f'{mu:g}')
    _sub(ode, 'mu2', f'{mu:g}')


# ---------------------------------------------------------------- entry points
def load_shapes(path, wall_id=0, align=True, standoff=0.9, crop_u=None, crop_v=None):
    """Shapes of a wall_scene.json (`.json`) or a SceneModel YAML (`.yaml`/`.yml`)."""
    ext = os.path.splitext(path)[1].lower()
    if ext == '.json':
        with open(path) as f:
            scene = json.load(f)
        shapes = wall_scene_shapes(scene, wall_id, crop_u, crop_v)
        if align:
            wall = next(w for w in scene['walls'] if int(w['id']) == int(wall_id))
            shapes, _, _ = align_to_robot(shapes, _wall_front_corners(wall, crop_u, crop_v), standoff)
        return shapes
    if ext in ('.yaml', '.yml'):
        import yaml
        with open(path) as f:
            return scene_model_shapes(yaml.safe_load(f))
    raise ValueError(f'unsupported scene file {path!r}: expected .json (wall_scene) or .yaml (SceneModel)')


def generate_world(scene_path, out_path, **kwargs):
    friction = kwargs.pop('friction', 0.8)
    shapes = load_shapes(scene_path, **kwargs)
    with open(out_path, 'w') as f:
        f.write(build_world_sdf(shapes, friction=friction))
    return shapes
