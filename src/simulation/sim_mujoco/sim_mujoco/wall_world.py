"""Wall world for the MuJoCo backend, built from the scenes the coverage planner uses.

Inputs (same files as the coverage planner and `scene_model`):
  * `wall_scene.json` (walls with plane basis and uv extents, obstacles as oriented boxes);
  * a `scene_model` native YAML/JSON (`objects:` with id, type, size, pose{position, orientation_xyzw},
    role) or a cuRobo-style world dict (`cuboid:`, `sphere:`, ... with [x y z qw qx qy qz] poses).
Meshes and point clouds in a scene file are skipped (reported in the returned `skipped` list).

Everything is returned as MuJoCo `<geom>` elements for the worldbody. The scene frame is the robot
base frame; `offset` = (x, y, z, yaw) places the whole scene relative to it. Walls are boxes whose
front face lies on the scanned plane and which extend `thickness` behind it (against the outward
normal). Contact surfaces (role `contact_surface` and walls) get a higher friction.
"""
import json
import math
import pathlib
import xml.etree.ElementTree as ET

import yaml

WALL_RGBA = '0.75 0.75 0.7 1'
OBSTACLE_RGBA = '0.9 0.35 0.2 1'


def _fmt(values):
    return ' '.join(f'{v:.9g}' for v in values)


# ------------------------------------------------------------------ rotations (w, x, y, z)
def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def quat_rotate(q, v):
    w, x, y, z = q
    # v' = v + 2w (u x v) + 2 u x (u x v)
    ux, uy, uz = x, y, z
    cx, cy, cz = uy * v[2] - uz * v[1], uz * v[0] - ux * v[2], ux * v[1] - uy * v[0]
    dx, dy, dz = uy * cz - uz * cy, uz * cx - ux * cz, ux * cy - uy * cx
    return (v[0] + 2 * (w * cx + dx), v[1] + 2 * (w * cy + dy), v[2] + 2 * (w * cz + dz))


def quat_from_matrix(cols):
    """Unit quaternion of the rotation whose columns are the three given orthonormal vectors."""
    (m00, m10, m20), (m01, m11, m21), (m02, m12, m22) = cols
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1) * 2
        q = (s / 4, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s)
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1 + m00 - m11 - m22) * 2
        q = ((m21 - m12) / s, s / 4, (m01 + m10) / s, (m02 + m20) / s)
    elif m11 > m22:
        s = math.sqrt(1 + m11 - m00 - m22) * 2
        q = ((m02 - m20) / s, (m01 + m10) / s, s / 4, (m12 + m21) / s)
    else:
        s = math.sqrt(1 + m22 - m00 - m11) * 2
        q = ((m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, s / 4)
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def _normalize(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


# ------------------------------------------------------------------ scene -> primitive list
class Primitive:
    def __init__(self, name, kind, pos, quat, size, role='obstacle'):
        self.name, self.kind, self.pos, self.quat, self.size, self.role = name, kind, pos, quat, size, role

    def __repr__(self):
        return f'Primitive({self.name}, {self.kind}, pos={self.pos}, size={self.size}, role={self.role})'


def _free_rects(u0, u1, v0, v1, holes):
    """The rectangle (u0..u1, v0..v1) minus the rectangles `holes` (u_min, u_max, v_min, v_max), as a list of
    rectangles: cut along every hole's u edges, then keep the v stretches of each slab that no hole covers."""
    cuts = sorted({u0, u1} | {c for h in holes for c in h[:2] if u0 < c < u1})
    rects = []
    for a, b in zip(cuts, cuts[1:]):
        spans = sorted((max(h[2], v0), min(h[3], v1)) for h in holes if h[0] < b and h[1] > a and h[2] < v1 and h[3] > v0)
        v = v0
        for lo, hi in spans:
            if lo > v:
                rects.append((a, b, v, lo))
            v = max(v, hi)
        if v < v1:
            rects.append((a, b, v, v1))
    return rects


def walls_from_scan(scene, thickness=0.1, wall_ids=None, with_obstacles=True):
    """Primitives from a coverage-planner `wall_scene.json` dict.

    An obstacle with `"kind": "opening"` (a window or door hole) is not a box: the wall is cut around its
    `uv_on_wall` rectangle instead (the wall becomes several boxes, `wall_<id>`, `wall_<id>_1`, ...).
    Every other obstacle is an oriented box (`box_center_xyz`, `box_extents_uvn`, `box_orientation_xyzw`).
    """
    prims = []
    for wall in scene['walls']:
        if wall_ids is not None and wall['id'] not in wall_ids:
            continue
        u = _normalize(wall['basis_u'])
        n = _normalize(wall['normal_outward'])
        v = _cross(n, u)  # right-handed (u, v, n); equals basis_v up to scan noise
        u0, u1 = wall['u_range_m']
        v0, v1 = wall['v_range_m']
        holes = []
        if with_obstacles:
            holes = [(ob['uv_on_wall']['u'][0], ob['uv_on_wall']['u'][1], ob['uv_on_wall']['v'][0],
                      ob['uv_on_wall']['v'][1]) for ob in scene.get('obstacles', [])
                     if ob.get('kind') == 'opening' and ob.get('parent_wall') == wall['id']]
        # origin_xyz is on the plane at (u, v) = (0, 0)
        o = wall['origin_xyz']
        for i, (a, b, c, d) in enumerate(_free_rects(u0, u1, v0, v1, holes)):
            cu, cv = (a + b) / 2, (c + d) / 2
            centre = tuple(o[k] + u[k] * cu + v[k] * cv - n[k] * thickness / 2 for k in range(3))
            name = f'wall_{wall["id"]}' + (f'_{i}' if i else '')
            prims.append(Primitive(name, 'box', centre, quat_from_matrix([u, v, n]),
                                   ((b - a) / 2, (d - c) / 2, thickness / 2), 'contact_surface'))
    if with_obstacles:
        for ob in scene.get('obstacles', []):
            if wall_ids is not None and ob.get('parent_wall') not in wall_ids:
                continue
            if ob.get('kind') == 'opening':
                continue
            x, y, z, w = ob['box_orientation_xyzw']
            prims.append(Primitive(f'obstacle_{ob["id"]}', 'box', tuple(ob['box_center_xyz']),
                                   _normalize4((w, x, y, z)), tuple(e / 2 for e in ob['box_extents_uvn'])))
    return prims


def _normalize4(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def _native_pose(pose):
    pose = pose or {}
    pos = tuple(pose.get('position', (0, 0, 0)))
    x, y, z, w = pose.get('orientation_xyzw', (0, 0, 0, 1))
    return pos, _normalize4((w, x, y, z))


def objects_from_native(scene):
    prims = []
    for o in scene['objects']:
        pos, quat = _native_pose(o.get('pose'))
        role = o.get('role', 'obstacle')
        if role not in ('obstacle', 'contact_surface'):
            raise ValueError(f"object {o['id']}: role must be 'obstacle' or 'contact_surface'")
        kind = o['type']
        if kind == 'box':
            prims.append(Primitive(o['id'], 'box', pos, quat, tuple(s / 2 for s in o['size']), role))
        elif kind == 'sphere':
            prims.append(Primitive(o['id'], 'sphere', pos, quat, (o['radius'],), role))
        elif kind == 'cylinder':
            prims.append(Primitive(o['id'], 'cylinder', pos, quat, (o['radius'], o['height'] / 2), role))
        elif kind == 'capsule':
            prims.append(Primitive(o['id'], 'capsule', pos, quat, (o['radius'], o['length'] / 2), role))
    return prims


def objects_from_curobo(scene):
    prims = []

    def pose(d):
        p = d.get('pose', [0, 0, 0, 1, 0, 0, 0])
        return tuple(p[:3]), _normalize4(tuple(p[3:7]))
    for name, d in (scene.get('cuboid') or {}).items():
        pos, quat = pose(d)
        prims.append(Primitive(name, 'box', pos, quat, tuple(s / 2 for s in d['dims'])))
    for name, d in (scene.get('sphere') or {}).items():
        pos, quat = pose(d)
        prims.append(Primitive(name, 'sphere', tuple(d.get('position', pos)), quat, (d['radius'],)))
    for name, d in (scene.get('cylinder') or {}).items():
        pos, quat = pose(d)
        prims.append(Primitive(name, 'cylinder', pos, quat, (d['radius'], d['height'] / 2)))
    return prims


def load_scene(path, thickness=0.1, wall_ids=None, with_obstacles=True):
    """Parse a scene file; returns (primitives, skipped descriptions)."""
    path = pathlib.Path(path)
    text = path.read_text()
    scene = json.loads(text) if path.suffix == '.json' else yaml.safe_load(text)
    skipped = []
    if 'walls' in scene:
        return walls_from_scan(scene, thickness, wall_ids, with_obstacles), skipped
    if 'objects' in scene:
        skipped = [f"{o['id']} ({o['type']})" for o in scene['objects']
                   if o['type'] in ('mesh', 'pointcloud')]
        return objects_from_native(scene), skipped
    if any(k in scene for k in ('cuboid', 'sphere', 'cylinder', 'mesh')):
        skipped = [f'{k} (mesh)' for k in (scene.get('mesh') or {})]
        return objects_from_curobo(scene), skipped
    raise ValueError(f'{path}: not a wall_scene.json, scene_model scene or cuRobo world')


# ------------------------------------------------------------------ primitives -> MJCF
def apply_offset(prims, offset):
    """Return copies of the primitives moved by (x, y, z, yaw)."""
    x, y, z, yaw = offset
    q = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
    out = []
    for p in prims:
        pos = quat_rotate(q, p.pos)
        out.append(Primitive(p.name, p.kind, (pos[0] + x, pos[1] + y, pos[2] + z),
                             quat_mul(q, p.quat), p.size, p.role))
    return out


def primitives_to_geoms(prims, contact_friction=(1.0, 0.005, 0.0001)):
    geoms = []
    for p in prims:
        attrs = dict(name=p.name, type=p.kind, pos=_fmt(p.pos), quat=_fmt(p.quat), size=_fmt(p.size),
                     rgba=WALL_RGBA if p.role == 'contact_surface' else OBSTACLE_RGBA)
        if p.role == 'contact_surface':
            attrs['friction'] = _fmt(contact_friction)
        geoms.append(ET.Element('geom', attrs))
    return geoms


def wall_world_geoms(path, offset=(0, 0, 0, 0), thickness=0.1, wall_ids=None, with_obstacles=True):
    """`(geoms, primitives, skipped)` for a scene file, placed by `offset`."""
    prims, skipped = load_scene(path, thickness, wall_ids, with_obstacles)
    prims = apply_offset(prims, offset)
    return primitives_to_geoms(prims), prims, skipped
