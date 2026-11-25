"""Read the files each consumer is built from and reduce them to what check_sync compares. No ROS imports.

WallObs: the wall a file describes, in the scenario's wall coordinates (plane offset along the scenario normal,
u and v bounds of its front face, normal angle error). Sim files (SDF, MJCF) live in the base-start frame, so
their points are carried to the world frame with the scenario's base start pose first.
"""
import json
import math
import xml.etree.ElementTree as ET

import numpy as np

from .geometry import (angle_between, base_to_world, matrix_from_quat_xyzw, matrix_from_rpy, unit,
                       vec_base_to_world)


class WallObs:
    def __init__(self, label, frame=None, normal_err=0.0, plane_offsets=(), u=None, v=None, obstacles=None,
                 boxes=1):
        self.label = label
        self.frame = frame                 # frame_id the file names, None if it has none
        self.normal_err = normal_err       # rad between its normal and the scenario's
        self.plane_offset = float(np.mean(plane_offsets)) if len(plane_offsets) else None   # n_scn . p, mean
        self.plane_spread = float(np.ptp(plane_offsets)) if len(plane_offsets) else 0.0
        self.u = u                         # (min, max) in scenario u
        self.v = v
        self.obstacles = obstacles         # count of obstacle boxes (None: unknown)
        self.boxes = boxes

    @property
    def area(self):
        return (self.u[1] - self.u[0]) * (self.v[1] - self.v[0])


def _bounds_from_corners(scn, corners_world):
    uv = [scn.frame.uv(c) for c in corners_world]
    us, vs = [p[0] for p in uv], [p[1] for p in uv]
    return (min(us), max(us)), (min(vs), max(vs))


def read_json(path):
    with open(path) as f:
        return json.load(f)


# ------------------------------------------------------------------ wall_scene.json
def wall_obs_from_scene(scn, doc, label, wall_id=0, base_frame=False):
    """Observation of one wall of a wall_scene.json dict. base_frame: the file is in the base-start frame."""
    walls = [w for w in doc.get('walls', []) if int(w.get('id', 0)) == int(wall_id)]
    if not walls:
        raise ValueError(f'{label}: no wall with id {wall_id}')
    w = walls[0]
    o = np.array(w['origin_xyz'], float)
    u, v, n = (unit(w[k]) for k in ('basis_u', 'basis_v', 'normal_outward'))
    if base_frame:
        o = base_to_world(o, scn.base_pose)
        u, v, n = (vec_base_to_world(x, scn.base_pose) for x in (u, v, n))
    u0, u1 = w['u_range_m']
    v0, v1 = w['v_range_m']
    corners = [o + u * a + v * b for a in (u0, u1) for b in (v0, v1)]
    ub, vb = _bounds_from_corners(scn, corners)
    return WallObs(label, frame=doc.get('frame_id'), normal_err=angle_between(n, scn.frame.n),
                   plane_offsets=[float(scn.frame.n @ c) for c in corners], u=ub, v=vb,
                   obstacles=sum(1 for ob in doc.get('obstacles', [])
                                 if ob.get('kind', 'box') != 'opening' and int(ob.get('parent_wall', wall_id)) == int(wall_id)))


# ------------------------------------------------------------------ boxes (SDF / MJCF)
def _obs_from_boxes(scn, label, boxes, n_obstacles):
    """boxes: [(centre_base, R_base, full sizes)] of the wall (front face = local +z); base-start frame -> world."""
    corners, normals = [], []
    for c, R, size in boxes:
        c_w = base_to_world(c, scn.base_pose)
        R_w = np.column_stack([vec_base_to_world(R[:, i], scn.base_pose) for i in range(3)])
        normals.append(R_w[:, 2])
        for a in (-0.5, 0.5):
            for b in (-0.5, 0.5):
                corners.append(c_w + R_w[:, 0] * size[0] * a + R_w[:, 1] * size[1] * b + R_w[:, 2] * size[2] * 0.5)
    ub, vb = _bounds_from_corners(scn, corners)
    err = max(angle_between(n, scn.frame.n) for n in normals)
    return WallObs(label, frame=None, normal_err=err, plane_offsets=[float(scn.frame.n @ c) for c in corners],
                   u=ub, v=vb, obstacles=n_obstacles, boxes=len(boxes))


def wall_obs_from_sdf(scn, path, label='gazebo wall.sdf'):
    root = ET.parse(path).getroot()
    boxes, obstacles = [], 0
    for m in root.iter('model'):
        name = m.get('name', '')
        box = m.find('.//collision/geometry/box/size')
        if box is None or not (name.startswith('wall_') or name.startswith('obstacle_')):
            continue
        if name.startswith('obstacle_'):
            obstacles += 1
            continue
        pose = [float(x) for x in (m.findtext('pose') or '0 0 0 0 0 0').split()]
        size = [float(x) for x in box.text.split()]
        boxes.append((np.array(pose[:3]), matrix_from_rpy(*pose[3:6]), size))
    if not boxes:
        raise ValueError(f'{path}: no wall_* box model found (is it a wall_world_gen.py world?)')
    return _obs_from_boxes(scn, label, boxes, obstacles)


def wall_obs_from_mjcf(scn, path, label='mujoco MJCF'):
    root = ET.parse(path).getroot()
    boxes, obstacles = [], 0
    for g in root.iter('geom'):
        name = g.get('name', '')
        if g.get('type') != 'box':
            continue
        if name.startswith('obstacle_'):
            obstacles += 1
            continue
        if not name.startswith('wall_'):
            continue
        pos = [float(x) for x in g.get('pos', '0 0 0').split()]
        q = [float(x) for x in g.get('quat', '1 0 0 0').split()]       # MuJoCo: w x y z
        half = [float(x) for x in g.get('size').split()]
        boxes.append((np.array(pos), matrix_from_quat_xyzw((q[1], q[2], q[3], q[0])), [2 * h for h in half]))
    if not boxes:
        raise ValueError(f'{path}: no wall_* box geom found')
    return _obs_from_boxes(scn, label, boxes, obstacles)


# ------------------------------------------------------------------ coverage_output.json
class PlanObs:
    def __init__(self, label, frame, positions, area_m2, source_scene):
        self.label, self.frame, self.positions = label, frame, positions
        self.area_m2, self.source_scene = area_m2, source_scene


def read_plan_output(path, label='coverage_output.json'):
    """The planner's output: poses of the final (lifted and expanded) path, or of the flat path as a fallback."""
    doc = read_json(path)
    block = doc.get('final_lifted_and_expanded') or doc.get('received_from_coverage_planner_node') or doc
    pth = block.get('path') or {}
    poses = pth.get('poses', [])
    pos = np.array([[p['position'][k] for k in 'xyz'] for p in poses], float).reshape(-1, 3)
    recv = doc.get('received_from_coverage_planner_node', {})
    return PlanObs(label, pth.get('frame_id'), pos, recv.get('coverage_area_m2'), doc.get('wall_scene_json'))


# ------------------------------------------------------------------ rviz_args.yaml
def read_rviz_args(path):
    import yaml
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    block = doc.get('scenario_sync', doc)
    if not isinstance(block, dict):
        raise ValueError(f'{path}: no scenario_sync block')
    return block


def yaw_error(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)
