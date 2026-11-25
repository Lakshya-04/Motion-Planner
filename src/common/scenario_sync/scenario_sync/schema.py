"""scenario.yaml: schema, validation and the resolved Scenario object. No ROS imports.

Schema (schema_version 1), all lengths in metres, angles in radians:

    schema_version: 1
    name: smoke_full                    # [A-Za-z0-9_.-]+
    description: free text              # optional
    world_frame: odom                   # where the base starts at base.x/y/yaw (default odom)
    wall:                               # the FULL wall
      point: [x, y, z]                  # a point on the wall plane = (u, v) (0, 0), world frame
      normal: [nx, ny, 0]               # outward normal (points at the robot), horizontal
      u_range: [u0, u1]                 # along the wall, u = n x up; for n = -x, u is +y
      v_range: [v0, v1]                 # v is metres DOWN from point (v = -z for a vertical wall)
    patch:                              # what is planned and executed
      mode: full | crop                 # full: the whole wall; crop: the window below
      u_range: [u0, u1]                 # crop only, must lie inside the wall
      v_range: [v0, v1]
    obstacles:                          # optional
      - {kind: box, u: [a, b], v: [c, d], depth: 0.03}     # protrusion standing `depth` off the wall
      - {kind: opening, u: [a, b], v: [c, d]}              # a hole through the wall
    base:
      x: 0.0                            # base start pose in world_frame
      y: 0.0
      yaw: 0.0
      standoff_m: 0.98                  # base-to-wall-plane distance along the normal
      standoff_advisor_json: path       # optional: standoff_m := its `recommended_standoff_m`; the base is moved along
                                        # the normal to that distance (relative paths: next to scenario.yaml)
    robot_profile: ur10e_telescopic
    tool_profile: spray_gun             # coverage tool profile name
    lift_range_m: [0.0, 1.5]            # telescopic lift stroke the scenario may use
    consumers:                          # optional per-consumer settings
      gazebo: {sim_wall_offset_m: 0.065}   # the sim wall stands this much nearer the base than the planned plane
      mujoco: {}
      rviz: {fixed_frame: odom}         # default: world_frame
    tolerances: {plane_m: 0.005, extent_m: 0.01, angle_rad: 0.01, pose_m: 0.001, yaw_rad: 0.01}

Either base.x/base.y or base.standoff_m (or both, then they must agree) may be omitted: a missing position is
placed on the wall's u = 0 line at the standoff; a missing standoff is measured from the position; a missing yaw
faces the wall. The base must stand on the robot's side of the wall.
"""
import json
import math
import os
import re

import numpy as np

from .geometry import WallFrame

SCHEMA_VERSION = 1
NAME_RE = re.compile(r'^[A-Za-z0-9_.-]+$')
DEFAULT_TOL = {'plane_m': 0.005, 'extent_m': 0.01, 'angle_rad': 0.01, 'pose_m': 0.001, 'yaw_rad': 0.01}
TOP_KEYS = {'schema_version', 'name', 'description', 'world_frame', 'wall', 'patch', 'obstacles', 'base',
            'robot_profile', 'tool_profile', 'lift_range_m', 'consumers', 'tolerances'}
CONSUMERS = {'gazebo', 'mujoco', 'rviz'}


class ScenarioError(ValueError):
    """The scenario is invalid; `.errors` lists every problem found."""

    def __init__(self, errors, source=''):
        self.errors = list(errors)
        self.source = source
        super().__init__((f'{source}: ' if source else '') + '; '.join(self.errors))


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _range(doc, key, errs, where):
    r = doc.get(key)
    if not (isinstance(r, (list, tuple)) and len(r) == 2 and all(_num(x) for x in r)):
        errs.append(f'{where}.{key} must be [min, max] numbers')
        return None
    if not r[0] < r[1]:
        errs.append(f'{where}.{key} {list(r)} must have min < max')
        return None
    return [float(r[0]), float(r[1])]


def _vec(doc, key, n, errs, where):
    r = doc.get(key)
    if not (isinstance(r, (list, tuple)) and len(r) == n and all(_num(x) for x in r)):
        errs.append(f'{where}.{key} must be {n} numbers')
        return None
    return [float(x) for x in r]


def _unknown(doc, allowed, errs, where):
    for k in doc:
        if k not in allowed:
            errs.append(f'{where}: unknown key {k!r} (allowed: {", ".join(sorted(allowed))})')


class Scenario:
    """A validated scenario. Attributes are plain python values, `frame` is the wall's WallFrame."""

    def __init__(self):
        self.notes = []

    # derived geometry --------------------------------------------------
    @property
    def base_pose(self):
        return (self.base_x, self.base_y, self.base_yaw)

    @property
    def wall_area(self):
        return (self.wall_u[1] - self.wall_u[0]) * (self.wall_v[1] - self.wall_v[0])

    @property
    def patch_area(self):
        return (self.patch_u[1] - self.patch_u[0]) * (self.patch_v[1] - self.patch_v[0])

    @property
    def patch_fraction(self):
        return self.patch_area / self.wall_area

    def z_range(self, v_range):
        """Heights (z0, z1) of a v window (v is down from the wall point)."""
        zp = float(self.frame.point[2])
        return zp - v_range[1], zp - v_range[0]

    @property
    def wall_z(self):
        return self.z_range(self.wall_v)

    @property
    def patch_z(self):
        return self.z_range(self.patch_v)

    def consumer(self, name):
        return self.consumers.get(name, {})

    def tol(self, key):
        return self.tolerances[key]

    def summary(self):
        return (f'{self.name}: wall {self.wall_u[1] - self.wall_u[0]:.3f} x {self.wall_v[1] - self.wall_v[0]:.3f} m '
                f'(plane {self.frame.offset:+.3f} along n), patch {self.patch_mode} '
                f'{self.patch_u[1] - self.patch_u[0]:.3f} x {self.patch_v[1] - self.patch_v[0]:.3f} m, '
                f'base ({self.base_x:.3f}, {self.base_y:.3f}, yaw {self.base_yaw:.3f}) standoff {self.standoff_m:.3f} m, '
                f'frame {self.world_frame}')


def validate(doc, base_dir='.', source=''):
    """Validate a parsed scenario dict and return the resolved Scenario; raises ScenarioError listing all problems."""
    errs = []
    if not isinstance(doc, dict):
        raise ScenarioError(['the document must be a mapping'], source)
    _unknown(doc, TOP_KEYS, errs, 'scenario')
    sv = doc.get('schema_version', SCHEMA_VERSION)
    if sv != SCHEMA_VERSION:
        errs.append(f'schema_version {sv!r} is not supported (this tool reads {SCHEMA_VERSION})')
    s = Scenario()
    s.name = doc.get('name')
    if not (isinstance(s.name, str) and NAME_RE.match(s.name)):
        errs.append('name is required and must match [A-Za-z0-9_.-]+')
    s.description = str(doc.get('description', ''))
    s.world_frame = doc.get('world_frame', 'odom')
    if not (isinstance(s.world_frame, str) and s.world_frame.strip() and ' ' not in s.world_frame):
        errs.append('world_frame must be a non-empty frame name without spaces')

    # wall
    wall = doc.get('wall')
    point = normal = s.wall_u = s.wall_v = None
    if not isinstance(wall, dict):
        errs.append('wall is required (point, normal, u_range, v_range)')
    else:
        _unknown(wall, {'point', 'normal', 'u_range', 'v_range'}, errs, 'wall')
        point = _vec(wall, 'point', 3, errs, 'wall')
        normal = _vec(wall, 'normal', 3, errs, 'wall')
        if normal is not None:
            if abs(normal[2]) > 1e-9:
                errs.append('wall.normal must be horizontal (z = 0): only vertical walls are supported')
            elif math.hypot(normal[0], normal[1]) < 1e-9:
                errs.append('wall.normal must not be zero')
            else:
                nn = math.hypot(normal[0], normal[1])
                if abs(nn - 1.0) > 1e-3:
                    s.notes.append(f'wall.normal normalised (length {nn:.4f})')
                normal = [normal[0] / nn, normal[1] / nn, 0.0]
        s.wall_u = _range(wall, 'u_range', errs, 'wall')
        s.wall_v = _range(wall, 'v_range', errs, 'wall')
    s.frame = WallFrame(point, normal) if point and normal and abs(normal[2]) < 1e-9 and \
        math.hypot(normal[0], normal[1]) > 0 else None

    # patch
    patch = doc.get('patch', {'mode': 'full'})
    s.patch_mode = 'full'
    s.patch_u, s.patch_v = s.wall_u, s.wall_v
    if not isinstance(patch, dict):
        errs.append('patch must be a mapping')
    else:
        _unknown(patch, {'mode', 'u_range', 'v_range'}, errs, 'patch')
        mode = patch.get('mode', 'full')
        if mode not in ('full', 'crop'):
            errs.append(f"patch.mode must be 'full' or 'crop', got {mode!r}")
        elif mode == 'crop':
            s.patch_mode = 'crop'
            pu = _range(patch, 'u_range', errs, 'patch')
            pv = _range(patch, 'v_range', errs, 'patch')
            if pu and pv and s.wall_u and s.wall_v:
                eps = 1e-9
                if pu[0] < s.wall_u[0] - eps or pu[1] > s.wall_u[1] + eps:
                    errs.append(f'patch.u_range {pu} is not inside wall.u_range {s.wall_u}')
                if pv[0] < s.wall_v[0] - eps or pv[1] > s.wall_v[1] + eps:
                    errs.append(f'patch.v_range {pv} is not inside wall.v_range {s.wall_v}')
            s.patch_u, s.patch_v = pu, pv
        elif 'u_range' in patch or 'v_range' in patch:
            errs.append("patch.u_range / v_range are only used with mode 'crop' (mode 'full' is the whole wall)")

    # obstacles
    s.obstacles = []
    obs = doc.get('obstacles', [])
    if not isinstance(obs, list):
        errs.append('obstacles must be a list')
        obs = []
    for i, ob in enumerate(obs):
        w = f'obstacles[{i}]'
        if not isinstance(ob, dict):
            errs.append(f'{w} must be a mapping')
            continue
        _unknown(ob, {'kind', 'u', 'v', 'depth'}, errs, w)
        kind = ob.get('kind')
        if kind not in ('box', 'opening'):
            errs.append(f"{w}.kind must be 'box' or 'opening'")
            continue
        ou = _range(ob, 'u', errs, w)
        ov = _range(ob, 'v', errs, w)
        depth = 0.0
        if kind == 'box':
            depth = ob.get('depth')
            if not (_num(depth) and depth > 0):
                errs.append(f'{w}.depth must be a positive number for a box')
                depth = 0.0
        elif 'depth' in ob:
            errs.append(f'{w}: an opening has no depth')
        if ou and ov and s.wall_u and s.wall_v and (
                ou[0] < s.wall_u[0] or ou[1] > s.wall_u[1] or ov[0] < s.wall_v[0] or ov[1] > s.wall_v[1]):
            errs.append(f'{w} (u {ou}, v {ov}) is not inside the wall')
        if ou and ov:
            s.obstacles.append({'kind': kind, 'u': ou, 'v': ov, 'depth': float(depth)})

    # base
    base = doc.get('base')
    s.base_x = s.base_y = s.base_yaw = s.standoff_m = None
    s.standoff_source = 'scenario'
    if not isinstance(base, dict):
        errs.append('base is required (x, y, yaw and/or standoff_m)')
        base = {}
    _unknown(base, {'x', 'y', 'yaw', 'standoff_m', 'standoff_advisor_json'}, errs, 'base')
    for k in ('x', 'y', 'yaw', 'standoff_m'):
        if k in base and not _num(base[k]):
            errs.append(f'base.{k} must be a number')
    standoff = base.get('standoff_m') if _num(base.get('standoff_m')) else None
    advisor = base.get('standoff_advisor_json')
    if advisor is not None:
        path = advisor if os.path.isabs(str(advisor)) else os.path.join(base_dir, str(advisor))
        try:
            with open(path) as f:
                rec = json.load(f)["recommended_standoff_m"]
            if not _num(rec):
                raise TypeError('not a number')
            standoff = float(rec)
            s.standoff_source = f'advisor {path}'
            s.notes.append(f'base.standoff_m {standoff:.4f} read from {path}')
        except (OSError, ValueError, KeyError, TypeError) as e:
            errs.append(f'base.standoff_advisor_json {path}: cannot read recommended_standoff_m ({e})')
    if standoff is not None and not standoff > 0:
        errs.append(f'base standoff {standoff} must be > 0')
        standoff = None
    has_xy = _num(base.get('x')) and _num(base.get('y'))
    if ('x' in base) != ('y' in base):
        errs.append('base.x and base.y go together')
    if not has_xy and standoff is None:
        errs.append('base needs x and y, or a standoff (standoff_m or standoff_advisor_json)')

    if s.frame is not None and not errs:
        f = s.frame
        if has_xy:
            bx, by = float(base['x']), float(base['y'])
            dist = f.dist((bx, by, 0.0))
            if standoff is None:
                standoff = dist
                s.standoff_source = 'measured from base x, y'
            elif advisor is not None:
                # the advisor decides the distance: slide the base along the normal to it
                bx, by = (np.array([bx, by]) + f.n[:2] * (standoff - dist)).tolist()
                s.notes.append(f'base moved along the wall normal by {standoff - dist:+.4f} m to the advised standoff')
            elif abs(dist - standoff) > 1e-6 + 1e-3:
                errs.append(f'base.standoff_m {standoff:.4f} disagrees with the base-to-wall distance {dist:.4f} '
                            f'from base x, y (plane {f.offset:+.4f} along n); fix one of them')
        else:
            lateral = f.point + f.u * 0.5 * (s.wall_u[0] + s.wall_u[1])
            b = lateral[:2] + f.n[:2] * standoff
            bx, by = float(b[0]), float(b[1])
            s.notes.append('base x, y placed on the wall centre line at the standoff')
        if standoff is not None and standoff <= 0:
            errs.append('the base must stand on the robot side of the wall (standoff > 0)')
        s.base_x, s.base_y, s.standoff_m = bx, by, float(standoff)
        s.base_yaw = float(base['yaw']) if _num(base.get('yaw')) else math.atan2(-f.n[1], -f.n[0])
        if 'yaw' not in base:
            s.notes.append('base.yaw faces the wall')

    # robot / tool / lift
    for key in ('robot_profile', 'tool_profile'):
        val = doc.get(key)
        if not (isinstance(val, str) and val.strip()):
            errs.append(f'{key} is required (a profile name)')
        setattr(s, key, val)
    lift = doc.get('lift_range_m')
    s.lift_range_m = None
    if lift is None:
        errs.append('lift_range_m is required, [min, max] (use [0, 0] for a fixed lift)')
    elif not (isinstance(lift, (list, tuple)) and len(lift) == 2 and all(_num(x) for x in lift)
              and 0 <= lift[0] <= lift[1]):
        errs.append('lift_range_m must be [min, max] with 0 <= min <= max')
    else:
        s.lift_range_m = [float(lift[0]), float(lift[1])]

    # consumers and tolerances
    cons = doc.get('consumers', {})
    s.consumers = {}
    if not isinstance(cons, dict):
        errs.append('consumers must be a mapping')
    else:
        for k, v in cons.items():
            if k not in CONSUMERS:
                errs.append(f'consumers.{k}: unknown consumer (allowed: {", ".join(sorted(CONSUMERS))})')
            elif not isinstance(v, dict):
                errs.append(f'consumers.{k} must be a mapping')
            else:
                s.consumers[k] = dict(v)
        g = s.consumers.get('gazebo', {})
        if 'sim_wall_offset_m' in g and not _num(g['sim_wall_offset_m']):
            errs.append('consumers.gazebo.sim_wall_offset_m must be a number')
        r = s.consumers.get('rviz', {})
        if 'fixed_frame' in r and not isinstance(r['fixed_frame'], str):
            errs.append('consumers.rviz.fixed_frame must be a frame name')
    tol = doc.get('tolerances', {})
    s.tolerances = dict(DEFAULT_TOL)
    if not isinstance(tol, dict):
        errs.append('tolerances must be a mapping')
    else:
        for k, v in tol.items():
            if k not in DEFAULT_TOL:
                errs.append(f'tolerances.{k}: unknown (allowed: {", ".join(DEFAULT_TOL)})')
            elif not (_num(v) and v > 0):
                errs.append(f'tolerances.{k} must be a positive number')
            else:
                s.tolerances[k] = float(v)

    if errs:
        raise ScenarioError(errs, source)
    return s


def load_scenario(path):
    """Read and validate a scenario.yaml."""
    import yaml
    with open(path) as f:
        try:
            doc = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise ScenarioError([f'not valid YAML: {e}'], str(path))
    return validate(doc, base_dir=os.path.dirname(os.path.abspath(path)), source=str(path))
