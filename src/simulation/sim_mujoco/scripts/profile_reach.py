#!/usr/bin/env python3
"""Where to put the smoke wall for a robot profile, and the tool orientation that goes with it (pure Python, no ROS).

The MuJoCo passes were written for the UR10e on the telescopic lift: wall plane x = 0.98 m, contact patch y 0..0.3 and
z 1.0..1.3 m in base_footprint, tool roll `tool_frame_yaw_deg` -90. Another arm needs another wall distance (a UFactory
xArm 7 reaches 0.7 m, a UR10e 1.3 m) and possibly another roll. This module computes them from the profile YAML
(`robot.xacro_args.arm`, `base.lift_mount_height_m`, `lift.mount_height_m`, `lift.stroke_m`, `base.footprint`,
`robot.xacro_args.tcp_xyz`) and a table of published arm reaches.

    profile_reach.py --profile xarm7_telescopic [--standoff 0.03] [--shell | --field NAME | (JSON)]

`--shell` prints `NAME=value` lines for `eval` in the pass scripts (manager_pass_common.sh, mp_profile_init).

HOW THE WALL DISTANCE IS CHOSEN (every number marked GUESS is a conservative estimate that a simulator run has to
confirm; nothing here was run in a simulator):

* The stock wall is the one configuration known to work (UR10e, wall_x 0.98, band 1.0..1.3, patch y 0..0.3). The
  radial distance from the arm base to the farthest point of the patch the arm has to reach, with the tool horizontal
  and the lift free to raise the shoulder, is
      r = sqrt((wall_x - tcp_z)^2 + y_far^2 + dz^2)
  with y_far = 0.3 (patch far edge) and dz = the vertical offset the lift cannot remove (the band edge farthest from the
  shoulder's travel [mount_z, mount_z + stroke]). For the stock UR10e r / reach = RHO (computed below, about 0.69).
  GUESS: the same fraction of the published reach is workable for every other arm. It ignores the wrist-to-flange
  offset (the UR10e calibration absorbs it for the UR10e only) and the 7-DOF arms' extra dexterity, so it is
  conservative for the 7-DOF arms in the sense of a short wall distance, and not checked for any of them.
* The band is centred 0.02 m below the retracted shoulder height (as the stock band is for the UR10e: shoulder
  1.17 m, centre 1.15 m) and is 0.3 m high; the telescopic lift supplies the vertical travel.
* The wall is kept at least MIN_BASE_CLEARANCE_M in front of the base footprint (GUESS), so that a short arm on a long
  base does not need the base inside the wall; the result is then flagged in `warnings`.
* Published reaches (ARM_REACH_M, m, vendor datasheets, quoted from memory of the public data sheets, to be checked):
  the figure is to the flange or last wrist axis depending on the vendor; the table makes no distinction.

TOOL ORIENTATION. `tool_frame_yaw_deg` rolls the tool about the wall normal (coverage_planner `liftCoveragePathPose`:
tool z into the wall, tool x = the wall's u axis, rotated by this angle about z). -90 was found for the UR10e by
trying: with the ready posture's wrist_3 at +pi/2 the arm folds away from the lift column, and a 90 degree mismatch made
the arm collide with the lift on every contact path (docs/sim_mujoco.md). Whether another arm needs a different roll
depends on its flange convention and joint ranges, which the profile does not state (there is no tcp_rpy field), so
it is NOT derived: ARM_TOOL_YAW_DEG is empty and every arm gets -90, unless the profile has
`coverage: {tool_frame_yaw_deg: ..}` or the caller sets MANAGER_PASS_TOOL_YAW. A 7-DOF arm with a +-360 degree last
joint can usually reach either sign; the pre-contact pose here is planned (ready_pose.py --pose), not a stored joint
pose, so a wrong roll shows up as a planning failure, not as a collision.
"""
import argparse
import json
import math
import os
import pathlib
import shlex
import sys

HERE = pathlib.Path(__file__).resolve().parent

DEFAULT_PROFILE = 'ur10e_telescopic'

# The stock wall (src/simulation/sim_mujoco/test/data): known to work for ur10e_telescopic.
STOCK_WALL_X = 0.98
STOCK_Z_LO, STOCK_Z_HI = 1.0, 1.3
STOCK_Y_LO, STOCK_Y_HI = 0.0, 0.3
STOCK_ARM = 'ur10e'
STOCK_TCP_Z = 0.165
BAND_CENTRE_BELOW_SHOULDER_M = 0.02   # stock: shoulder 1.17, band centre 1.15
MIN_BASE_CLEARANCE_M = 0.10           # GUESS
DEFAULT_TOOL_YAW_DEG = -90.0
FREE_STANDOFF_M = 0.03                # the stock FREE pose is 3 cm in front of the wall

# Published reach in metres (vendor data sheets). generic_6dof has none: pass --reach.
ARM_REACH_M = {
    'ur3e': 0.500, 'ur5e': 0.850, 'ur10e': 1.300, 'ur16e': 0.900, 'ur20': 1.750,
    'panda': 0.855, 'fr3': 0.855, 'iiwa7': 0.800, 'kinova_gen3': 0.902, 'xarm7': 0.700,
}
# Per-arm roll about the wall normal, degrees. Empty on purpose: see the module docstring.
ARM_TOOL_YAW_DEG = {}


def profiles_dir():
    env = os.environ.get('MP_PROFILE_DIR')
    if env:
        return pathlib.Path(env)
    try:
        from ament_index_python.packages import get_package_share_directory
        return pathlib.Path(get_package_share_directory('mobile_manipulator_description')) / 'config' / 'profiles'
    except Exception:  # noqa: BLE001  (no ROS environment: use the source tree)
        return HERE.parents[2] / 'description' / 'mobile_manipulator_description' / 'config' / 'profiles'


def load_profile(name_or_path):
    import yaml
    p = pathlib.Path(name_or_path)
    if not p.suffix:
        p = profiles_dir() / f'{name_or_path}.yaml'
    with open(p) as f:
        return yaml.safe_load(f)


def joint_names(profile):
    """Lift joint first, then the arm joints, as the planning group `lift_arm` orders them."""
    j = profile['joints']
    return [j['lift']] + list(j['arm'])


def arm_name(profile):
    return (profile.get('robot', {}).get('xacro_args') or {}).get('arm', 'generic_6dof')


def tcp_z(profile):
    t = str((profile.get('robot', {}).get('xacro_args') or {}).get('tcp_xyz', f'0 0 {STOCK_TCP_Z}')).split()
    return float(t[2])


def mount_z(profile):
    """Arm base height above base_footprint with the lift retracted."""
    return float(profile['base']['lift_mount_height_m']) + float(profile['lift']['mount_height_m'])


def lift_stroke(profile):
    return float((profile.get('lift') or {}).get('stroke_m', 0.0))


def _vertical_offset(z_lo, z_hi, shoulder_lo, shoulder_hi):
    """Largest vertical distance from a band point to the shoulder's travel [shoulder_lo, shoulder_hi]."""
    return max(max(shoulder_lo - z_lo, 0.0), max(z_hi - shoulder_hi, 0.0))


def _radial(wall_x, tcp, y_far, dz):
    return math.sqrt(max(wall_x - tcp, 0.0) ** 2 + y_far ** 2 + dz ** 2)


def stock_reach_fraction():
    """RHO: radial distance over published reach of the stock wall on the stock UR10e profile (retracted shoulder
    1.17 m, stroke 1.5 m). The one calibration constant; see the module docstring."""
    shoulder = STOCK_Z_LO + 0.5 * (STOCK_Z_HI - STOCK_Z_LO) + BAND_CENTRE_BELOW_SHOULDER_M
    dz = _vertical_offset(STOCK_Z_LO, STOCK_Z_HI, shoulder, shoulder + 1.5)
    return _radial(STOCK_WALL_X, STOCK_TCP_Z, STOCK_Y_HI, dz) / ARM_REACH_M[STOCK_ARM]


def tool_quaternion(yaw_deg):
    """Tool orientation for a wall whose normal points -x (the wall at +x): tool z = +x, tool x = wall u (+y) rolled by
    yaw_deg about z, as liftCoveragePathPose builds it. Returns (qx, qy, qz, qw) of the tool frame in base_footprint."""
    th = math.radians(yaw_deg)
    c, s = math.cos(th), math.sin(th)
    xa, ya, za = (0.0, c, s), (0.0, -s, c), (1.0, 0.0, 0.0)   # columns of the rotation matrix
    m = [[xa[0], ya[0], za[0]], [xa[1], ya[1], za[1]], [xa[2], ya[2], za[2]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0.0:
        w = math.sqrt(1.0 + tr) / 2.0
        q = ((m[2][1] - m[1][2]) / (4 * w), (m[0][2] - m[2][0]) / (4 * w), (m[1][0] - m[0][1]) / (4 * w), w)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        sx = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        q = (sx / 4, (m[0][1] + m[1][0]) / sx, (m[0][2] + m[2][0]) / sx, (m[2][1] - m[1][2]) / sx)
    elif m[1][1] > m[2][2]:
        sy = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        q = ((m[0][1] + m[1][0]) / sy, sy / 4, (m[1][2] + m[2][1]) / sy, (m[0][2] - m[2][0]) / sy)
    else:
        sz = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        q = ((m[0][2] + m[2][0]) / sz, (m[1][2] + m[2][1]) / sz, sz / 4, (m[1][0] - m[0][1]) / sz)
    n = math.sqrt(sum(v * v for v in q))
    return tuple(v / n for v in q)


def tool_yaw_deg(profile):
    own = (profile.get('coverage') or {}).get('tool_frame_yaw_deg')
    if own is not None:
        return float(own)
    return ARM_TOOL_YAW_DEG.get(arm_name(profile), DEFAULT_TOOL_YAW_DEG)


def derive(profile, standoff=FREE_STANDOFF_M, reach=None):
    """Wall placement for a profile. Returns a dict (see the keys below); raises ValueError if the arm's reach is unknown
    or too short for a wall at all."""
    arm = arm_name(profile)
    reach = reach if reach is not None else ARM_REACH_M.get(arm)
    if reach is None:
        raise ValueError(f'no published reach for arm {arm!r}: pass --reach')
    warnings = []
    zb0 = mount_z(profile)
    stroke = lift_stroke(profile)
    centre = zb0 - BAND_CENTRE_BELOW_SHOULDER_M
    z_lo = centre - 0.5 * (STOCK_Z_HI - STOCK_Z_LO)
    z_hi = centre + 0.5 * (STOCK_Z_HI - STOCK_Z_LO)
    dzv = _vertical_offset(z_lo, z_hi, zb0, zb0 + stroke)
    tcp = tcp_z(profile)
    r_max = stock_reach_fraction() * reach
    horizontal_sq = r_max ** 2 - STOCK_Y_HI ** 2 - dzv ** 2
    if horizontal_sq <= 0.0:
        raise ValueError(f'reach {reach} m is too short for a wall band of this height on {arm}')
    wall_x = tcp + math.sqrt(horizontal_sq)
    front = max(pt[0] for pt in profile['base']['footprint'])
    if wall_x < front + MIN_BASE_CLEARANCE_M:
        wall_x = front + MIN_BASE_CLEARANCE_M
        warnings.append(f'wall_x raised to {wall_x:.2f} m to stay {MIN_BASE_CLEARANCE_M} m in front of the footprint; '
                        f'the arm may not reach it')
    wall_x, z_lo, z_hi = round(wall_x, 2), round(z_lo, 2), round(z_hi, 2)
    yaw = tool_yaw_deg(profile)
    y_mid = 0.5 * (STOCK_Y_LO + STOCK_Y_HI)
    z_mid = 0.5 * (z_lo + z_hi)
    free_pose = [wall_x - standoff, y_mid, z_mid, *tool_quaternion(yaw)]
    return {
        'profile_arm': arm, 'reach_m': reach, 'wall_x': wall_x, 'wall_dx': round(wall_x - STOCK_WALL_X, 2),
        'z_lo': z_lo, 'z_hi': z_hi, 'wall_dz': round(z_lo - STOCK_Z_LO, 2), 'y_lo': STOCK_Y_LO, 'y_hi': STOCK_Y_HI,
        'tool_yaw_deg': yaw, 'free_pose': free_pose, 'standoff_m': standoff, 'arm_mount_z': round(zb0, 3),
        'lift_stroke': stroke, 'tcp_xyz': str((profile.get('robot', {}).get('xacro_args') or {}).get('tcp_xyz', '')),
        'warnings': warnings,
    }


def shell_lines(d):
    pose = ' '.join(f'{v:.5f}' for v in d['free_pose'])
    items = [
        ('MP_ARM', d['profile_arm']), ('MP_WALL_X', f"{d['wall_x']:.2f}"), ('MP_WALL_DX', f"{d['wall_dx']:.2f}"),
        ('MP_Z_LO', f"{d['z_lo']:.2f}"), ('MP_Z_HI', f"{d['z_hi']:.2f}"), ('MP_WALL_DZ', f"{d['wall_dz']:.2f}"),
        ('MP_TOOL_YAW', f"{d['tool_yaw_deg']:.1f}"), ('MP_FREE_POSE', pose),
        ('MP_ARM_MOUNT_Z', f"{d['arm_mount_z']:.3f}"), ('MP_LIFT_STROKE', f"{d['lift_stroke']:g}"),
        ('MP_TCP_XYZ', d['tcp_xyz']),
    ]
    return [f'{k}={shlex.quote(v)}' for k, v in items]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--profile', default=os.environ.get('MP_PROFILE', DEFAULT_PROFILE), help='profile name or YAML path')
    ap.add_argument('--standoff', type=float, default=FREE_STANDOFF_M, help='tool point to wall for the free pose, m')
    ap.add_argument('--reach', type=float, help='arm reach in m (arms not in the table)')
    ap.add_argument('--shell', action='store_true', help='print NAME=value lines for eval')
    ap.add_argument('--field', help='print one value: a key of the JSON output (lists space separated)')
    args = ap.parse_args()
    try:
        d = derive(load_profile(args.profile), args.standoff, args.reach)
    except (ValueError, KeyError, OSError) as e:
        print(f'profile_reach: {e}', file=sys.stderr)
        return 2
    for w in d['warnings']:
        print(f'profile_reach: warning: {w}', file=sys.stderr)
    if args.shell:
        print('\n'.join(shell_lines(d)))
    elif args.field:
        v = d[args.field]
        print(' '.join(f'{x:.5f}' for x in v) if isinstance(v, list) else v)
    else:
        print(json.dumps(d, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
