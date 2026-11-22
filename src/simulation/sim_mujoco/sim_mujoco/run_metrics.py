"""ROS-free run-quality numbers of a multi-stand pass, read from manager.log (and the wall scene).

Three questions that a coverage score does not answer:

* lift used: how much of the lift's stroke the run moved through (min, max, span, path travel, from the ARMSTATE
  lines; the first joint of `q` is the lift);
* chassis-to-wall clearance: the smallest distance of the base footprint to the wall plane at each stand (stand poses
  from the `stand N: arm base (x, y) yaw a reaches ...` lines, the footprint rectangle and the arm mount offset of
  the profile);
* tool reach fraction: distance arm base -> tool over the arm's reach, per stand, from ARMSTATE `tcp_in_odom` and the
  stand's arm base (the arm base is `arm_mount_z_m` + lift above the floor).

Log lines (manipulation_manager_node.cpp):
    ARMSTATE docked_wall_column_4 stand 0: q=[0.0455, 2.7983, ...] tcp_in_odom=(0.940, 0.810, 1.010)
    stand 0: arm base (0.180, 0.010) yaw 1.571 reaches 36 pose(s)
"""
import json
import math
import re

# ur10e_telescopic profile (description/config/profiles/ur10e_telescopic.yaml, base_check_arm_mount_z_m 1.17)
DEFAULTS = dict(stroke_m=1.5, arm_mount_z_m=1.17, reach_m=1.3, arm_mount_xy=(0.0, 0.0),
                footprint=((-0.45, -0.30), (0.45, -0.30), (0.45, 0.30), (-0.45, 0.30)))
LOW_LIFT_SPAN_FRACTION = 0.25      # a run whose lift span is under this share of the stroke is flagged
MIN_CLEARANCE_M = 0.30             # chassis closer to the wall than this is flagged
MAX_REACH_FRACTION = 0.85          # tool farther than this share of the reach from the arm base is flagged

_NUM = r'[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?'
_ARMSTATE = re.compile(
    rf'ARMSTATE\s+(?P<tag>\S+)\s+stand\s+(?P<stand>\d+):\s+q=\[(?P<q>[^\]]*)\]\s+'
    rf'tcp_in_\w+=\(\s*(?P<x>{_NUM}),\s*(?P<y>{_NUM}),\s*(?P<z>{_NUM})\s*\)')
_STAND = re.compile(
    rf'stand\s+(?P<stand>\d+):\s+arm base\s+\(\s*(?P<x>{_NUM}),\s*(?P<y>{_NUM})\s*\)\s+yaw\s+(?P<yaw>{_NUM})'
    rf'\s+reaches\s+(?P<n>\d+)')


def parse_log(lines):
    """-> (armstates, stands): armstates = [{tag, stand, q, tcp}], stands = {index: {x, y, yaw, poses}}.
    A stand line seen twice (a re-plan) keeps the last one."""
    arm, stands = [], {}
    for ln in lines:
        m = _ARMSTATE.search(ln)
        if m:
            try:
                q = [float(v) for v in m['q'].split(',') if v.strip()]
            except ValueError:
                continue
            if q:
                arm.append(dict(tag=m['tag'], stand=int(m['stand']), q=q,
                                tcp=(float(m['x']), float(m['y']), float(m['z']))))
            continue
        m = _STAND.search(ln)
        if m:
            stands[int(m['stand'])] = dict(x=float(m['x']), y=float(m['y']), yaw=float(m['yaw']), poses=int(m['n']))
    return arm, stands


def lift_stats(armstates, stroke_m=DEFAULTS['stroke_m']):
    """Lift range from the ARMSTATE samples (q[0]): min, max, span (max - min), travel (sum of |step|, samples in log
    order), span as a fraction of the stroke, and the same min/max for the docked samples only."""
    lifts = [a['q'][0] for a in armstates]
    if not lifts:
        return None
    docked = [a['q'][0] for a in armstates if a['tag'].startswith('docked')]
    out = dict(n=len(lifts), min=min(lifts), max=max(lifts), span=max(lifts) - min(lifts),
               travel=sum(abs(b - a) for a, b in zip(lifts, lifts[1:])), stroke=stroke_m)
    out['span_fraction'] = out['span'] / stroke_m if stroke_m > 0 else float('nan')
    out['docked_min'] = min(docked) if docked else None
    out['docked_max'] = max(docked) if docked else None
    return out


def base_pose(stand, arm_mount_xy=DEFAULTS['arm_mount_xy']):
    """Base footprint origin and yaw of a stand (the stand pose is the arm base: base = arm base - R(yaw) mount)."""
    c, s = math.cos(stand['yaw']), math.sin(stand['yaw'])
    mx, my = arm_mount_xy
    return stand['x'] - (c * mx - s * my), stand['y'] - (s * mx + c * my), stand['yaw']


def footprint_corners(stand, footprint=DEFAULTS['footprint'], arm_mount_xy=DEFAULTS['arm_mount_xy']):
    bx, by, yaw = base_pose(stand, arm_mount_xy)
    c, s = math.cos(yaw), math.sin(yaw)
    return [(bx + c * px - s * py, by + s * px + c * py) for px, py in footprint]


def wall_distance(point_xy, plane_abcd):
    """Signed distance of a point to a vertical wall plane a x + b y + c z + d = 0 in the xy plane. For the scene's
    plane [-1, 0, 0, x_wall] it is x_wall - x: positive on the robot's side, negative past the wall."""
    a, b, _, d = plane_abcd
    n = math.hypot(a, b)
    if n == 0:
        raise ValueError('plane has no horizontal normal')
    return (a * point_xy[0] + b * point_xy[1] + d) / n


def chassis_clearance(stands, plane_abcd, footprint=DEFAULTS['footprint'], arm_mount_xy=DEFAULTS['arm_mount_xy']):
    """-> {stand index: dict(chassis=min corner distance to the wall plane, arm_base=arm base distance)}."""
    out = {}
    for i, st in sorted(stands.items()):
        corners = footprint_corners(st, footprint, arm_mount_xy)
        out[i] = dict(chassis=min(wall_distance(c, plane_abcd) for c in corners),
                      arm_base=wall_distance((st['x'], st['y']), plane_abcd))
    return out


def reach_fractions(armstates, stands, arm_mount_z_m=DEFAULTS['arm_mount_z_m'], reach_m=DEFAULTS['reach_m'],
                    docked_only=True):
    """-> {stand: dict(n, mean, max, min_dist, max_dist)} of |tcp - arm base| / reach. The arm base is at the stand's
    (x, y) and z = arm_mount_z_m + lift. Samples of a stand with no `arm base` line are skipped."""
    acc = {}
    for a in armstates:
        st = stands.get(a['stand'])
        if st is None or (docked_only and not a['tag'].startswith('docked')):
            continue
        d = math.dist((st['x'], st['y'], arm_mount_z_m + a['q'][0]), a['tcp'])
        acc.setdefault(a['stand'], []).append(d)
    return {i: dict(n=len(v), mean=sum(v) / len(v) / reach_m, max=max(v) / reach_m,
                    min_dist=min(v), max_dist=max(v), reach=reach_m) for i, v in sorted(acc.items())}


def wall_plane_from_scene(path):
    """plane_abcd of the first wall of a wall_scene.json."""
    with open(path) as f:
        return json.load(f)['walls'][0]['plane_abcd']


def evaluate(lines, plane_abcd=None, **cfg):
    """All three metrics plus the flags (list of strings) from manager.log lines."""
    c = dict(DEFAULTS)
    c.update({k: v for k, v in cfg.items() if v is not None})
    arm, stands = parse_log(lines)
    res = dict(stands=stands, lift=lift_stats(arm, c['stroke_m']),
               reach=reach_fractions(arm, stands, c['arm_mount_z_m'], c['reach_m']),
               clearance=(chassis_clearance(stands, plane_abcd, c['footprint'], c['arm_mount_xy'])
                          if plane_abcd is not None else {}))
    flags = []
    if not arm:
        flags.append('no ARMSTATE lines (lift and reach not measured)')
    if not stands:
        flags.append('no stand lines (clearance and reach not measured)')
    if res['lift'] and res['lift']['span_fraction'] < LOW_LIFT_SPAN_FRACTION:
        flags.append(f"lift not used: span {res['lift']['span']:.2f} m is {res['lift']['span_fraction']:.0%} "
                     f"of the {c['stroke_m']:.2f} m stroke")
    for i, v in res['clearance'].items():
        if v['chassis'] < MIN_CLEARANCE_M:
            flags.append(f"stand {i}: chassis {v['chassis']:.2f} m from the wall (under {MIN_CLEARANCE_M:.2f} m)")
    for i, v in res['reach'].items():
        if v['max'] > MAX_REACH_FRACTION:
            flags.append(f"stand {i}: tool at {v['max']:.0%} of the reach (over {MAX_REACH_FRACTION:.0%})")
    res['flags'] = flags
    return res


def format_table(res):
    """The metrics as text lines."""
    out = []
    L = res['lift']
    if L:
        dk = '' if L['docked_min'] is None else f", docked {L['docked_min']:.3f} to {L['docked_max']:.3f} m"
        out.append(f"lift: {L['min']:.3f} to {L['max']:.3f} m (span {L['span']:.3f} m = {L['span_fraction']:.0%} of the "
                   f"{L['stroke']:.2f} m stroke, travel {L['travel']:.2f} m, {L['n']} samples){dk}")
    else:
        out.append('lift: no ARMSTATE samples')
    out.append('stand  arm base (x, y)    yaw   chassis-wall  armbase-wall  tool reach (mean / max)  tool dist (m)  samples')
    for i, st in sorted(res['stands'].items()):
        cl, rc = res['clearance'].get(i), res['reach'].get(i)
        cs = f"{cl['chassis']:.3f} m" if cl else 'n/a'
        ab = f"{cl['arm_base']:.3f} m" if cl else 'n/a'
        rs = f"{rc['mean']:.0%} / {rc['max']:.0%}" if rc else 'n/a'
        ds = f"{rc['min_dist']:.2f} to {rc['max_dist']:.2f}" if rc else 'n/a'
        out.append(f"{i:5d}  ({st['x']:.3f}, {st['y']:.3f})  {st['yaw']:5.3f}  {cs:>12}  {ab:>12}  {rs:>23}  {ds:>13}  "
                   f"{rc['n'] if rc else 0:7d}")
    out += [f'FLAG: {f}' for f in res['flags']] or ['no flags']
    return out
