#!/usr/bin/env python3
"""check_sync: do Gazebo, MuJoCo, RViz and the coverage plan describe the scenario in scenario.yaml?

    python3 -m scenario_sync.check_sync scenario.yaml \\
        [--plan-scene wall_scene.json] [--sim-scene LABEL=wall_scene.json ...] \\
        [--gazebo-sdf wall.sdf] [--mujoco-mjcf wall_world.xml] \\
        [--plan-output coverage_output.json] [--rviz-args rviz_args.yaml] [--gen-dir DIR] [--strict] [--format md]

--gen-dir takes every file scenario_gen.py wrote to DIR. Exit status: 0 all PASS (WARN and SKIP do not fail unless
--strict), 1 at least one FAIL, 2 the scenario cannot be read. Tolerances are the scenario's `tolerances`.

What is compared, per source:
  * frame: the frame name the file uses vs the world frame (base_link / base_footprint pass with a WARN when the
    base starts at the origin, where they coincide; anything else, e.g. map, link_base, fails);
  * wall plane: normal angle and the plane's distance from the base start (scenario standoff, minus the Gazebo
    sim_wall_offset_m for the Gazebo world);
  * extents: the u and v bounds of the wall's face against the full wall (sim files, sim scenes) or the work patch
    (plan scene), and the plan scene / plan poses inside the wall;
  * plan output: poses inside the patch (+ --plan-margin), distance of the poses from the wall plane
    (<= --plan-plane-tol), the planner's area vs the patch and vs the wall ("covers X % of the wall");
  * RViz params: fixed and plan frame, base x, y, yaw.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

from . import sources
from .schema import ScenarioError, load_scenario

PASS, WARN, FAIL, SKIP = 'PASS', 'WARN', 'FAIL', 'SKIP'
IDENTITY_FRAMES = ('base_link', 'base_footprint')


class Check:
    def __init__(self, source, name, status, detail):
        self.source, self.name, self.status, self.detail = source, name, status, detail

    def as_dict(self):
        return {'source': self.source, 'check': self.name, 'status': self.status, 'detail': self.detail}


def _base_is_identity(scn):
    return abs(scn.base_x) < 1e-6 and abs(scn.base_y) < 1e-6 and abs(scn.base_yaw) < 1e-6


def frame_check(scn, source, name, frame, expected=None):
    expected = expected or scn.world_frame
    if frame is None:
        return Check(source, name, SKIP, 'the file names no frame')
    if frame == expected:
        return Check(source, name, PASS, f"'{frame}'")
    if frame in IDENTITY_FRAMES and expected == scn.world_frame and _base_is_identity(scn):
        return Check(source, name, WARN, f"'{frame}' vs world frame '{expected}': the same place only while the base "
                                         'stands at the origin (a different name for tf and RViz)')
    return Check(source, name, FAIL, f"'{frame}' vs expected '{expected}'")


def _rng(r):
    return f'{r[0] + 0.0:.3f}..{r[1] + 0.0:.3f}'


def _wall_checks(scn, obs, expect_u, expect_v, what, plane_shift=0.0, plane_note=''):
    """Normal, plane distance and extents of a WallObs against the expected window."""
    out = []
    src = obs.label
    ang = obs.normal_err
    out.append(Check(src, 'wall normal', PASS if ang <= scn.tol('angle_rad') else FAIL,
                     f'{math.degrees(ang):.2f} deg from the scenario normal (tol {math.degrees(scn.tol("angle_rad")):.2f})'))
    exp = scn.frame.offset + plane_shift
    d = obs.plane_offset - exp
    dist_obs = float(scn.frame.n[:2] @ np.array([scn.base_x, scn.base_y])) - obs.plane_offset
    dist_exp = scn.standoff_m - plane_shift
    ok = abs(d) <= scn.tol('plane_m') and obs.plane_spread <= scn.tol('plane_m')
    detail = f'{dist_obs:.3f} m from the base start, expected {dist_exp:.3f} m (diff {d:+.3f} m, tol {scn.tol("plane_m"):.3f})'
    if obs.plane_spread > scn.tol('plane_m'):
        detail += f'; the face is not planar (spread {obs.plane_spread:.3f} m)'
    out.append(Check(src, 'wall plane distance', PASS if ok else FAIL, detail + plane_note))
    te = scn.tol('extent_m')
    dus = (obs.u[0] - expect_u[0], obs.u[1] - expect_u[1])
    dvs = (obs.v[0] - expect_v[0], obs.v[1] - expect_v[1])
    ok = max(map(abs, dus + dvs)) <= te
    zo = scn.z_range(obs.v)
    ze = scn.z_range(expect_v)
    out.append(Check(src, f'extents vs {what}', PASS if ok else FAIL,
                     f'u {_rng(obs.u)} (expected {_rng(expect_u)}), v {_rng(obs.v)} (expected {_rng(expect_v)}); '
                     f'{obs.u[1] - obs.u[0]:.3f} x {obs.v[1] - obs.v[0]:.3f} m, z {_rng(sorted(zo))} '
                     f'(expected {_rng(sorted(ze))}), max diff {max(map(abs, dus + dvs)):.3f} m (tol {te:.3f})'))
    return out


def _inside_wall(scn, obs, what):
    te = scn.tol('extent_m')
    wu, wv = scn.wall_u, scn.wall_v
    ok = (obs.u[0] >= wu[0] - te and obs.u[1] <= wu[1] + te and obs.v[0] >= wv[0] - te and obs.v[1] <= wv[1] + te)
    return Check(obs.label, f'{what} inside the wall', PASS if ok else FAIL,
                 f'u {_rng(obs.u)} v {_rng(obs.v)} vs wall u {_rng(wu)} v {_rng(wv)}')


def _coverage_row(scn, obs_label, area, what):
    frac = area / scn.wall_area
    if frac >= 0.995:
        return Check(obs_label, 'share of the wall', PASS, f'{what} covers {100 * frac:.1f} % of the wall '
                                                           f'({area:.3f} of {scn.wall_area:.3f} m^2)')
    kind = 'crop declared in the scenario' if scn.patch_mode == 'crop' else 'the scenario plans the whole wall'
    return Check(obs_label, 'share of the wall', WARN, f'{what} covers {100 * frac:.1f} % of the wall '
                                                       f'({area:.3f} of {scn.wall_area:.3f} m^2; {kind})')


def check_plan_scene(scn, doc, label, wall_id=0, base_frame=False):
    out = []
    obs = sources.wall_obs_from_scene(scn, doc, label, wall_id, base_frame)
    out.append(frame_check(scn, label, 'frame', obs.frame))
    out += _wall_checks(scn, obs, scn.patch_u, scn.patch_v, 'work patch')
    out.append(_inside_wall(scn, obs, 'plan scene'))
    out.append(_coverage_row(scn, label, obs.area, 'the planned window'))
    return out


def check_sim_scene(scn, doc, label, wall_id=0, base_frame=False):
    out = []
    obs = sources.wall_obs_from_scene(scn, doc, label, wall_id, base_frame)
    out.append(frame_check(scn, label, 'frame', obs.frame))
    out += _wall_checks(scn, obs, scn.wall_u, scn.wall_v, 'full wall')
    out += _obstacle_row(scn, obs)
    return out


def _expected_boxes(scn):
    return sum(1 for o in scn.obstacles if o['kind'] == 'box')


def _obstacle_row(scn, obs):
    if obs.obstacles is None:
        return []
    exp = _expected_boxes(scn)
    return [Check(obs.label, 'obstacle boxes', PASS if obs.obstacles == exp else FAIL,
                  f'{obs.obstacles} in the file, {exp} in the scenario')]


def check_sdf(scn, path, label='gazebo wall.sdf'):
    obs = sources.wall_obs_from_sdf(scn, path, label)
    off = float(scn.consumer('gazebo').get('sim_wall_offset_m', 0.0))
    note = f' (scenario standoff {scn.standoff_m:.3f} m minus the sim wall offset {off:.3f} m)' if off else ''
    out = _wall_checks(scn, obs, scn.wall_u, scn.wall_v, 'full wall', plane_shift=off, plane_note=note)
    return out + _obstacle_row(scn, obs)


def check_mjcf(scn, path, label='mujoco MJCF'):
    obs = sources.wall_obs_from_mjcf(scn, path, label)
    off = float(scn.consumer('mujoco').get('sim_wall_offset_m', 0.0))
    out = _wall_checks(scn, obs, scn.wall_u, scn.wall_v, 'full wall', plane_shift=off)
    return out + _obstacle_row(scn, obs)


def check_plan_output(scn, plan, margin=0.02, plane_tol=0.10, min_patch_fraction=0.5):
    out = []
    label = plan.label
    out.append(frame_check(scn, label, 'frame', plan.frame))
    if not len(plan.positions):
        return out + [Check(label, 'poses', FAIL, 'the plan has no poses')]
    uv = np.array([scn.frame.uv(p) for p in plan.positions])
    dist = np.array([scn.frame.dist(p) for p in plan.positions])
    umin, vmin = uv.min(axis=0)
    umax, vmax = uv.max(axis=0)
    pu, pv = scn.patch_u, scn.patch_v
    inside = (umin >= pu[0] - margin and umax <= pu[1] + margin and vmin >= pv[0] - margin and vmax <= pv[1] + margin)
    out.append(Check(label, 'plan bounding box inside the patch', PASS if inside else FAIL,
                     f'{len(uv)} poses, u {_rng((umin, umax))} v {_rng((vmin, vmax))} vs patch u {_rng(pu)} '
                     f'v {_rng(pv)} (+- {margin:.3f} m)'))
    dm = float(np.mean(dist))
    out.append(Check(label, 'distance of the poses from the wall plane', PASS if abs(dm) <= plane_tol else FAIL,
                     f'mean {dm:+.3f} m (range {dist.min():+.3f}..{dist.max():+.3f}), tol {plane_tol:.2f} m '
                     '(tool standoff and the sim wall offset)'))
    bbox = max(umax - umin, 0.0) * max(vmax - vmin, 0.0)
    area = float(plan.area_m2) if plan.area_m2 else bbox
    basis = 'planner coverage_area' if plan.area_m2 else 'pose bounding box'
    fp = area / scn.patch_area
    out.append(Check(label, 'plan area vs the patch', PASS if fp >= min_patch_fraction else FAIL,
                     f'{area:.3f} m^2 ({basis}) = {100 * fp:.0f} % of the {scn.patch_area:.3f} m^2 patch '
                     f'(min {100 * min_patch_fraction:.0f} %)'))
    out.append(_coverage_row(scn, label, area, f'the plan ({basis})'))
    return out


def check_rviz(scn, args, label='rviz_args.yaml', base_dir='.'):
    out = []
    exp = scn.consumer('rviz').get('fixed_frame', scn.world_frame)
    out.append(frame_check(scn, label, 'fixed frame', args.get('fixed_frame'), expected=exp))
    out.append(frame_check(scn, label, 'plan frame', args.get('plan_frame')))
    tp, ty = scn.tol('pose_m'), scn.tol('yaw_rad')
    miss = [k for k in ('base_x', 'base_y', 'base_yaw') if not isinstance(args.get(k), (int, float))]
    if miss:
        out.append(Check(label, 'base start pose', FAIL, f'missing {", ".join(miss)}'))
    else:
        dx, dy = args['base_x'] - scn.base_x, args['base_y'] - scn.base_y
        dyaw = sources.yaw_error(args['base_yaw'], scn.base_yaw)
        ok = max(abs(dx), abs(dy)) <= tp and dyaw <= ty
        out.append(Check(label, 'base start pose', PASS if ok else FAIL,
                         f'({args["base_x"]:.3f}, {args["base_y"]:.3f}, yaw {args["base_yaw"]:.3f}) vs scenario '
                         f'({scn.base_x:.3f}, {scn.base_y:.3f}, yaw {scn.base_yaw:.3f})'))
    sj = args.get('wall_scene_json')
    if sj:
        path = sj if os.path.isabs(sj) else os.path.join(base_dir, sj)
        if os.path.exists(path):
            out += check_plan_scene(scn, sources.read_json(path), f'{label} wall_scene_json')
        else:
            out.append(Check(label, 'wall_scene_json', FAIL, f'{path} does not exist'))
    else:
        out.append(Check(label, 'wall_scene_json', SKIP, 'not set'))
    return out


def run_checks(scn, plan_scene=None, sim_scenes=(), gazebo_sdf=None, mujoco_mjcf=None, plan_output=None,
               rviz_args=None, plan_margin=0.02, plan_plane_tol=0.10, min_patch_fraction=0.5):
    """All checks for the files given; the scenario's own consistency comes first."""
    out = [Check('scenario', 'base standoff', PASS,
                 f'{scn.standoff_m:.3f} m from base ({scn.base_x:.3f}, {scn.base_y:.3f}) to the wall plane ({scn.standoff_source})'),
           Check('scenario', 'patch inside the wall', PASS,
                 f'{scn.patch_mode} patch u {_rng(scn.patch_u)} v {_rng(scn.patch_v)} in wall u {_rng(scn.wall_u)} '
                 f'v {_rng(scn.wall_v)}'),
           _coverage_row(scn, 'scenario', scn.patch_area, 'the work patch')]
    if plan_scene:
        out += check_plan_scene(scn, sources.read_json(plan_scene), f'plan scene {os.path.basename(plan_scene)}')
    for label, path in sim_scenes:
        out += check_sim_scene(scn, sources.read_json(path), label)
    if gazebo_sdf:
        out += check_sdf(scn, gazebo_sdf)
    if mujoco_mjcf:
        out += check_mjcf(scn, mujoco_mjcf)
    if plan_output:
        out += check_plan_output(scn, sources.read_plan_output(plan_output), plan_margin, plan_plane_tol,
                                 min_patch_fraction)
    if rviz_args:
        out += check_rviz(scn, sources.read_rviz_args(rviz_args), base_dir=os.path.dirname(os.path.abspath(rviz_args)))
    return out


def verdict(checks, strict=False):
    bad = {FAIL, WARN} if strict else {FAIL}
    return 1 if any(c.status in bad for c in checks) else 0


def render(checks, fmt='text'):
    rows = [(c.source, c.name, c.status, c.detail) for c in checks]
    if fmt == 'json':
        return json.dumps([c.as_dict() for c in checks], indent=2)
    if fmt == 'md':
        lines = ['| Source | Check | Status | Detail |', '|---|---|---|---|']
        lines += [f'| {s} | {n} | {st} | {d.replace("|", "/")} |' for s, n, st, d in rows]
        return '\n'.join(lines)
    w = [max(len(r[i]) for r in rows + [('Source', 'Check', 'Status', '')]) for i in range(3)]
    lines = [f'{"Source".ljust(w[0])}  {"Check".ljust(w[1])}  {"Status".ljust(w[2])}  Detail']
    lines += [f'{s.ljust(w[0])}  {n.ljust(w[1])}  {st.ljust(w[2])}  {d}' for s, n, st, d in rows]
    return '\n'.join(lines)


def gen_dir_files(gen_dir):
    """The artifacts of scenario_gen.py in a directory, as run_checks keyword arguments."""
    def have(rel):
        p = os.path.join(gen_dir, rel)
        return p if os.path.exists(p) else None
    kw = {'plan_scene': have('plan/wall_scene.json'), 'gazebo_sdf': have('gazebo/wall.sdf'),
          'mujoco_mjcf': have('mujoco/wall_world.xml'), 'rviz_args': have('rviz_args.yaml'), 'sim_scenes': []}
    return kw


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scenario')
    ap.add_argument('--plan-scene', help='the wall_scene.json the coverage planner reads')
    ap.add_argument('--sim-scene', action='append', default=[], metavar='LABEL=PATH',
                    help='a wall_scene.json a simulator builds its wall from (world frame; repeatable)')
    ap.add_argument('--sim-scene-base-frame', action='store_true', help='the --sim-scene files are in the base-start frame')
    ap.add_argument('--gazebo-sdf')
    ap.add_argument('--mujoco-mjcf')
    ap.add_argument('--plan-output', help='coverage_output.json')
    ap.add_argument('--rviz-args')
    ap.add_argument('--gen-dir', help='a directory written by scenario_gen.py')
    ap.add_argument('--plan-margin', type=float, default=0.02, help='slack of the plan bounding box in the patch (m)')
    ap.add_argument('--plan-plane-tol', type=float, default=0.10, help='allowed mean distance of plan poses from the plane (m)')
    ap.add_argument('--min-patch-fraction', type=float, default=0.5)
    ap.add_argument('--strict', action='store_true', help='WARN fails too')
    ap.add_argument('--format', choices=('text', 'md', 'json'), default='text')
    a = ap.parse_args(argv)
    try:
        scn = load_scenario(a.scenario)
    except (ScenarioError, OSError) as e:
        print(f'check_sync: {e}', file=sys.stderr)
        return 2
    kw = {'plan_scene': a.plan_scene, 'gazebo_sdf': a.gazebo_sdf, 'mujoco_mjcf': a.mujoco_mjcf,
          'plan_output': a.plan_output, 'rviz_args': a.rviz_args, 'sim_scenes': []}
    if a.gen_dir:
        for k, v in gen_dir_files(a.gen_dir).items():
            if not kw.get(k):
                kw[k] = v
    for item in a.sim_scene:
        if '=' not in item:
            print('check_sync: --sim-scene wants LABEL=PATH', file=sys.stderr)
            return 2
        label, path = item.split('=', 1)
        kw['sim_scenes'].append((label, path))
    try:
        checks = run_checks(scn, plan_margin=a.plan_margin, plan_plane_tol=a.plan_plane_tol,
                            min_patch_fraction=a.min_patch_fraction, **kw)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as e:
        print(f'check_sync: cannot read an input: {type(e).__name__}: {e}', file=sys.stderr)
        return 2
    print(render(checks, a.format))
    rc = verdict(checks, a.strict)
    n_fail = sum(c.status == FAIL for c in checks)
    n_warn = sum(c.status == WARN for c in checks)
    print(f'\ncheck_sync {scn.name}: {"MISMATCH" if rc else "in sync"} ({n_fail} fail, {n_warn} warn, '
          f'{sum(c.status == PASS for c in checks)} pass)', file=sys.stderr if a.format == 'json' else sys.stdout)
    return rc


if __name__ == '__main__':
    sys.exit(main())
