"""Touch-up coverage plan: what a finished or aborted run left undone, as a coverage_output.json the manager can run.

Pure python + numpy (PyYAML only for ``load_tool_coverage``); no ROS. Uses the paint core of ``coverage_paint``:

    1. replay run.csv on the wall grid -> cells NOT_DONE (never swept in contact) or FAILED (a failed / partial column);
    2. group those cells into vertical strips: the strips are ``swath_m`` wide, the pitch of strips that touch is
       ``swath_m * (1 - overlap_pct / 100)`` (the planner's strip spacing), each strip runs along the wall's v axis;
    3. every run of undone cells along a strip is one stroke; strokes shorter than ``min_length_m`` are dropped, and the area
       of the undone cells no kept stroke reaches is reported as ``ignored_m2`` (nothing is hidden);
    4. strips are ordered along u and alternate direction (serpentine), one pass, no connector segments: the manager drives
       between columns itself.

The file has the keys ``CoverageOutputJsonSource`` (manipulation_manager) reads: ``final_lifted_and_expanded.path.poses[]``
(position + quaternion orientation) and ``final_lifted_and_expanded.segments[]`` (start_index / end_index of one stroke); the
other keys follow the planner's writer (``wall_coverage_demo_node``) so the file also loads in ``coverage_replay.py --plan``.
``check_manager_schema`` repeats the manager's parsing on a plan and is used by the tests.

Assumption: the manager's strokes run along world z, so the wall's v axis must be (close to) up; sweep_direction
'horizontal' tool profiles are rejected.
"""
import json
import math

import numpy as np

from . import coverage_paint as cp

DEFAULT_MIN_LENGTH_M = 0.10
DEFAULT_POSE_STEP_M = 0.05
DEFAULT_OVERLAP_PCT = 20.0
MANAGER_X_QUANTUM_M = 1e-3     # CoverageOutputJsonSource groups poses of one column by this rounding


def load_tool_coverage(path, tool_name='contact_wiper'):
    """(swath_m, overlap_pct, sweep_direction) of a tool in a tool_profiles.yaml (coverage_planner/config)."""
    import yaml
    with open(path) as f:
        doc = yaml.safe_load(f)
    cov = doc['tools'][tool_name]['coverage']
    return float(cov['swath_m']), float(cov.get('overlap_pct', 0.0)), str(cov.get('sweep_direction', 'vertical'))


def undone_mask(grid):
    return grid.status != cp.DONE


def _runs(col, min_gap_cells):
    """[(j0, j1)] inclusive runs of True in a 1D bool array; runs closer than min_gap_cells are merged."""
    idx = np.flatnonzero(col)
    if idx.size == 0:
        return []
    out = [[int(idx[0]), int(idx[0])]]
    for j in idx[1:]:
        if j - out[-1][1] - 1 <= min_gap_cells:
            out[-1][1] = int(j)
        else:
            out.append([int(j), int(j)])
    return [tuple(r) for r in out]


def plan_strips(grid, swath_m=None, overlap_pct=DEFAULT_OVERLAP_PCT, min_length_m=DEFAULT_MIN_LENGTH_M, max_gap_m=None):
    """Strips covering the undone cells of ``grid``.

    Returns (strips, report). strips: [{'u': centre, 'runs': [(v_lo, v_hi), ...]}] ordered along u; report: the numbers.
    max_gap_m: done stretches shorter than this inside a strip are stroked through (default: one cell).
    """
    sw = float(swath_m if swath_m is not None else grid.swath)
    pitch = sw * (1.0 - overlap_pct / 100.0)
    if sw <= 0 or pitch <= 0:
        raise ValueError('swath must be positive and overlap_pct < 100')
    cell, w = grid.cell, grid.wall
    u0, v0 = w.u_range[0], w.v_range[0]
    gap_cells = 0 if max_gap_m is None else int(max_gap_m / cell + 1e-9)
    und = undone_mask(grid)
    col_any = und.any(axis=0)
    band_cells = max(1, int(round(sw / cell)))
    strips, prev_centre, i = [], None, 0
    width = w.u_range[1] - w.u_range[0]
    while i < grid.nu:
        if not col_any[i]:
            i += 1
            continue
        left = u0 + i * cell
        if prev_centre is not None and left - (prev_centre + sw / 2.0) < cell * 0.5 + 1e-9:
            centre = prev_centre + pitch                 # touching the previous strip: keep the planner's overlap
        else:
            centre = left + sw / 2.0
        if width > sw:
            centre = min(max(centre, w.u_range[0] + sw / 2.0), w.u_range[1] - sw / 2.0)
        else:
            centre = (w.u_range[0] + w.u_range[1]) / 2.0
        ia = max(0, int(math.floor((centre - sw / 2.0 - u0) / cell + 1e-9)))
        ib = min(grid.nu - 1, ia + band_cells - 1)
        band = und[:, ia:ib + 1].any(axis=1)
        runs = []
        for j0, j1 in _runs(band, gap_cells):
            lo, hi = v0 + j0 * cell, min(w.v_range[1], v0 + (j1 + 1) * cell)
            if hi - lo >= min_length_m - 1e-9:
                runs.append((lo, hi))
        if runs:
            strips.append({'u': centre, 'runs': runs})
        prev_centre = centre
        i = ib + 1
    # what the kept strokes reach, repainted on a scratch grid: the rest of the undone cells is the ignored area
    scratch = cp.CoverageGrid(w, cell, sw, grid.force_threshold, grid.plane_tol)
    for s in strips:
        for lo, hi in s['runs']:
            scratch._paint((s['u'], lo + cell / 2), (s['u'], hi - cell / 2), cp.DONE)
    left_over = und & (scratch.status != cp.DONE)
    a = cell * cell
    stroke_len = sum(hi - lo for s in strips for lo, hi in s['runs'])
    report = {'undone_m2': round(int(und.sum()) * a, 4), 'planned_strips': len(strips),
              'planned_strokes': sum(len(s['runs']) for s in strips), 'stroke_length_m': round(stroke_len, 3),
              'ignored_m2': round(int(left_over.sum()) * a, 4), 'min_length_m': min_length_m, 'swath_m': sw,
              'overlap_pct': overlap_pct, 'percent_done_before': round(grid.percent_done(), 2)}
    return strips, report


def _quat_from_matrix(m):
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
        q = [0.0] * 4
        q[i] = 0.25 * s
        q[j] = (m[j, i] + m[i, j]) / s
        q[k] = (m[k, i] + m[i, k]) / s
        q[3] = (m[k, j] - m[j, k]) / s
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def wall_tool_quaternion(wall):
    """Tool orientation facing the wall (z = -normal, x along u): used when no reference plan gives one."""
    z = -wall.n
    x = wall.u - (wall.u @ z) * z
    x /= np.linalg.norm(x)
    return _quat_from_matrix(np.column_stack([x, np.cross(z, x), z]))


def reference_from_plan(wall, plan_path):
    """(quaternion, d_offset, frame_id) of an existing plan: its first orientation, its mean distance from the wall plane
    along the normal, its frame. So the touch-up poses are at the same stand-off and tool attitude as the original."""
    with open(plan_path) as f:
        d = json.load(f)
    path = d['final_lifted_and_expanded']['path']
    poses = path['poses']
    o = poses[0]['orientation']
    ds = [wall.to_uvd([p['position'][c] for c in 'xyz'])[2] for p in poses]
    return (o['x'], o['y'], o['z'], o['w']), float(np.mean(ds)), path.get('frame_id', '')


def build_plan(strips, wall, report, quaternion=None, d_offset=0.0, frame_id='', pose_step_m=DEFAULT_POSE_STEP_M,
               tool=None, wall_id=None):
    """coverage_output.json dict for ``strips`` (serpentine: strip k runs up for even k, down for odd k)."""
    q = quaternion or wall_tool_quaternion(wall)
    poses, segs = [], []
    for k, s in enumerate(strips):
        up = (k % 2 == 0)
        runs = sorted(s['runs'], reverse=not up)
        for lo, hi in runs:
            n = max(1, int(math.ceil((hi - lo) / pose_step_m - 1e-9)))
            vs = [lo + (hi - lo) * t / n for t in range(n + 1)]
            if not up:
                vs.reverse()
            a = len(poses)
            for v in vs:
                x, y, z = wall.to_world(s['u'], v, d_offset)
                poses.append({'position': {'x': x, 'y': y, 'z': z},
                              'orientation': {'x': q[0], 'y': q[1], 'z': q[2], 'w': q[3]}})
            segs.append({'label': 'touch_up_strip_%d' % k, 'start_index': a, 'end_index': len(poses) - 1,
                         'render_start_index': a, 'render_end_index': len(poses) - 1, 'mode': 'cartesian', 'planner': '',
                         'requires_contact': True, 'requires_retraction': True, 'priority': 0, 'tags': ['touch_up']})
    path = {'frame_id': frame_id or wall.frame_id or 'link_base', 'poses': poses}
    return {'wall_scene_json': '', 'wall_id': int(wall.wall_id if wall_id is None else wall_id),
            'resolution_m': report.get('cell_m', 0.0), 'tool_profile': tool or {'name': 'touch_up',
                                                                              'swath_m': report['swath_m'],
                                                                              'overlap_pct': report['overlap_pct'],
                                                                              'sweep_direction': 'vertical', 'passes': 1},
            'touch_up': report,
            'final_lifted_and_expanded': {'path': path, 'segments': segs}}


def check_manager_schema(doc, wall_dir=(1.0, 0.0)):
    """The parsing of CoverageOutputJsonSource on ``doc``: raises KeyError / ValueError where the manager would throw.
    Returns the columns as [(x_key, z_min, z_max)] in the manager's order (ascending along the wall tangent)."""
    plan = doc['final_lifted_and_expanded']
    poses = plan['path']['poses']
    tx, ty = wall_dir[1], -wall_dir[0]
    seg_of = [-1] * len(poses)
    for n, sj in enumerate(plan.get('segments', [])):
        a, b = int(sj.get('start_index', 0)), int(sj.get('end_index', 0))
        if not (0 <= a <= b < len(poses)):
            raise ValueError('segment %d indices %d..%d outside %d poses' % (n, a, b, len(poses)))
        for i in range(a, b + 1):
            seg_of[i] = n
    cols = {}
    for p in poses:
        pos, ori = p['position'], p['orientation']
        vals = [float(pos[c]) for c in 'xyz'] + [float(ori[c]) for c in 'xyzw']
        if not all(math.isfinite(v) for v in vals):
            raise ValueError('non-finite pose')
        cols.setdefault(round((vals[0] * tx + vals[1] * ty) / MANAGER_X_QUANTUM_M), []).append(vals[2])
    return [(k, min(z), max(z)) for k, z in sorted(cols.items())]


def touch_up(csv_path, wall_path=None, wall_id=0, swath_m=cp.DEFAULT_SWATH_M, overlap_pct=DEFAULT_OVERLAP_PCT,
             min_length_m=DEFAULT_MIN_LENGTH_M, cell_m=cp.DEFAULT_CELL_M, force_n=cp.DEFAULT_FORCE_N,
             tol_m=cp.DEFAULT_PLANE_TOL_M, require_active=False, plan_path=None, failed=(), partial=(),
             pose_step_m=DEFAULT_POSE_STEP_M, max_gap_m=None, frame_id=''):
    """run.csv + wall json -> (plan dict or None, report). plan is None when nothing is left to do (report['message'])."""
    wall = cp.load_wall(wall_path or csv_path + '.wall.json', wall_id)
    grid = cp.CoverageGrid(wall, cell_m, swath_m, force_n, tol_m)
    cp.replay(grid, cp.read_run_csv(csv_path), require_active)
    q, d_off, frame = None, 0.0, frame_id
    if plan_path:
        grid.set_columns(cp.load_plan_columns(plan_path))
        grid.mark_outcome(partial=partial, failed=failed)
        q, d_off, f = reference_from_plan(wall, plan_path)
        frame = frame or f
    return touch_up_grid(grid, overlap_pct, min_length_m, pose_step_m, max_gap_m, q, d_off, frame)


def touch_up_grid(grid, overlap_pct=DEFAULT_OVERLAP_PCT, min_length_m=DEFAULT_MIN_LENGTH_M, pose_step_m=DEFAULT_POSE_STEP_M,
                  max_gap_m=None, quaternion=None, d_offset=0.0, frame_id=''):
    strips, rep = plan_strips(grid, grid.swath, overlap_pct, min_length_m, max_gap_m)
    rep['cell_m'] = grid.cell
    if not strips:
        if rep['undone_m2'] <= 0:
            rep['message'] = 'wall fully covered (%.1f %% done): nothing to touch up' % rep['percent_done_before']
        else:
            rep['message'] = ('no strip left: all %.4f m2 of undone area is below the minimum stroke length of %.2f m '
                              '(ignored area)' % (rep['undone_m2'], min_length_m))
        return None, rep
    plan = build_plan(strips, grid.wall, rep, quaternion, d_offset, frame_id, pose_step_m)
    rep['message'] = '%d strokes in %d strips, %.4f m2 ignored (strokes under %.2f m)' % (
        rep['planned_strokes'], rep['planned_strips'], rep['ignored_m2'], min_length_m)
    return plan, rep
