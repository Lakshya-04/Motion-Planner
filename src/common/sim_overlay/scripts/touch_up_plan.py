#!/usr/bin/env python3
"""Touch-up plan of a finished or aborted run: the strips that are still undone, as a coverage_output.json for the manager.

    touch_up_plan.py run.csv [--wall run.csv.wall.json] [--out touch_up_coverage.json] [--tool-profiles tool_profiles.yaml
        --tool contact_wiper | --swath 0.14 --overlap-pct 20] [--min-length 0.10] [--plan coverage_output.json
        --failed 3 7 --partial 5] [--cell 0.02] [--force 1.0] [--tol 0.03] [--require-active] [--max-gap M] [--pose-step 0.05]

No ROS. Prints one JSON line (numbers, message) and writes the plan only when something is left; exit code 0 either way.
--plan takes the tool attitude, stand-off and frame from the original plan (and its columns for --failed / --partial).
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from sim_overlay import coverage_paint as cp  # noqa: E402
from sim_overlay import touch_up as tu  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv')
    ap.add_argument('--wall')
    ap.add_argument('--wall-id', type=int, default=0)
    ap.add_argument('--out')
    ap.add_argument('--tool-profiles', help='tool_profiles.yaml: swath, overlap and sweep direction come from --tool')
    ap.add_argument('--tool', default='contact_wiper')
    ap.add_argument('--swath', type=float, help='m (default: tool profile, else %s)' % cp.DEFAULT_SWATH_M)
    ap.add_argument('--overlap-pct', type=float, help='percent (default: tool profile, else %s)' % tu.DEFAULT_OVERLAP_PCT)
    ap.add_argument('--min-length', type=float, default=tu.DEFAULT_MIN_LENGTH_M, help='drop strokes shorter than this [m]')
    ap.add_argument('--max-gap', type=float, help='stroke through done stretches shorter than this [m] (default: none)')
    ap.add_argument('--pose-step', type=float, default=tu.DEFAULT_POSE_STEP_M)
    ap.add_argument('--cell', type=float, default=cp.DEFAULT_CELL_M)
    ap.add_argument('--force', type=float, default=cp.DEFAULT_FORCE_N)
    ap.add_argument('--tol', type=float, default=cp.DEFAULT_PLANE_TOL_M)
    ap.add_argument('--require-active', action='store_true')
    ap.add_argument('--plan')
    ap.add_argument('--failed', type=int, nargs='*', default=[])
    ap.add_argument('--partial', type=int, nargs='*', default=[])
    ap.add_argument('--frame-id', default='')
    a = ap.parse_args(argv)
    swath, overlap = a.swath, a.overlap_pct
    if a.tool_profiles:
        s, o, sweep = tu.load_tool_coverage(a.tool_profiles, a.tool)
        if sweep != 'vertical':
            sys.exit('touch_up_plan: sweep_direction %r not supported (the manager strokes along z: vertical only)' % sweep)
        swath = s if swath is None else swath
        overlap = o if overlap is None else overlap
    swath = cp.DEFAULT_SWATH_M if swath is None else swath
    overlap = tu.DEFAULT_OVERLAP_PCT if overlap is None else overlap
    plan, rep = tu.touch_up(a.csv, a.wall, a.wall_id, swath, overlap, a.min_length, a.cell, a.force, a.tol, a.require_active,
                            a.plan, a.failed, a.partial, a.pose_step, a.max_gap, a.frame_id)
    out = a.out or os.path.splitext(a.csv)[0] + '.touch_up.json'
    if plan is not None:
        tu.check_manager_schema(plan)
        with open(out, 'w') as f:
            json.dump(plan, f, indent=2)
        rep['plan'] = out
    print(json.dumps(rep))
    print(rep['message'], file=sys.stderr)
    return rep


if __name__ == '__main__':
    main()
