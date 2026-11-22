#!/usr/bin/env python3
"""Offline sweep paint of a recorded run: run.csv (t,x,y,z,force[,active]) + run.csv.wall.json -> PNG and numbers.

    coverage_replay.py run.csv [--wall run.csv.wall.json] [--out run.coverage.png] [--swath 0.14] [--cell 0.02]
        [--force 1.0] [--tol 0.03] [--plan coverage_output.json --failed 3 7 --partial 5] [--overlay-json feed.json]
        [--require-active] [--title TEXT] [--wall-id 0]

No ROS. Prints one JSON line with the percent covered. --overlay-json writes the sim-neutral primitives for the MuJoCo viewer
feed / gz_overlay.py --from-json. --plan with --failed/--partial paints those columns orange where nothing was swept.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from sim_overlay import coverage_paint as cp  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv')
    ap.add_argument('--wall')
    ap.add_argument('--wall-id', type=int, default=0)
    ap.add_argument('--out')
    ap.add_argument('--swath', type=float, default=cp.DEFAULT_SWATH_M)
    ap.add_argument('--cell', type=float, default=cp.DEFAULT_CELL_M)
    ap.add_argument('--force', type=float, default=cp.DEFAULT_FORCE_N, help='contact threshold N (<= 0 ignores force)')
    ap.add_argument('--tol', type=float, default=cp.DEFAULT_PLANE_TOL_M)
    ap.add_argument('--require-active', action='store_true')
    ap.add_argument('--plan')
    ap.add_argument('--failed', type=int, nargs='*', default=[])
    ap.add_argument('--partial', type=int, nargs='*', default=[])
    ap.add_argument('--overlay-json')
    ap.add_argument('--title', default='')
    a = ap.parse_args(argv)
    wall = cp.load_wall(a.wall or a.csv + '.wall.json', a.wall_id)
    grid = cp.CoverageGrid(wall, a.cell, a.swath, a.force, a.tol)
    cp.replay(grid, cp.read_run_csv(a.csv), a.require_active)
    if a.plan:
        grid.set_columns(cp.load_plan_columns(a.plan))
    grid.mark_outcome(partial=a.partial, failed=a.failed)
    out = a.out or os.path.splitext(a.csv)[0] + '.coverage.png'
    cp.render_png(grid, out, a.title or os.path.basename(a.csv))
    if a.overlay_json:
        from sim_overlay.mujoco_overlay import save_overlay
        from sim_overlay.overlay import OverlayState
        st = OverlayState()
        st.update('/mp_viz/coverage_paint', grid.marker_dicts())
        save_overlay(a.overlay_json, st.primitives())
    res = dict(grid.summary(), png=out)
    print(json.dumps(res))
    return res


if __name__ == '__main__':
    main()
