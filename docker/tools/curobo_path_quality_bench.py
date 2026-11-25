#!/usr/bin/env python3
"""Free-space path quality of the cuRobo backend, default mode versus the optimal mode (needs a GPU).

Draws M random start/goal pairs inside the joint limits (kept only when cuRobo's own state check accepts both),
plans every pair with a default planner and with an optimal-mode planner (curobo_bridge.planner.CuroboPlanner,
free_space_optimize), and prints, per pair and as a summary, the weighted joint path length (each joint
normalised by its range, curobo_bridge.path_quality), the duration of the returned trajectory, the planning time
and the efficiency against the straight joint-space line.

Run it where cuRobo and torch are installed (the gpu image's venv), with the bridge package on the path:

  docker run --rm --gpus all -v "$PWD":/repo -v /abs/assets:/abs/assets:ro motion-planner:gpu \\
    env PYTHONPATH=/repo/src/manipulation/curobo_bridge /opt/venv/bin/python \\
    /repo/docker/tools/curobo_path_quality_bench.py --robot /abs/assets/lift_arm.yml -m 20 \\
    --prismatic lift_joint --budget 20 --candidates 6 --objective length [--json out.json]

Written for the GPU lane; the table code is host-testable (--selftest runs it on synthetic numbers).
"""
import argparse
import json
import statistics
import sys


def summarize(rows):
    """Rows: dicts with default/optimal entries {ok, length, duration, time}. Returns the table text."""
    lines = ['pair | default len  dur(s)  t(s) | optimal len  dur(s)  t(s) | len ratio  dur ratio',
             '-----+----------------------------+---------------------------+--------------------']
    ratios_l, ratios_d = [], []
    for i, r in enumerate(rows):
        d, o = r['default'], r['optimal']

        def cell(x):
            return f"{x['length']:8.3f} {x['duration']:7.2f} {x['time']:6.1f}" if x['ok'] else '    failed' + ' ' * 14
        rl = o['length'] / d['length'] if d['ok'] and o['ok'] and d['length'] > 1e-9 else None
        rd = o['duration'] / d['duration'] if d['ok'] and o['ok'] and d['duration'] > 1e-9 else None
        if rl is not None:
            ratios_l.append(rl)
        if rd is not None:
            ratios_d.append(rd)
        lines.append(f"{i:4d} | {cell(d)} | {cell(o)} | {'' if rl is None else f'{rl:8.3f}'}  "
                     f"{'' if rd is None else f'{rd:8.3f}'}")
    n = len(rows)
    for mode in ('default', 'optimal'):
        ok = [r[mode] for r in rows if r[mode]['ok']]
        if ok:
            lines.append(f"{mode:8s}: solved {len(ok)}/{n}, mean length {statistics.mean(x['length'] for x in ok):.3f}, "
                         f"mean duration {statistics.mean(x['duration'] for x in ok):.2f} s, "
                         f"mean planning time {statistics.mean(x['time'] for x in ok):.1f} s")
        else:
            lines.append(f'{mode:8s}: solved 0/{n}')
    if ratios_l:
        lines.append(f'optimal / default (pairs both solved): length mean {statistics.mean(ratios_l):.3f}, '
                     f'worst {max(ratios_l):.3f}; duration mean {statistics.mean(ratios_d):.3f}, worst {max(ratios_d):.3f}')
    return '\n'.join(lines)


def _selftest():
    ok = lambda l, d, t: {'ok': True, 'length': l, 'duration': d, 'time': t}  # noqa: E731
    print(summarize([{'default': ok(2.0, 4.0, 1.0), 'optimal': ok(1.0, 2.0, 9.0)},
                     {'default': {'ok': False}, 'optimal': ok(1.0, 2.0, 9.0)}]))


def _plan_all(planner, pairs, weights, path_quality):
    out = []
    for start, goal in pairs:
        r = planner.plan_joint(start, goal)
        if r.success:
            out.append({'ok': True, 'length': path_quality.path_length(r.positions, weights),
                        'duration': r.duration, 'time': r.planning_time_s, 'route': r.route})
        else:
            out.append({'ok': False, 'reason': r.failure_reason_code})
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robot', help='cuRobo robot yml (absolute path)')
    ap.add_argument('-m', '--pairs', type=int, default=20)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--prismatic', nargs='*', default=[])
    ap.add_argument('--budget', type=float, default=20.0)
    ap.add_argument('--candidates', type=int, default=6)
    ap.add_argument('--objective', choices=('length', 'duration'), default='length')
    ap.add_argument('--max-boxes', type=int, default=16)
    ap.add_argument('--json', help='also write the raw rows here')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args(argv)
    if a.selftest:
        return _selftest()
    if not a.robot:
        ap.error('--robot is required')
    import numpy as np
    from curobo_bridge import path_quality
    from curobo_bridge.planner import CuroboPlanner

    rng = np.random.default_rng(a.seed)
    default = CuroboPlanner(a.robot, max_boxes=a.max_boxes, prismatic_joints=a.prismatic)
    lo, hi = default._joint_limits()
    lo, hi = np.array(lo), np.array(hi)
    weights = path_quality.joint_weights(lo, hi)
    pairs = []
    while len(pairs) < a.pairs:      # valid, distinct endpoints only
        s, g = (rng.uniform(lo, hi) for _ in range(2))
        if all(default.check_states([s.tolist(), g.tolist()])):
            pairs.append((s.tolist(), g.tolist()))
    base = _plan_all(default, pairs, weights, path_quality)
    default.close()
    optimal = CuroboPlanner(a.robot, max_boxes=a.max_boxes, prismatic_joints=a.prismatic,
                            free_space_optimize=True, free_space_time_budget_s=a.budget,
                            free_space_num_candidates=a.candidates, free_space_objective=a.objective)
    best = _plan_all(optimal, pairs, weights, path_quality)
    optimal.close()
    rows = [{'start': s, 'goal': g, 'default': d, 'optimal': o} for (s, g), d, o in zip(pairs, base, best)]
    print(summarize(rows))
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(rows, f, indent=1)


if __name__ == '__main__':
    sys.exit(main())
