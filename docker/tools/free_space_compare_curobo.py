#!/usr/bin/env python3
"""cuRobo side of the free-space comparison (needs the GPU image; docs/free_space_comparison.md).

Reads the result file of free_space_compare_node (its `header` and `transition` lines hold the joint names and the
resolved start and goal joint states) and plans every transition with cuRobo in two modes, `curobo_default` and
`curobo_optimal` (CuroboPlanner free_space_optimize, docs/curobo_free_space_optimal.md), writing the same JSON lines
(type run: success, planning_s, duration_s, positions in the MoveIt group's joint order). The TCP length is added
afterwards by `free_space_compare_node -p fk_in:=...`, so this script needs no kinematics of its own.

  PYTHONPATH=/repo/src/manipulation/curobo_bridge /opt/venv/bin/python docker/tools/free_space_compare_curobo.py \\
    --moveit-results OUT/moveit_results.jsonl --robot /abs/assets/lift_arm.yml --prismatic lift_joint \\
    --out OUT/curobo_results.jsonl [--world-npy cloud_in_curobo_base_frame.npy] [--budget 20] [--candidates 6]

The cuRobo world is empty unless --world-npy (an N x 3 array in the cuRobo base frame) is given: a collision free
cuRobo plan is then only collision free against that world. State this next to any number.
"""
import argparse
import json
import sys
import time


def reorder(values, src_names, dst_names):
    """values in src_names order -> dst_names order; None when a name is missing."""
    idx = {n: i for i, n in enumerate(src_names)}
    if any(n not in idx for n in dst_names):
        return None
    return [values[idx[n]] for n in dst_names]


def read_moveit(path):
    header, transitions = {}, []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            if r.get('type') == 'header' and not header:
                header = r
            elif r.get('type') == 'transition':
                transitions.append(r)
    return header, transitions


def plan_all(planner, mode, header, transitions, out):
    names = header['joint_names']
    cnames = planner.joint_names
    for t in transitions:
        s = reorder(t['start'], names, cnames)
        g = reorder(t['goal_q'], names, cnames)
        rec = {'type': 'run', 'transition': t['transition'], 'mode': mode, 'rep': 0, 'success': False}
        if s is None or g is None:
            rec['reason'] = 'joint names differ between MoveIt group and cuRobo robot'
        else:
            t0 = time.time()
            r = planner.plan_joint(s, g)
            rec.update(success=bool(r.success), planning_s=time.time() - t0, route=r.route or mode,
                       reason=r.failure_reason_code)
            if r.success:
                rec['duration_s'] = r.duration
                rec['points'] = len(r.positions)
                rec['positions'] = [reorder(p, cnames, names) for p in r.positions]
        out.write(json.dumps(rec) + '\n')
        out.flush()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--moveit-results', required=True)
    ap.add_argument('--robot', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--prismatic', nargs='*', default=[])
    ap.add_argument('--world-npy')
    ap.add_argument('--budget', type=float, default=20.0)
    ap.add_argument('--candidates', type=int, default=6)
    ap.add_argument('--objective', choices=('length', 'duration'), default='length')
    a = ap.parse_args(argv)
    header, transitions = read_moveit(a.moveit_results)
    if not header or not transitions:
        print('free_space_compare_curobo: no header/transition lines in the MoveIt result file', file=sys.stderr)
        return 1
    from curobo_bridge.planner import CuroboPlanner
    world = None
    if a.world_npy:
        import numpy as np
        world = np.load(a.world_npy)
    with open(a.out, 'w') as out:
        for mode, kwargs in (('curobo_default', {}),
                             ('curobo_optimal', dict(free_space_optimize=True, free_space_time_budget_s=a.budget,
                                                     free_space_num_candidates=a.candidates,
                                                     free_space_objective=a.objective))):
            planner = CuroboPlanner(a.robot, prismatic_joints=a.prismatic, **kwargs)
            if world is not None:
                planner.set_world(points=world)
            plan_all(planner, mode, header, transitions, out)
            planner.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
