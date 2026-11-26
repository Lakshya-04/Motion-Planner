#!/usr/bin/env python3
"""How long does a cuRobo plan take on the 7-joint lift + arm config, idle and under load?

Runs inside motion-planner:gpu with a lift_arm.yml from generate_curobo_7dof.sh:

    python curobo_plan_timing.py --robot DIR/lift_arm.yml --label idle --out idle.json
    python curobo_plan_timing.py --robot DIR/lift_arm.yml --label vram4 --ballast-gb 12     # leaves ~4 GB on a 16 GB card
    python curobo_plan_timing.py --robot DIR/lift_arm.yml --label loaded --gpu-load --cpu-load 4

Problems: the logged docked poses of docker/tools/curobo_lowlift_poses.json, docked -> stow and stow -> docked, at one
lift height (default 0.05, where the sphere model accepts them), empty world. Each problem is timed `--repeats` times.
Reported per condition: construct + warmup, VRAM, success rate, wall time median / p95 / max, cuRobo's own planning
time, and how many plans ended on the graph-seed fallback. --gpu-load and --cpu-load are crude stand-ins for a
simulator running on the same machine, not the simulator.
"""
import argparse
import json
import multiprocessing as mp
import pathlib
import statistics
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'src/manipulation/curobo_bridge'))
sys.path.insert(0, '/repo/src/manipulation/curobo_bridge')


def _ballast(gb, ready, stop):
    import torch
    keep = torch.empty(int(gb * 2 ** 30), dtype=torch.uint8, device='cuda')
    keep.fill_(1)
    ready.set()
    stop.wait()


def _gpu_load(size, ready, stop):
    import torch
    x = torch.randn(size, size, device='cuda')
    ready.set()
    while not stop.is_set():
        float((x @ x).sum())


def _cpu_load(stop):
    while not stop.is_set():
        sum(i * i for i in range(20000))


def pct(values, p):
    if not values:
        return float('nan')
    s = sorted(values)
    return s[min(len(s) - 1, int(round(p / 100.0 * (len(s) - 1))))]


def vram_used_mib():
    import torch
    free, total = torch.cuda.mem_get_info()
    return (total - free) / 2 ** 20


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot', required=True)
    ap.add_argument('--poses', default=str(HERE / 'curobo_lowlift_poses.json'))
    ap.add_argument('--lift', type=float, default=0.05)
    ap.add_argument('--repeats', type=int, default=2)
    ap.add_argument('--max-attempts', type=int, default=20, help='cuRobo attempts per plan (node default 20)')
    ap.add_argument('--label', default='run')
    ap.add_argument('--ballast-gb', type=float, default=0.0, help='GPU memory held by another process')
    ap.add_argument('--gpu-load', action='store_true', help='another process keeps the GPU busy with matmuls')
    ap.add_argument('--cpu-load', type=int, default=0, help='number of busy CPU processes')
    ap.add_argument('--out')
    args = ap.parse_args()

    ctx = mp.get_context('spawn')
    stop = ctx.Event()
    procs = []

    def start(target, *a):
        ready = ctx.Event()
        p = ctx.Process(target=target, args=(*a, ready, stop), daemon=True)
        p.start()
        if not ready.wait(300):
            raise SystemExit(f'{target.__name__} did not start')
        procs.append(p)

    if args.ballast_gb > 0:
        start(_ballast, args.ballast_gb)
    if args.gpu_load:
        start(_gpu_load, 4096)
    for _ in range(args.cpu_load):
        p = ctx.Process(target=_cpu_load, args=(stop,), daemon=True)
        p.start()
        procs.append(p)

    import torch
    base_mib = vram_used_mib()
    from curobo_bridge.planner import CuroboPlanner
    t0 = time.monotonic()
    planner = CuroboPlanner(robot=args.robot, max_boxes=16, prismatic_joints=['lift_joint'],
                            max_attempts=args.max_attempts, precheck_endpoints=True)
    construct_s = time.monotonic() - t0
    peak_mib = vram_used_mib() - base_mib
    print(f'[{args.label}] construct + warmup {construct_s:.1f} s, VRAM above base {peak_mib:.0f} MiB '
          f'(base {base_mib:.0f} MiB)', flush=True)

    poses = json.load(open(args.poses))
    stow = list(poses['stow'])
    docked = {n: [args.lift] + list(q[1:]) for n, q in poses['docked'].items()}
    ok = planner.check_states(list(docked.values()))
    docked = {n: q for (n, q), v in zip(docked.items(), ok) if v}
    print(f'[{args.label}] {len(docked)} of {len(poses["docked"])} docked poses valid at lift {args.lift}', flush=True)
    problems = []
    for n, q in docked.items():
        problems += [(f'{n}->stow', q, stow), (f'stow->{n}', stow, q)]

    rows = []
    for rep in range(args.repeats):
        for name, a, b in problems:
            t = time.monotonic()
            r = planner.plan_joint(a, b)
            wall = time.monotonic() - t
            rows.append({'problem': name, 'rep': rep, 'success': bool(r.success), 'wall_s': wall,
                         'curobo_s': float(r.planning_time_s), 'route': r.route, 'code': r.failure_reason_code})
    peak_run_mib = vram_used_mib() - base_mib
    stop.set()

    ok_rows = [r for r in rows if r['success']]
    walls = [r['wall_s'] for r in ok_rows]
    summary = {
        'label': args.label, 'gpu': torch.cuda.get_device_name(0), 'ballast_gb': args.ballast_gb,
        'gpu_load': args.gpu_load, 'cpu_load': args.cpu_load, 'max_attempts': args.max_attempts, 'lift': args.lift,
        'construct_s': construct_s, 'vram_above_base_mib_after_construct': peak_mib,
        'vram_above_base_mib_after_plans': peak_run_mib, 'plans': len(rows), 'succeeded': len(ok_rows),
        'wall_median_s': statistics.median(walls) if walls else None, 'wall_p95_s': pct(walls, 95),
        'wall_max_s': max(walls) if walls else None,
        'curobo_median_s': statistics.median([r['curobo_s'] for r in ok_rows]) if ok_rows else None,
        'graph_seed_plans': sum(1 for r in ok_rows if r['route'] == 'graph_seed'),
        'failure_codes': sorted({r['code'] for r in rows if not r['success']}),
        'slowest': sorted(rows, key=lambda r: -r['wall_s'])[:3],
    }
    print(json.dumps({k: v for k, v in summary.items() if k != 'slowest'}, indent=1), flush=True)
    if args.out:
        json.dump({'summary': summary, 'rows': rows}, open(args.out, 'w'), indent=1, default=str)
        print('wrote', args.out)
    planner.close()
    for p in procs:
        p.join(5)


if __name__ == '__main__':
    main()
