#!/usr/bin/env python3
"""Per-segment-type comparison of two MuJoCo pass output dirs (e.g. MoveIt/OMPL reference vs cuRobo backend).

usage: compare_backends.py LABEL=RUN_DIR [LABEL=RUN_DIR ...] [--json out.json]

Per segment_exec line of manager.log: planning_ms, execution_ms, traj_s, fallback flag, planner/backend, outcome.
Executed joint path length (rad, 7 joints incl. lift as logged in run_film.npz) and TCP path (m, run.csv) are
computed over the window [log stamp - execution_ms, log stamp] of each segment: run_film.npz 't' is on the same
epoch clock as the manager log stamps; run.csv 't' is relative, so its offset is fitted on the press segments
(first sample with force>1 N of each stroke vs the start of each press window). Both are EXECUTED (controller) paths, not planned ones;
a segment's window can include ~0.1-1 s of settle. Peak force comes from run.csv per window (force column).
"""
import json, os, re, statistics, sys
import numpy as np

SEG = re.compile(r'\[(\d+\.\d+)\] \[manipulation_manager\]: segment_exec id=(\d+) label=(\S+) outcome=(\S+) mode=(\S+) planner=(\S+) '
                 r'fsp_backend=(\S+) .*?reason=(\S+) stage=(\S+) planning_ms=(\d+) execution_ms=(\d+) fallback=(\S+) .*?traj_pts=(\d+) traj_s=([\d.]+)')


def seg_type(label):
    return re.sub(r'^wall_column_\d+_', '', label)


def load(run_dir):
    segs = []
    for line in open(os.path.join(run_dir, 'manager.log'), errors='replace'):
        m = SEG.search(line)
        if m:
            e, _id, label, outc, mode, planner, be, reason, stage, pl, ex, fb, pts, ts = m.groups()
            segs.append(dict(end=float(e), label=label, type=seg_type(label), outcome=outc, mode=mode, backend=be, reason=reason,
                             planning_ms=int(pl), execution_ms=int(ex), fallback=(fb == 'true'), traj_pts=int(pts), traj_s=float(ts)))
    z = np.load(os.path.join(run_dir, 'run_film.npz'))
    ft, fq = z['t'], z['q']
    rows = np.genfromtxt(os.path.join(run_dir, 'run.csv'), delimiter=',', names=True)
    rt, xyz, force, active = rows['t'], np.c_[rows['x'], rows['y'], rows['z']], rows['force'], rows['active']
    offs = []
    for s in segs:
        if s['type'] == 'press' and s['outcome'] == 'completed':
            t0 = s['end'] - s['execution_ms'] / 1000.0
            offs.append(t0)
    # fit offset: first active sample of each stroke (runs of active==1 separated by >5 s)
    idx = np.flatnonzero(force > 1.0)
    starts = [rt[idx[0]]] + [rt[b] for a, b in zip(idx[:-1], idx[1:]) if rt[b] - rt[a] > 5.0] if len(idx) else []
    off = None
    if offs and len(starts) >= len(offs):
        d = [a - b for a, b in zip(sorted(offs), starts[:len(offs)])]
        off = statistics.median(d)
        spread = max(d) - min(d)
        if spread > 3.0:                      # stroke starts do not line up with the press windows: do not trust it
            print('WARNING: run.csv clock offset spread %.1f s, TCP/force per segment skipped' % spread, file=sys.stderr)
            off = None
    for s in segs:
        t1, t0 = s['end'], s['end'] - s['execution_ms'] / 1000.0
        i0, i1 = np.searchsorted(ft, [t0, t1])
        q = fq[i0:i1 + 1]
        s['joint_path_rad'] = float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum()) if len(q) > 1 else 0.0
        s['joint_straight_rad'] = float(np.linalg.norm(q[-1] - q[0])) if len(q) > 1 else 0.0
        if off is not None:
            j0, j1 = np.searchsorted(rt + off, [t0, t1])
            p = xyz[j0:j1 + 1]
            s['tcp_path_m'] = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) if len(p) > 1 else 0.0
            s['peak_force_n'] = float(force[j0:j1 + 1].max()) if j1 >= j0 and len(force[j0:j1 + 1]) else None
    return segs, off


def med(v):
    v = [x for x in v if x is not None]
    return statistics.median(v) if v else None


def summarise(segs):
    out = {}
    for ty in sorted({s['type'] for s in segs}):
        g = [s for s in segs if s['type'] == ty]
        fsp = [x for x in g if x['mode'] == 'fsp']
        out[ty] = dict(ompl_in_curobo_run=sum(x['backend'] == 'ompl' for x in fsp) if any(x['backend'] == 'curobo' for x in segs) else 0,n=len(g), completed=sum(s['outcome'] == 'completed' for s in g), failed=sum(s['outcome'] != 'completed' for s in g),
                       backends=sorted({s['backend'] for s in g}), fallback=sum(s['fallback'] for s in g),
                       planning_ms_med=med(s['planning_ms'] for s in g), planning_ms_max=max(s['planning_ms'] for s in g),
                       exec_s_med=med(s['execution_ms'] / 1000 for s in g), traj_s_med=med(s['traj_s'] for s in g),
                       joint_rad_med=med(s['joint_path_rad'] for s in g), tcp_m_med=med(s.get('tcp_path_m') for s in g),
                       peak_force_n=max([s.get('peak_force_n') or 0 for s in g]))
    return out


def main(argv):
    jo = None
    if '--json' in argv:
        i = argv.index('--json'); jo = argv[i + 1]; argv = argv[:i] + argv[i + 2:]
    res = {}
    for a in argv:
        lab, d = a.split('=', 1)
        segs, off = load(d)
        res[lab] = dict(segments=segs, summary=summarise(segs), run_csv_offset=off)
        print('\n## %s  (%s)  segments=%d  run.csv clock offset=%s' % (lab, d, len(segs), off))
        print('| type | n ok/fail | backend | fallbacks (flag / ompl-in-curobo-run) | plan ms med (max) | exec s med | joint rad med | TCP m med | peak N |')
        print('|---|---|---|---|---|---|---|---|---|')
        f = lambda v, p='.2f': '-' if v is None else format(v, p)
        for ty, r in res[lab]['summary'].items():
            print('| %s | %d/%d | %s | %d / %d | %s (%s) | %s | %s | %s | %s |' % (ty, r['completed'], r['failed'], ','.join(r['backends']), r['fallback'], r['ompl_in_curobo_run'],
                  f(r['planning_ms_med'], '.0f'), f(r['planning_ms_max'], '.0f'), f(r['exec_s_med'], '.1f'), f(r['joint_rad_med']), f(r['tcp_m_med']), f(r['peak_force_n'], '.1f')))
    if jo:
        json.dump(res, open(jo, 'w'), indent=1)


if __name__ == '__main__':
    main(sys.argv[1:])
