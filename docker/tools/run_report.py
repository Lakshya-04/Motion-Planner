#!/usr/bin/env python3
"""One report for a finished run directory: report.md, report.png and report.json.

ROS-free. Reads what run_multi_stand_pass.sh (MuJoCo) or the Gazebo pass leaves in the run directory:
coverage_output.json, run.csv (+ run.csv.wall.json), manager.log, optionally a verify json, a score card json,
run_film.npz and *.contacts.csv. Missing files only skip their section (with a note); the tool never fails on a
partial directory.

  run_report.py RUN_DIR [--out DIR] [--target-force 10] [--band 1.5] [--engaged-force 1] [--pitch M]

Sections: result, force (score card when present, else computed from run.csv), per-column peak force (columns found
by clustering the engaged samples along the wall's u axis), time split (contact stroke from run.csv, free-space and
other motion from the manager log segment_exec lines, idle = the rest), free-space path efficiency (run_film.npz),
coverage map (sim_overlay coverage_paint when importable), plan redundancy (plan_redundancy_report binary when on PATH,
else a python mirror), run metrics (sim_mujoco run_metrics), flags and a one-line verdict. Exit code 0 always.
"""
import argparse
import csv
import glob
import importlib.util
import json
import math
import os
import re
import shutil
import statistics
import subprocess
import sys

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
PEAK_FLAG_N = 40.0
REACH_FLAG = 0.85
CHASSIS_FLAG_M = 0.30
FREE_SHARE_FLAG = 0.25
SPIKE_FACTOR = 2.0
SEG_KEYS = re.compile(r'(\w+)=(\[[^\]]*\]|\S+)')
SEG_LINE = re.compile(r'segment_exec\s')
COLS_LINE = re.compile(r'columns_done\s*\[(?P<d>[^\]]*)\]\s*partial\s*\[(?P<p>[^\]]*)\]\s*failed\s*\[(?P<f>[^\]]*)\]')


def _load_by_path(name, path):
    if not os.path.isfile(path):
        return None
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    try:
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
    except Exception:
        sys.modules.pop(name, None)
        return None
    return mod


def _ints(s):
    return [int(x) for x in re.findall(r'-?\d+', s)]


def _read_text(path):
    try:
        with open(path, errors='replace') as f:
            return f.read().splitlines()
    except OSError:
        return []


def _read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


# ---- readers -------------------------------------------------------------------------------------------------

def read_run_csv(path):
    """-> dict of numpy arrays t, xyz (n,3), force; empty arrays when the file is missing."""
    t, p, f = [], [], []
    if os.path.isfile(path):
        with open(path, newline='') as fh:
            for rec in csv.DictReader(fh):
                try:
                    tt, ff = float(rec['t']), float(rec['force'])
                    xyz = (float(rec['x']), float(rec['y']), float(rec['z']))
                except (KeyError, ValueError, TypeError):
                    continue
                if math.isfinite(ff):
                    t.append(tt), p.append(xyz), f.append(ff)
    return dict(t=np.array(t), xyz=np.array(p).reshape(-1, 3), force=np.array(f))


def parse_segments(lines):
    out = []
    for ln in lines:
        if not SEG_LINE.search(ln):
            continue
        d = {k: v for k, v in SEG_KEYS.findall(ln.split('segment_exec', 1)[1])}
        try:
            out.append(dict(label=d.get('label', ''), mode=d.get('mode', ''), outcome=d.get('outcome', ''),
                            traj_s=float(d.get('traj_s', 0) or 0), exec_ms=float(d.get('execution_ms', 0) or 0)))
        except ValueError:
            continue
    return out


def parse_columns(manager_lines, client_lines, verify):
    """columns done/partial/failed: the manager log line, else the client log, else the verify json."""
    for lines in (manager_lines, client_lines):
        for ln in reversed(lines):
            m = COLS_LINE.search(ln)
            if m:
                return dict(done=_ints(m['d']), partial=_ints(m['p']), failed=_ints(m['f']), source='log')
    if isinstance(verify, dict):
        def pick(*keys):
            for k in keys:
                if isinstance(verify.get(k), list):
                    return [int(x) for x in verify[k]]
            return None
        d, p, f = pick('columns_done', 'done_columns'), pick('partial', 'columns_partial'), pick('failed', 'columns_failed')
        if d is not None or f is not None:
            return dict(done=d or [], partial=p or [], failed=f or [], source='verify json')
    return None


def find_jsons(run_dir):
    """-> (score card dict or None, verify dict or None) among the run directory's json files."""
    card = verify = None
    for path in sorted(glob.glob(os.path.join(run_dir, '*.json'))):
        base = os.path.basename(path)
        if base in ('coverage_output.json', 'run.csv.wall.json', 'report.json'):
            continue
        d = _read_json(path)
        if not isinstance(d, dict):
            continue
        if isinstance(d.get('force'), dict) and 'peak_n' in d['force']:
            card = card or d
        elif any(k in d for k in ('columns_done', 'done_columns', 'failed_segment_ids')):
            verify = verify or d
    return card, verify


# ---- analysis ------------------------------------------------------------------------------------------------

def engaged_mask(force, engaged_n):
    return force >= engaged_n


def force_stats(run, card, target_n, band_n, engaged_n):
    f = run['force']
    res = dict(source=None, peak_n=None, rms_error_n=None, in_band_fraction=None, engaged_samples=0)
    if card is not None:
        fc = card['force']
        res.update(source='score card', peak_n=fc.get('peak_n'), rms_error_n=fc.get('rms_error_n'),
                   in_band_fraction=fc.get('in_band_fraction'))
    if len(f):
        m = engaged_mask(f, engaged_n)
        res['engaged_samples'] = int(m.sum())
        if res['source'] is None:
            res['source'] = 'run.csv'
            res['peak_n'] = float(f.max())
            if m.any():
                err = f[m] - target_n
                res['rms_error_n'] = float(math.sqrt(np.mean(err ** 2)))
                res['in_band_fraction'] = float(np.mean(np.abs(err) <= band_n))
        res['peak_csv_n'] = float(f.max())
        res['rms_force_n'] = float(math.sqrt(np.mean(f[m] ** 2))) if m.any() else 0.0
    return res


def engaged_runs(run, engaged_n):
    """Index ranges [a, b] (inclusive) of consecutive engaged samples."""
    m = engaged_mask(run['force'], engaged_n)
    runs, a = [], None
    for i, v in enumerate(m):
        if v and a is None:
            a = i
        elif not v and a is not None:
            runs.append((a, i - 1))
            a = None
    if a is not None:
        runs.append((a, len(m) - 1))
    return runs


def column_table(run, wall, engaged_n, target_n, band_n, gap_m=0.05):
    """Per-column force: engaged samples clustered along the wall's u axis (a column is a vertical stroke)."""
    m = engaged_mask(run['force'], engaged_n) if len(run['force']) else np.zeros(0, bool)
    if not m.any() or wall is None:
        return []
    u_axis = np.array(wall['u'], float)
    u_axis /= np.linalg.norm(u_axis)
    u = (run['xyz'][m] - np.array(wall['origin'], float)) @ u_axis
    f = run['force'][m]
    order = np.argsort(u)
    u, f = u[order], f[order]
    cuts = np.where(np.diff(u) > gap_m)[0] + 1
    rows = []
    for k, (ug, fg) in enumerate(zip(np.split(u, cuts), np.split(f, cuts))):
        err = fg - target_n
        rows.append(dict(index=k, u_m=float(ug.mean()), samples=int(len(fg)), peak_n=float(fg.max()),
                         mean_n=float(fg.mean()), in_band=float(np.mean(np.abs(err) <= band_n))))
    return rows


def time_split(run, segments, engaged_n):
    if not len(run['t']):
        return None
    t, f = run['t'], run['force']
    duration = float(t[-1] - t[0])
    dt = np.diff(t, prepend=t[0])
    dt = np.where(dt > 1.0, 0.0, dt)            # a recording gap is not contact time
    contact = float(dt[engaged_mask(f, engaged_n)].sum())
    free = sum(s['traj_s'] for s in segments if s['mode'] and s['mode'] != 'cartesian')
    cart = sum(s['traj_s'] for s in segments if s['mode'] == 'cartesian')
    other = max(0.0, cart - contact)
    idle = max(0.0, duration - contact - free - other)
    return dict(duration_s=duration, contact_s=contact, free_space_s=float(free), other_motion_s=float(other),
                idle_s=float(idle), free_space_share=(free / duration if duration > 0 else 0.0),
                free_space_segments=sum(1 for s in segments if s['mode'] and s['mode'] != 'cartesian'),
                segments=len(segments))


def path_efficiency(run, npz_path, engaged_n, min_gap_s=1.0):
    """Joint and TCP path against the straight line, per gap between engaged runs (needs run_film.npz)."""
    if not os.path.isfile(npz_path) or not len(run['t']):
        return None
    z = np.load(npz_path)
    if 't' not in z or 'q' not in z:
        return None
    ft, fq = z['t'], z['q']
    runs = engaged_runs(run, engaged_n)
    t0 = float(ft[0])
    rt = run['t']
    offset = 0.0 if abs(float(rt[0]) - t0) < 5.0 else t0 - float(rt[0])    # film clock vs run.csv clock
    gaps = []
    for (_, b0), (a1, _) in zip(runs[:-1], runs[1:]):
        ta, tb = float(rt[b0]), float(rt[a1])
        if tb - ta < min_gap_s:
            continue
        i0, i1 = np.searchsorted(ft, [ta + offset, tb + offset])
        seg = fq[i0:i1 + 1]
        c0, c1 = b0, a1
        xyz = run['xyz'][c0:c1 + 1]
        if len(seg) < 2 or len(xyz) < 2:
            continue
        jp = float(np.linalg.norm(np.diff(seg, axis=0), axis=1).sum())
        js = float(np.linalg.norm(seg[-1] - seg[0]))
        tp = float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())
        ts = float(np.linalg.norm(xyz[-1] - xyz[0]))
        gaps.append(dict(t0=ta, t1=tb, joint_path=jp, joint_straight=js,
                         joint_eff=(js / jp if jp > 1e-6 else None), tcp_path_m=tp, tcp_straight_m=ts,
                         tcp_eff=(ts / tp if tp > 1e-6 else None)))
    if not gaps:
        return dict(gaps=[], note='no gap between engaged runs longer than %.1f s' % min_gap_s)
    je = [g['joint_eff'] for g in gaps if g['joint_eff'] is not None]
    te = [g['tcp_eff'] for g in gaps if g['tcp_eff'] is not None]
    return dict(gaps=gaps, joint_eff_mean=(statistics.mean(je) if je else None),
                joint_eff_min=(min(je) if je else None), tcp_eff_mean=(statistics.mean(te) if te else None))


def coverage_map(run, wall, run_dir, engaged_n):
    """Paint the run with sim_overlay.coverage_paint. -> (grid, summary) or (None, note)."""
    sys.path.insert(0, os.path.join(REPO, 'src', 'common', 'sim_overlay'))
    try:
        from sim_overlay import coverage_paint as cp
    except Exception as e:  # noqa: BLE001
        return None, 'skipped: sim_overlay.coverage_paint not importable (%s)' % type(e).__name__
    finally:
        sys.path.pop(0)
    if wall is None or not len(run['t']):
        return None, 'skipped: no run.csv or run.csv.wall.json'
    try:
        grid = cp.CoverageGrid(cp.wall_from_dict(wall), force_threshold_n=engaged_n)
        cp.replay(grid, [(float(t), tuple(p), float(f), None) for t, p, f in zip(run['t'], run['xyz'], run['force'])])
        return grid, grid.summary()
    except Exception as e:  # noqa: BLE001
        return None, 'skipped: paint failed (%s)' % e


def _plan_geometry(doc):
    node = doc.get('final_lifted_and_expanded', doc)
    pts = []
    for p in node['path']['poses']:
        q = p['position'] if 'position' in p else p['pose']['position']
        pts.append((q['x'], q['y'], q['z']))
    return np.array(pts), node.get('segments') or []


def plan_redundancy(doc, run_dir, pitch=None, jump_factor=2.0):
    """The coverage_planner binary when on PATH, else a python mirror (jumps over 2x pitch, strips sharing cells)."""
    exe = shutil.which('plan_redundancy_report')
    path = os.path.join(run_dir, 'coverage_output.json')
    if exe and os.path.isfile(path):
        cmd = [exe, path] + (['--pitch', str(pitch)] if pitch else [])
        out = os.path.join(run_dir, '.plan_redundancy.json')
        try:
            subprocess.run(cmd + ['--json-out', out], capture_output=True, timeout=60)
            d = _read_json(out)
            if os.path.isfile(out):
                os.remove(out)
            if d:
                d['source'] = 'plan_redundancy_report'
                return d
        except (OSError, subprocess.SubprocessError):
            pass
    try:
        pts, segs = _plan_geometry(doc)
    except (KeyError, TypeError):
        return None
    strips = [s for s in segs if s.get('mode') == 'cartesian'
              and 0 <= s.get('start_index', 0) <= s.get('end_index', 0) < len(pts)]
    if not strips:
        return dict(source='python mirror', strips=0, jumps_over_2x_pitch=0, double_sweep_cells=0, cells=0)
    travel = []
    for a, b in zip(strips[:-1], strips[1:]):
        i, j = a['end_index'], b['start_index']
        if j >= i:
            travel.append(float(np.linalg.norm(np.diff(pts[i:j + 1], axis=0), axis=1).sum()))
        else:
            travel.append(float(np.linalg.norm(pts[i] - pts[j])))
    nz = [x for x in travel if x > 1e-6]
    p = pitch or (statistics.median(nz) if nz else 0.0)
    jumps = sum(1 for x in travel if p > 0 and x > jump_factor * p + 1e-9)
    allp = np.vstack([pts[s['start_index']:s['end_index'] + 1] for s in strips])
    ext = allp.max(0) - allp.min(0)
    ax = list(np.argsort(-ext)[:2])
    cell = p / 2 if p > 0 else 0.05
    hits = {}
    for k, s in enumerate(strips):
        sid = sum(1 for q in strips[:k + 1] if 'second_pass' not in (q.get('tags') or []))
        seg = pts[s['start_index']:s['end_index'] + 1]
        for a, b in zip(seg[:-1], seg[1:]) if len(seg) > 1 else [(seg[0], seg[0])]:
            n = max(1, int(math.ceil(np.linalg.norm(b - a) / (cell / 2))))
            for t in np.linspace(0, 1, n + 1):
                q = a + t * (b - a)
                hits.setdefault((math.floor(q[ax[0]] / cell), math.floor(q[ax[1]] / cell)), set()).add(sid)
    dbl = sum(1 for v in hits.values() if len(v) > 1)
    return dict(source='python mirror', strips=len(strips), pitch_m=p, jumps_over_2x_pitch=jumps,
                max_jump_m=max(travel) if travel else 0.0, cells=len(hits), double_sweep_cells=dbl,
                double_sweep_fraction=dbl / len(hits) if hits else 0.0)


def run_metrics(run_dir):
    rm = _load_by_path('sim_mujoco_run_metrics',
                       os.path.join(REPO, 'src', 'simulation', 'sim_mujoco', 'sim_mujoco', 'run_metrics.py'))
    log = os.path.join(run_dir, 'manager.log')
    if rm is None or not os.path.isfile(log):
        return None
    scene = os.path.join(run_dir, 'data', 'wall_pass_coverage_scene.json')
    plane = None
    if os.path.isfile(scene):
        try:
            plane = rm.wall_plane_from_scene(scene)
        except (KeyError, IndexError, ValueError):
            plane = None
    with open(log, errors='replace') as f:
        return rm.evaluate(f, plane)


def retract_spikes(run, engaged_n):
    """Stroke ends where the force at the first sample after the stroke (the retract start) is over the limit and
    over SPIKE_FACTOR times the stroke's median force."""
    spikes = []
    f = run['force']
    for a, b in engaged_runs(run, engaged_n):
        if b + 1 >= len(f):
            continue
        nxt = float(f[b + 1])
        med = float(np.median(f[a:b + 1]))
        peak_tail = max(nxt, float(f[b]))
        if peak_tail > PEAK_FLAG_N and peak_tail > SPIKE_FACTOR * max(med, 1e-9):
            spikes.append(dict(t=float(run['t'][b + 1]), force_n=peak_tail, stroke_median_n=med))
    return spikes


# ---- assemble ------------------------------------------------------------------------------------------------

def build_report(run_dir, target_n=10.0, band_n=1.5, engaged_n=1.0, pitch=None):
    notes = []
    mlines = _read_text(os.path.join(run_dir, 'manager.log'))
    clines = _read_text(os.path.join(run_dir, 'client.log'))
    if not mlines:
        notes.append('manager.log missing or empty: no segment, result or metric sections')
    run = read_run_csv(os.path.join(run_dir, 'run.csv'))
    if not len(run['t']):
        notes.append('run.csv missing or empty: force, time split and map sections skipped')
    wall = _read_json(os.path.join(run_dir, 'run.csv.wall.json'))
    card, verify = find_jsons(run_dir)
    cov_doc = _read_json(os.path.join(run_dir, 'coverage_output.json'))
    segments = parse_segments(mlines)
    rep = dict(run_dir=os.path.abspath(run_dir), notes=notes)
    rep['columns'] = parse_columns(mlines, clines, verify)
    rep['force'] = force_stats(run, card, target_n, band_n, engaged_n)
    rep['per_column'] = column_table(run, wall, engaged_n, target_n, band_n)
    rep['time'] = time_split(run, segments, engaged_n)
    rep['path_efficiency'] = path_efficiency(run, os.path.join(run_dir, 'run_film.npz'), engaged_n)
    if rep['path_efficiency'] is None:
        notes.append('run_film.npz missing: free-space path efficiency skipped')
    grid, cov = coverage_map(run, wall, run_dir, engaged_n)
    rep['coverage'] = cov if grid is not None else None
    if grid is None:
        notes.append('coverage map ' + cov)
    rep['redundancy'] = plan_redundancy(cov_doc, run_dir, pitch) if cov_doc else None
    if cov_doc is None:
        notes.append('coverage_output.json missing: plan redundancy skipped')
    metrics = run_metrics(run_dir)
    rep['metrics'] = metrics
    if metrics is None:
        notes.append('run metrics skipped (no manager.log or run_metrics not importable)')
    contacts = sorted(glob.glob(os.path.join(run_dir, '*.contacts.csv')))
    rep['contacts_files'] = [os.path.basename(c) for c in contacts]
    rep['segments_failed'] = [s['label'] for s in segments if s['outcome'].startswith('failed')]

    flags = []
    pk = rep['force']['peak_n']
    if pk is not None and pk > PEAK_FLAG_N:
        flags.append('peak force %.1f N over %.0f N' % (pk, PEAK_FLAG_N))
    if metrics:
        for i, v in sorted(metrics['reach'].items()):
            if v['max'] > REACH_FLAG:
                flags.append('stand %s: tool at %.0f%% of the reach (over %.0f%%)' % (i, 100 * v['max'], 100 * REACH_FLAG))
        for i, v in sorted(metrics['clearance'].items()):
            if v['chassis'] < CHASSIS_FLAG_M:
                flags.append('stand %s: chassis %.2f m from the wall (under %.2f m)' % (i, v['chassis'], CHASSIS_FLAG_M))
        if metrics.get('lift') and metrics['lift'].get('span_fraction', 1) < 0.25:
            flags.append('lift barely used (span %.0f%% of the stroke)' % (100 * metrics['lift']['span_fraction']))
    ts = rep['time']
    if ts and ts['free_space_share'] > FREE_SHARE_FLAG:
        flags.append('free-space motion is %.0f%% of the run (over %.0f%%)' % (100 * ts['free_space_share'], 100 * FREE_SHARE_FLAG))
    cols = rep['columns']
    if cols and cols['failed']:
        flags.append('failed columns: %s' % cols['failed'])
    if cols and cols['partial']:
        flags.append('partial columns: %s' % cols['partial'])
    for s in retract_spikes(run, engaged_n):
        flags.append('spike at the first retract sample: %.1f N at t=%.1f s (stroke median %.1f N)'
                     % (s['force_n'], s['t'], s['stroke_median_n']))
    red = rep['redundancy']
    if red and red.get('jumps_over_2x_pitch'):
        flags.append('plan has %d jump(s) over 2x pitch' % red['jumps_over_2x_pitch'])
    rep['flags'] = flags
    rep['verdict'] = verdict(rep)
    rep['_grid'] = grid
    rep['_run'] = run
    return rep


def verdict(rep):
    c = rep['columns']
    if c:
        n = len(c['done']) + len(c['partial']) + len(c['failed'])
        head = '%d/%d columns done' % (len(c['done']), n)
    else:
        head = 'columns unknown'
    f = rep['force']
    bits = [head]
    if f['peak_n'] is not None:
        bits.append('peak %.1f N' % f['peak_n'])
    if f['in_band_fraction'] is not None:
        bits.append('%.0f%% in band' % (100 * f['in_band_fraction']))
    if rep['coverage']:
        bits.append('coverage %.1f%%' % rep['coverage']['percent_done'])
    n = len(rep['flags'])
    return ('FLAGGED' if n else 'CLEAN') + ': ' + ', '.join(bits) + ('; %d flag(s)' % n if n else '; no flags')


# ---- output --------------------------------------------------------------------------------------------------

def _fmt(v, spec, none='n/a'):
    return none if v is None else format(v, spec)


def to_markdown(rep):
    L = ['# Run report', '', '`%s`' % rep['run_dir'], '', '**Verdict:** ' + rep['verdict'], '']
    c = rep['columns']
    L += ['## Result', '']
    if c:
        L += ['columns done %s, partial %s, failed %s (from %s)' % (c['done'], c['partial'], c['failed'], c['source'])]
    else:
        L += ['no columns line in manager.log, client.log or a verify json']
    f = rep['force']
    L += ['', '## Force (%s)' % f['source'], '',
          '| peak N | rms error N | rms force N | in band | engaged samples |', '|---|---|---|---|---|',
          '| %s | %s | %s | %s | %d |' % (_fmt(f['peak_n'], '.1f'), _fmt(f['rms_error_n'], '.2f'),
                                        _fmt(f.get('rms_force_n'), '.2f'),
                                        _fmt(None if f['in_band_fraction'] is None else 100 * f['in_band_fraction'], '.0f') + ' %',
                                        f['engaged_samples'])]
    if rep['per_column']:
        L += ['', '## Per-column peak force', '', '| column (by u) | u m | samples | peak N | mean N | in band |', '|---|---|---|---|---|---|']
        L += ['| %d | %.3f | %d | %.1f | %.1f | %.0f %% |' % (r['index'], r['u_m'], r['samples'], r['peak_n'], r['mean_n'],
                                                            100 * r['in_band']) for r in rep['per_column']]
    t = rep['time']
    if t:
        L += ['', '## Time split', '', '| contact | free-space | other motion | idle | total |', '|---|---|---|---|---|']
        d = max(t['duration_s'], 1e-9)
        cell = lambda v: '%.0f s (%.0f %%)' % (v, 100 * v / d)  # noqa: E731
        L += ['| %s | %s | %s | %s | %.0f s |' % (cell(t['contact_s']), cell(t['free_space_s']), cell(t['other_motion_s']),
                                                  cell(t['idle_s']), t['duration_s']),
              '', 'free-space = traj_s of the %d non-cartesian segment_exec lines (of %d); other motion = cartesian traj_s '
              'minus contact; idle = the rest.' % (t['free_space_segments'], t['segments'])]
    pe = rep['path_efficiency']
    if pe:
        L += ['', '## Free-space path efficiency', '']
        if pe['gaps']:
            L += ['straight-line distance over path length per gap between strokes (1 = straight): joint mean %s, worst %s; TCP mean %s over %d gap(s)'
                  % (_fmt(pe['joint_eff_mean'], '.2f'), _fmt(pe['joint_eff_min'], '.2f'), _fmt(pe['tcp_eff_mean'], '.2f'),
                     len(pe['gaps']))]
        else:
            L += [pe.get('note', 'no gaps')]
    L += ['', '## Coverage', '']
    cv = rep['coverage']
    L += ['%.1f %% of the wall swept (%.2f of %.2f m2); map in report.png' % (cv['percent_done'], cv['done_m2'], cv['wall_m2'])
          ] if cv else ['skipped (see notes)']
    L += ['', '## Plan redundancy', '']
    r = rep['redundancy']
    if r:
        L += ['%s: %s strips, %s jump(s) over 2x pitch, %s of %s cells swept by more than one strip'
              % (r.get('source'), r.get('strips'), r.get('jumps_over_2x_pitch'),
                 r.get('cells_swept_more_than_expected', r.get('double_sweep_cells')), r.get('cells'))]
    else:
        L += ['skipped']
    m = rep['metrics']
    L += ['', '## Run metrics', '']
    if m:
        lf = m['lift']
        if lf:
            L += ['lift %.3f to %.3f m (span %.3f m, %.0f %% of the stroke)' % (lf['min'], lf['max'], lf['span'], 100 * lf['span_fraction']), '']
        L += ['| stand | chassis-wall m | tool reach max |', '|---|---|---|']
        for i in sorted(m['stands']):
            cl, rc = m['clearance'].get(i), m['reach'].get(i)
            L += ['| %s | %s | %s |' % (i, _fmt(cl and cl['chassis'], '.3f'), _fmt(rc and 100 * rc['max'], '.0f') + ' %')]
    else:
        L += ['skipped']
    L += ['', '## Flags', ''] + (['- ' + x for x in rep['flags']] or ['none'])
    if rep['notes']:
        L += ['', '## Notes', ''] + ['- ' + x for x in rep['notes']]
    return '\n'.join(L) + '\n'


def render_png(rep, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    run = rep['_run']
    a = ax[0, 0]
    if len(run['t']):
        a.plot(run['t'], run['force'], lw=0.6)
        a.axhline(PEAK_FLAG_N, color='r', ls='--', lw=0.8)
    a.set(title='force', xlabel='t [s]', ylabel='N')
    a = ax[0, 1]
    pc = rep['per_column']
    if pc:
        a.bar([r['index'] for r in pc], [r['peak_n'] for r in pc])
        a.axhline(PEAK_FLAG_N, color='r', ls='--', lw=0.8)
    a.set(title='peak force per column (by u)', xlabel='column', ylabel='N')
    a = ax[1, 0]
    t = rep['time']
    if t:
        vals = [t['contact_s'], t['free_space_s'], t['other_motion_s'], t['idle_s']]
        a.barh(['contact', 'free-space', 'other motion', 'idle'], vals)
    a.set(title='time split [s]')
    a = ax[1, 1]
    g = rep['_grid']
    if g is not None:
        (u0, u1), (v0, v1) = g.wall.u_range, g.wall.v_range
        a.imshow(g.status, origin='lower', extent=(u0, u0 + g.nu * g.cell, v0, v0 + g.nv * g.cell), vmin=0, vmax=2,
                 cmap='RdYlGn', interpolation='nearest')
        a.set(title='coverage %.1f %% (green = done)' % rep['coverage']['percent_done'], xlabel='u [m]', ylabel='v [m]')
    else:
        a.text(0.5, 0.5, 'coverage map skipped', ha='center')
        a.set_axis_off()
    fig.suptitle(rep['verdict'], fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def write_outputs(rep, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'report.md'), 'w') as f:
        f.write(to_markdown(rep))
    clean = {k: v for k, v in rep.items() if not k.startswith('_')}
    with open(os.path.join(out_dir, 'report.json'), 'w') as f:
        json.dump(clean, f, indent=2, default=lambda o: o.item() if hasattr(o, 'item') else str(o))
    try:
        render_png(rep, os.path.join(out_dir, 'report.png'))
    except Exception as e:  # noqa: BLE001
        rep['notes'].append('report.png not written: %s' % e)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('run_dir')
    ap.add_argument('--out', help='output directory (default: RUN_DIR)')
    ap.add_argument('--target-force', type=float, default=10.0)
    ap.add_argument('--band', type=float, default=1.5)
    ap.add_argument('--engaged-force', type=float, default=1.0)
    ap.add_argument('--pitch', type=float, default=None)
    a = ap.parse_args(argv)
    if not os.path.isdir(a.run_dir):
        print('run_report: no such directory %s' % a.run_dir, file=sys.stderr)
        return 2
    rep = build_report(a.run_dir, a.target_force, a.band, a.engaged_force, a.pitch)
    write_outputs(rep, a.out or a.run_dir)
    print('VERDICT: ' + rep['verdict'])
    for fl in rep['flags']:
        print('FLAG: ' + fl)
    print('wrote report.md, report.png, report.json in %s' % (a.out or a.run_dir))
    return 0


if __name__ == '__main__':
    sys.exit(main())
