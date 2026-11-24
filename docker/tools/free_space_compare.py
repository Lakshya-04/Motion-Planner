#!/usr/bin/env python3
"""Free-space transition comparison: transition lists in, table and chart out (host-testable, numpy only).

The planning itself runs elsewhere (docs/free_space_comparison.md): `free_space_compare_node` (MoveIt modes, in the
sim image) and `free_space_compare_curobo.py` (cuRobo, on the GPU lane) write JSON lines; this tool

  from-log LOG [--out F]     turn the ARMSTATE lines of a run's manager.log (and its segment_exec lines) into a
                             transitions file (start joint state = one ARMSTATE, goal joint state = the next one that
                             follows a free-space move), classified stow / stand_change / column_change
  template [--out F]         a hand list to edit: retract -> next-column approach, stow <-> dock, stand change
  analyze RES... --out-dir D read result files, write results.csv, results_table.markdown and results.png

Metrics per (transition, mode): success, planning time, duration (after TOTG), joint path length weighted by
1/range of every joint (the lift's metres and the arm's radians become fractions of travel), TCP path length, and the
baselines: the straight joint line (joint_ratio = joint path / straight) and the Euclidean chord between the start
and goal tool positions (tcp_ratio = TCP path / chord).
"""
import argparse
import csv
import json
import math
import os
import re
import statistics
import sys

ARMSTATE = re.compile(r'ARMSTATE (\S+) stand (\d+): q=\[([^\]]*)\] tcp_in_\S+=\(([^)]*)\)')
SEGMENT = re.compile(r'segment_exec (.*)')
KV = re.compile(r'(\w+)=(\S+)')


# ---------------------------------------------------------------- log parsing

def parse_manager_log(lines):
    """Returns (armstates, segments). armstates: dicts tag, stand, q, tcp; segments: dicts of the segment_exec key=value."""
    arm, segs = [], []
    for line in lines:
        m = ARMSTATE.search(line)
        if m:
            try:
                q = [float(v) for v in m.group(3).split(',') if v.strip()]
                tcp = [float(v) for v in m.group(4).split(',')]
            except ValueError:
                continue
            if q:
                arm.append({'tag': m.group(1), 'stand': int(m.group(2)), 'q': q, 'tcp': tcp})
            continue
        m = SEGMENT.search(line)
        if m:
            segs.append(dict(KV.findall(m.group(1))))
    return arm, segs


def classify(a, b):
    """Transition class of two consecutive ARMSTATE entries."""
    ta, tb = a['tag'], b['tag']
    if 'stow' in ta or 'stow' in tb:
        return 'stow'
    if a['stand'] != b['stand']:
        return 'stand_change'
    return 'column_change'


def _free_space_pair(a, b):
    """A free-space move lies between a finished dock (or the arrival / stow pose) and the next dock approach."""
    starts = a['tag'].startswith(('docked_', 'stow', 'arrival'))
    ends = b['tag'].startswith(('before_dock_', 'stow', 'docked_'))
    return starts and ends and a['q'] != b['q']


def transitions_from_log(lines):
    arm, segs = parse_manager_log(lines)
    out = []
    for a, b in zip(arm, arm[1:]):
        if len(a['q']) != len(b['q']) or not _free_space_pair(a, b):
            continue
        out.append({'name': f"{a['tag']}->{b['tag']}", 'class': classify(a, b), 'start': a['q'], 'goal_q': b['q'],
                    'source': 'manager.log ARMSTATE (may span several segments: retract, approach)'})
    logged = [{'label': s.get('label', ''), 'mode': s.get('mode', ''), 'planner': s.get('planner', ''),
               'mode_switch': s.get('mode_switch', ''), 'traj_pts': s.get('traj_pts', ''),
               'traj_s': s.get('traj_s', ''), 'reason': s.get('reason', '')}
              for s in segs if s.get('mode') == 'fsp' or s.get('mode_switch') == 'true']
    return {'transitions': out, 'logged_segments': logged}


def template():
    """Placeholders to edit (joint order of the planning group, lift first for the telescopic profiles)."""
    z = [0.0] * 7
    return {'_comment': 'EDIT the joint states; goal_q or goal_pose (x y z qx qy qz qw in the planning frame) per entry.',
            'transitions': [
                {'name': 'retract_to_next_column_approach', 'class': 'column_change', 'start': z, 'goal_q': z},
                {'name': 'dock_to_stow', 'class': 'stow', 'start': z, 'goal_q': z},
                {'name': 'stow_to_dock', 'class': 'stow', 'start': z, 'goal_q': z},
                {'name': 'stand_change_retract_to_next_stand_approach', 'class': 'stand_change', 'start': z,
                 'goal_q': z}]}


# ---------------------------------------------------------------- metrics

def weights_from_ranges(ranges):
    return [1.0 / r if (r is not None and math.isfinite(r) and r > 1e-9) else 1.0 for r in ranges]


def weighted_distance(a, b, w):
    return math.sqrt(sum(((x - y) * (w[i] if i < len(w) else 1.0)) ** 2 for i, (x, y) in enumerate(zip(a, b))))


def weighted_length(path, w):
    return sum(weighted_distance(p, q, w) for p, q in zip(path, path[1:]))


def chord(p, q):
    return math.dist(p, q)


def ratio(value, baseline):
    """value / baseline; None when the baseline is (near) zero, so a no-move transition does not divide by zero."""
    if value is None or baseline is None or baseline < 1e-9:
        return None
    return value / baseline


def load_results(paths):
    """Header (first one wins), transitions by name, runs list. Records of the same (transition, mode, rep) later in
    the list replace earlier ones (a cuRobo file with FK added replaces the same file without)."""
    header, transitions, runs = {}, {}, {}
    for p in paths:
        with open(p) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                t = r.get('type')
                if t == 'header' and not header:
                    header = r
                elif t == 'transition':
                    transitions.setdefault(r['transition'], r)
                elif t == 'run':
                    runs[(r['transition'], r['mode'], r.get('rep', 0))] = r
    return header, transitions, list(runs.values())


def build_rows(header, transitions, runs):
    w = weights_from_ranges(header.get('ranges', []))
    rows = []
    for r in runs:
        t = transitions.get(r['transition'])
        if t is None:
            continue
        ch = chord(t['tcp_start'], t['tcp_goal']) if t.get('tcp_start') and t.get('tcp_goal') else None
        straight = weighted_distance(t['start'], t['goal_q'], w)
        row = {'transition': r['transition'], 'class': t.get('class', ''), 'mode': r['mode'],
               'rep': r.get('rep', 0), 'success': bool(r.get('success')), 'route': r.get('route', ''),
               'planning_s': r.get('planning_s'), 'duration_s': r.get('duration_s'), 'points': r.get('points'),
               'checked': r.get('checked', True), 'reason': r.get('reason', ''), 'chord_m': ch,
               'straight_joint': straight, 'joint_len': None, 'joint_ratio': None, 'tcp_len': r.get('tcp_length'),
               'tcp_ratio': None}
        if r.get('success') and r.get('positions'):
            row['joint_len'] = weighted_length(r['positions'], w)
            row['joint_ratio'] = ratio(row['joint_len'], straight)
            row['tcp_ratio'] = ratio(row['tcp_len'], ch)
        rows.append(row)
    return rows


def best_modes(rows):
    """Per transition: the mode with the smallest joint length, TCP length and duration among CHECKED successes."""
    out = {}
    for name in sorted({r['transition'] for r in rows}):
        ok = [r for r in rows if r['transition'] == name and r['success'] and r['checked']]
        pick = {}
        for key in ('joint_len', 'tcp_len', 'duration_s'):
            cand = [r for r in ok if r.get(key) is not None]
            pick[key] = min(cand, key=lambda r: r[key])['mode'] if cand else ''
        out[name] = pick
    return out


def class_summary(rows):
    """Median ratios per (class, mode) over the successful checked runs."""
    groups = {}
    for r in rows:
        groups.setdefault((r['class'], r['mode']), []).append(r)
    out = []
    for (cls, mode), rs in sorted(groups.items()):
        ok = [r for r in rs if r['success'] and r['checked']]

        def med(key):
            v = [r[key] for r in ok if r.get(key) is not None]
            return statistics.median(v) if v else None
        out.append({'class': cls, 'mode': mode, 'runs': len(rs), 'solved': len(ok), 'joint_ratio': med('joint_ratio'),
                    'tcp_ratio': med('tcp_ratio'), 'duration_s': med('duration_s'), 'planning_s': med('planning_s')})
    return out


# ---------------------------------------------------------------- output

CSV_FIELDS = ['transition', 'class', 'mode', 'rep', 'success', 'checked', 'route', 'reason', 'planning_s', 'duration_s',
              'points', 'joint_len', 'straight_joint', 'joint_ratio', 'tcp_len', 'chord_m', 'tcp_ratio']


def write_csv(rows, path):
    with open(path, 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=CSV_FIELDS, extrasaction='ignore')
        wr.writeheader()
        for r in rows:
            wr.writerow(r)


def _f(v, fmt='{:.2f}'):
    return '-' if v is None else fmt.format(v)


def markdown(rows, logged=None):
    lines = ['# Free-space comparison', '',
             'joint_ratio = weighted joint path / straight joint line; tcp_ratio = TCP path / Euclidean chord; '
             'duration after TOTG. `straight_joint` is an unchecked baseline. 1.00 = as short as the baseline.', '']
    for name in sorted({r['transition'] for r in rows}):
        rs = [r for r in rows if r['transition'] == name]
        ch = rs[0]['chord_m']
        lines += [f"## {name} ({rs[0]['class']}, chord {_f(ch, '{:.3f}')} m)", '',
                  '| mode | ok | route | plan s | dur s | pts | joint len | joint ratio | TCP m | TCP ratio |',
                  '|---|---|---|---|---|---|---|---|---|---|']
        for r in rs:
            lines.append(
                f"| {r['mode']} | {'yes' if r['success'] else 'NO ' + str(r['reason'])} | {r['route']} | "
                f"{_f(r['planning_s'], '{:.1f}')} | {_f(r['duration_s'], '{:.1f}')} | {r['points'] or '-'} | "
                f"{_f(r['joint_len'], '{:.3f}')} | {_f(r['joint_ratio'])} | {_f(r['tcp_len'], '{:.3f}')} | "
                f"{_f(r['tcp_ratio'])} |")
        lines.append('')
    lines += ['## Best mode per transition (checked successes)', '', '| transition | joint length | TCP length | duration |',
              '|---|---|---|---|']
    for name, p in best_modes(rows).items():
        lines.append(f"| {name} | {p['joint_len']} | {p['tcp_len']} | {p['duration_s']} |")
    lines += ['', '## Median per class and mode', '',
              '| class | mode | solved | joint ratio | TCP ratio | duration s | planning s |', '|---|---|---|---|---|---|---|']
    for s in class_summary(rows):
        lines.append(f"| {s['class']} | {s['mode']} | {s['solved']}/{s['runs']} | {_f(s['joint_ratio'])} | "
                     f"{_f(s['tcp_ratio'])} | {_f(s['duration_s'], '{:.1f}')} | {_f(s['planning_s'], '{:.1f}')} |")
    if logged:
        lines += ['', '## Free-space segments of the logged run', '', '| label | mode | planner | mode_switch | pts | s | reason |',
                  '|---|---|---|---|---|---|---|']
        for s in logged:
            lines.append(f"| {s['label']} | {s['mode']} | {s['planner']} | {s['mode_switch']} | {s['traj_pts']} | "
                         f"{s['traj_s']} | {s['reason']} |")
    return '\n'.join(lines) + '\n'


def write_chart(rows, path):
    """Three panels per transition: TCP path / chord, joint path / straight joint, duration (grouped bars by mode)."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('free_space_compare: matplotlib missing, no PNG', file=sys.stderr)
        return False
    names = sorted({r['transition'] for r in rows})
    modes = sorted({r['mode'] for r in rows if r['mode'] != 'straight_joint'})
    if not names or not modes:
        return False
    fig, axes = plt.subplots(3, 1, figsize=(max(8, 1.6 * len(names) + 4), 10), sharex=True)
    width = 0.8 / len(modes)
    for ax, key, title in zip(axes, ('tcp_ratio', 'joint_ratio', 'duration_s'),
                              ('TCP path / chord (1 = straight line)', 'joint path / straight joint line',
                               'duration after TOTG (s)')):
        for k, mode in enumerate(modes):
            vals = []
            for n in names:
                match = [r for r in rows if r['transition'] == n and r['mode'] == mode and r['success']]
                vals.append(match[0][key] if match and match[0].get(key) is not None else 0.0)
            ax.bar([i + k * width for i in range(len(names))], vals, width, label=mode)
        ax.set_title(title, loc='left', fontsize=10)
        ax.grid(axis='y', alpha=0.3)
    axes[0].legend(ncol=3, fontsize=8)
    axes[-1].set_xticks([i + 0.4 - width / 2 for i in range(len(names))])
    axes[-1].set_xticklabels(names, rotation=20, ha='right', fontsize=8)
    fig.text(0.01, 0.005, 'a missing bar = the mode failed (or has no value)', fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    return True


# ---------------------------------------------------------------- cli

def cmd_analyze(a):
    header, transitions, runs = load_results(a.results)
    rows = build_rows(header, transitions, runs)
    if not rows:
        print('free_space_compare: no runs in the result files', file=sys.stderr)
        return 1
    os.makedirs(a.out_dir, exist_ok=True)
    logged = None
    if a.transitions:
        with open(a.transitions) as f:
            logged = json.load(f).get('logged_segments')
    write_csv(rows, os.path.join(a.out_dir, 'results.csv'))
    with open(os.path.join(a.out_dir, 'results_table.markdown'), 'w') as f:
        f.write(markdown(rows, logged))
    png = write_chart(rows, os.path.join(a.out_dir, 'results.png'))
    print(f"{len(rows)} runs -> {a.out_dir}/results.csv results_table.markdown" + (' results.png' if png else ''))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('from-log')
    p.add_argument('log')
    p.add_argument('--out')
    p = sub.add_parser('template')
    p.add_argument('--out')
    p = sub.add_parser('analyze')
    p.add_argument('results', nargs='+')
    p.add_argument('--out-dir', required=True)
    p.add_argument('--transitions', help='transitions file from from-log (adds the logged segments table)')
    a = ap.parse_args(argv)
    if a.cmd == 'analyze':
        return cmd_analyze(a)
    if a.cmd == 'from-log':
        with open(a.log, errors='replace') as f:
            doc = transitions_from_log(f)
        if not doc['transitions']:
            print('free_space_compare: no free-space transition found in the ARMSTATE lines', file=sys.stderr)
            return 1
    else:
        doc = template()
    text = json.dumps(doc, indent=1)
    if a.out:
        with open(a.out, 'w') as f:
            f.write(text + '\n')
    else:
        print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
