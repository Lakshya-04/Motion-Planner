"""Host tests for docker/tools/free_space_compare.py (analyser, log parser) and the cuRobo stage's helpers."""
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import free_space_compare as fsc  # noqa: E402
import free_space_compare_curobo as fcc  # noqa: E402

LOG = """\
[m]: ARMSTATE arrival stand 0: q=[0.10, 0.0, -1.5, 1.5, -1.5, -1.5, 0.0] tcp_in_odom=(0.1, 0.0, 1.3)
[m]: ARMSTATE before_dock_wall_column_0 stand 0: q=[0.20, 1.0, -1.5, 1.5, -1.5, -1.5, 0.0] tcp_in_odom=(0.8, 0.3, 1.1)
[m]: ARMSTATE docked_wall_column_0 stand 0: q=[0.20, 1.1, -1.5, 1.5, -1.5, -1.5, 0.0] tcp_in_odom=(0.9, 0.3, 1.1)
[m]: ARMSTATE before_dock_wall_column_1 stand 0: q=[0.30, 1.1, -1.5, 1.5, -1.5, -1.5, 0.0] tcp_in_odom=(0.8, 0.4, 1.1)
[m]: ARMSTATE docked_wall_column_1 stand 0: q=[0.30, 1.2, -1.5, 1.5, -1.5, -1.5, 0.0] tcp_in_odom=(0.9, 0.4, 1.1)
[m]: ARMSTATE stowed stand 0: q=[0.0, 0.0, -1.5, 1.5, -1.5, -1.5, 0.0] tcp_in_odom=(0.1, 0.0, 1.3)
[m]: ARMSTATE before_dock_wall_column_0 stand 1: q=[0.50, 1.0, -1.5, 1.5, -1.5, -1.5, 0.0] tcp_in_odom=(0.8, 1.5, 1.1)
[m]: ARMSTATE broken stand 1: q=[a, b] tcp_in_odom=(1, 2, 3)
[m]: segment_exec id=1 label=wall_column_1_approach outcome=completed mode=fsp planner=ompl planning_ms=5 mode_switch=true traj_pts=70 traj_s=7.5 reason=none
[m]: segment_exec id=2 label=wall_column_1_contact outcome=completed mode=cartesian planner=ompl traj_pts=9 traj_s=20
"""


def test_parse_log_skips_broken_lines():
    arm, segs = fsc.parse_manager_log(LOG.splitlines())
    assert len(arm) == 7 and arm[1]['stand'] == 0 and arm[1]['q'][0] == 0.20
    assert len(segs) == 2 and segs[0]['label'] == 'wall_column_1_approach'


def test_transitions_from_log_classes():
    doc = fsc.transitions_from_log(LOG.splitlines())
    classes = {t['name']: t['class'] for t in doc['transitions']}
    assert classes['docked_wall_column_0->before_dock_wall_column_1'] == 'column_change'
    assert classes['docked_wall_column_1->stowed'] == 'stow'
    assert classes['arrival->before_dock_wall_column_0'] == 'column_change'
    # stowed (stand 0) -> before_dock (stand 1): stow class wins, a stand change is also visible in the tags
    assert 'stowed->before_dock_wall_column_0' in classes
    assert len(doc['logged_segments']) == 1 and doc['logged_segments'][0]['mode_switch'] == 'true'


def test_template_is_valid_json():
    t = fsc.template()
    json.dumps(t)
    assert {x['class'] for x in t['transitions']} == {'column_change', 'stow', 'stand_change'}


def test_weighted_length_and_ratio():
    w = fsc.weights_from_ranges([2.0, 4.0, 0.0])
    assert w == [0.5, 0.25, 1.0]
    assert math.isclose(fsc.weighted_length([[0, 0, 0], [1, 0, 0], [1, 2, 0]], w), 1.0)
    assert fsc.ratio(2.0, 0.0) is None and fsc.ratio(None, 1.0) is None and fsc.ratio(3.0, 1.5) == 2.0


def _write(path, recs):
    with open(path, 'w') as f:
        for r in recs:
            f.write(json.dumps(r) + '\n')


def _results(tmp_path):
    header = {'type': 'header', 'ranges': [2.0, 4.0]}
    tr = {'type': 'transition', 'transition': 'a', 'class': 'column_change', 'start': [0, 0], 'goal_q': [1, 2],
          'tcp_start': [0, 0, 0], 'tcp_goal': [0.1, 0, 0]}
    runs = [
        {'type': 'run', 'transition': 'a', 'mode': 'straight_joint', 'success': True, 'checked': False,
         'positions': [[0, 0], [1, 2]], 'duration_s': 2.0, 'tcp_length': 0.5, 'planning_s': 0.0},
        {'type': 'run', 'transition': 'a', 'mode': 'ompl_default', 'success': True, 'route': 'ompl',
         'positions': [[0, 0], [1, 0], [1, 2]], 'duration_s': 7.0, 'tcp_length': 1.0, 'planning_s': 0.3},
        {'type': 'run', 'transition': 'a', 'mode': 'cartesian', 'success': True, 'route': 'cartesian',
         'positions': [[0, 0], [0.5, 1.2], [1, 2]], 'duration_s': 3.0, 'tcp_length': 0.1, 'planning_s': 0.1},
        {'type': 'run', 'transition': 'a', 'mode': 'cartesian_joint', 'success': False, 'reason': 'joint_goal_branch_mismatch'},
    ]
    m = tmp_path / 'm.jsonl'
    _write(m, [header, tr] + runs)
    return m


def test_build_rows_and_best(tmp_path):
    header, tr, runs = fsc.load_results([str(_results(tmp_path))])
    rows = fsc.build_rows(header, tr, runs)
    by = {r['mode']: r for r in rows}
    assert math.isclose(by['ompl_default']['tcp_ratio'], 10.0)       # 1.0 m of TCP path for a 0.1 m chord
    assert math.isclose(by['cartesian']['tcp_ratio'], 1.0)
    assert by['ompl_default']['joint_ratio'] > 1.0
    assert by['cartesian_joint']['joint_len'] is None and not by['cartesian_joint']['success']
    best = fsc.best_modes(rows)['a']
    # the unchecked straight baseline never counts as a winner
    assert best['tcp_len'] == 'cartesian' and best['duration_s'] == 'cartesian'
    assert best['joint_len'] == 'cartesian'


def test_later_record_replaces_earlier(tmp_path):
    m = _results(tmp_path)
    extra = tmp_path / 'fk.jsonl'
    _write(extra, [{'type': 'run', 'transition': 'a', 'mode': 'ompl_default', 'rep': 0, 'success': True,
                    'positions': [[0, 0], [1, 2]], 'duration_s': 1.0, 'tcp_length': 0.2}])
    header, tr, runs = fsc.load_results([str(m), str(extra)])
    rows = [r for r in fsc.build_rows(header, tr, runs) if r['mode'] == 'ompl_default']
    assert len(rows) == 1 and rows[0]['duration_s'] == 1.0


def test_zero_chord_gives_no_ratio(tmp_path):
    header = {'type': 'header', 'ranges': [1.0]}
    tr = {'type': 'transition', 'transition': 'z', 'start': [0], 'goal_q': [0], 'tcp_start': [0, 0, 0], 'tcp_goal': [0, 0, 0]}
    run = {'type': 'run', 'transition': 'z', 'mode': 'direct', 'success': True, 'positions': [[0], [0]], 'tcp_length': 0.0}
    p = tmp_path / 'z.jsonl'
    _write(p, [header, tr, run])
    h, t, r = fsc.load_results([str(p)])
    row = fsc.build_rows(h, t, r)[0]
    assert row['tcp_ratio'] is None and row['joint_ratio'] is None


def test_cli_analyze_writes_files(tmp_path):
    m = _results(tmp_path)
    out = tmp_path / 'out'
    assert fsc.main(['analyze', str(m), '--out-dir', str(out)]) == 0
    assert (out / 'results.csv').read_text().startswith('transition,class,mode')
    md = (out / 'results_table.markdown').read_text()
    assert '| cartesian |' in md and 'joint_goal_branch_mismatch' in md and 'Best mode per transition' in md


def test_cli_from_log_and_template(tmp_path):
    log = tmp_path / 'manager.log'
    log.write_text(LOG)
    out = tmp_path / 't.json'
    assert fsc.main(['from-log', str(log), '--out', str(out)]) == 0
    assert json.loads(out.read_text())['transitions']
    empty = tmp_path / 'e.log'
    empty.write_text('nothing here\n')
    assert fsc.main(['from-log', str(empty)]) == 1
    assert fsc.main(['template', '--out', str(tmp_path / 'h.json')]) == 0


def test_curobo_reorder():
    assert fcc.reorder([1, 2, 3], ['lift', 'a', 'b'], ['a', 'b', 'lift']) == [2, 3, 1]
    assert fcc.reorder([1], ['x'], ['y']) is None


def test_curobo_plan_all_with_fake_planner(tmp_path):
    class R:
        success = True
        route = 'trajopt'
        failure_reason_code = 'none'
        duration = 2.0
        positions = [[0, 0], [1, 1]]

    class P:
        joint_names = ['b', 'a']

        def plan_joint(self, s, g):
            assert s == [0.2, 0.1] and g == [2.0, 1.0]   # reordered into the cuRobo order
            return R()

    header = {'joint_names': ['a', 'b']}
    tr = [{'transition': 't', 'start': [0.1, 0.2], 'goal_q': [1.0, 2.0]}]
    import io
    buf = io.StringIO()
    fcc.plan_all(P(), 'curobo_default', header, tr, buf)
    rec = json.loads(buf.getvalue())
    assert rec['success'] and rec['mode'] == 'curobo_default' and rec['duration_s'] == 2.0
