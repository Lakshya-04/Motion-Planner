import json
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))

from sim_overlay import coverage_paint as cp  # noqa: E402
from sim_overlay import touch_up as tu  # noqa: E402

WALL = {'origin': [1.0, 0, 0], 'u': [0, 1, 0], 'v': [0, 0, 1], 'u_range': [0, 1.0], 'v_range': [0, 1.0]}


def make_grid(swath=0.14):
    g = cp.CoverageGrid(cp.wall_from_dict(WALL), 0.02, swath)
    g.status[:] = cp.DONE
    return g


def test_missing_strip_gives_exactly_that_strip():
    g = make_grid()
    g.status[:, 14:21] = cp.NOT_DONE                       # u 0.28 .. 0.42 = one swath, full height
    plan, rep = tu.touch_up_grid(g)
    cols = tu.check_manager_schema(plan)
    assert len(cols) == 1 and rep['planned_strips'] == 1 and rep['ignored_m2'] == 0
    (key, z0, z1), = cols
    assert key == pytest.approx(-350, abs=1)   # manager key: -y for the default wall_direction and z0 == pytest.approx(0.0) and z1 == pytest.approx(1.0)
    poses = plan['final_lifted_and_expanded']['path']['poses']
    assert {round(p['position']['y'], 3) for p in poses} == {0.35} and {p['position']['x'] for p in poses} == {1.0}
    assert len(plan['final_lifted_and_expanded']['segments']) == 1


def test_serpentine_and_overlap_pitch():
    g = make_grid()
    g.status[:, 10:40] = cp.NOT_DONE                       # 0.6 m wide: touching strips at pitch 0.14 * 0.8
    plan, rep = tu.touch_up_grid(g, overlap_pct=20.0)
    segs = plan['final_lifted_and_expanded']['segments']
    poses = plan['final_lifted_and_expanded']['path']['poses']
    ys = [poses[s['start_index']]['position']['y'] for s in segs]
    assert len(segs) >= 4 and np.allclose(np.diff(ys), 0.112, atol=1e-6)
    ups = [poses[s['end_index']]['position']['z'] > poses[s['start_index']]['position']['z'] for s in segs]
    assert ups == [i % 2 == 0 for i in range(len(segs))]
    # every undone cell lies within half a swath of some stroke
    assert rep['ignored_m2'] == 0


def test_fully_covered_is_empty_with_message():
    plan, rep = tu.touch_up_grid(make_grid())
    assert plan is None and 'fully covered' in rep['message'] and rep['planned_strips'] == 0


def test_speckle_below_minimum_is_ignored_and_reported():
    g = make_grid()
    g.status[10:12, 5:7] = cp.NOT_DONE                      # 4 cm blobs
    g.status[40:42, 30:32] = cp.NOT_DONE
    plan, rep = tu.touch_up_grid(g, min_length_m=0.10)
    assert plan is None and 'ignored area' in rep['message']
    assert rep['ignored_m2'] == pytest.approx(8 * 0.0004) and rep['undone_m2'] == pytest.approx(8 * 0.0004)


def test_strip_plus_speckle_keeps_strip_reports_speckle():
    g = make_grid()
    g.status[:, 14:21] = cp.NOT_DONE
    g.status[10:12, 40:42] = cp.NOT_DONE
    plan, rep = tu.touch_up_grid(g)
    assert rep['planned_strips'] == 1 and rep['ignored_m2'] == pytest.approx(4 * 0.0004)


def test_obstacle_splits_strip_in_two_strokes_same_direction():
    g = make_grid()
    g.status[:, 14:21] = cp.NOT_DONE
    g.status[20:30, 14:21] = cp.DONE                        # 0.2 m done stretch in the middle
    plan, rep = tu.touch_up_grid(g)
    segs = plan['final_lifted_and_expanded']['segments']
    assert len(segs) == 2 and rep['planned_strips'] == 1
    assert len(tu.check_manager_schema(plan)) == 1


def test_run_csv_end_to_end_and_cli(tmp_path):
    import touch_up_plan
    csv = tmp_path / 'run.csv'
    rows = ['t,x,y,z,force']
    t = 0.0
    for u in (0.07, 0.21, 0.49, 0.63, 0.77, 0.91):          # strokes of swath 0.14; the strip at u 0.35 is missing
        for k in range(21):
            rows.append('%.2f,%.3f,%.3f,%.3f,5.0' % (t, 1.0, u, 0.05 * k))
            t += 0.1
        t += 5
    csv.write_text('\n'.join(rows) + '\n')
    (tmp_path / 'run.csv.wall.json').write_text(json.dumps(dict(WALL, u_range=[0, 0.98])))
    out = tmp_path / 'tu.json'
    rep = touch_up_plan.main([str(csv), '--out', str(out), '--swath', '0.14', '--min-length', '0.1'])
    assert rep['planned_strips'] == 1 and out.exists()
    doc = json.loads(out.read_text())
    (key, z0, z1), = tu.check_manager_schema(doc)
    assert key == pytest.approx(-350, abs=20)
    cols = cp.load_plan_columns(str(out))                   # also loads in coverage_replay.py --plan
    assert len(cols) == 1


def test_schema_rejects_bad_segment():
    g = make_grid()
    g.status[:, 14:21] = cp.NOT_DONE
    plan, _ = tu.touch_up_grid(g)
    plan['final_lifted_and_expanded']['segments'][0]['end_index'] = 999
    with pytest.raises(ValueError):
        tu.check_manager_schema(plan)


def test_horizontal_tool_profile_rejected(tmp_path):
    import touch_up_plan
    y = tmp_path / 't.yaml'
    y.write_text('tools:\n  w:\n    coverage: {swath_m: 0.2, overlap_pct: 10, sweep_direction: horizontal}\n')
    (tmp_path / 'run.csv').write_text('t,x,y,z,force\n')
    with pytest.raises(SystemExit):
        touch_up_plan.main([str(tmp_path / 'run.csv'), '--wall', str(tmp_path / 'w.json'), '--tool-profiles', str(y),
                            '--tool', 'w'])


def test_tool_profile_values():
    here = os.path.join(HERE, '..', '..', '..', 'perception', 'coverage_planner', 'config', 'tool_profiles.yaml')
    s, o, d = tu.load_tool_coverage(here)
    assert (s, o, d) == (0.20, 20.0, 'vertical')
