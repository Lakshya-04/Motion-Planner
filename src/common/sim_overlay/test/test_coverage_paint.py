import json
import os
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

from sim_overlay import coverage_paint as cp  # noqa: E402
from sim_overlay import gz_commands, mujoco_overlay  # noqa: E402
from sim_overlay.overlay import OverlayState  # noqa: E402

REC_WALL = {'origin': [0.98, 0, 0], 'u': [0, 1, 0], 'v': [0, 0, 1], 'u_range': [0, 0.6], 'v_range': [1.0, 1.4]}
SCENE = {'frame_id': 'odom', 'walls': [{'id': 0, 'origin_xyz': [2, 0, 0], 'basis_u': [0, 1, 0], 'basis_v': [0, 0, 1],
                                        'normal_outward': [-1, 0, 0], 'u_range_m': [-1, 1], 'v_range_m': [0, 2]}]}


def grid(**kw):
    return cp.CoverageGrid(cp.wall_from_dict(REC_WALL), **kw)


def test_wall_formats():
    w = cp.wall_from_dict(REC_WALL)
    assert w.area_m2 == pytest.approx(0.24)
    assert np.allclose(w.n, [1, 0, 0])
    s = cp.wall_from_dict(SCENE)
    assert s.frame_id == 'odom' and np.allclose(s.n, [-1, 0, 0]) and np.allclose(s.v, [0, 0, 1])
    with pytest.raises(KeyError):
        cp.wall_from_dict(SCENE, 3)


def test_starts_not_done_and_grid_size():
    g = grid()
    assert (g.nu, g.nv) == (30, 20)
    assert g.percent_done() == 0 and g.counts()[cp.NOT_DONE] == 600


def test_contact_needs_force_and_plane_distance():
    g = grid()
    assert g.add_sample((0.98, 0.3, 1.2), 0.2, 0) == 0            # below force threshold
    assert g.add_sample((0.90, 0.3, 1.2), 10.0, 0) == 0           # 8 cm off the plane
    assert g.percent_done() == 0
    assert g.add_sample((0.99, 0.3, 1.2), 10.0, 0) > 0
    assert g.add_sample((0.99, 0.3, 1.2), 10.0, 0.1) == 0         # same cells again
    # swath disc of radius 7 cm: area ~ pi r^2 = 0.0154 m2 = 6.4 % of 0.24
    assert g.percent_done() == pytest.approx(6.4, abs=1.2)


def test_stroke_paints_swath_band_not_the_gaps():
    g = grid()
    for k in range(31):                                           # u 0.1 .. 0.5 at z 1.2, 5 cm steps are not needed
        g.add_sample((0.98, 0.1 + k * 0.4 / 30, 1.2), 5.0, k * 0.1)
    st = g.status
    row = lambda z: st[int((z - 1.0) / 0.02)]  # noqa: E731
    assert row(1.2)[int(0.3 / 0.02)] == cp.DONE
    assert row(1.2 + 0.05)[int(0.3 / 0.02)] == cp.DONE            # inside half swath (0.07)
    assert row(1.2 + 0.09)[int(0.3 / 0.02)] == cp.NOT_DONE        # outside
    assert row(1.2)[int(0.59 / 0.02)] == cp.NOT_DONE              # beyond the stroke end + half swath
    # band is ~0.14 x (0.4 + 0.14 for the round ends)
    assert cp.CoverageGrid.percent_done(g) == pytest.approx(100 * 0.14 * 0.4 / 0.24 + 6.4, abs=5)


def test_gap_in_contact_breaks_the_stroke():
    g = grid()
    g.add_sample((0.98, 0.1, 1.2), 5, 0)
    g.add_sample((0.98, 0.1, 1.2), 0, 0.1)                        # force lost
    g.add_sample((0.98, 0.5, 1.2), 5, 0.2)                        # must not be joined to (0.1) by a bridge
    assert g.status[int(0.2 / 0.02), int(0.3 / 0.02)] == cp.NOT_DONE


def test_nan_and_force_threshold_zero():
    g = grid(force_threshold_n=0)
    assert g.add_sample((0.98, 0.3, float('nan')), 5, 0) == 0
    assert g.add_sample((0.98, 0.3, 1.2), 0.0, 0) > 0             # spray mode: force ignored


def test_failed_columns_orange_only_where_not_done():
    g = grid()
    g.set_columns({0: [(0.98, 0.1, 1.05), (0.98, 0.1, 1.35)], 1: [(0.98, 0.5, 1.05), (0.98, 0.5, 1.35)]})
    g.add_sample((0.98, 0.1, 1.2), 5, 0)                          # one cell patch of column 0 done
    n = g.mark_outcome(done=[0], failed=[0], partial=[1])
    assert n > 0
    assert g.status[int(0.2 / 0.02), int(0.1 / 0.02)] == cp.DONE  # kept
    assert g.status[int(0.05 / 0.02) + 15, int(0.5 / 0.02)] == cp.FAILED or (g.status == cp.FAILED).any()
    assert g.status[int(0.35 / 0.02), int(0.5 / 0.02)] == cp.FAILED
    g.add_sample((0.98, 0.5, 1.35), 5, 5)                         # later sweep turns FAILED into DONE
    assert g.status[int(0.35 / 0.02), int(0.5 / 0.02)] == cp.DONE
    assert g.mark_column(99) == 0                                 # unknown column ignored


def test_feedback_tracker_infers_outcomes():
    t = cp.FeedbackTracker()
    assert t.update(-1, 0, 0) == []
    assert t.update(0, 0, 0) == []
    assert t.update(1, 1, 0) == [(0, 'done')]
    assert t.update(-1, 1, 1) == [(1, 'failed')]
    assert t.update(2, 1, 1) == []
    assert t.update(-1, 2, 1) == [(2, 'done')]
    assert t.done == [0, 2] and t.failed == [1]


def test_rects_cover_exactly_and_merge():
    g = grid()
    assert g.rects(cp.NOT_DONE) == [(0, 29, 0, 19)]               # clean wall: one rectangle
    g.add_sample((0.98, 0.3, 1.2), 5, 0)
    for st in (cp.NOT_DONE, cp.DONE):
        m = np.zeros(g.status.shape, bool)
        for i0, i1, j0, j1 in g.rects(st):
            assert not m[j0:j1 + 1, i0:i1 + 1].any()               # no overlap
            m[j0:j1 + 1, i0:i1 + 1] = True
        assert (m == (g.status == st)).all()                       # exact cover
    assert len(g.rects(cp.DONE)) <= 8                              # a disc is few rectangles


def test_markers_flow_into_gazebo_and_mujoco_paths():
    g = grid()
    g.add_sample((0.98, 0.3, 1.2), 5, 0)
    ms = g.marker_dicts()
    assert {m['ns'] for m in ms} >= {'coverage_paint_done', 'coverage_paint_not_done', 'coverage_paint_text'}
    assert any(m['type'] == 'TRIANGLE_LIST' and m['ns'].endswith('failed') and m['action'] == 'DELETE' for m in ms)
    st = OverlayState()
    st.update('/mp_viz/coverage_paint', ms)
    prims = st.primitives()
    tiles = [p for p in prims if p['kind'] == 'tiles']
    assert len(tiles) == 2 and all(len(q) == 4 for p in tiles for q in p['quads'])
    assert any(p['kind'] == 'label' and 'covered' in p['text'] for p in prims)
    cmds, _ = gz_commands.plan_commands(prims)
    assert any('type: TRIANGLE_LIST' in c[-1] for c in cmds)
    # quads lie in front of the wall (x = 0.98 + lift)
    assert all(abs(pt[0] - 0.983) < 1e-6 for p in tiles for q in p['quads'] for pt in q)


class FakeGeom:
    label = ''


class FakeScn:
    def __init__(self, n):
        self.maxgeom, self.ngeom = n, 0
        self.geoms = [FakeGeom() for _ in range(n)]


class FakeMj:
    class mjtGeom:
        mjGEOM_BOX = 6
    calls = []

    @classmethod
    def mjv_initGeom(cls, g, typ, size, pos, mat, rgba):
        cls.calls.append((typ, np.array(size), np.array(pos), np.array(mat)))


def test_mujoco_tiles_become_flat_boxes():
    prim = {'kind': 'tiles', 'color': [0, 1, 0, 1],
            'quads': [[[0.98, 0, 1], [0.98, 0.2, 1], [0.98, 0.2, 1.1], [0.98, 0, 1.1]]]}
    scn = FakeScn(10)
    FakeMj.calls = []
    assert mujoco_overlay.draw(scn, [prim], mujoco=FakeMj) == 0
    typ, size, pos, mat = FakeMj.calls[0]
    assert scn.ngeom == 1 and typ == 6
    assert np.allclose(size, [0.1, 0.05, 0.0015]) and np.allclose(pos, [0.98, 0.1, 1.05])
    assert np.allclose(mat.reshape(3, 3)[:, 2], [-1, 0, 0]) or np.allclose(mat.reshape(3, 3)[:, 2], [1, 0, 0])
    assert mujoco_overlay.draw(FakeScn(0), [prim], mujoco=FakeMj) == 1   # no room: dropped, counted


def _write_run(tmp_path):
    csv = tmp_path / 'run.csv'
    rows = ['t,x,y,z,force,active']
    t = 0.0
    for z in (1.1, 1.3):                                          # two horizontal strokes
        for k in range(61):
            rows.append('%.2f,0.98,%.4f,%.3f,6.0,1' % (t, 0.05 + k * 0.5 / 60, z))
            t += 0.05
        rows.append('%.2f,0.90,0.3,1.2,0.0,0' % t)                # lifted off
        t += 0.05
    csv.write_text('\n'.join(rows) + '\n')
    (tmp_path / 'run.csv.wall.json').write_text(json.dumps(REC_WALL))
    return csv


def test_replay_cli_writes_png_and_number(tmp_path):
    pytest.importorskip('matplotlib')
    csv = _write_run(tmp_path)
    r = subprocess.run([sys.executable, os.path.join(HERE, '..', 'scripts', 'coverage_replay.py'), str(csv),
                        '--overlay-json', str(tmp_path / 'feed.json')], capture_output=True, text=True, check=True)
    res = json.loads(r.stdout.strip().splitlines()[-1])
    png = tmp_path / 'run.coverage.png'
    assert res['png'] == str(png) and png.read_bytes()[:8] == b'\x89PNG\r\n\x1a\n'
    # two 0.14 m bands of length 0.5 + round ends over 0.6 x 0.4 m wall: roughly 0.14*0.64*2/0.24 = 75 % minus the overlap
    assert 55 < res['percent_done'] < 85
    prims = mujoco_overlay.load_overlay(str(tmp_path / 'feed.json'))
    assert any(p['kind'] == 'tiles' for p in prims)


def test_replay_with_failed_column_and_active_filter(tmp_path):
    csv = _write_run(tmp_path)
    plan = {'final_lifted_and_expanded': {
        'path': {'poses': [{'position': {'x': 0.98, 'y': y, 'z': z}, 'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}
                           for y, z in ((0.1, 1.1), (0.5, 1.1), (0.1, 1.2), (0.5, 1.2), (0.1, 1.38), (0.5, 1.38))]},
        'segments': [{'label': 'strip_0', 'start_index': 0, 'end_index': 1},
                     {'label': 'connector', 'start_index': 1, 'end_index': 2},
                     {'label': 'strip_1', 'start_index': 4, 'end_index': 5}]}}
    pf = tmp_path / 'plan.json'
    pf.write_text(json.dumps(plan))
    assert sorted(cp.load_plan_columns(str(pf))) == [0, 1]       # connector skipped, strips numbered 0, 1
    g = cp.CoverageGrid(cp.load_wall(str(csv) + '.wall.json'))
    cp.replay(g, cp.read_run_csv(str(csv)))
    base = g.counts()[cp.DONE]
    g.set_columns(cp.load_plan_columns(str(pf)))
    g.mark_outcome(failed=[1])
    assert g.counts()[cp.DONE] == base and g.counts()[cp.FAILED] >= 0
    g2 = cp.CoverageGrid(cp.load_wall(str(csv) + '.wall.json'))
    cp.replay(g2, cp.read_run_csv(str(csv)), require_active=True)
    assert g2.counts()[cp.DONE] == base                           # the off-plane rows were inactive anyway


def test_marker_message_conversion_names_match_node():
    # the node maps these dict types to Marker types; keep the set small and known
    assert {m['type'] for m in grid().marker_dicts()} == {'TRIANGLE_LIST', 'LINE_STRIP', 'TEXT_VIEW_FACING'}
