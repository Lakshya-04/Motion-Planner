"""Host tests of the pure logic of mp_viz_scene.py (no rclpy)."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import mp_viz_scene_logic as L  # noqa: E402

PTS = [(float(i), 0.0, 0.0) for i in range(10)]


def test_segments_split_and_colour():
    segs = [{'label': 'strip0', 'start_index': 0, 'end_index': 3},
            {'label': 'connector1', 'start_index': 3, 'end_index': 4},
            {'label': 'strip1', 'start_index': 4, 'end_index': 8}]
    out = L.segment_polylines(segs, PTS)
    assert [o['connector'] for o in out] == [False, True, False]
    assert out[1]['rgb'] == L.CONNECTOR_RGB
    assert out[0]['rgb'] == L.palette(0) and out[2]['rgb'] == L.palette(1)
    assert len(L.segment_polylines(segs, PTS, include_connectors=False)) == 2


def test_render_indices_widen_connector_and_bad_ranges_skipped():
    segs = [{'label': 'connector', 'start_index': 4, 'end_index': 4, 'render_start_index': 3, 'render_end_index': 5},
            {'label': 'bad', 'start_index': 5, 'end_index': 99},
            {'label': 'single', 'start_index': 2, 'end_index': 2}]
    out = L.segment_polylines(segs, PTS)
    assert len(out) == 1 and len(out[0]['points']) == 3


def test_path_fallback():
    assert len(L.path_fallback(PTS)) == 1
    assert L.path_fallback(PTS[:1]) == []


def test_subsample_keeps_ends():
    s = L.subsample(list(range(100)), 5)
    assert len(s) == 5 and s[0] == 0 and s[-1] == 99
    assert L.subsample([1, 2], 5) == [1, 2]


def test_wall_normal_from_tool():
    # identity: tool z = world z, so the normal is -z
    assert L.wall_normal_from_tool((0, 0, 0, 1)) == pytest.approx((0, 0, -1))
    # rotate +90 deg about y: tool z -> world +x, normal -> -x
    h = math.sqrt(0.5)
    assert L.wall_normal_from_tool((0, h, 0, h)) == pytest.approx((-1, 0, 0), abs=1e-9)


def test_stand_specs_labels():
    s = L.stand_specs([{'x': 1, 'y': 2, 'yaw': math.pi, 'travel_s': 2.5, 'target_indices': [0, 1]},
                       {'x': 0, 'y': 0, 'target_indices': []}])
    assert s[0]['label'] == '#0  2 tgt  2.5s'
    assert s[1]['label'] == '#1  0 tgt'
    assert s[0]['quat'][2] == pytest.approx(1.0)


def test_footprint_and_circle():
    fp = L.footprint_corners(0, 0, 0, 2.0, 1.0)
    assert len(fp) == 5 and fp[0] == fp[-1]
    assert max(p[0] for p in fp) == pytest.approx(1.0)
    rot = L.footprint_corners(0, 0, math.pi / 2, 2.0, 1.0)
    assert max(p[1] for p in rot) == pytest.approx(1.0)
    c = L.circle_points(1, 1, 2.0)
    assert all(math.hypot(x - 1, y - 1) == pytest.approx(2.0) for x, y, _ in c)


def test_reach_rgb_ramp():
    assert L.reach_rgb(0, 1) == (0.0, 0.0, 1.0)
    assert L.reach_rgb(1, 1) == (1.0, 0.0, 0.0)
    assert L.reach_rgb(5, 0) == (0.0, 0.0, 1.0)
