"""Host tests of the pure logic of mp_viz_markers.py (no rclpy)."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import mp_viz_logic as L  # noqa: E402


def test_magnitude():
    assert L.force_magnitude(3, 4, 0) == pytest.approx(5.0)


def test_colour_ramp_green_to_red():
    assert L.force_to_rgb(0, 50) == (0.0, 1.0, 0.0)
    assert L.force_to_rgb(25, 50) == (1.0, 1.0, 0.0)
    assert L.force_to_rgb(50, 50) == (1.0, 0.0, 0.0)
    assert L.force_to_rgb(500, 50) == (1.0, 0.0, 0.0)
    assert L.force_to_rgb(-3, 50) == (0.0, 1.0, 0.0)


def test_colour_grey_without_limit_or_force():
    assert L.force_to_rgb(10, 0) == (0.5, 0.5, 0.5)
    assert L.force_to_rgb(float('nan'), 50) == (0.5, 0.5, 0.5)


def test_trace_is_capped_and_keeps_newest():
    b = L.TraceBuffer(3)
    for i in range(10):
        b.add(i, 0, 0, i)
    assert len(b) == 3
    assert [p[0] for p in b.points()] == [7, 8, 9]


def test_trace_min_step_drops_close_samples():
    b = L.TraceBuffer(10, min_step_m=0.01)
    assert b.add(0, 0, 0, 1)
    assert not b.add(0.001, 0, 0, 1)
    assert b.add(0.02, 0, 0, 1)
    assert len(b) == 2
    b.clear()
    assert len(b) == 0


def test_trace_rejects_bad_cap():
    with pytest.raises(ValueError):
        L.TraceBuffer(0)


def test_readout():
    t = L.format_readout('pressing', 1, 3, 12.34, 50.0, column=4)
    assert t == 'pressing\nstand 2/3  column 4\nforce 12.3 N / 50 N'
    assert 'OVER LIMIT' in L.format_readout('stroking', 0, 1, 60.0, 50.0)
    assert L.format_readout('', 0, 0, None, 50.0) == 'no mission feedback\nstand -\nforce - N'
    assert 'force - N' in L.format_readout('x', 0, 0, math.nan, 50.0)


def test_line_list_points_breaks_between_strokes():
    pts = [(0, 0, 0), (1, 0, 0), (2, 0, 0), None, (5, 0, 0), (6, 0, 0), None, None, (9, 0, 0)]
    out = L.line_list_points(pts)
    assert out == [(0, 0, 0), (1, 0, 0), (1, 0, 0), (2, 0, 0), (5, 0, 0), (6, 0, 0)]   # no segment joins two strokes, a lone point draws nothing
    assert L.line_list_points([]) == []
    assert L.line_list_points([None, None]) == []
