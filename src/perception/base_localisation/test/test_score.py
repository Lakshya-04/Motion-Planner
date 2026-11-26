import math

import numpy as np
import pytest

from base_localisation.score import align_first, score


def arc(n=200, r=2.0):
    a = np.linspace(0, math.pi, n)
    return np.column_stack((r * np.sin(a), r * (1 - np.cos(a)), a))


def test_identical_tracks_score_zero():
    t = np.linspace(0, 10, 200)
    tr = arc()
    s = score(t, tr, t, tr)
    assert s['rms_m'] < 1e-9 and s['final_yaw_deg'] == pytest.approx(0.0, abs=1e-9)
    assert s['path_m'] == pytest.approx(math.pi * 2.0, rel=1e-3)


def test_a_constant_start_offset_is_removed():
    t = np.linspace(0, 10, 200)
    tr = arc()
    assert score(t, tr, t, tr + np.array([0.3, -0.2, 0.0]))['rms_m'] < 1e-9
    rotated = np.column_stack((-tr[:, 1], tr[:, 0], tr[:, 2] + math.pi / 2))
    out = align_first(tr, rotated)
    assert np.allclose(out[:, :2], tr[:, :2], atol=1e-9)
    assert np.allclose(np.sin(out[:, 2] - tr[:, 2]), 0.0, atol=1e-9)


def test_scale_error_shows_up_as_position_error():
    t = np.linspace(0, 10, 200)
    tr = arc()
    est = tr.copy()
    est[:, :2] *= 1.02
    s = score(t, tr, t, est)
    assert s['final_m'] == pytest.approx(0.02 * 4.0, rel=0.05)


def test_estimate_at_other_times_is_matched_to_the_nearest_sample():
    t = np.linspace(0, 10, 200)
    tr = arc()
    t_est = np.linspace(0.01, 10.01, 150)
    est = np.column_stack([np.interp(t_est, t, tr[:, i]) for i in range(3)])
    assert score(t, tr, t_est, est)['rms_m'] < 0.03
