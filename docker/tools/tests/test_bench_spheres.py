"""Tests for the result summary of bench_spheres.py (pure python, no GPU, no meshes).

    python -m pytest docker/tools/tests/test_bench_spheres.py
"""
import math
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import bench_spheres as bs  # noqa: E402


def _p(success, wall_ms, valid=True):
    return {'success': success, 'wall_s': wall_ms / 1e3, 'endpoints_valid': [valid, True]}


# 2 fast successes, 1 slow failure, 1 slow failure whose start is invalid under this set
RESULTS = [_p(True, 10), _p(True, 30), _p(False, 1000), _p(False, 2000, valid=False)]


def test_success_counts_every_problem_and_valid_success_leaves_invalid_endpoints_out():
    s = bs.summarize_problems(RESULTS)
    assert (s['n'], s['ok'], s['endpoints_invalid'], s['n_valid']) == (4, 2, 1, 3)
    assert s['success'] == pytest.approx(2 / 4)
    assert s['success_valid'] == pytest.approx(2 / 3)


def test_median_and_p95_cover_successful_plans_only_all_runs_median_is_separate():
    s = bs.summarize_problems(RESULTS)
    assert s['time_ms_median'] == pytest.approx(20.0)
    assert s['time_ms_p95'] <= 30.0
    assert s['time_ms_median_all'] == pytest.approx(515.0)


def test_no_successes_gives_nan_times_not_an_error():
    s = bs.summarize_problems([_p(False, 5), _p(False, 7, valid=False)])
    assert s['success'] == 0 and math.isnan(s['time_ms_median'])
