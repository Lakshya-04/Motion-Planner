"""path_quality and the optimal-mode candidate selection: numpy only, no torch / cuRobo."""
import logging

import numpy as np
import pytest

from curobo_bridge import path_quality as pq
from curobo_bridge.planner import CuroboPlanner, PlanResult


def test_path_length_weighted_and_degenerate():
    pts = [[0, 0], [3, 4], [3, 4]]
    assert pq.path_length(pts) == pytest.approx(5.0)
    assert pq.path_length(pts, [2.0, 2.0]) == pytest.approx(10.0)
    assert pq.path_length([[1, 2]]) == 0.0 and pq.path_length([]) == 0.0
    with pytest.raises(ValueError):
        pq.path_length(pts, [1.0])


def test_joint_weights_normalise_by_range():
    w = pq.joint_weights([0.0, -3.14], [1.0, 3.14])
    assert w == pytest.approx([1.0, 1 / 6.28])
    assert pq.joint_weights([0.0], [0.0]) == [1.0]


def test_straight_duration_trapezoid_and_triangle():
    # distance 1 rad, vmax 1, amax 1: triangular (v^2/a = 1 >= 1): T = 2
    assert pq.straight_duration([0], [1], [1.0], [1.0]) == pytest.approx(2.0)
    # distance 1, vmax 0.5, amax 1: trapezoid, T = 1/0.5 + 0.5/1 = 2.5
    assert pq.straight_duration([0], [1], [0.5], [1.0]) == pytest.approx(2.5)
    assert pq.straight_duration([0], [0], [1.0], [1.0]) == 0.0
    # the slowest joint (relative to its limit) sets the time
    assert pq.straight_duration([0, 0], [1, 1], [0.5, 5.0], [1.0, 9.0]) == pytest.approx(2.5)


def test_straight_line_endpoints_monotone_and_within_limits():
    rows = pq.straight_line([0, 0], [1, -2], 0.01, [1.0, 1.0], [2.0, 2.0])
    q = np.array(rows)
    assert q[0] == pytest.approx([0, 0]) and q[-1] == pytest.approx([1, -2])
    assert np.all(np.diff(q[:, 0]) >= -1e-12)
    v = np.abs(np.diff(q, axis=0)) / 0.01
    assert v.max() <= 1.0 + 1e-6
    # straight line length is the euclidean distance, whatever the timing
    assert pq.path_length(rows) == pytest.approx(np.hypot(1, 2))
    assert pq.duration(rows, 0.01) >= pq.straight_duration([0, 0], [1, -2], [1, 1], [2, 2]) - 0.01


def test_straight_line_same_endpoints():
    assert len(pq.straight_line([1, 1], [1, 1], 0.1, [1, 1], [1, 1])) == 2


def test_efficiency():
    assert pq.efficiency(2.0, 1.0) == 0.5
    assert pq.efficiency(1.0, 1.0) == 1.0
    assert pq.efficiency(0.5, 1.0) == 1.0       # capped
    assert pq.efficiency(0.0, 0.0) == 1.0


def _cand(name, pos, dt=0.1, valid=True):
    return pq.Candidate(name, pos, dt, valid)


def test_select_best_prefers_shorter_valid_and_straight_wins_ties():
    straight = _cand('straight', [[0], [1]])
    detour = _cand('trajopt-0', [[0], [2], [1]])
    assert pq.select_best([detour, straight], 'length').name == 'straight'
    assert pq.select_best([_cand('s', [[0], [1]], valid=False), detour], 'length').name == 'trajopt-0'
    assert pq.select_best([_cand('s', [[0], [1]], valid=False)], 'length') is None
    a, b = _cand('a', [[0], [1]]), _cand('b', [[0], [1]])
    assert pq.select_best([a, b], 'length').name == 'a'


def test_select_best_by_duration_and_secondary_key():
    fast_long = _cand('fast', [[0], [2]], dt=0.1)          # length 2, duration 0.1
    slow_short = _cand('slow', [[0], [0.5], [1]], dt=0.5)  # length 1, duration 1.0
    assert pq.select_best([fast_long, slow_short], 'duration').name == 'fast'
    assert pq.select_best([fast_long, slow_short], 'length').name == 'slow'
    with pytest.raises(ValueError):
        pq.select_best([fast_long], 'speed')


def test_summary_line_format():
    best, base = pq.score(_cand('trajopt-1', [[0], [2], [1]])), pq.score(_cand('straight', [[0], [1]]))
    line = pq.summary_line(4, best, 'length', base)
    assert line.startswith('free-space plan: candidates 4, best length=3.0000 '
                               '(baseline straight 1.0000, efficiency 0.333)')


# ---- candidate loop of the planner, with a fake cuRobo -------------------------------------------------------

class _Torch:
    def manual_seed(self, s):
        self.seed = s


class _Planner:
    def __init__(self, paths):
        self.paths, self.calls = list(paths), []

    def plan_cspace(self, tgt, cur, **kw):
        self.calls.append(kw)
        return self.paths.pop(0) if self.paths else None


def _make(paths, straight_valid, objective='length', candidates=3):
    p = CuroboPlanner.__new__(CuroboPlanner)
    p._torch, p._planner = _Torch(), _Planner(paths)
    p._optimize, p._budget_s, p._num_candidates, p._objective = True, 100.0, candidates, objective
    p._finetune_attempts, p._seed0, p._interpolation_dt, p._max_attempts = 4, 7, 0.1, 10
    p._lift_joints, p._lift_cost_weight = [], 1.0
    p._joint_limits = lambda: ([-1.0], [1.0])
    p._va_limits = lambda: ([1.0], [2.0])
    p.check_motion = lambda rows, step: straight_valid
    p._wrap = lambda res, last=None: (PlanResult(success=True, joint_names=['j'], positions=res, dt=0.1)
                                      if res is not None else PlanResult(failure_reason_code='no_path'))
    CuroboPlanner.joint_names = property(lambda self: ['j'])
    return p


def test_optimal_returns_straight_when_unobstructed(caplog):
    p = _make([[[0.0], [1.5], [0.5]]] * 3, straight_valid=True)
    with caplog.at_level(logging.INFO):
        out = p._plan_joint_optimal([0.0], [0.5], None, None)
    assert out.success and out.route == 'straight'
    assert out.positions[0] == [0.0] and out.positions[-1] == [0.5]
    assert 'free-space plan: candidates 4, best length=0.2500 (baseline straight 0.2500, efficiency 1.000)' in caplog.text
    assert len(p._planner.calls) == 3 and p._torch.seed == 9


def test_optimal_picks_shortest_trajopt_when_straight_blocked():
    long_ = [[0.0], [1.0], [-1.0], [0.5]]
    short = [[0.0], [0.7], [0.5]]
    p = _make([long_, short, long_], straight_valid=False)
    out = p._plan_joint_optimal([0.0], [0.5], None, None)
    assert out.route != 'straight' and out.positions == short


def test_optimal_fails_when_nothing_valid():
    p = _make([], straight_valid=False)
    assert p._plan_joint_optimal([0.0], [0.5], None, None).failure_reason_code == 'no_path'


def test_optimal_stops_starting_candidates_after_budget():
    p = _make([[[0.0], [0.5]]] * 5, straight_valid=False, candidates=5)
    p._budget_s = 0.0
    p._plan_joint_optimal([0.0], [0.5], None, None)
    assert len(p._planner.calls) == 1        # the first candidate always runs


def test_lift_cost_weight_prefers_arm_motion():
    names = ['lift', 'a1']
    base = [1.0, 1.0]
    a = [[0, 0], [0.9, 0.1]]   # mostly lift
    b = [[0, 0], [0.1, 1.1]]   # mostly arm
    assert pq.apply_lift_cost_weight(base, names, ['lift'], 1.0) == base
    w2 = pq.apply_lift_cost_weight(base, names, ['lift'], 2.0)
    assert w2 == [2.0, 1.0]
    assert pq.path_length(a, base) < pq.path_length(b, base)
    assert pq.path_length(b, w2) < pq.path_length(a, w2)
