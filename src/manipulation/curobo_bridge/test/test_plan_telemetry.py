import logging
from types import SimpleNamespace

from curobo_bridge.planner import CuroboPlanner, PlanResult


def _planner(attempts=7, graph=None):
    p = CuroboPlanner.__new__(CuroboPlanner)
    p._planner = SimpleNamespace(_last_planning_attempts=attempts,
                                 _last_graph_call_telemetry=graph if graph is not None else [])
    return p


def test_telemetry_reads_cuRobo_counters():
    graph = [SimpleNamespace(solve_time=0.25, success=False), SimpleNamespace(solve_time=0.5, success=True)]
    out = PlanResult()
    _planner(attempts=7, graph=graph)._add_telemetry(out)
    assert out.attempts == 7
    assert out.graph_calls == 2
    assert abs(out.graph_solve_s - 0.75) < 1e-12


def test_telemetry_is_zero_when_cuRobo_has_no_counters():
    p = CuroboPlanner.__new__(CuroboPlanner)
    p._planner = SimpleNamespace()
    out = PlanResult()
    p._add_telemetry(out)
    assert (out.attempts, out.graph_calls, out.graph_solve_s) == (0, 0, 0.0)


def test_plan_joint_times_the_call_and_logs_one_line(caplog):
    p = _planner()
    p._plan_joint = lambda start, goal: PlanResult(success=True, route='trajopt', attempts=1, planning_time_s=0.02)
    with caplog.at_level(logging.INFO):
        out = p.plan_joint([0.0], [1.0])
    assert out.wall_s >= 0.0 and out.success
    lines = [r.getMessage() for r in caplog.records if r.getMessage().startswith('plan_joint:')]
    assert len(lines) == 1 and 'route=trajopt' in lines[0] and 'attempts=1' in lines[0]


def test_a_failed_plan_logs_its_reason_and_attempts(caplog):
    p = _planner()
    p._plan_joint = lambda start, goal: PlanResult(failure_reason_code='no_path', attempts=20, planning_time_s=5.4)
    with caplog.at_level(logging.INFO):
        p.plan_joint([0.0], [1.0])
    msg = [r.getMessage() for r in caplog.records if r.getMessage().startswith('plan_joint:')][0]
    assert 'failed=no_path' in msg and 'attempts=20' in msg
