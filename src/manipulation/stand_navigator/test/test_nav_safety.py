import math
import struct

import pytest

from stand_navigator import nav_safety as ns
from stand_navigator.core import BaseProfile, StandController, Tolerance, wrap
from stand_navigator.path_planner import PathFollower

P = BaseProfile(drive='omni', max_lin=0.5, max_ang=1.0, max_lin_acc=0.5, max_ang_acc=1.0)


def test_gate_rejects_non_finite():
    for bad in [(float('nan'), 0, 0), (0, float('inf'), 0), (0, 0, float('-inf'))]:
        cmd, v = ns.gate_command(bad, (0, 0, 0), 0.02, P)
        assert v.action == ns.ABORT and v.code == ns.BAD_COMMAND and cmd == (0.0, 0.0, 0.0)


def test_gate_rejects_over_limit_and_clamps_slight_excess():
    _, v = ns.gate_command((0.9, 0, 0), (0.9, 0, 0), 0.02, P)
    assert v.code == ns.BAD_COMMAND and 'limits' in v.reason
    _, v = ns.gate_command((0, 0, 3.0), (0, 0, 3.0), 0.02, P)
    assert v.code == ns.BAD_COMMAND
    cmd, v = ns.gate_command((0.51, 0, 0), (0.505, 0, 0), 0.5, P)
    assert v.ok and cmd[0] == pytest.approx(0.5)


def test_gate_limits_acceleration():
    cmd, v = ns.gate_command((0.5, 0, 0), (0, 0, 0), 0.1, P)
    assert v.ok and cmd[0] <= 0.5 * 0.1 * 1.05 + 1e-9 and cmd[0] > 0.04


@pytest.mark.parametrize('drive,start,goal', [
    ('omni', (0, 0, 0), (2.0, 1.0, 1.0)), ('diff', (0, 0, 0), (-1.5, 0.5, 2.5)),
    ('omni', (0, 0, 0), (0.5, 0, 0.0))])
def test_gate_never_changes_a_healthy_run(drive, start, goal):
    prof = BaseProfile(drive=drive, max_lin=0.5, max_ang=1.0, max_lin_acc=0.5, max_ang_acc=1.0)
    for ctl in (StandController(prof, goal), PathFollower(prof, [start[:2], goal[:2]], goal)):
        x, y, yaw = start
        sent, dt = (0.0, 0.0, 0.0), 0.02
        mon = ns.SafetyMonitor(ns.SafetyConfig(), 0.0)
        for i in range(int(90 / dt)):
            t = i * dt
            cmd = ctl.step((x, y, yaw), dt)
            out, v = ns.gate_command(cmd, sent, dt, prof)
            assert v.ok and out == pytest.approx(cmd, abs=1e-12)
            d, e = ctl.errors((x, y, yaw))
            vv = mon.check(t, t, ns.progress_metric(d, e, mon.cfg), d <= 0.02 and abs(e) <= 0.02)
            assert vv.ok, (drive, t, vv.reason)
            sent = out
            c, s = math.cos(yaw), math.sin(yaw)
            x += (c * out[0] - s * out[1]) * dt
            y += (s * out[0] + c * out[1]) * dt
            yaw = wrap(yaw + out[2] * dt)
            if ctl.arrived((x, y, yaw), math.hypot(out[0], out[1]), abs(out[2])):
                break
        else:
            pytest.fail('did not arrive')


def test_stale_odom_aborts_with_reason():
    m = ns.SafetyMonitor(ns.SafetyConfig(odom_timeout_s=0.5), 0.0)
    assert m.check(1.0, 0.8).ok
    v = m.check(1.0, 0.4)
    assert v.action == ns.ABORT and v.code == ns.NO_ODOM and 'stale' in v.reason
    assert m.check(1.0, None).code == ns.NO_ODOM


def test_stuck_aborts_after_window_and_progress_resets():
    cfg = ns.SafetyConfig(stuck_window_s=5.0, stuck_min_progress=0.02)
    m = ns.SafetyMonitor(cfg, 0.0)
    metric = 2.0
    for t in range(0, 100):          # creeping 0.03 per second is progress
        metric -= 0.03
        assert m.check(float(t), float(t), metric).ok
    m = ns.SafetyMonitor(cfg, 0.0)
    assert m.check(0.0, 0.0, 2.0).ok
    assert m.check(4.9, 4.9, 1.999).ok
    v = m.check(5.2, 5.2, 1.999)
    assert v.action == ns.ABORT and v.code == ns.NAVIGATION_STUCK and 'no progress' in v.reason


def test_stuck_off_and_settled_not_stuck():
    m = ns.SafetyMonitor(ns.SafetyConfig(stuck_window_s=0.0), 0.0)
    assert all(m.check(t, t, 1.0).ok for t in range(0, 500, 10))
    m = ns.SafetyMonitor(ns.SafetyConfig(stuck_window_s=2.0), 0.0)
    assert all(m.check(t, t, 0.0, settled=True).ok for t in range(0, 50))


def test_obstacle_stop_hold_then_abort_and_resume():
    cfg = ns.SafetyConfig(front_stop_m=0.3, obstacle_hold_s=2.0, stuck_window_s=1.0)
    m = ns.SafetyMonitor(cfg, 0.0)
    assert m.check(0.0, 0.0, 1.0, clearance=2.0, clearance_stamp=0.0).ok
    v = m.check(0.5, 0.5, 1.0, clearance=0.2, clearance_stamp=0.5)
    assert v.action == ns.HOLD and v.code == ns.OBSTACLE
    assert m.check(2.0, 2.0, 1.0, clearance=0.2, clearance_stamp=2.0).action == ns.HOLD   # not stuck while held
    v = m.check(3.0, 3.0, 1.0, clearance=0.2, clearance_stamp=3.0)
    assert v.action == ns.ABORT and v.code == ns.OBSTACLE and '0.20 m' in v.reason
    m = ns.SafetyMonitor(cfg, 0.0)
    m.check(0.0, 0.0, 1.0, clearance=0.2, clearance_stamp=0.0)
    assert m.check(1.0, 1.0, 1.0, clearance=1.0, clearance_stamp=1.0).ok     # cleared
    assert m.hold_since is None


def test_obstacle_stop_distance_grows_with_speed():
    m = ns.SafetyMonitor(ns.SafetyConfig(front_stop_m=0.3), 0.0)
    assert m.check(0.0, 0.0, clearance=0.5, clearance_stamp=0.0, speed=0.0).ok
    assert m.check(0.0, 0.0, clearance=0.5, clearance_stamp=0.0, speed=0.5, max_lin_acc=0.5).action == ns.HOLD


def test_obstacle_sensor_missing_or_stale_aborts_only_when_enabled():
    on = ns.SafetyMonitor(ns.SafetyConfig(front_stop_m=0.3), 0.0)
    assert on.check(1.0, 1.0, clearance=None, clearance_stamp=None).code == ns.OBSTACLE
    assert on.check(5.0, 5.0, clearance=3.0, clearance_stamp=1.0).code == ns.OBSTACLE
    off = ns.SafetyMonitor(ns.SafetyConfig(), 0.0)
    assert off.check(1.0, 1.0, clearance=None, clearance_stamp=None).ok


def test_corridor_clearance_and_scan_points():
    pts = [(1.0, 0.0), (0.3, 0.9), (-0.3, 0.0), (2.0, 0.1)]
    assert ns.corridor_clearance(pts, 0.0, 0.4) == pytest.approx(1.0)
    assert ns.corridor_clearance(pts, math.pi / 2, 0.4, 5.0) == pytest.approx(0.9)
    assert ns.corridor_clearance([], 0.0, 0.4, 3.0) == 3.0
    assert ns.corridor_clearance([(1.0, 0.5)], 0.0, 0.4) == 5.0       # beside the corridor
    ranges = [float('inf'), 1.0, float('nan'), 0.05, 9.0]
    p = ns.scan_to_points(ranges, -math.pi / 2, math.pi / 4, 0.1, 8.0)
    assert len(p) == 1 and p[0][0] == pytest.approx(1.0 * math.cos(-math.pi / 4))


def test_cloud_to_points_filters_height_and_nan():
    step = 16
    rows = [(1.0, 0.0, 0.5), (2.0, 0.0, 3.0), (float('nan'), 0.0, 0.5), (0.7, -0.1, -0.1)]
    data = b''.join(struct.pack('<ffff', x, y, z, 1.0) for x, y, z in rows)
    pts = ns.cloud_to_points(data, step, (0, 4, 8))
    assert pts == [(pytest.approx(1.0), 0.0), (pytest.approx(0.7), pytest.approx(-0.1))]


def test_held_direction_still_looks_at_the_obstacle():
    h = ns.HeldDirection()
    assert h.look((0.3, 0.0, 0.0)) == (0.3, 0.0, 0.0)
    h.hold((0.3, 0.0, 0.0))
    # stopped: clearance must still be measured along the old direction
    assert h.look((0.0, 0.0, 0.0)) == (0.3, 0.0, 0.0)
    pts = [(0.5, 0.0)]
    cmd = h.look((0.0, 0.0, 0.0))
    assert ns.corridor_clearance(pts, math.atan2(cmd[1], cmd[0]), 0.4) == pytest.approx(0.5)


def test_held_direction_before_any_motion_and_after_resume():
    h = ns.HeldDirection()
    assert h.look((0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0)
    h.hold((0.0, 0.0, 0.0))
    assert h.last is None
    h.hold((0.0, 0.2, 0.0))
    assert h.look((0.1, 0.0, 0.0)) == (0.1, 0.0, 0.0)
