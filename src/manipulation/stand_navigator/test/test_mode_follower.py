import logging
import math

import pytest

from stand_navigator.core import BaseProfile, Tolerance, wrap
from stand_navigator.mode_follower import ModeFollower
from stand_navigator.mode_planner import plan_with_modes
from stand_navigator.path_planner import scene_obstacles
from test_mode_planner import ACK, CRAB, FP, MID, R, SCENE, SPIN, prof, ranger

TOL = Tolerance(0.02, 0.02, 0.02)


def drive_sim(profile, plan, start, goal, set_mode=None, dt=0.02, t_max=240.0):
    """Follow the plan on a planar kinematic body (unicycle for non-holonomic modes, full for omni and crab).

    Returns (pose, time, log) where log has (mode, vx, vy, wz) per step; asserts the mode rules every step."""
    f = ModeFollower(profile, plan, goal, TOL, set_mode)
    modes = {m.name: m for m in profile.drive_modes()}
    x, y, yaw = start
    t, log = 0.0, []
    while t < t_max:
        vx, vy, wz = f.step((x, y, yaw), dt)
        m = modes[f.current_mode]
        if not m.can_strafe:
            assert vy == 0.0, (f.phase, vy)
        if m.min_turn_radius:
            assert abs(wz) <= abs(vx) / m.min_turn_radius + 1e-9, (vx, wz)
        assert math.hypot(vx, vy) <= max(m.max_lin, m.turning_speed) + 1e-9 and abs(wz) <= m.max_ang + 1e-9
        c, s = math.cos(yaw), math.sin(yaw)
        x, y, yaw, t = x + (c * vx - s * vy) * dt, y + (s * vx + c * vy) * dt, wrap(yaw + wz * dt), t + dt
        log.append((f.current_mode, vx, vy, wz))
        if f.arrived((x, y, yaw), math.hypot(vx, vy), abs(wz)):
            break
    return (x, y, yaw), t, log, f


def check_arrived(f, pose, goal):
    d, e = f.errors(pose)
    assert d <= TOL.xy and abs(e) <= TOL.yaw, (pose, goal)


@pytest.mark.parametrize('goal', [(3, 2, 1.0), (0, 1.0, 0.0), (-2, 0.5, 3.0), (2, -2, -1.5), (0.5, 0.0, 2.0)])
def test_ackermann_follower_reaches_stand_within_tolerance(goal):
    p = prof('4ws', [ACK])
    start = (0.0, 0.0, 0.0)
    plan = plan_with_modes(p, start, goal)
    pose, t, log, f = drive_sim(p, plan, start, goal)
    check_arrived(f, pose, goal)
    assert t < 60


def test_ackermann_follower_around_obstacles():
    obs = scene_obstacles(SCENE) + [MID]
    p = prof('4ws', [ACK])
    goal = (2.0, 1.5, 0.0)
    plan = plan_with_modes(p, (0.0, 0.0, 0.0), goal, obs)
    pose, _, _, f = drive_sim(p, plan, (0.0, 0.0, 0.0), goal)
    check_arrived(f, pose, goal)


@pytest.mark.parametrize('drive', ['diff', 'skid'])
def test_diff_follower_unicycle(drive):
    p = prof(drive)
    obs = scene_obstacles(SCENE) + [MID]
    start, goal = (0.0, 0.0, 0.2), (2.0, 0.0, 0.7)
    plan = plan_with_modes(p, start, goal, obs)
    pose, _, log, f = drive_sim(p, plan, start, goal)
    check_arrived(f, pose, goal)
    assert all(vy == 0.0 for _, _, vy, _ in log)
    # turning only in place: whenever it moves forward it is not rotating on its own beyond the small
    # heading correction, and whenever it rotates fast it is not translating
    assert all(abs(vx) < 0.05 for _, vx, _, wz in log if abs(wz) > 0.3)


def test_omni_follower():
    p = prof('omni')
    start, goal = (0.0, 0.0, 0.0), (2.0, 1.0, 1.2)
    plan = plan_with_modes(p, start, goal)
    pose, _, _, f = drive_sim(p, plan, start, goal)
    check_arrived(f, pose, goal)


def test_crab_follower_keeps_heading():
    p = prof('4ws', [CRAB])
    start, goal = (0.0, 0.0, 0.3), (0.0, 0.9, 0.3)
    plan = plan_with_modes(p, start, goal)
    pose, _, log, f = drive_sim(p, plan, start, goal)
    check_arrived(f, pose, goal)
    assert max(abs(wz) for *_, wz in log) < 0.05


def test_ranger_dock_switches_modes_and_logs():
    p = ranger()
    start, goal = (0.0, 0.0, 0.0), (5.0, 0.0, 0.0)
    plan = plan_with_modes(p, start, goal, dock_dir=math.pi / 2)
    calls = []

    def set_mode(old, new):
        calls.append((old, new))
        return True

    seen = []

    class Grab(logging.Handler):
        def emit(self, record):
            seen.append(record.getMessage())

    lg = logging.getLogger('stand_navigator.modes')
    old = lg.level
    lg.addHandler(Grab())
    lg.setLevel(logging.INFO)
    try:
        pose, _, log, f = drive_sim(p, plan, start, goal, set_mode)
    finally:
        lg.handlers = [h for h in lg.handlers if not isinstance(h, Grab)]
        lg.setLevel(old)
    check_arrived(f, pose, goal)
    assert calls == [('ackermann', 'crab')]
    assert [m for m in dict.fromkeys(m for m, *_ in log)] == ['ackermann', 'crab']
    assert any('mode change ackermann -> crab' in m for m in seen)
    # the last metre is sideways in crab mode: vy only, heading unchanged
    crab = [(vx, vy, wz) for m, vx, vy, wz in log if m == 'crab']
    assert max(abs(vy) for _, vy, _ in crab) > 0.3 and max(abs(wz) for *_, wz in crab) < 0.05


def test_mode_switch_waits_until_the_driver_is_ready():
    p = ranger()
    start, goal = (0.0, 0.0, 0.0), (0.0, 0.8, 0.0)
    plan = plan_with_modes(p, start, goal)
    assert plan.modes == ['crab']
    n = {'k': 0}

    def slow(old, new):
        n['k'] += 1
        return n['k'] > 50

    f = ModeFollower(p, plan, goal, TOL, slow)
    for _ in range(50):
        assert f.step((0.0, 0.0, 0.0), 0.02) == (0.0, 0.0, 0.0)
    assert f.step((0.0, 0.0, 0.0), 0.02) != (0.0, 0.0, 0.0)
    assert f.current_mode == 'crab'


def test_spin_leg_rotates_in_place_at_its_own_rate():
    p = ranger()
    start, goal = (0.0, 0.0, 0.0), (0.0, 0.0, 2.5)
    plan = plan_with_modes(p, start, goal)
    pose, t, log, f = drive_sim(p, plan, start, goal)
    check_arrived(f, pose, goal)
    assert {m for m, *_ in log} == {'spin'} and all(vx == 0.0 and vy == 0.0 for _, vx, vy, _ in log)
    assert max(abs(wz) for *_, wz in log) > 1.5          # faster than the profile's 1.5 rad/s drive-mode limit


def test_follower_is_deterministic():
    p = prof('4ws', [ACK])
    plan = plan_with_modes(p, (0, 0, 0), (2, -2, -1.5))
    a = drive_sim(p, plan, (0, 0, 0), (2, -2, -1.5))
    b = drive_sim(p, plan, (0, 0, 0), (2, -2, -1.5))
    assert a[0] == b[0] and a[1] == b[1]


def test_follower_surface_matches_other_controllers():
    p = BaseProfile(drive='diff', footprint=FP)
    plan = plan_with_modes(p, (0, 0, 0), (1, 0, 0))
    f = ModeFollower(p, plan, (1, 0, 0), TOL)
    assert f.phase and hasattr(f, 'cmd') and f.errors((0, 0, 0))[0] == pytest.approx(1.0)
    f.cmd = (0.0, 0.0, 0.0)
    assert R > 0 and SPIN.can_spin_in_place
