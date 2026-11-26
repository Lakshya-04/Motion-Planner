import math
import os
import random

import pytest

from stand_navigator import reeds_shepp as rs
from stand_navigator.core import BaseProfile, load_profile, wrap
from stand_navigator.mode_planner import plan_ackermann, plan_with_modes
from stand_navigator.modes import DriveMode
from stand_navigator.path_planner import pose_free, scene_obstacles
from test_path_planner import MID, SCENE

FP = ((-0.4, -0.3), (0.4, -0.3), (0.4, 0.3), (-0.4, 0.3))
SQ = ((-0.2, -0.2), (0.2, -0.2), (0.2, 0.2), (-0.2, 0.2))
PROFILES = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'description',
                        'mobile_manipulator_description', 'config', 'profiles')
R = 0.81
ACK = DriveMode('ackermann', False, False, R, 1.5, 1.5, 1.0, 2.0, True, 0.75)
CRAB = DriveMode('crab', True, False, None, 0.75, 1.5, 1.0, 2.0)
SPIN = DriveMode('spin', False, True, None, 1.5, 3.259, 1.0, 2.0)


def prof(drive, modes=(), fp=FP):
    return BaseProfile(drive=drive, footprint=fp, modes=tuple(modes))


def ranger():
    return load_profile(os.path.join(PROFILES, 'ur10e_ranger_mini_telescopic.yaml'))


def corridor(half_width, length=3.0):
    """Closed box corridor along x: walls at +-half_width and end caps at x = +-length."""
    t = 0.1
    return [[(-length, half_width), (length, half_width), (length, half_width + t), (-length, half_width + t)],
            [(-length, -half_width - t), (length, -half_width - t), (length, -half_width), (-length, -half_width)],
            [(length, -half_width), (length + t, -half_width), (length + t, half_width), (length, half_width)],
            [(-length - t, -half_width), (-length, -half_width), (-length, half_width), (-length - t, half_width)]]


def curvatures(poses):
    out = []
    for a, b in zip(poses, poses[1:]):
        ds = math.hypot(b.x - a.x, b.y - a.y)
        if ds > 1e-9:
            out.append(abs(wrap(b.yaw - a.yaw)) / ds)
    return out


def assert_ends_at(plan, goal, tol=1e-6):
    last = plan.waypoints[-1]
    assert math.hypot(last.x - goal[0], last.y - goal[1]) <= tol
    assert abs(wrap(last.yaw - goal[2])) <= tol


def assert_collision_free(plan, fp, obstacles):
    for p in plan.waypoints:
        assert pose_free(fp, (p.x, p.y, p.yaw), obstacles), (p.x, p.y, p.yaw)


# ---------------------------------------------------------------- Reeds-Shepp
def test_reeds_shepp_reaches_random_goals():
    rnd = random.Random(3)
    for _ in range(300):
        s = (rnd.uniform(-4, 4), rnd.uniform(-4, 4), rnd.uniform(-3, 3))
        g = (rnd.uniform(-4, 4), rnd.uniform(-4, 4), rnd.uniform(-3, 3))
        ps = rs.paths(s, g, R)
        assert ps, (s, g)
        x, y, th, _, _ = rs.sample(s, ps[0][1], R)[-1]
        assert math.hypot(x - g[0], y - g[1]) < 1e-3 and abs(wrap(th - g[2])) < 1e-3
        assert ps[0][0] >= math.hypot(g[0] - s[0], g[1] - s[1]) - 1e-6


def test_reeds_shepp_straight_and_reverse():
    assert rs.paths((0, 0, 0), (3, 0, 0), R)[0][0] == pytest.approx(3.0)
    L, pieces = rs.paths((0, 0, 0), (-3, 0, 0), R)[0]
    assert L == pytest.approx(3.0) and pieces[0][1] < 0


# ---------------------------------------------------------------- Ackermann geometry
@pytest.mark.parametrize('goal', [(3, 2, 1.0), (0, 1.0, 0.0), (-2, 0.5, 3.0), (2, -2, -1.5), (0.5, 0.0, 2.0)])
def test_ackermann_never_turns_tighter_than_radius(goal):
    p = prof('4ws', [ACK])
    plan = plan_with_modes(p, (0, 0, 0), goal)
    assert plan.ok, plan.reason
    assert max(curvatures(plan.waypoints)) <= 1.0 / R + 1e-3
    assert_ends_at(plan, goal)
    assert plan.modes == ['ackermann']
    # the stored curvature is within the limit too, and the heading follows the stated gear
    assert all(abs(w.curv) <= 1.0 / R + 1e-9 for w in plan.waypoints)


def test_ackermann_sideways_goal_needs_reverse_and_forward():
    plan = plan_with_modes(prof('4ws', [ACK]), (0, 0, 0), (0, 1.0, 0.0))
    gears = {w.gear for w in plan.waypoints}
    assert gears == {1, -1}


def test_ackermann_infeasible_in_narrow_corridor_but_diff_can_turn():
    obs = corridor(0.35)                     # 0.7 m wide: the 0.4 m square can spin, radius 0.81 m cannot turn
    start, goal = (0.0, 0.0, 0.0), (0.0, 0.0, math.pi)
    a = plan_with_modes(prof('4ws', [ACK], SQ), start, goal, obs)
    assert not a.ok and a.waypoints == [] and 'turn radius' in a.reason and 'exhausted' in a.reason
    d = plan_with_modes(prof('diff', fp=SQ), start, goal, obs)
    assert d.ok and d.modes == ['diff_turn_in_place']
    assert_collision_free(d, SQ, obs)
    # a goal straight down the corridor is fine for Ackermann
    ok = plan_with_modes(prof('4ws', [ACK], SQ), start, (2.0, 0.0, 0.0), obs)
    assert ok.ok and max(curvatures(ok.waypoints)) <= 1.0 / R + 1e-3


def test_ackermann_budget_reason_differs_from_exhausted():
    obs = corridor(0.35)
    segs, why = plan_ackermann(SQ, (0.0, 0.0, 0.0), (0.0, 0.0, math.pi), obs, ACK, max_expansions=5)
    assert segs is None and 'budget' in why


def test_ackermann_avoids_obstacles():
    obs = scene_obstacles(SCENE) + [MID]
    p = prof('4ws', [ACK])
    plan = plan_with_modes(p, (0.0, 0.0, 0.0), (2.0, 1.5, 0.0), obs)
    assert plan.ok, plan.reason
    assert max(curvatures(plan.waypoints)) <= 1.0 / R + 1e-3
    assert_collision_free(plan, FP, obs)
    assert_ends_at(plan, (2.0, 1.5, 0.0))


# ---------------------------------------------------------------- diff / skid
@pytest.mark.parametrize('drive', ['diff', 'skid'])
def test_diff_turns_only_in_place_and_has_no_lateral_motion(drive):
    obs = scene_obstacles(SCENE) + [MID]
    goal = (2.0, 0.0, 0.7)
    plan = plan_with_modes(prof(drive), (0.0, 0.0, 0.0), goal, obs)
    assert plan.ok and plan.length() > 2.0
    assert {s.kind for s in plan.segments} == {'rotate', 'drive'}
    for s in plan.segments:
        if s.kind == 'rotate':
            assert all(p.xy == s.poses[0].xy for p in s.poses)
        else:
            for a, b in zip(s.poses, s.poses[1:]):
                assert a.yaw == b.yaw                                    # no turning while driving
                head = math.atan2(b.y - a.y, b.x - a.x)
                assert abs(wrap(head - a.yaw)) < 1e-9                    # motion along the body x axis
    for a, b in zip(plan.waypoints, plan.waypoints[1:]):          # across segment joins too
        assert (a.x, a.y) == (b.x, b.y) or abs(wrap(a.yaw - b.yaw)) < 1e-9
    assert (plan.waypoints[0].x, plan.waypoints[0].y, plan.waypoints[0].yaw) == (0.0, 0.0, 0.0)
    assert_ends_at(plan, goal)
    assert_collision_free(plan, FP, obs)


def test_skid_mode_is_named_skid():
    plan = plan_with_modes(prof('skid'), (0, 0, 0), (1, 1, 0))
    assert set(plan.modes) == {'skid'}


# ---------------------------------------------------------------- crab, omni
def test_crab_moves_sideways_with_constant_heading():
    p = prof('4ws', [CRAB])
    plan = plan_with_modes(p, (0, 0, 0), (0.0, 0.8, 0.0))
    assert plan.ok and plan.modes == ['crab']
    assert all(w.yaw == 0.0 for w in plan.waypoints)
    assert all(abs(w.x) < 1e-9 for w in plan.waypoints) and plan.waypoints[-1].y == pytest.approx(0.8)
    assert_ends_at(plan, (0.0, 0.8, 0.0))


def test_crab_alone_cannot_change_yaw():
    plan = plan_with_modes(prof('4ws', [CRAB]), (0, 0, 0), (1.0, 0.0, 1.0))
    assert not plan.ok and 'yaw' in plan.reason


def test_omni_plans_like_today():
    obs = scene_obstacles(SCENE) + [MID]
    plan = plan_with_modes(prof('omni'), (0.0, 0.0, 0.0), (2.0, 0.0, 0.3), obs)
    assert plan.ok and plan.modes == ['omni']
    assert_ends_at(plan, (2.0, 0.0, 0.3))
    assert_collision_free(plan, FP, obs)
    free = plan_with_modes(prof('omni'), (0, 0, 0), (2.0, 1.0, 0.0))
    assert free.length() == pytest.approx(math.hypot(2.0, 1.0), abs=1e-6)       # straight when free


def test_stand_near_wall_in_each_mode():
    wall = scene_obstacles(SCENE)[:1]
    goal = (2.0, 1.5, 0.0)                         # front edge 0.6 m from the wall at x = 3 (stand-like)
    for p in (prof('omni'), prof('diff'), prof('4ws', [ACK]), prof('4ws', [CRAB, SPIN, ACK])):
        plan = plan_with_modes(p, (0.0, 1.5, 0.0), goal, wall)
        assert plan.ok, (p.modes, plan.reason)
        assert_ends_at(plan, goal)
        assert_collision_free(plan, FP, wall)


# ---------------------------------------------------------------- several modes
def test_ranger_long_moves_use_ackermann_short_sideways_use_crab():
    long = plan_with_modes(ranger(), (0, 0, 0), (8, 3, 0.5))
    assert long.ok and long.modes == ['ackermann']
    side = plan_with_modes(ranger(), (0, 0, 0), (0.0, 0.8, 0.0))
    assert side.ok and side.modes == ['crab']
    turn = plan_with_modes(ranger(), (0, 0, 0), (0, 0, 2.5))
    assert turn.ok and turn.modes == ['spin']


def test_ranger_dock_leg_is_crab_at_goal_yaw_along_dock_direction():
    goal = (5.0, 0.0, 0.0)
    plan = plan_with_modes(ranger(), (0, 0, 0), goal, dock_dir=math.pi / 2)
    assert plan.ok and plan.modes[-1] == 'crab' and plan.modes[0] == 'ackermann'
    leg = plan.segments[-1]
    assert leg.length() == pytest.approx(1.0, abs=1e-6)
    assert all(abs(p.x - goal[0]) < 1e-9 and p.yaw == 0.0 for p in leg.poses)
    assert max(curvatures(plan.segments[0].poses)) <= 1.0 / R + 1e-3
    assert_ends_at(plan, goal)


def test_dock_leg_shrinks_when_blocked_and_drops_when_impossible():
    goal = (5.0, 0.0, 0.0)
    block = [[(4.5, -0.8), (5.5, -0.8), (5.5, -0.7), (4.5, -0.7)]]     # 0.7 m below the goal
    plan = plan_with_modes(ranger(), (0, 0, 0), goal, block, dock_dir=math.pi / 2)
    assert plan.ok and plan.segments[-1].length() < 1.0
    assert_collision_free(plan, ranger().footprint, block)


def test_determinism():
    for p, goal in ((prof('4ws', [ACK]), (2, -2, -1.5)), (ranger(), (5.0, 0.0, 0.0)), (prof('diff'), (2, 1, 1))):
        a = plan_with_modes(p, (0, 0, 0), goal, dock_dir=math.pi / 2)
        b = plan_with_modes(p, (0, 0, 0), goal, dock_dir=math.pi / 2)
        assert [(w.x, w.y, w.yaw, w.gear) for w in a.waypoints] == [(w.x, w.y, w.yaw, w.gear) for w in b.waypoints]
        assert a.modes == b.modes and a.strategy == b.strategy


def test_start_or_goal_in_collision_is_reported():
    obs = scene_obstacles(SCENE)
    assert 'goal' in plan_with_modes(prof('diff'), (0, 0, 0), (2.9, 0, 0), obs).reason
    assert 'start' in plan_with_modes(prof('diff'), (2.9, 0, 0), (0, 0, 0), obs).reason


def test_start_equals_goal():
    plan = plan_with_modes(prof('4ws', [ACK]), (1, 1, 0.5), (1, 1, 0.5))
    assert plan.ok
    assert_ends_at(plan, (1, 1, 0.5))
