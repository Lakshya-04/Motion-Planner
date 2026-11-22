import math

import pytest

from stand_navigator.core import BaseProfile, Tolerance, load_profile, wrap
from stand_navigator.path_planner import (
    PathFollower, footprint_corners, point_polygon_dist, plan_path, pose_free, pose_sequence, scene_obstacles,
    straight_line_free, time_parametrize)

FP = ((-0.4, -0.3), (0.4, -0.3), (0.4, 0.3), (-0.4, 0.3))
SCENE = {
    'walls': [{'id': 0, 'plane_abcd': [-1, 0, 0, 3.0], 'origin_xyz': [3.0, 0.0, 1.0], 'basis_u': [0, 1, 0],
               'normal_outward': [-1, 0, 0], 'u_range_m': [-3.0, 3.0], 'v_range_m': [0, 2]}],
    'obstacles': [{'id': 0, 'parent_wall': 0, 'protrusion_max_m': 0.3, 'uv_on_wall': {'u': [-0.4, 0.4]}}],
}
MID = [(1.0, -0.4), (1.4, -0.4), (1.4, 0.4), (1.0, 0.4)]        # free-standing box between start and goal


def prof(drive='omni'):
    return BaseProfile(drive=drive, footprint=FP)


def test_scene_obstacles_wall_and_box():
    obs = scene_obstacles(SCENE)
    assert len(obs) == 2
    xs = [p[0] for p in obs[1]]
    assert min(xs) == pytest.approx(2.7) and max(xs) == pytest.approx(3.0)     # protrudes toward -x


def test_horizontal_plane_ignored():
    assert scene_obstacles({'walls': [{'plane_abcd': [0, 0, 1, 0], 'origin_xyz': [0, 0, 0], 'basis_u': [1, 0, 0]}]}) == []


def test_pose_free_and_footprint_yaw():
    obs = scene_obstacles(SCENE)
    assert pose_free(FP, (1.0, 0.0, 0.0), obs)
    assert not pose_free(FP, (2.5, 0.0, 0.0), obs)        # front edge 2.9 inside the box (2.7)
    assert len(footprint_corners(FP, (0, 0, 1.0))) == 4


def test_straight_free_without_obstacles_and_blocked_by_box():
    obs = scene_obstacles(SCENE) + [MID]
    assert straight_line_free(True, FP, (0, 1.5, 0), (2.0, 1.5, 0), obs)
    assert not straight_line_free(True, FP, (0, 0, 0), (2.0, 0, 0), obs)
    assert straight_line_free(True, FP, (0, 0, 0), (1.0, 0, 0), [])


def test_stand_close_to_wall_is_still_reachable_straight():
    obs = scene_obstacles(SCENE)[:1]               # wall only; footprint front 0.55 m from the wall
    assert straight_line_free(False, FP, (0.0, 1.5, 0.0), (2.05, 1.5, 0.0), obs)


def test_plan_goes_around_box_and_is_collision_free():
    obs = scene_obstacles(SCENE) + [MID]
    start, goal = (0.0, 0.0, 0.0), (2.0, 0.0, 0.0)
    r = plan_path(FP, start, goal, obs)
    assert r.ok and r.length > r.straight and len(r.path) >= 3
    assert r.path[0] == (0.0, 0.0) and r.path[-1] == (2.0, 0.0)
    # every sample on the path keeps the circumscribed disc (0.5 m) off the obstacles, except near the ends
    for a, b in zip(r.path, r.path[1:]):
        for k in range(21):
            x, y = a[0] + (b[0] - a[0]) * k / 20, a[1] + (b[1] - a[1]) * k / 20
            if math.hypot(x - 2.0, y) > 0.7 and math.hypot(x, y) > 0.7:
                assert pose_free(FP, (x, y, math.atan2(b[1] - a[1], b[0] - a[0])), obs)


def test_plan_reports_blocked_goal_and_no_path():
    obs = scene_obstacles(SCENE)
    assert not plan_path(FP, (0, 0, 0), (2.8, 0.0, 0.0), obs).ok            # goal inside the wall
    ring = [[(0.9, -1.2), (1.0, -1.2), (1.0, 1.2), (0.9, 1.2)], [(0.9, 1.1), (3.0, 1.1), (3.0, 1.2), (0.9, 1.2)],
            [(0.9, -1.2), (3.0, -1.2), (3.0, -1.1), (0.9, -1.1)]]
    r = plan_path(FP, (0, 0, 0), (2.0, 0.0, 0), obs + ring)
    assert not r.ok and 'path' in r.reason


def test_time_parametrize_limits():
    p = prof()
    poses = pose_sequence([(0, 0), (1, 0), (1, 1)], 0.0, 1.5, True)
    seq = time_parametrize(poses, p, 0.05)
    assert seq[0][0] == 0 and seq[-1][1:3] == pytest.approx((1, 1))
    for a, b in zip(seq, seq[1:]):
        dt = b[0] - a[0]
        assert math.hypot(b[1] - a[1], b[2] - a[2]) / dt <= p.max_lin * 1.02
        assert abs(wrap(b[3] - a[3])) / dt <= p.max_ang * 1.02
    v = [math.hypot(b[1] - a[1], b[2] - a[2]) / (b[0] - a[0]) for a, b in zip(seq, seq[1:])]
    assert all(abs(v2 - v1) / 0.05 <= p.max_lin_acc * 1.1 for v1, v2 in zip(v, v[1:]))


@pytest.mark.parametrize('drive', ['omni', 'diff'])
def test_follower_drives_planned_path_to_goal(drive):
    p = prof(drive)
    obs = scene_obstacles(SCENE) + [MID]
    start, goal = (0.0, 0.0, 0.0), (2.0, 0.0, 0.3)
    r = plan_path(FP, start, goal, obs)
    f = PathFollower(p, r.path, goal, Tolerance())
    x, y, yaw = start
    dt, t, trav = 0.02, 0.0, 0.0
    while t < 120:
        vx, vy, wz = f.step((x, y, yaw), dt)
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = (c * vx - s * vy) * dt, (s * vx + c * vy) * dt
        x, y, yaw, t, trav = x + dx, y + dy, wrap(yaw + wz * dt), t + dt, trav + math.hypot(dx, dy)
        assert point_polygon_dist(x, y, MID) > 0.3      # the base centre keeps clear of the box
        if f.arrived((x, y, yaw), math.hypot(vx, vy), abs(wz)):
            break
    assert f.arrived((x, y, yaw), 0, 0), (x, y, yaw, t)
    assert trav < r.length * 1.3 + 0.3


def test_profile_loads_footprint(tmp_path):
    f = tmp_path / 'p.yaml'
    f.write_text('base:\n  drive_type: diff\n  footprint: [[-0.4,-0.3],[0.4,-0.3],[0.4,0.3],[-0.4,0.3]]\n')
    assert load_profile(str(f)).footprint[2] == (0.4, 0.3)
