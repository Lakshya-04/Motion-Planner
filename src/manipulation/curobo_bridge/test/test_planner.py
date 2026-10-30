"""Planner tests. Pure-Python parts always run; the GPU parts need CUDA + cuRobo (the gpu image)."""
import numpy as np
import pytest

from curobo_bridge.planner import Box, PlanResult, voxelize_points


def test_voxelize_groups_points_into_cubes():
    pts = np.array([[0.01, 0.01, 0.01], [0.02, 0.03, 0.04], [0.31, 0.0, 0.0]])
    boxes = voxelize_points(pts, voxel=0.1)
    assert len(boxes) == 2
    assert all(b.size == (0.1, 0.1, 0.1) for b in boxes)
    assert boxes[0].center == pytest.approx((0.05, 0.05, 0.05))


def test_voxelize_only_grows_when_capped():
    pts = np.random.default_rng(0).uniform(0, 1, size=(5000, 3))
    boxes = voxelize_points(pts, voxel=0.02, max_boxes=100)
    assert 0 < len(boxes) <= 100
    edge = boxes[0].size[0]
    assert edge > 0.02
    for p in pts:     # every input point stays inside some box: the obstacle never shrinks
        assert any(np.all(np.abs(p - np.array(b.center)) <= edge / 2 + 1e-9) for b in boxes)


def test_voxelize_empty():
    assert voxelize_points(np.zeros((0, 3))) == []


def test_plan_result_duration():
    r = PlanResult(success=True, positions=[[0.0], [1.0], [2.0]], dt=0.5)
    assert r.duration == pytest.approx(1.0)


# ---- GPU ------------------------------------------------------------------------------------
torch = pytest.importorskip('torch')
pytest.importorskip('curobo')
needs_gpu = pytest.mark.skipif(not torch.cuda.is_available(), reason='no CUDA device')


@pytest.fixture(scope='module')
def planner():
    if not torch.cuda.is_available():
        pytest.skip('no CUDA device')
    from curobo_bridge import CuroboPlanner
    return CuroboPlanner('franka.yml')


@needs_gpu
def test_robot_info(planner):
    assert planner.dimension == 7
    assert len(planner.joint_names) == 7
    assert len(planner.default_joint_state) == 7


@needs_gpu
def test_joint_plan_reaches_the_goal(planner):
    start = planner.default_joint_state
    goal = list(np.array(start) + np.array([0.4, -0.2, 0.2, 0.1, 0.0, 0.2, 0.0]))
    r = planner.plan_joint(start, goal)
    assert r.success, r.message
    assert r.joint_names == planner.joint_names
    assert len(r.positions) > 2 and r.dt > 0
    assert np.allclose(r.positions[0], start, atol=1e-2)
    assert np.allclose(r.positions[-1], goal, atol=2e-2)


@needs_gpu
def test_pose_plan_succeeds_with_a_box_in_the_world(planner):
    planner.set_world([Box('b', (0.5, 0.35, 0.3), (0.1, 0.1, 0.4))])
    r = planner.plan_pose(planner.default_joint_state, (0.5, 0.0, 0.3))
    planner.clear_world()
    assert r.success, r.message
    assert r.duration > 0


@needs_gpu
def test_start_inside_an_obstacle_is_refused_structurally(planner):
    n = planner.set_world([Box('swallow', (0.0, 0.0, 0.4), (3.0, 3.0, 3.0))])
    assert n == 1
    r = planner.plan_joint(planner.default_joint_state, list(np.array(planner.default_joint_state) + 0.1))
    planner.clear_world()
    assert not r.success
    assert r.failure_reason_code == 'no_path' and r.failure_stage == 'planning'


@needs_gpu
def test_wrong_dimension_is_an_invalid_request(planner):
    r = planner.plan_joint([0.0, 0.0], [0.1, 0.1])
    assert not r.success
    assert r.failure_reason_code == 'invalid_request' and r.failure_stage == 'precheck'


@needs_gpu
def test_points_become_obstacles_and_too_many_boxes_are_an_error(planner):
    pts = np.random.default_rng(1).uniform([0.4, -0.5, 0.0], [0.6, 0.5, 0.2], size=(3000, 3))
    n = planner.set_world(points=pts, point_voxel=0.02)
    assert 0 < n <= planner.max_boxes
    too_many = [Box(f'b{i}', (2.0, 2.0, 2.0), (0.01, 0.01, 0.01)) for i in range(planner.max_boxes + 1)]
    with pytest.raises(ValueError):
        planner.set_world(too_many)
    planner.clear_world()
