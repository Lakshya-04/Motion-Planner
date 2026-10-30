"""A ROS-free planner over cuRobo v2.

Mirrors the structured results of the motion_planner ports (success, failure code, failure stage,
message) so a C++ client or a ROS node can adapt it without parsing text. cuRobo itself is imported
lazily: importing this module works without a GPU stack, constructing a CuroboPlanner needs one.

Frames: everything is in the robot base frame, the same convention as the VAMP adapter. Poses are
[x, y, z] + quaternion [w, x, y, z] (cuRobo's order).
"""
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class Box:
    """An oriented box obstacle; `size` is the full extent along each local axis."""
    id: str
    center: Tuple[float, float, float]
    size: Tuple[float, float, float]
    quat_wxyz: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


@dataclass
class PlanResult:
    success: bool = False
    failure_reason_code: str = 'none'
    failure_stage: str = 'none'          # none | precheck | planning
    message: str = ''
    joint_names: List[str] = field(default_factory=list)
    positions: List[List[float]] = field(default_factory=list)   # one row per waypoint
    dt: float = 0.0                       # seconds between rows (cuRobo's interpolation step)
    planning_time_s: float = 0.0

    @property
    def duration(self) -> float:
        return self.dt * max(0, len(self.positions) - 1)


def voxelize_points(points: np.ndarray, voxel: float = 0.05, max_boxes: int = 2000) -> List[Box]:
    """Turn a point cloud into occupied-voxel boxes (cuRobo's scene has no raw point cloud type).

    Each occupied voxel becomes one cube of edge `voxel`. If that exceeds `max_boxes` the voxel size
    is doubled until it fits, so the obstacle only grows (conservative), never shrinks. For dense
    sensing use cuRobo's own ESDF mapper (see docs/curobo_v2_notes.md); this is the bridge for the
    sparse, already-filtered clouds the scene model hands over.
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if len(pts) == 0:
        return []
    size = float(voxel)
    for _ in range(12):
        cells = np.unique(np.floor(pts / size).astype(np.int64), axis=0)
        if len(cells) <= max_boxes:
            break
        size *= 2.0
    return [Box(f'pc_{i}', tuple(((c + 0.5) * size).tolist()), (size, size, size)) for i, c in enumerate(cells)]


class CuroboPlanner:
    """Joint-goal and pose-goal planning with cuRobo v2 on the GPU."""

    def __init__(self, robot: str = 'franka.yml', max_goalset: int = 1, max_boxes: int = 256,
                 warmup: bool = True):
        import torch
        from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
        from curobo.types import JointState

        if not torch.cuda.is_available():
            raise RuntimeError('cuRobo needs a CUDA device')
        self._torch = torch
        self._JointState = JointState
        # cuRobo allocates the obstacle cache up front; update_world can then swap scenes up to this size.
        self.max_boxes = max_boxes
        self._planner = MotionPlanner(MotionPlannerCfg.create(
            robot=robot, max_goalset=max_goalset, collision_cache={'cuboid': max_boxes}))
        if warmup:
            self._planner.warmup(enable_graph=True, num_warmup_iterations=3)
        self._world_set = False

    # ---- robot info -------------------------------------------------------------------------
    @property
    def joint_names(self) -> List[str]:
        return list(self._planner.joint_names)

    @property
    def dimension(self) -> int:
        return len(self.joint_names)

    @property
    def tool_frames(self) -> List[str]:
        return list(self._planner.tool_frames)

    @property
    def default_joint_state(self) -> List[float]:
        return self._planner.default_joint_state.position.detach().cpu().numpy().tolist()

    # ---- world ------------------------------------------------------------------------------
    def set_world(self, boxes: Sequence[Box] = (), points: Optional[np.ndarray] = None,
                  point_voxel: float = 0.05) -> int:
        """Replaces the world. Returns the number of boxes handed to cuRobo.

        Points are voxelized into whatever room the explicit boxes leave in the obstacle cache
        (`max_boxes`); explicit boxes beyond the cache are an error, not silently dropped.
        """
        from curobo.scene import Cuboid, Scene

        all_boxes = list(boxes)
        if len(all_boxes) > self.max_boxes:
            raise ValueError(f'{len(all_boxes)} boxes exceed the obstacle cache ({self.max_boxes})')
        if points is not None and len(points) > 0:
            all_boxes += voxelize_points(points, point_voxel, self.max_boxes - len(all_boxes))
        cuboids = [Cuboid(name=b.id, dims=list(b.size), pose=[*b.center, *b.quat_wxyz]) for b in all_boxes]
        self._planner.update_world(Scene(cuboid=cuboids))
        self._world_set = True
        return len(cuboids)

    def clear_world(self) -> None:
        from curobo.scene import Scene
        self._planner.update_world(Scene())
        self._world_set = False

    # ---- planning ---------------------------------------------------------------------------
    def plan_joint(self, start: Sequence[float], goal: Sequence[float]) -> PlanResult:
        """Collision-free joint-space move start -> goal."""
        bad = self._check_state(start, 'start') or self._check_state(goal, 'goal')
        if bad:
            return bad
        cur, tgt = self._state(start), self._state(goal)
        result = self._planner.plan_cspace(tgt, cur)
        return self._wrap(result, last=goal)

    def plan_pose(self, start: Sequence[float], position: Sequence[float],
                  quat_wxyz: Sequence[float] = (1.0, 0.0, 0.0, 0.0)) -> PlanResult:
        """Collision-free move to a tool pose (position + quaternion wxyz, robot base frame)."""
        bad = self._check_state(start, 'start')
        if bad:
            return bad
        from curobo.types import GoalToolPose
        t = self._torch
        goal = GoalToolPose(
            tool_frames=self._planner.tool_frames,
            position=t.tensor([[[[list(position)]]]], device='cuda', dtype=t.float32),
            quaternion=t.tensor([[[[list(quat_wxyz)]]]], device='cuda', dtype=t.float32))
        return self._wrap(self._planner.plan_pose(goal, self._state(start)))

    # ---- internals --------------------------------------------------------------------------
    def _state(self, q: Sequence[float]):
        t = self._torch
        return self._JointState.from_position(
            t.tensor([list(q)], device='cuda', dtype=t.float32), joint_names=self.joint_names)

    def _check_state(self, q: Sequence[float], what: str) -> Optional[PlanResult]:
        if len(q) != self.dimension:
            return PlanResult(failure_reason_code='invalid_request', failure_stage='precheck',
                              message=f'{what} must have {self.dimension} joint values, got {len(q)}')
        return None

    def _wrap(self, result, last: Optional[Sequence[float]] = None) -> PlanResult:
        out = PlanResult(joint_names=self.joint_names)
        if result is None or not bool(result.success.any()):
            out.failure_reason_code = 'no_path'
            out.failure_stage = 'planning'
            out.message = 'cuRobo found no collision-free trajectory'
            if result is not None and getattr(result, 'total_time', None) is not None:
                out.planning_time_s = float(result.total_time)
            return out
        plan = result.get_interpolated_plan()
        rows = plan.position.reshape(-1, plan.position.shape[-1]).detach().cpu().numpy()
        # The plan also carries locked joints (e.g. gripper fingers); keep the active ones, in order.
        cols = [list(plan.joint_names).index(n) for n in self.joint_names] if plan.joint_names else None
        out.positions = (rows[:, cols] if cols else rows).tolist()
        out.dt = float(self._planner.trajopt_solver.config.interpolation_dt)
        out.planning_time_s = float(getattr(result, 'total_time', 0.0) or 0.0)
        out.success = True
        out.failure_stage = 'none'
        return out
