"""ROS 2 shell over CuroboPlanner: services for joint / pose planning and the world.

Parameters: `robot` (cuRobo robot config: a bundled name like franka.yml or an absolute path to one
made by docker/tools/generate_curobo_robot.py), `max_boxes` (obstacle cache size). Services (under
the node name): ~/plan_joint, ~/plan_pose, ~/set_world. Everything is in the robot base frame.

The planner is built lazily-but-once at start-up (CUDA kernels compile on the first warmup, which
can take a minute); until then the services answer `unavailable` instead of blocking callers.
"""
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from curobo_bridge_msgs.srv import PlanJoint, PlanPose, SetWorld
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from curobo_bridge.planner import Box, CuroboPlanner, PlanResult


def _duration(seconds: float) -> Duration:
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


def to_trajectory(result: PlanResult) -> JointTrajectory:
    traj = JointTrajectory(joint_names=list(result.joint_names))
    for i, row in enumerate(result.positions):
        traj.points.append(JointTrajectoryPoint(positions=[float(v) for v in row],
                                                time_from_start=_duration(i * result.dt)))
    return traj


def fill(response, result: PlanResult):
    response.success = result.success
    response.failure_reason_code = result.failure_reason_code
    response.failure_stage = result.failure_stage
    response.message = result.message
    response.trajectory = to_trajectory(result)
    response.planning_time_s = result.planning_time_s
    return response


class CuroboPlannerNode(Node):

    def __init__(self, planner=None, **node_kwargs):
        super().__init__('curobo_planner', **node_kwargs)
        robot = self.declare_parameter('robot', 'franka.yml').value
        max_boxes = int(self.declare_parameter('max_boxes', 256).value)
        self._planner = planner
        if self._planner is None:
            try:
                self._planner = CuroboPlanner(robot, max_boxes=max_boxes)
                self.get_logger().info(
                    f'cuRobo ready: {self._planner.dimension} joints, tool {self._planner.tool_frames}')
            except Exception as e:  # noqa: BLE001 - report, keep serving "unavailable"
                self.get_logger().error(f'cuRobo planner unavailable: {e}')
        self.create_service(PlanJoint, '~/plan_joint', self._plan_joint)
        self.create_service(PlanPose, '~/plan_pose', self._plan_pose)
        self.create_service(SetWorld, '~/set_world', self._set_world)

    def _unavailable(self, response):
        return fill(response, PlanResult(failure_reason_code='unavailable', failure_stage='precheck',
                                         message='cuRobo planner is not running'))

    def _plan_joint(self, request, response):
        if self._planner is None:
            return self._unavailable(response)
        return fill(response, self._planner.plan_joint(list(request.start), list(request.goal)))

    def _plan_pose(self, request, response):
        if self._planner is None:
            return self._unavailable(response)
        p, q = request.goal.position, request.goal.orientation
        return fill(response, self._planner.plan_pose(
            list(request.start), (p.x, p.y, p.z), (q.w, q.x, q.y, q.z)))   # ROS xyzw -> cuRobo wxyz

    def _set_world(self, request, response):
        if self._planner is None:
            response.message = 'cuRobo planner is not running'
            return response
        boxes = [Box(b.id, (b.pose.position.x, b.pose.position.y, b.pose.position.z), tuple(b.size),
                     (b.pose.orientation.w, b.pose.orientation.x, b.pose.orientation.y,
                      b.pose.orientation.z)) for b in request.boxes]
        points = None
        if request.points.width * request.points.height > 0:
            raw = point_cloud2.read_points(request.points, field_names=('x', 'y', 'z'), skip_nans=True)
            points = np.column_stack([raw['x'], raw['y'], raw['z']]) if len(raw) else None
        try:
            response.box_count = self._planner.set_world(boxes, points, request.point_voxel or 0.05)
            response.success = True
        except ValueError as e:
            response.message = str(e)
        return response


def main():
    rclpy.init()
    node = CuroboPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
