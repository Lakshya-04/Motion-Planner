#!/usr/bin/env python3
"""End-to-end smoke test for the MoveIt config: needs `demo.launch.py` running.

For the 7-DOF `lift_arm` group and the 6-DOF `arm` group: FK of the current state, IK for a pose
8 cm away, then a collision-free plan (plan only) to that joint solution through move_group.
Exits non-zero with a message if any step fails.
"""
import sys

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions
from moveit_msgs.srv import GetPositionFK, GetPositionIK
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

TIMEOUT = 60.0


class Smoke(Node):

    def __init__(self):
        super().__init__('smoke_plan')
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.move = ActionClient(self, MoveGroup, '/move_action')

    def wait(self):
        for client, name in ((self.fk, 'compute_fk'), (self.ik, 'compute_ik')):
            if not client.wait_for_service(timeout_sec=TIMEOUT):
                raise RuntimeError(f'{name} is not available (is demo.launch.py running?)')
        if not self.move.wait_for_server(timeout_sec=TIMEOUT):
            raise RuntimeError('move_action is not available')

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=TIMEOUT)
        if future.result() is None:
            raise RuntimeError('service call timed out')
        return future.result()

    def check_group(self, group, tip='tcp_link', base='base_link'):
        fk_req = GetPositionFK.Request()
        fk_req.header.frame_id = base
        fk_req.fk_link_names = [tip]
        fk_req.robot_state.is_diff = True
        fk = self.call(self.fk, fk_req)
        if fk.error_code.val != 1 or not fk.pose_stamped:
            raise RuntimeError(f'[{group}] FK failed: {fk.error_code.val}')
        pose = fk.pose_stamped[0]
        pose.pose.position.x += 0.08  # a reachable nearby target

        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = group
        ik_req.ik_request.pose_stamped = pose
        ik_req.ik_request.ik_link_name = tip
        ik_req.ik_request.avoid_collisions = True
        ik_req.ik_request.timeout.sec = 1
        ik = self.call(self.ik, ik_req)
        if ik.error_code.val != 1:
            raise RuntimeError(f'[{group}] IK failed: {ik.error_code.val}')

        names = ik.solution.joint_state.name
        values = ik.solution.joint_state.position
        constraints = Constraints()
        group_joints = self.group_joints(group)
        for n, v in zip(names, values):
            if n in group_joints:
                c = JointConstraint(joint_name=n, position=v, tolerance_above=1e-3,
                                    tolerance_below=1e-3, weight=1.0)
                constraints.joint_constraints.append(c)

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = group
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.request.goal_constraints = [constraints]
        goal.planning_options = PlanningOptions(plan_only=True)
        send = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=TIMEOUT)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f'[{group}] plan goal rejected')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=TIMEOUT)
        result = result_future.result().result
        if result.error_code.val != 1:
            raise RuntimeError(f'[{group}] planning failed: {result.error_code.val}')
        points = result.planned_trajectory.joint_trajectory.points
        if len(points) < 2:
            raise RuntimeError(f'[{group}] empty trajectory')
        duration = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9
        print(f'[{group}] OK: IK solved, {len(points)} waypoints, {duration:.2f} s, '
              f'{len(result.planned_trajectory.joint_trajectory.joint_names)} joints')

    @staticmethod
    def group_joints(group):
        lift = ['lift_joint']
        return set(lift + [f'arm_{n}' for n in (
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
            'wrist_2_joint', 'wrist_3_joint', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5',
            'joint6')]) - (set() if group == 'lift_arm' else {'lift_joint'})


def main():
    rclpy.init()
    node = Smoke()
    try:
        node.wait()
        node.check_group('lift_arm')
        node.check_group('arm')
    except RuntimeError as error:
        print(f'FAIL: {error}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print('SMOKE OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
