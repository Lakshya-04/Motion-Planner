#!/usr/bin/env python3
"""Drives JtacController (jtac_controller_test.launch.py) through: a free-space move (proving
FollowJointTrajectory execution works, the same action MoveIt would call), then a hold phase
during which a mock wall-contact wrench is injected on ft_bridge_hardware's injection topic,
observing the arm deflect and relax -- the same Hooke-equilibrium physics
test_admittance_rule_integration.cpp (T4a) already proved, now exercised live under a real
controller_manager instead of a gtest.

Usage:
    ros2 run compliant_trajectory_controller send_test_trajectory_and_inject_wrench.py
"""
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINT_NAMES = [
    "lift_prismatic_joint", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# Same reference pose validated earlier this session (pose_conditioning_probe: condition number
# 17.5 at link_tcp; reference_pose_collision_probe: collision-free with 3x margin in both push
# directions, all joints within their real URDF limits) -- reused here rather than re-derived.
VALIDATED_POSE = [0.40, 0.0, -0.15, -0.15, 0.0, -0.15, 0.0]
# A further joint-space nudge from VALIDATED_POSE, mimicking "advance toward the wall" -- not a
# precise Cartesian move (this plain rclpy script has no kinematics_interface bindings), just
# something with a genuine Cartesian-Z component at this pose for admittance's selected Z axis to
# react to.
APPROACH_POSE = [0.40, 0.0, -0.35, -0.15, 0.0, -0.15, 0.0]

INJECTION_TOPIC = "/ft_bridge_hardware_uf_ft_sensor/injected_wrench"
INJECTION_FRAME = "link_tcp"


class TestDriver(Node):
    def __init__(self):
        super().__init__("jtac_test_driver")
        self._client = ActionClient(
            self, FollowJointTrajectory, "combined_arm_lift_controller/follow_joint_trajectory")
        self._wrench_pub = self.create_publisher(WrenchStamped, INJECTION_TOPIC, 10)
        self._latest_joint_state = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

    def _on_joint_state(self, msg):
        self._latest_joint_state = msg

    def waitForJointState(self, timeout_s=10.0):
        deadline = time.time() + timeout_s
        while self._latest_joint_state is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._latest_joint_state

    def sendTrajectory(self, target_positions, duration_s):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(JOINT_NAMES)
        point.time_from_start.sec = int(duration_s)
        point.time_from_start.nanosec = int((duration_s % 1.0) * 1e9)
        goal.trajectory.points = [point]

        self.get_logger().info(f"Waiting for action server...")
        self._client.wait_for_server()
        self.get_logger().info(f"Sending trajectory to {target_positions} over {duration_s}s")
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_s + 5.0)
        result = result_future.result()
        if result is None:
            self.get_logger().error("Timed out waiting for trajectory result.")
            return False
        ok = result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info(f"Trajectory finished: error_code={result.result.error_code}")
        return ok

    def publishWrench(self, force_z, duration_s, rate_hz=50.0):
        msg = WrenchStamped()
        msg.header.frame_id = INJECTION_FRAME
        msg.wrench.force.z = force_z
        n_ticks = int(duration_s * rate_hz)
        for _ in range(n_ticks):
            msg.header.stamp = self.get_clock().now().to_msg()
            self._wrench_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=1.0 / rate_hz)

    def printJointPositions(self, label):
        js = self.waitForJointState(timeout_s=2.0)
        if js is None:
            self.get_logger().warn(f"{label}: no /joint_states received")
            return
        by_name = dict(zip(js.name, js.position))
        ordered = [by_name.get(n, float("nan")) for n in JOINT_NAMES]
        self.get_logger().info(f"{label}: {['%.4f' % p for p in ordered]}")


def main():
    rclpy.init()
    node = TestDriver()

    node.get_logger().info("=== Phase 1: free-space move (regression check, no force) ===")
    node.printJointPositions("before move")
    if not node.sendTrajectory(VALIDATED_POSE, 4.0):
        node.get_logger().error("Phase 1 failed -- aborting.")
        rclpy.shutdown()
        sys.exit(1)
    time.sleep(0.5)
    node.printJointPositions("after move to VALIDATED_POSE")

    node.get_logger().info("=== Phase 2: approach (advance further, still no force) ===")
    if not node.sendTrajectory(APPROACH_POSE, 3.0):
        node.get_logger().error("Phase 2 failed -- aborting.")
        rclpy.shutdown()
        sys.exit(1)
    time.sleep(0.5)
    node.printJointPositions("after move to APPROACH_POSE (commanded)")

    node.get_logger().info(
        "=== Phase 3: inject a -6N wall-contact wrench (link_tcp +Z), hold 4s ===")
    node.publishWrench(force_z=-6.0, duration_s=4.0)
    node.printJointPositions("WHILE loaded (should differ from APPROACH_POSE if compliant)")

    node.get_logger().info("=== Phase 4: release the wrench, let the spring relax ===")
    node.publishWrench(force_z=0.0, duration_s=3.0)
    node.printJointPositions("after release (should relax back toward APPROACH_POSE)")

    node.get_logger().info("Done.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
