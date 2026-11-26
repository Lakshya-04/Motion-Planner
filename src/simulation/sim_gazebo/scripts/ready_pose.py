#!/usr/bin/env python3
"""Moves the arm to the UR10e ready pose of the MuJoCo manager pass (sim_mujoco smoke test FREE: tool pointing at the
wall, a short way off it) through the running controller's action and prints the tool pose reached in
base_footprint: `x y z qx qy qz qw`. The manager pass starts from here instead of the home pose.
Needs a launched sim (profile ur10e_telescopic) with a controller.

`--joints lift,pan,shoulder_lift,elbow,wrist_1,wrist_2,wrist_3` (the order of FREE) moves to another joint pose and
`--seconds S` sets the move time (default 8 s). `--planned` plans instead of interpolating in joint space: move_group
plans the joint goal with the robot and the wall in its scene (plan only), the controller executes the trajectory
(`--scale` is the velocity and acceleration scale). The spray pass seeds its first move this way (sim_mujoco's
ready_pose.py does the same): the stowed arm folded against the lift cannot interpolate straight to the stroke pose.
"""
import argparse
import math
import os
import sys

import rclpy
import rclpy.time
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions
from rclpy.action import ActionClient

from sim_gazebo import profile as prof
import smoke_test as st

PROFILE = 'ur10e_telescopic'
# lift + six UR10e joints, the same numbers as sim_mujoco's smoke test (the kinematics are the same description)
FREE = [0.4, -0.0363, 0.2528, 1.1964, 1.6924, -1.5345, 1.5708]


def planned_trajectory(node, joints, target, scale, group):
    """Plan-only MoveGroup goal on `group` (the profile's whole-body group; collision-checked, from the live state); the JointTrajectory or None."""
    client = ActionClient(node, MoveGroup, '/move_action')
    if not client.wait_for_server(timeout_sec=30.0):
        print('ready_pose: no /move_action server', file=sys.stderr)
        return None
    req = MotionPlanRequest()
    req.group_name = group
    req.num_planning_attempts = 5
    req.allowed_planning_time = 10.0
    req.max_velocity_scaling_factor = scale
    req.max_acceleration_scaling_factor = scale
    goal = Constraints()
    for name, q in zip(joints, target):
        goal.joint_constraints.append(JointConstraint(joint_name=name, position=float(q), tolerance_above=1e-3,
                                                      tolerance_below=1e-3, weight=1.0))
    req.goal_constraints.append(goal)
    g = MoveGroup.Goal()
    g.request = req
    g.planning_options = PlanningOptions()
    g.planning_options.plan_only = True
    fut = client.send_goal_async(g)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
    handle = fut.result()
    if handle is None or not handle.accepted:
        print('ready_pose: plan goal rejected', file=sys.stderr)
        return None
    res = handle.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=60.0)
    out = res.result()
    if out is None or out.result.error_code.val != 1:
        print(f'ready_pose: planning failed, MoveIt error {None if out is None else out.result.error_code.val}',
              file=sys.stderr)
        return None
    traj = out.result.planned_trajectory.joint_trajectory
    last = traj.points[-1].time_from_start
    print(f'ready_pose: planned {len(traj.points)} points, {last.sec + last.nanosec * 1e-9:.1f} s', flush=True)
    return traj


def execute(node, traj):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    fut = node.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    handle = fut.result()
    if handle is None or not handle.accepted:
        return False
    res = handle.get_result_async()
    last = traj.points[-1].time_from_start
    rclpy.spin_until_future_complete(node, res, timeout_sec=(last.sec + last.nanosec * 1e-9) * 6 + 30)
    return res.result() is not None and res.result().result.error_code == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--controller', default='combined_arm_lift_controller')
    ap.add_argument('--profile', default=PROFILE, help='robot profile (joint order and planning group)')
    ap.add_argument('--group', default='', help='MoveIt group of --planned; empty = the profile whole-body group')
    ap.add_argument('--seconds', type=float, default=8.0)
    ap.add_argument('--joints', help='7 comma separated joint values in the order of FREE')
    ap.add_argument('--planned', action='store_true', help='plan with move_group (collision-checked) instead of a joint move')
    ap.add_argument('--scale', type=float, default=0.3, help='velocity and acceleration scale of --planned')
    ap.add_argument('--settle', type=float, default=20.0, help='sim seconds to wait after the goal')
    args = ap.parse_args()
    target = FREE
    if args.joints:
        target = [float(v) for v in args.joints.split(',')]
        if len(target) != len(FREE):
            print(f'ready_pose: --joints needs {len(FREE)} values', file=sys.stderr)
            os._exit(1)
    profile = prof.load_profile(args.profile)
    joints = prof.controlled_joints(profile)
    rclpy.init()
    node = st.Probe(joints, args.controller)
    if not node.spin_until(lambda: node.sim_time is not None and not any(math.isnan(v) for v in node.positions())
                           and node.client.server_is_ready(), 90):
        print('ready_pose: no clock / joint states / action server', file=sys.stderr)
        os._exit(1)
    if args.planned:
        traj = planned_trajectory(node, joints, target, args.scale, args.group or profile.whole_body_group)
        ok = traj is not None and execute(node, traj)
    else:
        ok = node.send(target, args.seconds)
    node.sim_wait(args.settle)   # gz's position loop (jtac_position_only) settles slowly after the goal
    t = node.tf.lookup_transform('base_footprint', 'tcp_link', rclpy.time.Time()).transform
    print(f'ready_pose: ok {ok} tcp {t.translation.x:.4f} {t.translation.y:.4f} {t.translation.z:.4f} '
          f'q {t.rotation.x:.4f} {t.rotation.y:.4f} {t.rotation.z:.4f} {t.rotation.w:.4f}', flush=True)
    os._exit(0 if ok else 2)


if __name__ == '__main__':
    main()
