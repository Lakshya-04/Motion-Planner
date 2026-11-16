#!/usr/bin/env python3
"""Moves the arm to the smoke test's FREE pose (tool pointing at the wall, a short way off it) through the
controller's action and prints the tool pose reached in base_footprint: `x y z qx qy qz qw`. The manager pass
starts from here instead of the stowed pose, as the scripted wall pass does.

`--joints lift,pan,shoulder_lift,elbow,wrist_1,wrist_2,wrist_3` (the order of smoke_test.FREE) moves to another
joint pose instead and `--duration S` sets the move time (default 8 s): the spray pass seeds the first move with the
arm configuration of its first stroke, which is far from the wall (FREE is 3 cm from it), see run_spray_pass.sh.
`--planned` does not interpolate in joint space (no collision checking, the arm folded at the stow pose hits the lift or
itself): move_group plans the joint goal with the wall and the robot in its scene (plan only), the controller executes it."""
import argparse
import os
import sys
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions
from rclpy.action import ActionClient

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from smoke_test import FREE, JOINTS  # noqa: E402
from wall_pass_test import WallPass  # noqa: E402


def planned_move(node, target, scale):
    """Plan-only MoveGroup goal on lift_arm (collision-checked, from the live state), executed through the controller."""
    client = ActionClient(node, MoveGroup, '/move_action')
    if not client.wait_for_server(timeout_sec=30.0):
        print('ready_pose: no /move_action server', file=sys.stderr)
        return 3
    req = MotionPlanRequest()
    req.group_name = 'lift_arm'
    req.num_planning_attempts = 5
    req.allowed_planning_time = 10.0
    req.max_velocity_scaling_factor = scale
    req.max_acceleration_scaling_factor = scale
    goal = Constraints()
    for name, q in zip(JOINTS, target):
        goal.joint_constraints.append(JointConstraint(joint_name=name, position=float(q), tolerance_above=1e-3,
                                                      tolerance_below=1e-3, weight=1.0))
    req.goal_constraints.append(goal)
    g = MoveGroup.Goal()
    g.request = req
    g.planning_options = PlanningOptions()
    g.planning_options.plan_only = True
    fut = client.send_goal_async(g)
    while not fut.done():
        time.sleep(0.02)
    handle = fut.result()
    if not handle.accepted:
        print('ready_pose: plan goal rejected', file=sys.stderr)
        return 4
    res = handle.get_result_async()
    while not res.done():
        time.sleep(0.02)
    result = res.result().result
    if result.error_code.val != 1:
        print(f'ready_pose: planning failed, MoveIt error {result.error_code.val}', file=sys.stderr)
        return 5
    traj = result.planned_trajectory.joint_trajectory
    print(f'ready_pose: planned {len(traj.points)} points, '
          f'{traj.points[-1].time_from_start.sec + traj.points[-1].time_from_start.nanosec * 1e-9:.1f} s', flush=True)
    return node.execute(traj)


def execute(node, traj):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    fut = node.client.send_goal_async(goal)
    while not fut.done():
        time.sleep(0.01)
    handle = fut.result()
    if not handle.accepted:
        return 6
    res = handle.get_result_async()
    while not res.done():
        time.sleep(0.01)
    return res.result().result.error_code


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--joints', help='7 comma separated joint values in the order of smoke_test.FREE')
    ap.add_argument('--duration', type=float, default=8.0)
    ap.add_argument('--planned', action='store_true', help='plan with move_group (collision-checked) instead of a joint move')
    ap.add_argument('--scale', type=float, default=0.3, help='velocity and acceleration scale of --planned')
    args = ap.parse_args()
    target = FREE
    if args.joints:
        target = [float(v) for v in args.joints.split(',')]
        if len(target) != len(FREE):
            print(f'ready_pose: --joints needs {len(FREE)} values', file=sys.stderr)
            os._exit(1)
    rclpy.init()
    node = WallPass()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 60:
        js, _, _ = node.state()
        if js is not None and node.client.server_is_ready():
            break
        time.sleep(0.2)
    else:
        print('ready_pose: no joint states / action server', file=sys.stderr)
        os._exit(1)
    if args.planned:
        node.execute = lambda traj: execute(node, traj)
        code = planned_move(node, target, args.scale)
    else:
        code = node.move(target, args.duration)
    node.hold(1.0)
    p = node.tcp_pose()
    t, r = p.translation, p.rotation
    print(f'ready_pose: error_code {code} tcp {t.x:.4f} {t.y:.4f} {t.z:.4f} q {r.x:.4f} {r.y:.4f} {r.z:.4f} {r.w:.4f}',
          flush=True)
    os._exit(0 if code == 0 else 2)


if __name__ == '__main__':
    main()
