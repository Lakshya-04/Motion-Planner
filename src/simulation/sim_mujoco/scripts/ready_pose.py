#!/usr/bin/env python3
"""Moves the arm to the smoke test's FREE pose (tool pointing at the wall, a short way off it) through the
controller's action and prints the tool pose reached in base_footprint: `x y z qx qy qz qw`. The manager pass
starts from here instead of the stowed pose, as the scripted wall pass does.

`--joints lift,pan,shoulder_lift,elbow,wrist_1,wrist_2,wrist_3` (the order of smoke_test.FREE) moves to another
joint pose instead and `--duration S` sets the move time (default 8 s): the spray pass seeds the first move with the
arm configuration of its first stroke, which is far from the wall (FREE is 3 cm from it), see run_spray_pass.sh.
`--planned` does not interpolate in joint space (no collision checking, the arm folded at the stow pose hits the lift or
itself): move_group plans the joint goal with the wall and the robot in its scene (plan only), the controller executes it.

`--pose x y z qx qy qz qw` (tcp_link in base_footprint) is the profile-agnostic mode: move_group plans a pose goal (a
position sphere and an orientation constraint on the tcp) on the profile's whole-body group (lift + arm), plan only,
with the live state as the start, and the controller executes it. No joint names or joint angles are involved, so it
works for any arm; the pass scripts take the pose from profile_reach.py (the tool pointing at the wall, a short
way in front of it, at the wall's mid height) when MP_PROFILE is not the default UR10e profile. The joint names of the
state wait come from the profile (env MP_PROFILE, smoke_test.active_joints), the group and tcp link from its
`groups.whole_body` and `frames.tcp`. Not run in a simulator for any 7-DOF arm: a failed plan exits 5."""
import argparse
import os
import sys
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint, MotionPlanRequest, OrientationConstraint,
                             PlanningOptions, PositionConstraint)
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from smoke_test import DEFAULT_PROFILE, FREE, active_joints, profile_name  # noqa: E402
from wall_pass_test import WallPass  # noqa: E402


def group_and_tcp():
    """Planning group and tcp link of the active profile (the defaults for the UR10e profile)."""
    if profile_name() == DEFAULT_PROFILE:
        return 'lift_arm', 'tcp_link'
    from profile_reach import load_profile
    p = load_profile(profile_name())
    return p.get('groups', {}).get('whole_body', 'lift_arm'), p.get('frames', {}).get('tcp', 'tcp_link')


def pose_goal(link, pose, tol_pos, tol_ang):
    """Constraints for the tcp pose [x, y, z, qx, qy, qz, qw] in base_footprint."""
    x, y, z, qx, qy, qz, qw = pose
    target = Pose()
    target.position.x, target.position.y, target.position.z = float(x), float(y), float(z)
    target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = (
        float(qx), float(qy), float(qz), float(qw))
    pc = PositionConstraint()
    pc.header.frame_id = 'base_footprint'
    pc.link_name = link
    region = BoundingVolume()
    region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[float(tol_pos)]))
    region.primitive_poses.append(target)
    pc.constraint_region = region
    pc.weight = 1.0
    oc = OrientationConstraint()
    oc.header.frame_id = 'base_footprint'
    oc.link_name = link
    oc.orientation = target.orientation
    oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = float(tol_ang)
    oc.weight = 1.0
    goal = Constraints()
    goal.position_constraints.append(pc)
    goal.orientation_constraints.append(oc)
    return goal


def planned_move(node, target, scale, pose=None, tol_pos=0.005, tol_ang=0.05):
    """Plan-only MoveGroup goal on the whole-body group (collision-checked, from the live state), executed through the
    controller. `target` is a joint goal in the order of node.joints; `pose` (x y z qx qy qz qw) a tcp pose goal instead."""
    client = ActionClient(node, MoveGroup, '/move_action')
    if not client.wait_for_server(timeout_sec=30.0):
        print('ready_pose: no /move_action server', file=sys.stderr)
        return 3
    group, link = group_and_tcp()
    req = MotionPlanRequest()
    req.group_name = group
    req.num_planning_attempts = 5
    req.allowed_planning_time = 10.0
    req.max_velocity_scaling_factor = scale
    req.max_acceleration_scaling_factor = scale
    if pose is not None:
        req.goal_constraints.append(pose_goal(link, pose, tol_pos, tol_ang))
    else:
        goal = Constraints()
        for name, q in zip(node.joints, target):
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
    ap.add_argument('--joints', help='comma separated joint values, lift first (7 for the UR10e profile, the order of smoke_test.FREE)')
    ap.add_argument('--duration', type=float, default=8.0)
    ap.add_argument('--planned', action='store_true', help='plan with move_group (collision-checked) instead of a joint move')
    ap.add_argument('--scale', type=float, default=0.3, help='velocity and acceleration scale of --planned and --pose')
    ap.add_argument('--pose', type=float, nargs=7, metavar=('X', 'Y', 'Z', 'QX', 'QY', 'QZ', 'QW'),
                    help='plan to this tcp pose in base_footprint (any robot profile), see the module docstring')
    ap.add_argument('--tol-pos', type=float, default=0.005, help='--pose position tolerance, m')
    ap.add_argument('--tol-ang', type=float, default=0.05, help='--pose orientation tolerance per axis, rad')
    args = ap.parse_args()
    target = FREE
    if args.pose is None and profile_name() != DEFAULT_PROFILE and not args.joints:
        print(f'ready_pose: FREE is a UR10e joint pose; for MP_PROFILE={profile_name()} give --pose x y z qx qy qz qw',
              file=sys.stderr)
        os._exit(1)
    if args.joints:
        target = [float(v) for v in args.joints.split(',')]
        if len(target) != len(active_joints()):
            print(f'ready_pose: --joints needs {len(active_joints())} values (lift first)', file=sys.stderr)
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
    if args.planned or args.pose is not None:
        node.execute = lambda traj: execute(node, traj)
        code = planned_move(node, target, args.scale, args.pose, args.tol_pos, args.tol_ang)
    else:
        code = node.move(target, args.duration)
    node.hold(1.0)
    p = node.tcp_pose()
    t, r = p.translation, p.rotation
    if args.pose is not None:
        err = ((t.x - args.pose[0]) ** 2 + (t.y - args.pose[1]) ** 2 + (t.z - args.pose[2]) ** 2) ** 0.5
        print(f'ready_pose: position error {err * 1000:.1f} mm from the requested pose', flush=True)
    print(f'ready_pose: error_code {code} tcp {t.x:.4f} {t.y:.4f} {t.z:.4f} q {r.x:.4f} {r.y:.4f} {r.z:.4f} {r.w:.4f}',
          flush=True)
    os._exit(0 if code == 0 else 2)


if __name__ == '__main__':
    main()
