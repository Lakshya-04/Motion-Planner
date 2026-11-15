#!/usr/bin/env python3
"""Moves the arm to the UR10e ready pose of the MuJoCo manager pass (sim_mujoco smoke test FREE: tool pointing at the
wall, a short way off it) through the running controller's action and prints the tool pose reached in
base_footprint: `x y z qx qy qz qw`. The manager pass starts from here instead of the home pose.
Needs a launched sim (profile ur10e_telescopic) with a controller."""
import argparse
import math
import os
import sys

import rclpy
import rclpy.time

from sim_gazebo import profile as prof
import smoke_test as st

PROFILE = 'ur10e_telescopic'
# lift + six UR10e joints, the same numbers as sim_mujoco's smoke test (the kinematics are the same description)
FREE = [0.4, -0.0363, 0.2528, 1.1964, 1.6924, -1.5345, 1.5708]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--controller', default='combined_arm_lift_controller')
    ap.add_argument('--seconds', type=float, default=8.0)
    args = ap.parse_args()
    joints = prof.controlled_joints(prof.load_profile(PROFILE))
    rclpy.init()
    node = st.Probe(joints, args.controller)
    if not node.spin_until(lambda: node.sim_time is not None and not any(math.isnan(v) for v in node.positions())
                           and node.client.server_is_ready(), 90):
        print('ready_pose: no clock / joint states / action server', file=sys.stderr)
        os._exit(1)
    ok = node.send(FREE, args.seconds)
    node.sim_wait(20.0)   # gz's position loop (jtac_position_only) settles slowly after the goal
    t = node.tf.lookup_transform('base_footprint', 'tcp_link', rclpy.time.Time()).transform
    print(f'ready_pose: ok {ok} tcp {t.translation.x:.4f} {t.translation.y:.4f} {t.translation.z:.4f} '
          f'q {t.rotation.x:.4f} {t.rotation.y:.4f} {t.rotation.z:.4f} {t.rotation.w:.4f}', flush=True)
    os._exit(0 if ok else 2)


if __name__ == '__main__':
    main()
