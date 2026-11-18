#!/usr/bin/env python3
"""Blocks until the arm has stopped moving: every joint of /joint_states changed by less than --tol rad over the last
--window seconds (sim time), or --timeout sim seconds passed (exit 0 either way, the timeout is printed).

Why: under `jtac_position_only` gz_ros2_control's position loop has a 10 s time constant, so when a trajectory (the
manager's stow move) "completes" the joints are still far from its end (1.8 rad on wrist 2 seen) and creep on for 20 s or
more. The next plan starts from that live state, away from the pose the manager believes the arm is in, and the dock
moves are refused by the wall octomap. run_multi_stand_pass.sh runs this in front of the stand arrival command.
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tol', type=float, default=0.01, help='largest joint change over the window, rad')
    ap.add_argument('--window', type=float, default=3.0, help='s')
    ap.add_argument('--timeout', type=float, default=90.0, help='sim s')
    args = ap.parse_args()
    rclpy.init()
    node = Node('wait_arm_settled', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    hist = []   # (t, {name: pos})

    def cb(m):
        hist.append((node.get_clock().now().nanoseconds * 1e-9, dict(zip(m.name, m.position))))

    node.create_subscription(JointState, '/joint_states', cb, 10)
    t0 = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        if not hist:
            continue
        now = hist[-1][0]
        t0 = now if t0 is None else t0
        while len(hist) > 1 and hist[1][0] <= now - args.window:
            hist.pop(0)
        if now - hist[0][0] >= args.window * 0.9:
            names = [n for n in hist[-1][1] if n in hist[0][1]]
            err = max((abs(hist[-1][1][n] - hist[0][1][n]) for n in names), default=0.0)
            if err < args.tol:
                print(f'arm settled after {now - t0:.1f} s (max change {err:.4f} rad in {args.window:.0f} s)')
                return 0
        if now - t0 > args.timeout:
            print(f'arm not settled after {args.timeout:.0f} s, going on')
            return 0
    return 1


if __name__ == '__main__':
    sys.exit(main())
