#!/usr/bin/env python3
"""Record the arm/lift controller's reference, feedback and error per joint to a CSV (about 10 Hz).

Diagnostic for "the arm stops a few cm short of the commanded pose" in Gazebo: the trajectory controller publishes
control_msgs/JointTrajectoryControllerState (reference, feedback, error). The columns are
t, then <joint>_ref, <joint>_fb, <joint>_err for every joint, in the controller's joint order.

usage: joint_track_log.py --out FILE [--topic /lift_arm_controller/controller_state] [--rate 10] [--timeout SEC]
"""
import argparse
import csv
import time

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node


class JointTrackLog(Node):
    def __init__(self, topic, out, rate):
        super().__init__('joint_track_log')
        self._file = open(out, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._header = False
        self._period = 1.0 / max(rate, 0.1)
        self._last = 0.0
        self._t0 = time.monotonic()
        self.create_subscription(JointTrajectoryControllerState, topic, self._on_state, 10)

    def _on_state(self, msg):
        now = time.monotonic()
        if now - self._last < self._period:
            return
        self._last = now
        names = list(msg.joint_names)
        if not self._header:
            cols = ['t']
            for n in names:
                cols += [f'{n}_ref', f'{n}_fb', f'{n}_err']
            self._writer.writerow(cols)
            self._header = True
        row = [f'{now - self._t0:.3f}']
        for i in range(len(names)):
            ref = msg.reference.positions[i] if i < len(msg.reference.positions) else float('nan')
            fb = msg.feedback.positions[i] if i < len(msg.feedback.positions) else float('nan')
            err = msg.error.positions[i] if i < len(msg.error.positions) else float('nan')
            row += [f'{ref:.5f}', f'{fb:.5f}', f'{err:.5f}']
        self._writer.writerow(row)
        self._file.flush()

    def close(self):
        self._file.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--topic', default='/lift_arm_controller/controller_state')
    ap.add_argument('--rate', type=float, default=10.0)
    ap.add_argument('--timeout', type=float, default=0.0, help='stop after this many seconds (0 = until killed)')
    a = ap.parse_args()
    rclpy.init()
    node = JointTrackLog(a.topic, a.out, a.rate)
    deadline = time.monotonic() + a.timeout if a.timeout > 0 else None
    try:
        while rclpy.ok() and (deadline is None or time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
