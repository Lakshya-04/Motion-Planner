#!/usr/bin/env python3
"""Record the joint positions (/joint_states) to a CSV (about 10 Hz), one column per joint.

Diagnostic for "the arm stops a few cm short of the commanded pose" in Gazebo: with the JTAC controller there is no
controller_state topic (that is the plain joint_trajectory_controller's), so the measured positions are recorded and
compared offline with the plan (manager.log ARMSTATE lines, the executed trajectories).

usage: joint_track_log.py --out FILE [--topic /joint_states] [--rate 10] [--timeout SEC]
"""
import argparse
import csv
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointTrackLog(Node):
    def __init__(self, topic, out, rate):
        super().__init__('joint_track_log')
        self._file = open(out, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._header = False
        self._period = 1.0 / max(rate, 0.1)
        self._last = 0.0
        self._t0 = time.monotonic()
        self.create_subscription(JointState, topic, self._on_state, 10)

    def _on_state(self, msg):
        now = time.monotonic()
        if now - self._last < self._period:
            return
        self._last = now
        names = list(msg.name)
        if not self._header:
            self._names = names
            self._writer.writerow(['t'] + names)
            self._header = True
        pos = dict(zip(names, msg.position))
        self._writer.writerow([f'{now - self._t0:.3f}'] + [f'{pos.get(n, float("nan")):.5f}' for n in self._names])
        self._file.flush()

    def close(self):
        self._file.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--topic', default='/joint_states')
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
