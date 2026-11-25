#!/usr/bin/env python3
"""Logs /joint_states to a CSV (time, one column per joint) at a fixed rate and prints a WIND line the first time a
revolute joint is more than pi away from its first sample (a wound-up arm: the stow planner refuses to start there)."""
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Logger(Node):
    def __init__(self, path, hz):
        super().__init__('mp_joint_logger')
        self.f = open(path, 'w')
        self.names = None
        self.first = None
        self.flagged = set()
        self.period = 1.0 / hz
        self.last = 0.0
        self.create_subscription(JointState, '/joint_states', self.cb, 10)

    def cb(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        if t - self.last < self.period:
            return
        self.last = t
        order = sorted(range(len(msg.name)), key=lambda i: msg.name[i])
        names = [msg.name[i] for i in order]
        pos = [msg.position[i] for i in order]
        if self.names is None:
            self.names, self.first = names, pos
            self.f.write('t,' + ','.join(names) + '\n')
        self.f.write('%.2f,' % t + ','.join('%.4f' % p for p in pos) + '\n')
        self.f.flush()
        for n, p, p0 in zip(names, pos, self.first):
            if n.startswith('arm_') and abs(p) > math.pi + 0.05 and n not in self.flagged:
                self.flagged.add(n)
                self.get_logger().warn('WIND t=%.1f %s=%.2f (outside +-pi: wound a turn; first sample %.2f)' % (t, n, p, p0))


def main():
    rclpy.init()
    rclpy.spin(Logger(sys.argv[1] if len(sys.argv) > 1 else '/tmp/joints.csv', float(sys.argv[2]) if len(sys.argv) > 2 else 4.0))


if __name__ == '__main__':
    main()
