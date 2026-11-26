#!/usr/bin/env python3
"""Drive the base around a square and score odometry tracks against /odom_truth.

Run against a sim started with base_motion:=true and odom_realism on (run_localisation_check.sh does that).
Open-loop /cmd_vel: `--laps` laps of a square of side `--side` m (drive, then turn 90 degrees left), then a stop.
Every `--track name=/topic` (Odometry) is compared with /odom_truth after the first-sample alignment; prints one table
and writes the tracks to `--csv`. Exit code 0 when it ran and every track has data (it does not judge the numbers).
"""
import argparse
import math
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

from base_localisation.score import score


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Check(Node):
    def __init__(self, tracks):
        super().__init__('localisation_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.lock = threading.Lock()
        self.data = {name: [] for name in ['truth'] + [n for n, _ in tracks]}
        self.create_subscription(Odometry, '/odom_truth', self._cb('truth'), 50)
        for name, topic in tracks:
            self.create_subscription(Odometry, topic, self._cb(name), 50)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _cb(self, name):
        def cb(m):
            p = m.pose.pose
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            with self.lock:
                self.data[name].append((t, p.position.x, p.position.y, yaw_of(p.orientation)))
        return cb

    def drive(self, vx, wz, seconds):
        end = self.get_clock().now().nanoseconds * 1e-9 + seconds
        msg = Twist()
        msg.linear.x, msg.angular.z = vx, wz
        while self.get_clock().now().nanoseconds * 1e-9 < end:
            self.pub.publish(msg)
            time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--track', action='append', default=[], help='name=/topic (Odometry), repeatable')
    ap.add_argument('--side', type=float, default=2.0)
    ap.add_argument('--laps', type=int, default=2)
    ap.add_argument('--speed', type=float, default=0.3)
    ap.add_argument('--turn-rate', type=float, default=0.5)
    ap.add_argument('--csv', default='')
    ap.add_argument('--wait', type=float, default=90.0, help='seconds to wait for the topics')
    args = ap.parse_args()
    tracks = [tuple(t.split('=', 1)) for t in (args.track or ['raw=/odom'])]

    rclpy.init()
    node = Check(tracks)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    t0 = time.time()
    while time.time() - t0 < args.wait:
        with node.lock:
            if all(len(v) > 5 for v in node.data.values()):
                break
        time.sleep(0.5)
    else:
        with node.lock:
            print('FAIL: no data on', [k for k, v in node.data.items() if len(v) <= 5])
        return 1
    time.sleep(2.0)
    for lap in range(args.laps):
        for _ in range(4):
            node.drive(args.speed, 0.0, args.side / args.speed)
            node.drive(0.0, args.turn_rate, (math.pi / 2) / args.turn_rate)
    node.drive(0.0, 0.0, 2.0)
    time.sleep(1.0)

    with node.lock:
        arrays = {k: np.array(v) for k, v in node.data.items()}
    truth = arrays['truth']
    rows, rc = [], 0
    for name, _ in tracks:
        est = arrays[name]
        if len(est) < 10:
            rc = 1
            continue
        # compare from the start of the drive, so the idle seconds do not dilute the figures
        moving = np.nonzero(np.hypot(*(truth[:, 1:3] - truth[0, 1:3]).T) > 0.01)[0]
        i0 = max(int(moving[0]) - 5, 0) if len(moving) else 0
        t_ref = truth[i0:, 0]
        res = score(t_ref, truth[i0:, 1:], est[:, 0], est[:, 1:])
        rows.append((name, res))
    print(f'{"track":<14}{"path m":>8}{"final m":>9}{"rms m":>8}{"max m":>8}{"final yaw deg":>15}{"rms yaw deg":>13}')
    for name, r in rows:
        print(f'{name:<14}{r["path_m"]:>8.2f}{r["final_m"]:>9.3f}{r["rms_m"]:>8.3f}{r["max_m"]:>8.3f}'
              f'{r["final_yaw_deg"]:>15.2f}{r["rms_yaw_deg"]:>13.2f}')
    if args.csv:
        with open(args.csv, 'w') as f:
            f.write('track,t,x,y,yaw\n')
            for name, arr in arrays.items():
                for row in arr:
                    f.write(f'{name},{row[0]:.4f},{row[1]:.5f},{row[2]:.5f},{row[3]:.5f}\n')
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
