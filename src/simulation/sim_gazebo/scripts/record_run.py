#!/usr/bin/env python3
"""Records a Gazebo run driven by something else (the manipulation manager) as a score_run CSV.

Samples the tool point (tcp_link in base_footprint, from TF) and the ground-truth contact force
(/ft_sensor/ground_truth_wrench, normal component |Fz| along the tool) at the ground-truth rate, and writes
PATH and PATH.wall.json on SIGINT/SIGTERM or after --timeout seconds. The wall is the plane the tcp touches
(x = 0.98 in base_footprint: the tool tip is 6.5 cm short of the tcp, the world wall at 0.915), y 0..0.3, z 1.0..1.3
unless overridden. Same CSV as sim_mujoco/scripts/record_run.py, including the `active` column (the last
/tool/valve std_msgs/Bool, 0 when none was published: the spray pass) and `--frame odom` (the world frame after a base
move, with --wall-x and --wall-y in that frame).
"""
import argparse
import json
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


class Recorder(Node):
    def __init__(self, frame='base_footprint'):
        super().__init__('sim_gazebo_record_run')
        self.frame = frame
        self.valve = 0     # last /tool/valve state, the `active` column
        self.rows = []     # (sim t_s, x, y, z, |Fz|, valve), tool point in `frame`
        self.lock = threading.Lock()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(WrenchStamped, '/ft_sensor/ground_truth_wrench', self._truth, qos_profile_sensor_data)
        self.create_subscription(Bool, '/tool/valve', self._valve, 10)

    def _valve(self, m):
        self.valve = 1 if m.data else 0

    def _truth(self, m):
        try:
            t = self.tf.lookup_transform(self.frame, 'tcp_link', rclpy.time.Time()).transform.translation
        except Exception:  # noqa: BLE001  (no transform yet: skip the row)
            return
        with self.lock:
            self.rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, t.x, t.y, t.z,
                              abs(m.wrench.force.z), self.valve))

    def write(self, path, wall_x, wall_y, wall_z):
        with self.lock:
            rows = list(self.rows)
        t0 = rows[0][0] if rows else 0.0
        with open(path, 'w') as f:
            f.write('t,x,y,z,force,active\n')
            for t, x, y, z, fz, valve in rows:
                f.write(f'{t - t0:.4f},{x:.5f},{y:.5f},{z:.5f},{fz:.4f},{valve}\n')
        with open(path + '.wall.json', 'w') as f:
            json.dump({'origin': [wall_x, 0.0, 0.0], 'u': [0, 1, 0], 'v': [0, 0, 1],
                       'u_range': list(wall_y), 'v_range': list(wall_z)}, f)
        return len(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--record', required=True)
    ap.add_argument('--timeout', type=float, default=900.0)
    ap.add_argument('--wall-x', type=float, default=0.98)
    ap.add_argument('--wall-y', type=float, nargs=2, default=[0.0, 0.3])
    ap.add_argument('--wall-z', type=float, nargs=2, default=[1.0, 1.3])
    ap.add_argument('--frame', default='base_footprint', help='frame of the tool point and the wall (odom after a base move)')
    args = ap.parse_args()
    rclpy.init()
    node = Recorder(args.frame)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    t0 = time.monotonic()
    while not stop.is_set() and time.monotonic() - t0 < args.timeout:
        time.sleep(0.1)
    n = node.write(args.record, args.wall_x, tuple(args.wall_y), tuple(args.wall_z))
    print(f'recorded {n} samples to {args.record}', flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
