#!/usr/bin/env python3
"""Live octomap check in the MuJoCo sim (scripts/run_live_octomap.sh starts everything).

Against a sim launched with depth_sensor:=true, move_group:=true live_octomap:=true and the smoke wall (x = 0.98 m in
base_footprint, normal -x), with no octomap loaded from a file and no collision object for the wall, this checks

  1. MoveIt's occupancy map monitor built an octomap from /depth_sensor/points: /get_planning_scene returns a
     non-empty octomap (resolution, size);
  2. with --esdf (octomap_esdf_node running): the distance field made from that octomap is ~0 or negative at the wall
     plane, about the distance to the wall in front of it, and the wall is not at the wrong x (zero crossing);
  3. a collision-checked Cartesian path (/compute_cartesian_path, avoid_collisions true) from the FREE pose, tool
     axis +x, straight at the wall and 12 cm through it stops at the wall (before the 12 cm are done), while the same path without
     collision checking completes, and the path away from the wall completes with collision checking on.

Exit code 0 on success. Prints the numbers.
"""
import argparse
import os
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningSceneComponents, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPlanningScene
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from smoke_test import FREE, JOINTS, Smoke  # noqa: E402

WALL_X = 0.98


class Live(Smoke):
    def __init__(self, controller):
        super().__init__(controller)
        self.scene_srv = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.cart = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.esdf, self.info = None, None
        self.create_subscription(Float32MultiArray, '/octomap_esdf/esdf', lambda m: setattr(self, 'esdf', m), latched)
        self.create_subscription(Float64MultiArray, '/octomap_esdf/esdf_info', lambda m: setattr(self, 'info', m), latched)

    def call(self, client, req, timeout=30.0):
        fut = client.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def octomap(self):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.OCTOMAP | PlanningSceneComponents.WORLD_OBJECT_NAMES
        res = self.call(self.scene_srv, req)
        return res.scene.world if res else None

    def tcp(self):
        tf = self.tf_buffer.lookup_transform('base_footprint', 'tcp_link', rclpy.time.Time())
        return tf.transform

    def cartesian(self, dx, avoid):
        tcp = self.tcp()
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_footprint'
        start = RobotState()
        start.joint_state = JointState(name=list(JOINTS), position=list(self.state()[0]))
        req.start_state = start
        req.group_name = 'lift_arm'
        req.link_name = 'tcp_link'
        p = Pose()
        p.position.x, p.position.y, p.position.z = tcp.translation.x + dx, tcp.translation.y, tcp.translation.z
        p.orientation = tcp.rotation
        req.waypoints = [p]
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = avoid
        res = self.call(self.cart, req)
        return (res.fraction if res else -1.0), tcp.translation.x


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--controller', default='combined_arm_lift_controller')
    ap.add_argument('--esdf', action='store_true')
    ap.add_argument('--timeout', type=float, default=120.0)
    args = ap.parse_args()
    rclpy.init()
    node = Live(args.controller)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    failures = []

    def check(ok, text):
        print(('PASS  ' if ok else 'FAIL  ') + text, flush=True)
        if not ok:
            failures.append(text)

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.timeout:
        if node.state()[0] is not None and node.scene_srv.service_is_ready() and node.cart.service_is_ready() \
                and node.client.server_is_ready():
            break
        time.sleep(0.2)
    else:
        print('FAIL  startup: joint_states / move_group services / controller not available')
        os._exit(1)

    # 1. the octomap from the live cloud (no wall collision object, no file-loaded octomap in this setup)
    world = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < 90:
        world = node.octomap()
        if world is not None and len(world.octomap.octomap.data) > 0:
            break
        time.sleep(1.0)
    ok = world is not None and len(world.octomap.octomap.data) > 0
    check(ok, 'planning scene has an octomap built by the occupancy map monitor')
    if not ok:
        os._exit(1)
    res, size = world.octomap.octomap.resolution, len(world.octomap.octomap.data)
    print(f'      octomap: frame {world.octomap.header.frame_id}, resolution {res} m, '
          f'{size} bytes, collision objects {[o.id for o in world.collision_objects]}',
          flush=True)
    check(size > 100 and abs(res - 0.03) < 1e-6, f'octomap resolution {res} m, {size} bytes')
    check('detected_wall' not in [o.id for o in world.collision_objects], 'the wall is not a collision-object box')

    # 2. the distance field made from it
    if args.esdf:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 60 and (node.esdf is None or node.info is None):
            time.sleep(0.2)
        if node.esdf is None or node.info is None:
            check(False, 'no ESDF on /octomap_esdf/esdf')
        else:
            vs, cx, cy, cz, nx, ny, nz = node.info.data
            nx, ny, nz = int(nx), int(ny), int(nz)
            f = np.array(node.esdf.data, dtype=float).reshape(nx, ny, nz)
            origin = np.array([cx, cy, cz]) - 0.5 * np.array([nx, ny, nz]) * vs

            def at(x, y, z):
                i, j, k = ((np.array([x, y, z]) - origin) / vs).astype(int)
                return f[i, j, k]
            # along the sensor ray through (y 0.15, z 1.15): the first occupied voxel is the wall
            d_wall, d_front, d_far = at(WALL_X, 0.15, 1.15), at(WALL_X - 0.3, 0.15, 1.15), at(WALL_X - 0.6, 0.15, 1.15)
            xs = origin[0] + (np.arange(nx) + 0.5) * vs
            line = f[:, int((0.15 - origin[1]) / vs), int((1.15 - origin[2]) / vs)]
            x_zero = xs[int(np.argmax(line < 0))]  # first voxel inside an obstacle along +x
            print(f'      ESDF {nx}x{ny}x{nz} at {vs} m: d(wall plane) {d_wall:.3f}, d(0.3 m in front) {d_front:.3f}, '
                  f'd(0.6 m in front) {d_far:.3f}; first negative voxel on the ray at x = {x_zero:.3f}', flush=True)
            check(abs(d_wall) <= vs + 1e-6, f'ESDF at the wall plane |{d_wall:.3f}| <= one voxel')
            check(abs(d_front - 0.3) <= 1.5 * vs and abs(d_far - 0.6) <= 1.5 * vs,
                  'ESDF grows with the distance in front of the wall (0.3 m, 0.6 m)')
            check(abs(x_zero - WALL_X) <= vs + 1e-6, f'ESDF zero crossing at x = {x_zero:.3f} vs wall x = {WALL_X}')

    # 3. Cartesian approach checked against the live octomap
    code = node.move(FREE, 8.0)
    check(code == 0, f'moved to the FREE pose (error code {code})')
    time.sleep(1.0)
    back, x0 = node.cartesian(-0.15, True)
    through_checked, _ = node.cartesian(0.12, True)
    through_free, _ = node.cartesian(0.12, False)
    stop_x = x0 + through_checked * 0.12
    print(f'      tool x {x0:.3f} m; fraction away from the wall {back:.2f}; through the wall with collision '
          f'checking {through_checked:.2f} (stops at x = {stop_x:.3f}), without {through_free:.2f}', flush=True)
    check(back > 0.99, f'path away from the wall completes with collision checking (fraction {back:.2f})')
    check(through_free > 0.99, f'path through the wall completes without collision checking (fraction {through_free:.2f})')
    check(through_checked < 0.99 and stop_x < WALL_X + 0.06,
          f'path through the wall is stopped by the octomap before the end of the path (tool x = {stop_x:.3f}, wall at {WALL_X})')

    print('live octomap check:', 'PASS' if not failures else f'FAIL ({len(failures)})')
    os._exit(0 if not failures else 1)


if __name__ == '__main__':
    main()
