#!/usr/bin/env python3
"""Live wall detection check in the MuJoCo sim (scripts/run_depth_wall.sh starts everything).

Against a sim launched with depth_sensor:=true and the smoke wall (x = 0.98 m in base_footprint, normal -x),
a wall_detector node, and wall_coverage_service_node + coverage_planner_node pointed at the detector's
output file, this checks

  1. the simulated sensor: PointCloud2 on /depth_sensor/points in frame depth_sensor_link, enough points,
     all on the wall (sensor x = wall distance, measured against the known wall within `--tol-m`);
  2. the fitted plane (wall_detector ~/wall_scene, frame base_footprint) against the known wall: normal
     angle <= `--tol-deg` and the plane's distance from the wall's corners and centre <= `--tol-m`,
     the detected extents covering most of the part of the wall the sensor sees;
  3. the coverage planner accepts the written wall_scene.json + walls.pcd: ~/start, then a successful
     CoveragePlan with waypoints on the wall side of the robot (x near 0.98 minus the standoff);
  4. with --moveit: the wall is a collision object `detected_wall` in move_group's planning scene.

Exit code 0 on success. Prints the measured errors.
"""
import argparse
import json
import math
import struct
import sys
import time

import numpy as np
import rclpy
from coverage_planner_msgs.msg import CoveragePlan
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

WALL_X = 0.98
WALL_NORMAL = np.array([-1.0, 0.0, 0.0])
# the smoke wall (test/data/smoke_wall_scene.json): y 0.15 +- 0.4, z 1.15 +- 0.35
WALL_Y = (-0.25, 0.55)
WALL_Z = (0.80, 1.50)


def xyz(msg):
    assert msg.fields[0].name == 'x' and msg.point_step == 12
    p = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(-1, 3).astype(float)
    return p[np.isfinite(p).all(axis=1)]   # organised cloud: NaN rows are no return


class Check(Node):
    def __init__(self):
        super().__init__('sim_mujoco_depth_wall_test')
        self.cloud, self.scene, self.plan = None, None, None
        self.create_subscription(PointCloud2, '/depth_sensor/points', self._cloud, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/wall_detector/wall_scene', self._scene, latched)
        self.create_subscription(CoveragePlan, '/wall_coverage_service_node/coverage_plan', self._plan, 10)
        self.start = self.create_client(Trigger, '/wall_coverage_service_node/start')
        self.scene_srv = self.create_client(GetPlanningScene, '/get_planning_scene')

    def _cloud(self, m):
        self.cloud = m

    def _scene(self, m):
        self.scene = json.loads(m.data)

    def _plan(self, m):
        self.plan = m

    def wait(self, what, timeout):
        end = time.time() + timeout
        while time.time() < end and getattr(self, what) is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return getattr(self, what)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tol-m', type=float, default=0.01)
    ap.add_argument('--tol-deg', type=float, default=1.0)
    ap.add_argument('--sensor-x', type=float, default=0.1, help='sensor x in base_footprint (mounting)')
    ap.add_argument('--moveit', action='store_true')
    args = ap.parse_args()
    rclpy.init()
    node = Check()
    failures = []

    def check(ok, text):
        print(('PASS  ' if ok else 'FAIL  ') + text, flush=True)
        if not ok:
            failures.append(text)

    # 1. the sensor
    cloud = node.wait('cloud', 60)
    if cloud is None:
        print('FAIL  no point cloud on /depth_sensor/points')
        return 1
    p = xyz(cloud)
    dist = float(np.median(p[:, 0]))
    check(cloud.header.frame_id == 'depth_sensor_link', f'cloud frame {cloud.header.frame_id}')
    check(len(p) > 1000, f'{len(p)} points per scan')
    expect = WALL_X - args.sensor_x
    check(abs(dist - expect) < args.tol_m, f'median sensor-frame x {dist:.4f} m vs wall at {expect:.4f} m')

    # 2. the fitted plane
    scene = node.wait('scene', 90)
    if scene is None:
        print('FAIL  no wall scene from the detector')
        return 1
    wall = scene['walls'][0]
    n = np.array(wall['normal_outward'])
    a, b, c, d = wall['plane_abcd']
    angle = math.degrees(math.acos(max(-1.0, min(1.0, float(n @ WALL_NORMAL)))))
    corners = [(WALL_X, y, z) for y in WALL_Y for z in WALL_Z] + [(WALL_X, 0.15, 1.15)]
    err = max(abs(a * x + b * y + c * z + d) for x, y, z in corners)
    check(scene['frame_id'] == 'base_footprint', f'scene frame {scene["frame_id"]}')
    check(angle <= args.tol_deg, f'normal error {angle:.3f} deg (tolerance {args.tol_deg})')
    check(err <= args.tol_m, f'plane distance error at the wall corners/centre {err * 1000:.2f} mm (tolerance {args.tol_m * 1000:.0f})')
    du = wall['u_range_m'][1] - wall['u_range_m'][0]
    dv = wall['v_range_m'][1] - wall['v_range_m'][0]
    print(f'      fit: plane_abcd {[round(v, 4) for v in wall["plane_abcd"]]}, rms {wall["plane_rms_mm"]:.2f} mm, '
          f'extent {du:.2f} x {dv:.2f} m, {wall["num_inliers"]} inliers', flush=True)
    check(du > 0.6 and dv > 0.4, 'detected extent covers most of the wall the sensor sees')

    # 3. the coverage planner accepts it
    if not node.start.wait_for_service(timeout_sec=30):
        check(False, '/wall_coverage_service_node/start not available')
    else:
        r = node.start.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, r, timeout_sec=20)
        check(r.done() and r.result().success, f'coverage start: {r.result().message if r.done() else "timeout"}')
        plan = node.wait('plan', 60)
        if plan is None:
            check(False, 'no coverage plan received')
        else:
            check(plan.success, f'coverage plan success={plan.success} ({plan.message})')
            xs = [q.pose.position.x for q in plan.path.poses]
            check(plan.num_waypoints > 10 and len(xs) > 10, f'{plan.num_waypoints} waypoints, {len(plan.segments)} segments')
            if xs:
                print(f'      coverage x range {min(xs):.3f} .. {max(xs):.3f} m, area {plan.coverage_area:.2f} m^2, '
                      f'frame {plan.path.header.frame_id}', flush=True)
                check(0.5 < min(xs) and max(xs) < WALL_X + 0.02, 'coverage poses lie between the robot and the wall')

    # 4. MoveIt planning scene
    if args.moveit:
        ok = False
        if node.scene_srv.wait_for_service(timeout_sec=60):
            req = GetPlanningScene.Request()
            req.components.components = 8 | 16  # WORLD_OBJECT_NAMES | WORLD_OBJECT_GEOMETRY
            for _ in range(20):
                f = node.scene_srv.call_async(req)
                rclpy.spin_until_future_complete(node, f, timeout_sec=5)
                names = [o.id for o in f.result().scene.world.collision_objects] if f.done() else []
                if 'detected_wall' in names:
                    o = [o for o in f.result().scene.world.collision_objects if o.id == 'detected_wall'][0]
                    # MoveIt stores the shape pose in the object's own `pose`: compose it with the primitive's
                    q, pp = o.pose.orientation, o.primitive_poses[0].position
                    w, qx, qy, qz = q.w, q.x, q.y, q.z
                    rot = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * w), 2 * (qx * qz + qy * w)],
                                    [2 * (qx * qy + qz * w), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * w)],
                                    [2 * (qx * qz - qy * w), 2 * (qy * qz + qx * w), 1 - 2 * (qx * qx + qy * qy)]])
                    centre = np.array([o.pose.position.x, o.pose.position.y, o.pose.position.z]) + rot @ [pp.x, pp.y, pp.z]
                    print(f'      planning scene: detected_wall box {[round(v, 3) for v in o.primitives[0].dimensions]} '
                          f'centre ({centre[0]:.3f}, {centre[1]:.3f}, {centre[2]:.3f}) in {o.header.frame_id}', flush=True)
                    # front face on the wall plane, the box 5 cm behind it: centre x = 0.98 + 0.025
                    ok = abs(centre[0] - (WALL_X + 0.025)) < 0.01
                    break
                time.sleep(1.0)
        check(ok, 'detected wall is a collision object in the MoveIt planning scene')

    print('depth wall check:', 'PASS' if not failures else f'FAIL ({len(failures)})')
    rclpy.shutdown()
    return 0 if not failures else 1


if __name__ == '__main__':
    sys.exit(main())
