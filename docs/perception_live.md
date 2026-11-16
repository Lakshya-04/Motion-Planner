# Live perception: simulated depth sensor and wall detection

First slice of live perception, in the MuJoCo sim: a simulated depth sensor publishes a `PointCloud2`, the
`wall_detector` node fits the wall plane from it and writes the `wall_scene.json` the coverage planner
already reads. Status per item is in [ROADMAP.md](ROADMAP.md) section 1.

```
sim_mujoco plugin (mj_multiRay)  --PointCloud2 /depth_sensor/points, frame depth_sensor_link-->
wall_detector (TF to base_footprint, scene_model RANSAC + PCA plane fit)
   --> <output_dir>/wall_scene.json + walls.pcd     (coverage_planner: wall_coverage_service_node)
   --> ~/wall_scene (std_msgs/String, the same JSON)
   --> optional /collision_object box "detected_wall" (MoveIt planning scene)
```

## Simulated depth sensor (`sim_mujoco`)

Launch argument `depth_sensor:=true` (default `false`: the MJCF is then byte-identical to before; a generator test
checks it). Further arguments: `depth_sensor_pose:="x y z roll pitch yaw"` (mounting on the root link,
`base_footprint`, x forward and z up; default `0.1 0 1.15 0 0 0`), `depth_sensor_noise_m` (Gaussian range noise, default 0)
and `depth_sensor_rate_hz` (default 5). The generator option `depth_sensor=dict(pos=, rpy=, width=, height=,
hfov_deg=, vfov_deg=, min_range=, max_range=, rate_hz=, noise_std=)` (CLI `generate_mjcf --depth-sensor X Y Z R P Y`)
adds the site `depth_sensor` on the root body and the numeric `sim_mujoco_depth` to the MJCF. Defaults: 96 x 64 rays,
60 x 45 degrees, 0.1 to 5 m.

The plugin casts the pinhole ray grid with `mj_multiRay` (`MujocoSim::depthScan()`, ROS-free and gtest-covered) and
publishes `sensor_msgs/PointCloud2` (x y z float32, dense, misses dropped) on `/depth_sensor/points` in the frame
`depth_sensor_link`; the launch file adds the static TF `base_footprint -> depth_sensor_link` from the same pose.
With `base_motion:=true` the sensor rides on the base.

This is a geometric ray-cast, not a rendered depth image: no OpenGL, no texture, no RGB, no lens model, no
motion blur. Rays hit only the scene geoms (the wall world, geom group 1), not the robot, so the cloud has no
self-hits (a real sensor would see the arm: the `robot_self_filter` is for that, and is not in the loop here).

## Wall detector (`wall_detector`)

ROS-free core (`wall_detector/core/wall_detection.hpp`, 7 gtests) and a thin node. The plane fit is
`scene_model::fitPlane` (RANSAC, then PCA on the inliers, in the existing `scene_model` package), not PCL: one dominant
plane per batch needs nothing more. PCL multi-plane segmentation behind the same function is the planned
extension for several walls per scan (ROADMAP). `detectWall` drops non-finite points and points beyond
`max_range_m`, orients the normal toward the sensor and rejects fits with too few inliers.

```bash
ros2 launch wall_detector wall_detector.launch.py cloud_topic:=/depth_sensor/points target_frame:=base_footprint \
    output_dir:=/tmp/wall_detector                       # publish_collision_object:=true for MoveIt
```

Per batch of `accumulate_frames` (5) clouds it transforms the points to `target_frame` through TF and writes
`output_dir/wall_scene.json` (the schema of `coverage_planner/test/data/wall_scene.json`: `plane_abcd`,
`origin_xyz`, `basis_u/v`, `normal_outward`, `u_range_m`, `v_range_m`, flatness in mm; `obstacles` is empty)
and `walls.pcd` (the inliers, label = wall id), through a temporary file and a rename. The same JSON goes out on
`~/wall_scene` (transient local). Feed it to the coverage planner with
`ros2 launch coverage_planner wall_coverage_service.launch.py wall_scene_json:=<output_dir>/wall_scene.json` and call
`/wall_coverage_service_node/start`.

MoveIt: with `publish_collision_object:=true` the node adds a box `detected_wall` (the measured extent, 5 cm
thick behind the measured face) on `/collision_object`. Default off, because a contact tool must touch the wall:
turn it on for the approach and free-space planning, and allow the tool collisions (ACM) for the contact segment.
The MoveIt octomap route (`pointcloud_octomap_updater` on the live cloud) is not done: it is the better choice for
clutter and unknown space, and needs a sensor-frame `sensors.yaml` plus the self-filtering mask; see ROADMAP.

## Check

`scripts/run_depth_wall.sh` (in a sourced environment with `sim_mujoco`, `wall_detector`, `coverage_planner`)
starts the sim with the smoke wall (x = 0.98 m in `base_footprint`, normal -x) and the sensor at (0.1, 0, 1.15) with
3 mm range noise, the detector and the coverage planner nodes, then `scripts/depth_wall_test.py` checks:

1. the cloud: frame, point count, median sensor-frame x equal to the wall distance;
2. the fitted plane against the known wall: normal angle within 1 degree, plane distance at the wall corners and centre within
   1 cm; detected extent covers the part of the wall the sensor sees;
3. the coverage planner accepts the written scene: `~/start`, then a successful `CoveragePlan` with waypoints;
4. `DEPTH_WALL_MOVEIT=1`: `move_group` is started and `detected_wall` must appear in its planning scene at the
   expected pose.

Result (3 mm range noise, 4464 points per scan, 5 scans per fit): normal error 0.012 degrees, plane distance
error 0.13 mm at the corners and centre, rms 2.8 mm, extent 0.76 x 0.70 m, coverage plan 12 waypoints and 0.94 m^2
(MoveIt variant: `detected_wall` box centre x = 1.005 m, as expected, 0.7 m high). Docker image `motion-planner:build`
with the MuJoCo SDK mounted, as for the other `run_*.sh` scripts.

## Use for docking

With `target_frame:=odom` the plane is a world plane, and `stand_navigator`'s `wall_docking_node` servos the base to a standoff
and yaw from it (`WallDock` action, `docs/stand_navigation.md`, `scripts/run_wall_docking.sh`: 0.4 m and 15 degrees off, docked
within 5.2 mm and 0.01 degrees, three runs). `accumulate_frames:=3` keeps the fits frequent while the base moves.

## Limits

- One planar wall per scan; no obstacle detection (`obstacles` is empty), no merging of scans from different base
  poses, no doors or openings.
- The sim cloud is a clean ray-cast: no multipath, no dropouts, no temporal noise beyond the range Gaussian, no
  depth-dependent noise model, and no dynamic objects. The 1 cm / 1 degree numbers are the best case.
- A moving base: each cloud is transformed with TF at its own stamp, but the batch is fitted as one static scene; with
  `base_motion:=true` and a moving base the TF lookup uses the cloud time (falls back to the latest TF).
- No octomap or SceneModel update from the live cloud; the wall collision object is a single box.
