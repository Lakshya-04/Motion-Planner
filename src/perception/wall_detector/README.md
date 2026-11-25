# wall_detector

Wall detection from a live `PointCloud2`: a dominant plane fit (RANSAC plus PCA from `scene_model`) in a target frame, written as the `wall_scene.json` and `walls.pcd` that the coverage planner reads, optionally also published as a MoveIt collision object.

## Tier

ROS-free core library (`wall_detector_core`) plus a thin ROS node and an optional octomap-to-ESDF node.

## Inputs and outputs

- C++ (`wall_detector/core/wall_detection.hpp`): `detectWall(points, params)` returning a `WallDetection`, `transformPoints`, `wallSceneJson`, `writeWallScene`; `WallDetectionParams`.
- Node `wall_detector_node`: subscribes `cloud_topic` (default `/depth_sensor/points`); publishes `~/wall_scene` (`std_msgs/String`, the scene JSON, transient local) and, with `publish_collision_object`, a `moveit_msgs/CollisionObject`. Writes `wall_scene.json` and `walls.pcd` into `output_dir`.
- Parameters: `cloud_topic`, `target_frame` (default `base_footprint`), `output_dir`, `wall_id`, `accumulate_frames`, `inlier_threshold`, `ransac_iterations`, `min_inliers`, `max_range_m`, `min_inlier_fraction`, `publish_collision_object`, `collision_thickness_m`, `collision_object_id`.
- Node `octomap_esdf_node` (when octomap is found): polls MoveIt's `/get_planning_scene`, publishes `~/esdf` and `~/esdf_info`; parameters `center`, `dims`, `voxel_size`, `rate_hz`.
- Launch: `launch/wall_detector.launch.py`.

## Dependencies

- In this repo: `scene_model`.
- External: `Eigen3`, `nlohmann_json`; the nodes need `rclcpp`, `tf2_ros`, `sensor_msgs`, `std_msgs`, `moveit_msgs`, `octomap`.
- The core builds and runs without ROS sourced. No GPU. A simulator only as a source of clouds (the MuJoCo and Gazebo backends offer a simulated depth sensor).

## Use it on its own

```bash
ros2 launch wall_detector wall_detector.launch.py cloud_topic:=/depth_sensor/points target_frame:=base_footprint output_dir:=/tmp/wall_detector
```

## Tests

8 test cases in 2 files (`wall_detection_test.cpp`, `octree_esdf_test.cpp`), counted from the sources.

```bash
colcon test --packages-select wall_detector
```

## Perception in the loop (MuJoCo two-stand pass)

`MULTI_STAND_PERCEPTION=1 run_multi_stand_pass.sh` plans on a detected wall: `scripts/pcd_to_cloud.py` publishes the sim wall's
cloud (`walls_sim.pcd`) as `PointCloud2` on `/perception/points` in `odom`, this node runs on it (`target_frame:=odom`), the
scene is cropped to the contact patch and handed to the coverage planner ([sim_mujoco.md](../../../docs/sim_mujoco.md)).
`scripts/compare_wall_scene.py DETECTED.json TRUTH.json` (numpy only, exit 1 out of tolerance) reports the normal angle error (deg), the
plane offset error (mm), the rectangle's edge errors in the truth wall's u / v axes (mm) and IoU, and the recall of true obstacles.
`scripts/wall_fit_ref.py` is a python mirror of `detectWall` (same RANSAC + PCA steps and node defaults, different random draws) so
the comparison is tested on the host without ROS: `python3 -m pytest src/perception/wall_detector/test/python` (13 tests: clean and
1 / 3 / 5 mm noisy clouds, a tilted wall, outliers plus a floor, obstacle recall, a too-short wall, the CLI, and the pass's
`walls_sim.pcd` -> detect -> crop -> compare chain on `two_panels_plain`).

What is detected, honestly: from the sim's clean cloud (exact grid, no noise) the plane comes out exact (angle 0.000 deg, offset
0.00 mm, cropped extent equal to the baked planner scene). With 3 mm Gaussian noise on the same cloud: 0.001 deg, 0.01 mm. That is a
check of the plumbing, not of perception quality: the sim cloud has no sensor noise, no occlusion, no holes and one plane. The
detector writes no obstacles, so true obstacle recall is 0 (the pass warns, and plans without them), and the C++ node itself was
not run in this mode when it was written (host python only; the pass needs a sim run).

## Status and limits

- Fits one dominant plane; no obstacles, openings or non-planar surfaces in the output (the scene JSON has one wall and no obstacles).
- Tried on simulated depth clouds only; real sensor noise and range limits are untested. Nothing here has run on hardware.
- Needs a TF transform from the cloud frame to `target_frame`.

## Related docs

- [perception_live.md](../../../docs/perception_live.md)
- [scene_model.md](../../../docs/scene_model.md)
- [coverage_patterns.md](../../../docs/coverage_patterns.md)
