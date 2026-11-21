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

## Status and limits

- Fits one dominant plane; no obstacles, openings or non-planar surfaces in the output (the scene JSON has one wall and no obstacles).
- Tried on simulated depth clouds only; real sensor noise and range limits are untested. Nothing here has run on hardware.
- Needs a TF transform from the cloud frame to `target_frame`.

## Related docs

- [perception_live.md](../../../docs/perception_live.md)
- [scene_model.md](../../../docs/scene_model.md)
- [coverage_patterns.md](../../../docs/coverage_patterns.md)
