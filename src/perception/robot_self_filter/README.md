# robot_self_filter

Removes the robot's own body from point clouds, using link collision shapes parsed from the URDF, with per-link padding, a contact mode, speed padding, optional shadow removal and a minimum sensor distance.

## Tier

ROS-free core library (`robot_self_filter_core`) plus a ROS node (`self_filter_node`), built when `rclcpp` is found.

## Inputs and outputs

- C++ (namespace `robot_self_filter`): `UrdfRobot` (shapes from a URDF), `SelfFilter::filter(SelfFilterInput)` returning a `SelfFilterResult`, `SelfFilterConfig` and `Padding`.
- Node topics: subscribes `cloud_in` (`sensor_msgs/PointCloud2`) and `contact_mode` (`std_msgs/Bool`); publishes `cloud_out`. It looks up link poses and the cloud frame in TF at the cloud's own stamp.
- Parameters: `robot_description` (or the `robot_description` topic), `root_link`, `links`, `padding.sphere`, `padding.box`, `padding.cylinder_radius`, `padding.cylinder_height`, `padding_overrides`, `min_sensor_distance`, `shadow`, `contact_link`, `contact_padding`, `tf_timeout_sec`.
- Config: `config/self_filter.yaml`. Launch: `launch/self_filter.launch.py` (arguments `cloud_in`, `cloud_out`).

## Dependencies

- In this repo: `scene_model`.
- External: `Eigen3`, `tinyxml2`; the node needs `rclcpp`, `tf2_ros`, `sensor_msgs`, `std_msgs`, `geometry_msgs`.
- The core builds and runs without ROS sourced. No GPU. No simulator needed.

## Use it on its own

```bash
ros2 launch robot_self_filter self_filter.launch.py cloud_in:=/lidar/points cloud_out:=/lidar/points_filtered
```

## Tests

29 test cases in 4 files (counted from the sources): the filter, the URDF model, the node, and a model of the full robot description.

```bash
colcon test --packages-select robot_self_filter
```

## Status and limits

- Shadow removal is off by default because it deletes real surfaces behind a link.
- Padding values in the shipped config are examples; link names for per-link overrides depend on the arm.
- Evaluated on simulated and synthetic clouds only; nothing here has run on hardware.

## Related docs

- [robot_self_filter.md](../../../docs/robot_self_filter.md)
- [perception_live.md](../../../docs/perception_live.md)
- [moveit_config.md](../../../docs/moveit_config.md)
