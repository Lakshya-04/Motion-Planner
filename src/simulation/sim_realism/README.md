# sim_realism

ROS-side noise for the simulators' odometry and depth cloud, the same for MuJoCo and Gazebo. Off by default.

## Tier

Python ROS nodes over ROS-free numpy models (`odom_model.py`, `depth_model.py`, `presets.py`). Builds with colcon without any simulator.

## Inputs and outputs

- `odom_realism_node`: `/odom_truth` (`nav_msgs/Odometry`) -> `/odom` and TF `odom -> base_footprint` (`publish_tf`); `~/drift` (`Float64MultiArray`: position error m, yaw error rad, truth path m).
- `depth_realism_node`: `/depth_sensor/points_truth` (organised `PointCloud2`) -> `/depth_sensor/points`, same size, fields, frame and stamp.
- Parameters: `preset` (`off`, `good`, `typical`, `poor`), `seed`, and one override per model field (`presets.py`).
- Launch: `launch/realism.launch.py` (standalone); the simulator launches include it through `odom_realism:=` / `depth_realism:=`.

## Dependencies

External only: `rclpy`, `nav_msgs`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `tf2_ros`, `numpy`.

## Use it

```bash
ros2 launch sim_mujoco sim_mujoco.launch.py base_motion:=true depth_sensor:=true odom_realism:=typical depth_realism:=typical
scripts/run_realism_check.sh     # stand navigation, wall docking, depth wall with the nodes on
```

## Tests

`python3 -m pytest src/simulation/sim_realism/test` (the node tests need `rclpy` and skip without it).

## Status and limits

Presets are round numbers, not calibrated to any robot or sensor. No wheel physics, no lidar. Details, numbers and measured results: [docs/sim_realism.md](../../../docs/sim_realism.md).
