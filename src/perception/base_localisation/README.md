# base_localisation

Optional filtered base pose: `robot_localization`'s EKF over wheel odometry, an IMU and, if you have a lidar, scan matching. One config per drive class. Off by default; nothing else in the stack depends on it.

## Tier

Python ROS nodes over ROS-free numpy modules (`scan_match.py`, `sim_imu.py`, `ekf_params.py`, `score.py`), plus a launch file and three YAML configs. The EKF itself is the `ros-jazzy-robot-localization` package (in `docker/apt-extra.txt`). Builds with colcon without any simulator.

## Inputs and outputs

- EKF in: `/odom` (`nav_msgs/Odometry`, velocities only), `/imu` (`sensor_msgs/Imu`, gyro yaw rate only), `/scan_odom` (`nav_msgs/Odometry`, velocity only, from `scan_match_node`).
- EKF out: `/odometry/filtered` (`output_topic`) and TF `odom -> base_footprint` (`publish_tf`).
- `scan_match_node`: `/scan` (`sensor_msgs/LaserScan`) -> `/scan_odom`. Scan-to-scan point-to-line ICP, rejects bad matches.
- `sim_imu_node`: `/odom_truth` (or `/odom`) -> `/imu`, for the MuJoCo sim only (Gazebo has a real `imu` sensor).
- `launch/localisation.launch.py`: `platform:=diff|omni|ackermann`, `imu`, `scan_match`, topics, `publish_tf`, `params_file`.
- `config/ekf_<platform>.yaml`: the filter setup for that drive class.

## Dependencies

External: `robot_localization`, `rclpy`, `nav_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2_ros_py`, `numpy`, `scipy`, `yaml`.

## Use it

```bash
# simulator, either one (the EKF owns TF odom -> base_footprint, so odom_realism must be on)
ros2 launch sim_gazebo sim.launch.py base_motion:=true odom_realism:=typical imu:=true lidar:=true localisation:=true localisation_scan_match:=true
ros2 launch sim_mujoco sim_mujoco.launch.py base_motion:=true odom_realism:=typical imu:=true localisation:=true
# a real base
ros2 launch base_localisation localisation.launch.py platform:=diff odom_topic:=/wheel/odom imu_topic:=/imu/data
src/perception/base_localisation/scripts/run_localisation_check.sh    # Gazebo: EKF against truth
```

Consumers read `/odometry/filtered`, for example `stand_navigator` with `-p odom_topic:=/odometry/filtered`.

## Tests

`python3 -m pytest src/perception/base_localisation/test` (the node tests need `rclpy` and the launch test needs `launch_ros`; both skip without them).

## Status and limits

Run in Gazebo only, with the simulated odometry noise; not on a robot, MuJoCo not run. Details, numbers and the config per platform class: [docs/localisation.md](../../../docs/localisation.md).
