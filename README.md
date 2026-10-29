# Motion Planner

ROS 2 packages for wall coverage planning and arm motion planning, through
execution on a compliant (admittance) joint trajectory controller (JTAC).

## Packages

| Package | Description |
|---|---|
| [`mobile_manipulator_description`](src/description/mobile_manipulator_description) | Generic 7-DOF robot model: base + swappable telescopic lift + swappable 6-DOF arm (UR10e, generic) + tool |
| [`coverage_planner`](src/perception/coverage_planner) | Wall coverage path planning (boustrophedon passes over a wall point cloud) |
| [`coverage_planner_msgs`](src/perception/coverage_planner_msgs) | Messages and services for the coverage planner |
| [`motion_planner`](src/manipulation/motion_planner) | ROS-free motion planning core: planner, IK, collision and scene ports, `MotionPlanner` orchestration |
| [`motion_planner_moveit`](src/manipulation/motion_planner_moveit) | MoveIt / OMPL backend implementing those ports (Cartesian + free-space planners) |
| [`manipulation_manager`](src/manipulation/manipulation_manager) | Orchestrates coverage -> planning -> execution |
| [`compliant_trajectory_controller`](src/control/compliant_trajectory_controller) | JTAC: compliant trajectory controller for `ros2_control` |
| [`ft_bridge_hardware`](src/control/ft_bridge_hardware) | Force/torque sensor bridge (`ros2_control` hardware interface) |
| [`force_core`](src/common/force_core) | Force-control core library |
| [`core_types`](src/common/core_types) | ROS-free domain types and ports |
| [`ros_adapters`](src/common/ros_adapters) | Conversions between core types and ROS messages |

## Requirements

- ROS 2 (Jazzy recommended), `colcon`, MoveIt 2, `ros2_control`
- PCL, Eigen, nlohmann-json, Octomap

A robot description (URDF/xacro) and a MoveIt configuration for your arm are
not included; the packages expect them to be supplied by your own workspace.

## Build

```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/Lakshya-04/Motion-Planner.git
cd ~/ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## Test

```bash
colcon test && colcon test-result --verbose
```

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).



## Design

Every stage (perception, scene, surface, coverage, sequencing, motion planning, timing,
execution, control, and a slot for learned policies) is a swappable module selected by
config. Planning backends: MoveIt, cuRobo v2 and VAMP. See [docs/](docs/README.md), starting
with [architecture.md](docs/architecture.md).

## License

Apache License 2.0. See [LICENSE](LICENSE).
