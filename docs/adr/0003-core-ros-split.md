# 0003: Each module splits into a ROS-free core and a ROS shell

- Status: accepted
- Date: 2026-10-30

## Decision

Every module package has `core/` (plain C++17: algorithms, ports, data types; no ROS
headers; own CMake target and ROS-free tests) and `ros/` (node, parameters, interfaces,
message conversions, pluginlib registration). Implementations of a port are pluginlib
plugins selected by name. Cross-process traffic uses ROS 2 messages/services/actions, with
a core type and converter for each.

Follows the pattern of MoveIt (`moveit_core` vs `moveit_ros_*`), Nav2 (`nav2_core`
plugin interfaces) and ros2_control (plugin hardware/controllers).

## Consequences

- Porting to a new ROS distro touches only `ros/`, launch and config.
- Core logic is unit-tested without ROS and reusable outside ROS.
- Some conversion boilerplate per data type.
