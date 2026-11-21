# Package layout: `core/` + `ros/` in every module

Every module package has two halves:

```
<module>/
├── include/<module>/
│   ├── core/        # algorithms, interfaces (ports), data types — plain C++17, no ROS
│   └── ros/         # ROS 2 node / plugin / message conversions for this module
├── src/
│   ├── core/
│   └── ros/
├── test/
│   ├── core/        # gtest, runs with no ROS installed
│   └── ros/         # launch / integration tests
├── config/          # default YAML parameters
├── launch/
├── plugins.xml      # pluginlib description, when the module provides plugins (exported
│                    # under the `motion_planner` package for the planning ports)
├── CMakeLists.txt   # two targets: <module>_core (no rclcpp) and <module>_ros
└── package.xml
```

- **`core/` never depends on ROS.** It does not include `rclcpp`, messages, TF or MoveIt
  headers. It is built as its own library target (`<module>_core`) linked only against
  plain libraries (Eigen, PCL, ...). It does not change when the ROS version changes
  (Humble -> Jazzy -> next), and it can be reused outside ROS entirely.
- **`ros/` is the thin shell.** Parameters, topics/services/actions, TF lookups, message
  <-> core conversions, pluginlib registration, lifecycle. When the ROS version changes,
  only this folder (and launch/config) changes.
- Heavy external backends that are themselves ROS-bound (MoveIt) or not C++ (cuRobo, Python)
  live in their **own adapter package** (e.g. `motion_planner_moveit`, `curobo_bridge`) so the
  core package never pulls them in.
- Shared ROS-free types live in `core_types`; shared message conversions in `ros_adapters`.

## How modules talk to each other

This is the convention used across the mature ROS 2 projects:

| Where | Mechanism | Industry precedent |
|---|---|---|
| **Inside a process** (planner inside the manager, IK inside the planner) | C++ abstract interface in `core/` + implementation chosen at runtime by **pluginlib** from a YAML name | Nav2 (`nav2_core` interfaces: planners, controllers, costmap layers as plugins), MoveIt (planning, IK and collision plugins), ros2_control (hardware interfaces and controllers as plugins) |
| **Across processes** (perception node -> coverage node -> manager) | ROS 2 **messages, services, actions** defined in a `*_msgs` / `*_interfaces` package | Nav2 actions (`NavigateToPose`), MoveIt `move_group` actions, `FollowJointTrajectory` |
| **Algorithm vs. node** | ROS-independent library + thin node wrapper | MoveIt's split into `moveit_core` (no ROS node code) and `moveit_ros_*`; Autoware's practice of keeping algorithms in ROS-independent libraries with a separate node |

So "typed, serializable data" just means: each core data type that crosses a module
boundary (e.g. `core_types::Trajectory`, a coverage segment, a scene object) has a matching
ROS message and a converter in `ros/`. Then the same module can run either:

- **in-process** (fast, a direct C++ call through the interface), or
- **as its own node** (the message form), which also means it can be recorded with
  `ros2 bag` and replayed for debugging or for building training data.

## Rules

1. Interfaces (ports) and their data types are declared in `core/`.
2. `core/` compiles and its tests pass with no ROS environment sourced.
3. Every implementation of a port is a plugin selectable by name; the default lives in
   `config/`. Done for the planning ports (`Planner`, `IkPort`, `CollisionPort`, `ScenePort`);
   the other module families follow the same pattern when they get a second implementation. See
   [backend_selection.md](backend_selection.md).
4. Every port result is structured: success, failure code, failure stage, message.
5. Every cross-process data type has a `*_msgs` definition and a `ros/` converter, with a
   round-trip unit test.
6. Name ROS-version-specific code in `ros/` only; no `#if ROS_DISTRO` in `core/`.

## Current state of this repo

| Package | Status |
|---|---|
| `manipulation_manager` | already split into `core/` + `ros/`; the node builds its planners from `config/planning.yaml` (parameter `planning_config`) instead of linking a backend |
| `core_types`, `force_core`, `safety_monitor` | ROS-free already (pure core packages; `safety_monitor` has no node yet) |
| `ros_adapters` | ROS-only conversions (correct as is) |
| `motion_planner` | **done**: ROS-free core (ports, plugin base classes, `PlanningPipeline`, `MotionPlanner`, geometry helpers; its tests run without ROS) plus a small `motion_planner_ros` library (YAML selection loader, pluginlib registry; needs pluginlib and yaml-cpp, no node) |
| `motion_planner_moveit` | **done**: the MoveIt / OMPL adapter (the only package that depends on MoveIt); `plugins.xml` exports `moveit_cartesian`, `moveit_ompl`, `moveit_ik`, `moveit_collision`, `moveit_scene` |
| `curobo_planner` | the cuRobo client; `plugins.xml` exports `curobo`, `curobo_ik`, `curobo_collision`, `curobo_scene` |
| `vamp_planner` | the VAMP wrapper (built only where VAMP is installed); `plugins.xml` exports `vamp`, `vamp_collision`, `vamp_scene` |
| `coverage_planner` | **done**: `core/` + `ros/` split, 44 tests |
| `compliant_trajectory_controller`, `ft_bridge_hardware` | ros2_control plugins; extract controller math into `core/` |
