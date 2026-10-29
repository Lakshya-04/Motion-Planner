---
type: Reference
title: ros_adapters
description: Package README for the shared ROS-boundary adapter layer implementing core_types' ports -- RclcppLogger and the toCore()/toRos() conversion free functions.
tags: [ros-adapters, ports-and-adapters, package-readme]
generated: { by: claude-code/fable-5, at: 2026-07-25T06:27:42Z }
status: stable
---

# ros_adapters

Header-only, `INTERFACE`-library package holding the concrete, rclcpp-backed implementations of
`core_types`' ports. This is the only layer allowed to sit between `core_types` (pure C++,
no ROS deps) and `rclcpp`/ROS message types -- core logic (`core_types`, and any `*_core`
algorithm code built on it) never includes a ROS message header directly; it depends only on
`core_types`' abstract ports (`Logger`, `Pose`, `Trajectory`) and gets a real ROS-backed
implementation injected at the node boundary.

It exists to be shared: multiple ROS adapter packages (`motion_planner`,
`manipulation_manager`, and `serum_demo_bt`) need the same `RclcppLogger` and
`toCore()`/`toRos()` conversions, so they live here once instead of being duplicated or one package
depending on another just to reuse them.

## What's in scope here (and what isn't)

Only conversions between `core_types` and generic ROS message packages (`geometry_msgs`,
`trajectory_msgs`) belong in this package. Conversions to a *specific* package's own custom types or
interfaces do not: e.g. `manipulation_manager/include/manipulation_manager/ros/action_conversions.hpp`
converts `interfaces::action::ExecuteMotionSegments` goal/feedback/result to/from
`manipulation_manager`'s own `MotionSegmentSpec`/`MotionRunSummary`/`MotionSegmentExecMetrics` --
deliberately kept out of `ros_adapters` because those types aren't `core_types`, and this
package "doesn't (and shouldn't) depend on `interfaces`/`manipulation_manager`" (see that
file's header comment). It does mirror this package's inline-free-function `toCore()`/`toRos()`
style.

## Adapters

### `RclcppLogger` (`include/ros_adapters/rclcpp_logger.hpp`)

Implements the `core_types::Logger` port (`core_types/include/core_types/logger.hpp`,
`info()`/`warn()`, pure virtual). Wraps an `rclcpp::Logger`, forwarding to `RCLCPP_INFO`/`RCLCPP_WARN`:

```cpp
explicit RclcppLogger(rclcpp::Logger logger);
void info(const std::string & message) const override;
void warn(const std::string & message) const override;
```

Nodes construct it from their own logger and inject it into core logic at their boundary, e.g.:

```cpp
manager_.setLogger(std::make_shared<ros_adapters::RclcppLogger>(this->get_logger()));
```

Used this way by `ManipulationManagerNode`, `ManipulationManagerActionServer`, and `MotionValidator`
(all in `manipulation_manager`) to give `ManipulationManager::setLogger()` a real logging backend
without `ManipulationManager` itself depending on rclcpp. Before injection, core code defaults to
`core_types::NullLogger` (`core_types/include/core_types/null_logger.hpp`), which
silently discards every message.

### `ros_conversions.hpp` free functions (`include/ros_adapters/ros_conversions.hpp`)

Four overloaded, `inline` free functions converting between `core_types` POD types and ROS
messages:

| Function | Direction | Types |
|---|---|---|
| `toCore(const geometry_msgs::msg::Pose &)` | ROS -> core | `geometry_msgs::msg::Pose` -> `core_types::Pose` |
| `toRos(const core_types::Pose &)` | core -> ROS | `core_types::Pose` -> `geometry_msgs::msg::Pose` |
| `toCore(const trajectory_msgs::msg::JointTrajectory &)` | ROS -> core | `trajectory_msgs::msg::JointTrajectory` -> `core_types::Trajectory` |
| `toRos(const core_types::Trajectory &)` | core -> ROS | `core_types::Trajectory` -> `trajectory_msgs::msg::JointTrajectory` |

`Pose` conversion is a direct field-by-field copy (`position.{x,y,z}` <-> `{x,y,z}`,
`orientation.{x,y,z,w}` <-> `{qx,qy,qz,qw}`). `Trajectory` conversion copies `joint_names` and, per
point, `positions`/`velocities`/`accelerations`, converting `time_from_start` via
`rclcpp::Duration(point.time_from_start).seconds()` (ROS -> core) and
`rclcpp::Duration::from_seconds(point.time_from_start_sec)` (core -> ROS).

These implement the port role played by `core_types::Pose` and `core_types::Trajectory` --
plain structs with no ROS dependency of their own (per their header comments in
`core_types/include/core_types/pose.hpp` and `trajectory.hpp`), which this package is
responsible for translating at the ROS boundary. Unlike `Logger`, `Pose`/`Trajectory` aren't
abstract interfaces with a virtual dispatch point -- the "port" is just the plain type, and these
functions are the two-way boundary crossing for it.

## Ports implemented, cross-referenced against `core_types`

`core_types/include/core_types/` defines exactly two things `ros_adapters` adapts:

- **`Logger`** (`logger.hpp`) -- abstract port, `info()`/`warn()` pure virtual. Implemented here by
  `RclcppLogger`. (`NullLogger` in `null_logger.hpp` is `core_types`' own no-op default, not
  part of this package.)
- **`Pose`** / **`Trajectory`** (`pose.hpp`, `trajectory.hpp`) -- plain structs, no port interface
  to implement, but explicitly documented as crossing the ROS boundary only via an adapter. This
  package supplies that boundary via `toCore()`/`toRos()`.

There is no third port in `core_types` today (no execution or planning port lives there --
`manipulation_manager`'s `ExecutionPort` and `motion_planner`'s `Planner` interface are
defined in their own packages, not `core_types`, so they are out of scope for this package).

## Consumers

Declared as a `<depend>` / `find_package(ros_adapters REQUIRED)` and linked via the namespaced
target `ros_adapters::ros_adapters` by:

- **`motion_planner`** -- `CartesianPlanner` and `OmplPlanner`
  (`src/planners/cartesian_planner.cpp`, `src/planners/ompl_planner.cpp`) call
  `ros_adapters::toRos(pose)` to hand a core `Pose` to MoveIt and
  `ros_adapters::toCore(trajectory_msg.joint_trajectory)` to bring a planned MoveIt trajectory
  back into a core `Trajectory`.
- **`manipulation_manager`** -- `ManipulationManagerNode`, `ManipulationManagerActionServer`,
  and `MotionValidator` construct an `RclcppLogger` for `ManipulationManager::setLogger()`;
  `MoveItExecutionAdapter` (`src/ros/execution/moveit_execution_adapter.cpp`) calls
  `ros_adapters::toRos(request.trajectory)` to convert a core `Trajectory` into a
  `FollowJointTrajectory` goal; `ManipulationManagerActionServer`
  (`src/ros/manipulation_manager_action_server.cpp`) calls `ros_adapters::toCore(pose_stamped.pose)`
  converting incoming waypoints off the `ExecuteMotionSegments` action goal.
- **`serum_demo_bt`** (`applications`) -- `wall_cleaning_action_server.cpp` constructs an
  `RclcppLogger` from `this->get_logger()` and calls `ros_adapters::toCore(pose)` when building
  core waypoints from a behavior-tree action goal.

## Build

Header-only `INTERFACE` library (no `.cpp`, no compiled symbols). `CMakeLists.txt` exports the
namespaced target `ros_adapters::ros_adapters`, requiring downstream packages to
`find_package(ros_adapters REQUIRED)` rather than pulling in raw include paths. Depends on
`rclcpp`, `geometry_msgs`, `trajectory_msgs`, and `core_types` (propagated via
`ament_export_dependencies`).

## Tests

None. `BUILD_TESTING` in `CMakeLists.txt` only wires `ament_lint_auto`/`ament_lint_common` (with the
same `ament_cmake_copyright_FOUND`/`ament_cmake_cpplint_FOUND` skip used by `motion_planner`'s
and `core_types`' `CMakeLists.txt`, since this repo has no per-file copyright headers yet) --
there is no `test/` directory and no `ament_add_gtest` target, unlike `core_types` itself
(`test/test_pose.cpp`, `test/test_trajectory.cpp`). Correctness of the conversions is exercised only
indirectly, through the consumers listed above.
