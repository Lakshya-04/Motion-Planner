---
type: Reference
title: core_types
description: Package README for the pure C++ domain types + ports library at the bottom of the hexagonal split -- Pose, Trajectory, Logger, zero rclcpp/ROS message deps.
tags: [core-types, hexagonal-architecture, package-readme]
generated: { by: claude-code/fable-5, at: 2026-07-25T06:27:42Z }
status: stable
---

# core_types

Pure C++ domain types and ports for the manipulation/coverage stack -- no `rclcpp`, no ROS
message dependencies (`package.xml` description). This is the innermost layer of the repo's
ports-and-adapters ("hexagonal") split: `core_types` (types + ports) → `*_core` algorithm
packages → ROS adapter packages → `interfaces` (the wire contract).

## The litmus test

This package -- and any real `*_core` built on it -- must `colcon build` and run its unit tests
with ROS **not sourced**. `CMakeLists.txt` only `find_package(ament_cmake REQUIRED)`; the library
target is an `INTERFACE` library (header-only, no `.cpp` to compile) requiring only
`cxx_std_17`. `test/test_pose.cpp`'s file-level doc comment states this explicitly: it "also serves
as the litmus test that this package builds and runs its tests with ROS not sourced."

```bash
# with ROS unsourced:
colcon build --packages-select core_types
colcon test --packages-select core_types
```

## What's here

Four headers under `include/core_types/`, all in namespace `core_types`:

- **`pose.hpp` -- `struct Pose`**: plain position + orientation quaternion (`x, y, z, qx, qy, qz,
  qw`, `qw` defaulting to `1.0` for identity rotation). Adapters translate to/from
  `geometry_msgs::msg::Pose` at the ROS boundary; `Pose` itself never crosses that boundary.

- **`trajectory.hpp` -- `struct TrajectoryPoint` and `struct Trajectory`**: a plain
  time-parameterized joint trajectory. `TrajectoryPoint` holds `positions`, `velocities`,
  `accelerations` (each `std::vector<double>`, ordered per `Trajectory::joint_names`) and
  `time_from_start_sec`. `Trajectory` holds `joint_names` (`std::vector<std::string>`) and
  `points` (`std::vector<TrajectoryPoint>`, increasing `time_from_start_sec`). Adapters translate
  to/from `trajectory_msgs::msg::JointTrajectory`; this is the type that lets a `Planner` (see
  `motion_planner::Planner`) return what it planned without also executing it.

- **`logger.hpp` -- `class Logger`**: abstract logging port with two pure-virtual methods,
  `info(const std::string &)` and `warn(const std::string &)`, both `const`. Only info/warn are
  declared since those are the only levels any current core-logic call site uses. A concrete
  adapter (e.g. an `rclcpp::Logger` wrapper) is injected at construction time so core logic never
  depends on `rclcpp` directly.

- **`null_logger.hpp` -- `class NullLogger : public Logger`**: a no-op implementation (`info`/
  `warn` both empty bodies) used as the default logger before an adapter injects a real one. Keeps
  core code usable and testable without a logging backend.

That's the full surface: two POD types (`Pose`, `Trajectory`/`TrajectoryPoint`) and one port
(`Logger`) plus its null implementation. No math types (`Vec3`/`Quat`), no `ExecutionPort`, no
`ScenePort`/`IkPort`/`CollisionPort`/`Clock` -- those described in the broader architecture plan
either live elsewhere (e.g. `ExecutionPort` is defined in
`manipulation_manager/include/manipulation_manager/core/execution/execution_port.hpp`) or
aren't implemented yet.

## Who depends on this package

Grepped for `core_types` across `package.xml`/`CMakeLists.txt` and for `core_types::` in
source. Real consumers, all via `find_package(core_types REQUIRED)` +
`core_types::core_types`:

- **`ros_adapters`** -- the boundary-conversion package. `rclcpp_logger.hpp`'s `RclcppLogger`
  implements the `Logger` port over `rclcpp::Logger`; `ros_conversions.hpp`'s `toCore()`/`toRos()`
  free functions convert `geometry_msgs::msg::Pose` ↔ `core_types::Pose` and
  `trajectory_msgs::msg::JointTrajectory` ↔ `core_types::Trajectory`. Its own description calls
  itself "Generic ROS-backed implementations of core_types ports."
- **`motion_planner`** -- `motion_planner.hpp`'s `MotionPlanner` defaults its
  `logger_` member to `std::make_shared<core_types::NullLogger>()`; planning types and the
  `OmplPlanner`/wall-penetration check consume `Pose`/`Trajectory`.
- **`manipulation_manager`** -- `ManipulationManager`, `CoverageFeasibilityAnalyzer`,
  `CoverageSource`, `ExecutionPort`, `MotionExecReporter`, the MoveIt execution adapter, and marker
  utils all build on `Pose`/`Trajectory`; `ManipulationManager` also defaults its injected logger to
  `NullLogger` (`setLogger()` doc: "defaults to a no-op NullLogger").
- **`serum_demo_bt`** (in `applications/`) -- `wall_cleaning_action_server.cpp` includes
  `core_types/pose.hpp` and builds a `std::vector<core_types::Pose>` of core waypoints.

## The one-way dependency rule

Per `implementation_plan_manipulation_architecture.md` §2.3: **types ← cores ← adapters**. Cores
(`*_core` algorithm packages) never depend on adapters; nothing pure depends on `interfaces`
(the ROS msg/srv/action wire contract). `core_types` sits at the bottom of that arrow chain --
everything above it may depend on it, it depends on nothing in this repo. Keep it a domain/types
library only (types + ports + math), not a general-purpose utility drawer, so it doesn't accrete
feature logic that would pull in a real dependency and break the litmus test.

## Tests

`test/test_pose.cpp` and `test/test_trajectory.cpp`, both plain `gtest` (`ament_add_gtest`, no ROS
test fixtures):

- **`Pose`**: defaults to identity (`x=y=z=0`, `qx=qy=qz=0`, `qw=1`); fields are independently
  assignable.
- **`Trajectory`/`TrajectoryPoint`**: `Trajectory` defaults to empty (`joint_names`/`points` both
  empty); `TrajectoryPoint` defaults to zero `time_from_start_sec` and empty vectors; fields
  (`joint_names`, `positions`, `time_from_start_sec`) are independently assignable and preserved
  through a `push_back` onto `Trajectory::points`.

```bash
colcon test --packages-select core_types
colcon test-result --verbose --test-result-base build/core_types
```

## Package scope

Types + ports only, per §2.3's guardrail -- no feature logic, no ROS deps, no dependency on any
`*_core`/adapter/`interfaces` package in this repo. Boundary conversions
(`geometry_msgs`/`trajectory_msgs` ↔ these types) and concrete port implementations
(`RclcppLogger`) live one layer up, in `ros_adapters` -- see that package for the ROS side of
`Pose`/`Trajectory`/`Logger`.
