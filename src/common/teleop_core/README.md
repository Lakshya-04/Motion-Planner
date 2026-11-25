---
type: Reference
title: teleop_core
description: Package README for the ROS-free teleoperation logic library -- AUTO/TELEOP/HOLD mode manager, jog shaper, jog limiter. Eigen only, no node yet.
tags: [teleop-core, teleoperation, package-readme]
status: experimental
---

# teleop_core

Pure C++ logic around operator teleoperation (Eigen only; no `rclcpp`, no clock, no I/O). It builds and
tests with ROS not sourced. It is meant to sit around a Cartesian jog engine (MoveIt Servo, planned) and the
JTAC controller. **No node wraps it yet, and nothing has been run in a simulator or on hardware.**

| Item | Value |
|---|---|
| Version / licence | 0.1.0 / Apache-2.0 |
| Namespace | `teleop_core` |
| Dependencies | Eigen3 (build), `ament_cmake`; tests: `ament_cmake_gtest` |
| Target | static library `teleop_core` |

## What's here

- **`teleop_mode_manager.hpp` -- `TeleopModeManager`**: modes AUTO / TELEOP / HOLD, an explicit transition table,
  a reason code per request, a counter of refused requests, TELEOP input timeout falling back to HOLD.
- **`jog_shaper.hpp` -- `JogShaper`**: 6D normalised input to a base-frame twist: deadband, expo, scaling,
  axis mask, slew limit, watchdog, tool/base/world frame.
- **`jog_limiter.hpp` -- `JogLimiter`**: uniform safety speed scale, stop, fail-safe on a stale or invalid scale.

The transition table and the shaping pipeline are documented in [docs/teleop.md](../../../docs/teleop.md).

## Tests

`test/` has three gtest files (21, 18 and 11 tests), compiled and run with host g++ only; they have not been run
through `colcon test`.

```bash
colcon build --packages-select teleop_core && colcon test --packages-select teleop_core
```

## Not done

No ROS node, no MoveIt Servo wiring, no joint-space jog, untuned defaults.
