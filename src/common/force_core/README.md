# force_core

Pure C++ force-handling logic: the contact-force stage sequencing (approach, Cartesian stroke, retract), a force limiter, the wrench conditioning maths, payload identification and a bias tracker. No `rclcpp`, no ROS messages, no clock inside the maths.

## Tier

ROS-free core library (`ament_cmake` package, plain `add_library`, C++17).

## Inputs and outputs

No topics or services. It is a C++ library in namespace `force_core`; headers under `include/force_core/`:

- `force_stage_fsm.hpp`: `ForceStageFsm` (a state machine kept free of clock and I/O), `ForceFsmParams`, `ForceFsmStep`.
- `force_stage_sequencer.hpp`: `ForceStageSequencer` drives the FSM against a `core_types::CompliancePort` that you implement; reports `ForceSegmentTelemetry` and `ForceSequenceOutcome`. It runs its force trim loop on a `std::thread`.
- `force_limiter.hpp`: `ForceLimiter`, `NormalAxisLimiter` and their parameter structs.
- `wrench_conditioning.hpp`: `WrenchConditioner` (sign, bias, payload gravity, lever-arm move), `ConditionerConfig`, `Payload`, `FrameOffset`, `CalibrationSample` / `CalibrationResult`.
- `payload_excitation.hpp`: planning of static poses for payload identification (`ExcitationPlan`, `ExcitationOptions`).
- `bias_tracker.hpp`: `BiasTracker`, `BiasTrackerConfig`.

## Dependencies

- In this repo: `core_types` (`Wrench`, `CompliancePort`, `Pose`).
- External: C++17 compiler, `Threads`. `gtest` for the tests.
- Builds and runs without ROS sourced, without a GPU, without a simulator.

## Use it on its own

```cmake
find_package(force_core REQUIRED)
target_link_libraries(my_target force_core::force_core)
```

```bash
colcon build --packages-select core_types force_core   # ROS not sourced
```

## Tests

72 test cases in 6 files under `test/` (counted from the sources), plain `gtest`.

```bash
colcon test --packages-select force_core && colcon test-result --verbose
```

## Status and limits

- Unit-tested only; the sequencer and the limiter are exercised end to end in simulation through the compliant controller (see `compliant_trajectory_controller`), not on hardware. Nothing here has run on hardware.
- Thresholds and gains are starting values for a simulated arm and sensor, not tuned for a real one.
- `tare()` is deliberately not called by the sequencer; the caller owns it.

## Related docs

- [ft_sensing.md](../../../docs/ft_sensing.md)
- [hardware_bringup.md](../../../docs/hardware_bringup.md)
- [design_principles.md](../../../docs/design_principles.md)
- [package_layout.md](../../../docs/package_layout.md)
