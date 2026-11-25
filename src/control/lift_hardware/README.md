# lift_hardware

ros2_control system component for one prismatic lift joint, over an abstract `LiftTransport`, with a ROS-free core that homes, clamps, rate-limits, holds on a fault and reconnects. The protocol of a real device is not in this package.

## Tier

ROS node or plugin (pluginlib `hardware_interface::SystemInterface`) over a ROS-free core library (`lift_hardware_core`).

## Inputs and outputs

- Plugin `lift_hardware/PrismaticLiftSystem` (`lift_hardware.xml`, library `lift_hardware_ros`). One `<joint>`: command `position` (and `velocity`, exported and ignored, when the description lists it), state `position` and `velocity`. An optional `<gpio>` adds the states `homed`, `phase`, `fault_code`, `stop_confirmed` and the command `reset_fault`.
- Hardware parameters: `transport` (`mock` default, `device`) and the keys of `LiftConfig` (`core/lift_config.hpp`), listed in [docs/lift.md](../../../docs/lift.md#parameters).
- `LiftTransport` (`core/lift_transport.hpp`): connect, disconnect, read_stroke, move_to(height_m), stop, read (height, state, fault). Transports here: `MockTransport` (simulated trapezoidal motion with travel limits), `ScriptedTransport` (tests, fault injection), `DeviceTransport` (skeleton, every call returns `unsupported`).
- `LiftSupervisor` (`core/lift_supervisor.hpp`): `poll(now)`, `command(now, height)`, `reset_fault(now)`, `shutdown()`, `snapshot()`.

## Dependencies

- In this repo: none.
- External: `hardware_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle` (the plugin only; the core needs none of them).
- Needs ROS 2 and ros2_control for the plugin. No GPU, no simulator, no device.

## Use it on its own

Declare it as the lift's system block (the description's `hardware:=real` takes `real_lift_plugin:=lift_hardware/PrismaticLiftSystem`, see [docs/hardware_bringup.md](../../../docs/hardware_bringup.md)):

```xml
<ros2_control name="lift_system" type="system">
  <hardware>
    <plugin>lift_hardware/PrismaticLiftSystem</plugin>
    <param name="transport">mock</param>
    <param name="mock_stroke_m">0.8</param>
    <param name="max_velocity_mps">0.1</param>
    <param name="max_acceleration_mps2">0.08</param>
  </hardware>
  <joint name="lift_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

The core runs without ROS: link `lift_hardware_core`, build a `LiftSupervisor` over a transport and call `poll` then `command` each cycle.

## Tests

48 test cases in 4 files (`test_profile.cpp`, `test_lift_config.cpp`, `test_mock_transport.cpp`, `test_supervisor.cpp`), counted from the sources.

```bash
colcon test --packages-select lift_hardware
# or, without ROS: g++ -std=c++17 -Iinclude src/core/*.cpp test/core/test_supervisor.cpp -lgtest -lgtest_main -pthread
```

## Status and limits

- The core tests were built and run with host g++ and gtest. The plugin, `CMakeLists.txt`, `package.xml` and the plugin XML have not been built in a workspace (the plugin was syntax-checked against Humble headers only), and nothing has run on a device.
- A real device needs its own `LiftTransport`, written outside this repository; [docs/lift.md](../../../docs/lift.md#plugging-in-a-real-device) has the checklist. `transport_factory.cpp` knows only `mock` and `device`.
- Models one joint. The mimic stages of the description follow it and are not commanded.

## Related docs

[docs/lift.md](../../../docs/lift.md), [docs/robot_description.md](../../../docs/robot_description.md#lift-variants), [docs/package_layout.md](../../../docs/package_layout.md).
