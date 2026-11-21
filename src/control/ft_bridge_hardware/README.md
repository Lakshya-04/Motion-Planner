# ft_bridge_hardware

ros2_control sensor hardware component that bridges a `geometry_msgs/WrenchStamped` topic (a real F/T sensor driver, a conditioner, or a scriptable test topic) into the state interfaces `admittance_controller` expects, with a settable force-setpoint bias. No vendor changes.

## Tier

ROS node or plugin (pluginlib `hardware_interface::SensorInterface`).

## Inputs and outputs

- Plugin `ft_bridge_hardware/FtBridgeHardwareInterface` (`ft_bridge_hardware.xml`, library `ft_bridge_hardware`). It exports the sensor's `force.*` and `torque.*` state interfaces under the sensor name given in the ros2_control description.
- Hardware parameters (the `<param>` block, parsed in `ft_bridge_config.hpp`): `source_topic` (default is the topic name of one particular sensor driver), `injection_mode` (default false), `injection_topic` (default `~/injected_wrench`), `setpoint_topic` (default `~/target_wrench`, `geometry_msgs/Wrench`).
- With `injection_mode` true the component reads `injection_topic` instead of `source_topic`, so a controller can be driven with scripted forces.
- The setpoint is subtracted from the exported wrench, which turns the zero-seeking admittance law into a target-seeking one (`wrench_bias.hpp`).
- Plugin `ft_bridge_hardware/FtBridgeSimSystem` (`ft_bridge_sim_system.xml`, library `ft_bridge_hardware_gz`): the same bridge under the `gz_ros2_control` base class; built only where `gz_ros2_control` and `gz-sim8` are found.

## Dependencies

- In this repo: none.
- External: `hardware_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle`, `geometry_msgs`; optional `gz_ros2_control`.
- Needs ROS 2 and ros2_control. No GPU. No simulator needed.

## Use it on its own

Declare it in a ros2_control description as a sensor (see `mobile_manipulator_description/urdf/mobile_manipulator.ros2_control.xacro` for the working block):

```xml
<sensor name="ft_sensor">
  <hardware>
    <plugin>ft_bridge_hardware/FtBridgeHardwareInterface</plugin>
    <param name="source_topic">/my_sensor/wrench</param>
  </hardware>
</sensor>
```

## Tests

8 test cases in 2 files (`test_wrench_bias.cpp`, `test_ft_bridge_config.cpp`), counted from the sources.

```bash
colcon test --packages-select ft_bridge_hardware
```

## Status and limits

- The tests cover config parsing and the bias maths; the component itself runs inside the simulations and the mock-hardware demo. Nothing here has run on hardware.
- The default `source_topic` names one particular driver's topic: always set it for your sensor.

## Related docs

- [ft_sensing.md](../../../docs/ft_sensing.md)
- [hardware_bringup.md](../../../docs/hardware_bringup.md)
- [simulation.md](../../../docs/simulation.md)
