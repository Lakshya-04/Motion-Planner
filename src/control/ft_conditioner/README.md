# ft_conditioner

ROS node around `force_core`'s wrench conditioning: a raw force/torque sensor wrench in, the contact wrench at the tool point (in the tool axes) out, with services to identify payload mass, centre of gravity and bias from static poses.

## Tier

ROS node (library `ft_conditioner_lib` plus executable `wrench_conditioner_node`).

## Inputs and outputs

- Subscribes: `input_topic` (default `wrench_raw`, `geometry_msgs/WrenchStamped`); optionally `bias_tracking.contact_free_topic` (default `contact_free`, `std_msgs/Bool`) and `bias_tracking.joint_states_topic` (default `joint_states`) when bias tracking is on.
- Publishes: `output_topic` (default `wrench_conditioned`, `WrenchStamped`, frame = `tool_frame`); `~/tracked_bias` (`WrenchStamped`); `~/recalibrate_needed` (`std_msgs/Bool`).
- Services (`std_srvs/Trigger`): `~/capture_pose`, `~/identify`, `~/clear_samples`, `~/reset_bias_tracker`, `~/plan_excitation`.
- Parameters (selection): `world_frame`, `tool_frame`, `sensor_frame`, `sign`, `payload_mass_kg`, `payload_cog_xyz`, `bias`, `gravity_world`, `tool_in_sensor_from_tf`, `tool_in_sensor_xyz`, `tool_in_sensor_rpy`, `max_tf_age`, the `capture_*` limits, the `bias_tracking.*` and `excitation.*` groups. All are declared in `src/wrench_conditioner_node.cpp`.
- Launch: `launch/ft_conditioner.launch.py` (arguments mirror the main parameters).

## Dependencies

- In this repo: `force_core`, `core_types`.
- External: `rclcpp`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`, `tf2_ros`, `launch_ros`, `python3-yaml`.
- Needs ROS. No GPU. No simulator needed (it is fed by whatever publishes the raw wrench).

## Use it on its own

```bash
ros2 launch ft_conditioner ft_conditioner.launch.py input_topic:=/my_sensor/wrench payload_mass_kg:=0.8
```

With no TF frame for the sensor, add `tool_in_sensor_from_tf:=false tool_in_sensor_xyz:='[0,0,0.1]'`. Feed `output_topic` to `ft_bridge_hardware` (`source_topic`) or to your own controller.

## Tests

18 test cases in `test/wrench_conditioner_node_test.cpp` (counted from the source).

```bash
colcon test --packages-select ft_conditioner
```

## Status and limits

- Used in the MuJoCo and Gazebo simulations (`conditioner:=true`); its calibration services were exercised there only. Nothing here has run on hardware.
- It starts uncalibrated (payload 0) unless the parameters are set.
- Identification quality depends on static, contact-free poses; see the doc for the checks.

## Related docs

- [ft_sensing.md](../../../docs/ft_sensing.md)
- [calibration.md](../../../docs/calibration.md)
- [hardware_bringup.md](../../../docs/hardware_bringup.md)
