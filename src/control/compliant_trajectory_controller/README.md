# compliant_trajectory_controller

JTAC: a fused joint-trajectory and admittance controller for the lift plus arm chain. It follows a planned joint trajectory while overlaying a bounded, force-driven Cartesian compliance offset, and can freeze trajectory time until a commanded contact force converges.

## Tier

ROS node or plugin (pluginlib `controller_interface::ControllerInterface`, plus demo and probe executables).

## Inputs and outputs

- Plugin `compliant_trajectory_controller/JtacController` (`jtac_controller.xml`, library `jtac_controller`). It replaces `joint_trajectory_controller` and offers the same action.
- Action: `<controller_name>/follow_joint_trajectory` (`control_msgs/FollowJointTrajectory`).
- Topics (under the controller node): subscribes `~/target_force_n` and `~/max_offset_m` (`std_msgs/Float64`); publishes `~/measured_wrench` (`WrenchStamped`) and `~/debug_offset` (`Float64MultiArray`).
- Interfaces: position and velocity commands for the lift and arm joints; position/velocity state; an F/T sensor state (for example from `ft_bridge_hardware`).
- Parameters: the `admittance_controller` parameter schema (joints, interfaces, `kinematics.*`, `ft_sensor.*`, `control.frame.id`) plus `enable_redundancy_weighting`, `force_limiter.*` and `command_guard.*`.
- Config: `config/jtac_controller.yaml` (controller manager entry and test-robot values), `config/hardware_template.yaml` (every value to set for a real sensor; not loaded by any launch file).
- Launch: `launch/jtac_compliance_demo.launch.py`. Executables built here include `jtac_compliance_demo_node` and several probes (`moveit_jtac_execution_probe`, `wall_surface_probe`, `jtac_wall_approach_test_node`, ...).

## Dependencies

- In this repo: `core_types`, `force_core`, `ros_adapters`, `ft_bridge_hardware`, `mobile_manipulator_description`, `mobile_manipulator_profile`.
- External: ros2_control, `admittance_controller`, `kinematics_interface_kdl`, MoveIt (`moveit_core`, `moveit_ros_planning`), `octomap`, `rclcpp_action`, `realtime_tools`.
- Needs ROS 2 and MoveIt. No GPU. Runs against mock hardware; no simulator needed.

## Use it on its own

```bash
ros2 launch compliant_trajectory_controller jtac_compliance_demo.launch.py profile:=ur10e_telescopic
```

For a controller manager of your own, set `type: compliant_trajectory_controller/JtacController` as in `config/jtac_controller.yaml`; `robot_description` must be given to the controller's own node (the file explains how).

## Tests

26 test cases in 5 files (counted from the sources): admittance integration, redundancy weighting, compliance port, pose inputs, force-stage sequencer integration.

```bash
colcon test --packages-select compliant_trajectory_controller
```

## Status and limits

- Shown in mock hardware, MuJoCo and Gazebo passes. Nothing here has run on hardware; the commissioning steps are written but unverified (see `docs/hardware_bringup.md`).
- Written for a lift plus 6-DOF arm (7 actuated joints); the redundancy weighting needs `kinematics.tip`, `control.frame.id` and `ft_sensor.frame.id` to be the same frame.
- It only rotates wrenches; the lever-arm move and gravity belong to `ft_conditioner` or the controller's own compensation.

## Related docs

- [ft_sensing.md](../../../docs/ft_sensing.md)
- [hardware_bringup.md](../../../docs/hardware_bringup.md)
- [design_principles.md](../../../docs/design_principles.md)
- [sim_mujoco.md](../../../docs/sim_mujoco.md)
