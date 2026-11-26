# cartesian_tracking_controller

A chainable ros2_control controller that moves the tool contact point to a reference pose with joint velocity commands. It is a thin plugin around `diff_ik_core::CartesianTracker` (see `docs/development_plan.md`, section 10): the controller reads joint positions, gets the flange pose and Jacobian from `kinematics_interface` (KDL), and hands them to the tracker.

## Tier

ROS node or plugin (pluginlib `controller_interface::ChainableControllerInterface`).

## Inputs and outputs

- Plugin `cartesian_tracking_controller/CartesianTrackingController` (`cartesian_tracking_controller.xml`).
- State interfaces: `<joint>/position` for every joint in `joints`.
- Command interfaces: `<joint>/velocity` (required) and optionally `<joint>/position` (the tracker's integrated next position; while holding, the position at the start of the hold).
- Reference, in the kinematics base frame (frame ids are not checked):
  - `~/reference_pose` (`geometry_msgs/PoseStamped`): wanted contact pose.
  - `~/reference_twist` (`geometry_msgs/TwistStamped`): feedforward, optional; none or stale means zero.
  - Chained: the same 13 values as exported reference interfaces `<controller>/pose.position.{x,y,z}`, `pose.orientation.{x,y,z,w}`, `twist.linear.{x,y,z}`, `twist.angular.{x,y,z}`. Topics are ignored while chained.
- `~/tool_offset` (`std_msgs/Float64MultiArray`, `[x y z roll pitch yaw]`, flange to contact): swapped in at the start of the next update. Wrong size or non-finite is ignored with a warning.
- `~/plane_measurement` (`std_msgs/Float64MultiArray`, `[nx ny nz standoff_error ax ay az]`, see `planeServoTwist`): only read when `plane.enabled` is true; a missing or stale one switches the servo off for that step.
- `~/diagnostics` (`std_msgs/Float64MultiArray`, labels in `layout.dim`): `[status, position_error, orientation_error, sigma_min, twist_scale, joint_limit_margin]`. The last five are NaN while holding.

Status codes: 0 ok, 1 no reference yet, 2 stale reference, 3 invalid reference (non-finite, zero quaternion), 4 kinematics failed, 5 tracker rejected its input, 6 bad joint state. Anything but 0 holds: velocity 0, position held. The reference is stale after `reference_timeout` seconds without a new pose (<= 0 turns it off). Messages from before activation are ignored.

## Parameters

`joints` (ordered, lift first when present, at most 8), `command_interfaces` (`velocity`, `position`), `kinematics.{base,tip,alpha,plugin_name,plugin_package}` (tip is the flange), `tool.xyz`, `tool.rpy`, `max_linear_twist`, `max_angular_twist`, `feedback.{kp_linear,kp_angular,max_linear,max_angular}`, `plane.{enabled,kp_distance,kp_tilt,max_linear,max_angular}`, `sigma_threshold`, `lambda_max`, `joint_weights`, `joint_acceleration_limits` (empty = unlimited), `position_margin`, `reference_timeout`, `diagnostics_rate`. Position and velocity limits come from the URDF (a joint without a velocity limit fails configure). `dt` is 1 / update rate. All are read at configure; only the tool offset changes at run time. `config/cartesian_tracking_controller.yaml` is an example for the demo robot.

Configure checks that `joints` equals the movable joints from `kinematics.base` to `kinematics.tip`, in order.

## Dependencies

`diff_ik_core`, `controller_interface`, `kinematics_interface` (+ `kinematics_interface_kdl` at run time), `realtime_tools`, `urdf`, `geometry_msgs`, `std_msgs`.

## Tests

```bash
colcon test --packages-select cartesian_tracking_controller
```

`test_cartesian_tracking_controller`: the controller is loaded with hand-made interfaces on the demo URDF of `compliant_trajectory_controller` (read from the source tree). It covers configure and activate, a wrong joint order, a constant twist (the commanded joint velocities times the contact Jacobian give the twist, diagnostics filled), a wrong against the right tool offset and the hot swap from the wrong to the right one, a NaN pose and a NaN twist holding (velocity 0, position held), a stale reference, and chained mode giving the same commands as the topic path.

## Status and limits

- Simulation-only math: no simulator run, no controller_manager run, no hardware. The tests step `update()` by hand with fixed joint positions (no plant), so nothing closes the loop.
- No stability analysis (contact stiffness, loop delay, discretisation).
- No null-space costs are set, so the redundancy (lift plus arm) is used for the minimum-norm solution only.
- Real-time behaviour is not measured. `CartesianTracker::step` does not allocate (see `diff_ik_core`); the KDL calls, the Eigen temporaries in the controller and the diagnostics path were not checked.
- Frame ids on the reference are not checked, and the orientation is read as an `x y z w` quaternion and normalised.
- Not covered by tests: the plane servo path, the plane measurement topic, joint limit behaviour near a stop, the position command interface beyond the hold check, a joint list with a continuous joint.
