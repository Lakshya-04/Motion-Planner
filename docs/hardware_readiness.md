# Hardware readiness: what is untested and the commissioning order

**Nothing in this repository has run on a real robot.** This page is the honest list of what that means, in
the order a commissioning should go. Each item names the file and the parameter that holds the value, and
carries one status:

- **verified-in-sim**: exercised in MuJoCo or Gazebo (see [sim_mujoco.md](sim_mujoco.md), [sim_gazebo.md](sim_gazebo.md)); a simulator is not a robot.
- **host-tested**: covered by ROS-free unit tests (gtest or pytest) on a host machine only.
- **untested**: no test or simulation covers it, or it does not exist yet.

Every number in the files cited is a placeholder or a simulation value. A value this page calls "set by
commissioning" has no known-good setting; do not copy a simulation number onto a robot. The procedure for the
force path in detail is [hardware_bringup.md](hardware_bringup.md); calibrations are in
[calibration.md](calibration.md). Neither this software nor those procedures replace a hardware safety
function (see the e-stop item below).

## Commissioning order

Work top to bottom; stop at the first item that fails and treat that as a finding.

| # | Item | Status | Where the value lives | Value |
|---|---|---|---|---|
| 1 | Lift homing and stroke limit | host-tested (supervisor against a mock and a scripted transport); the real transport is untested | [`lift_system_example.yaml`](../src/control/lift_hardware/config/lift_system_example.yaml): `home_height_m`, `home_timeout_s`, `stroke_m`, `tolerance_m`, `move_timeout_s`; defaults in [`lift_config.hpp`](../src/control/lift_hardware/include/lift_hardware/core/lift_config.hpp) | set by commissioning |
| 2 | Lift velocity and acceleration limits | host-tested; verified-in-sim with the simulated lift | same file: `max_velocity_mps`, `max_acceleration_mps2`; reference stroke in a profile under [`config/profiles/`](../src/description/mobile_manipulator_description/config/profiles/) (`lift.stroke_m`, `lift.max_velocity_mps`) | set by commissioning |
| 3 | Lift device transport, fault codes, reconnect | untested: `transport: device` is an unimplemented skeleton ([`device_transport.hpp`](../src/control/lift_hardware/include/lift_hardware/core/device_transport.hpp) returns `kUnsupported`) | `reconnect_initial_s`, `reconnect_max_s`, `max_io_failures` in the example yaml; [lift.md](lift.md) | set by commissioning |
| 4 | F/T sign and frame | verified-in-sim (both simulators use sign +1); untested on a real sensor | [`hardware_template.yaml`](../src/control/compliant_trajectory_controller/config/hardware_template.yaml), `wrench_conditioner`: `sign`, `sensor_frame`, `tool_frame`, `world_frame`; JTAC `ft_sensor.frame.id` = `control.frame.id` | set by commissioning |
| 5 | F/T bias | host-tested (conditioner node test); services exercised only in simulation | `wrench_conditioner.bias`; services `~/capture_pose`, `~/identify`, `~/reset_bias_tracker` ([ft_conditioner README](../src/control/ft_conditioner/README.md)) | set by commissioning |
| 6 | Payload gravity compensation (mass, centre of gravity, lever arm) | verified-in-sim (conditioner error against ground truth under 0.005 N in both simulators, per [ROADMAP.md](ROADMAP.md) section 8); untested with a real tool | `wrench_conditioner.payload_mass_kg`, `payload_cog_xyz`, `tool_in_sensor_from_tf`, `tool_in_sensor_xyz`; JTAC `gravity_compensation.CoG.force` must stay 0 so gravity is not compensated twice | set by commissioning |
| 7 | Sensor noise filtering | untested: no tuned value exists | JTAC `ft_sensor.filter_coefficient` (1.0 means no smoothing, correct only for noise-free simulators); `control.ft.deadband_n`, `control.ft.clamp_n` in the tool profile block | set by commissioning |
| 8 | Controller (admittance) gains | verified-in-sim in MuJoCo (the force held close to its target); not production gains, no stability analysis for moving references | [`hardware_template.yaml`](../src/control/compliant_trajectory_controller/config/hardware_template.yaml) and [`jtac_controller.yaml`](../src/control/compliant_trajectory_controller/config/jtac_controller.yaml): `admittance.mass`, `admittance.damping_ratio`, `admittance.stiffness`, `admittance.joint_damping`; start with `admittance.selected_axes` all false | set by commissioning, per tool and per robot |
| 9 | Force limits per tool (target, abort, torque abort, speeds, offset cap) | host-tested (loader checks, `force_core` tests); not tied to any stop path | [`tool_profiles.yaml`](../src/perception/coverage_planner/config/tool_profiles.yaml): `task.target_force_n`, `task.overload_abort_n`, `task.tangential_torque_abort_nm`, `task.ee_speed_mps`, `task.approach_speed_mps`, `control.admittance.wrench_reference_z_n` (must equal `target_force_n`) | set by commissioning; commission well below the shipped values |
| 10 | Safety monitor limits | host-tested (gtests on distance sources, sphere model, curobo yaml, monitor core); no node, no topics, no consumer, no simulation run, not a certified safety function | `Params` in [`safety_monitor.hpp`](../src/control/safety_monitor/include/safety_monitor/safety_monitor.hpp): `stop_time_s`, `reaction_time_s`, `human_speed_mps`, `static_margin_m`, `person_intrusion_m`, `self_margin_m`, `static_slow_band_m`, `person_slow_band_m`, `stale_timeout_s`, `min_clear_time_s` | set by commissioning and a risk assessment |
| 11 | E-stop and protective-stop path | core only: `SafetyChain` ([safety_chain.md](safety_chain.md), host gtests) combines the inputs into a latched stop state, but no node or consumer uses it; the abort thresholds in item 9 and the monitor in item 10 do not connect to an e-stop or to the arm controller's own protective stop ([ROADMAP.md](ROADMAP.md) section 8) | the robot's own safety controller and a safety-rated stop circuit, outside this repository | set by commissioning, per the site risk assessment |
| 12 | Arm driver selection and joint order | host-tested (`real_driver:=mock` wiring, 28 pytest tests); no driver installed, plugin and parameter names are from public documentation | [`hardware_drivers.yaml`](../src/description/mobile_manipulator_description/config/hardware_drivers.yaml); `hardware_real.launch.py` arguments `real_driver`, `real_params`, `real_lift_plugin` | set by commissioning |
| 13 | Base navigation limits | verified-in-sim (planar-velocity base in both simulators, `/cmd_vel`, `/odom`); host-tested (`stand_navigator` pytest); a real drive controller does not exist | profile `base.max_linear_velocity_mps`, `base.max_angular_velocity_radps`, `base.max_linear_acceleration_mps2`, `base.max_angular_acceleration_radps2` under [`config/profiles/`](../src/description/mobile_manipulator_description/config/profiles/) (read by [`core.py`](../src/manipulation/stand_navigator/stand_navigator/core.py)); node parameters `tolerance_xy_m`, `timeout_s`, `odom_timeout_s` in [`ros_node.py`](../src/manipulation/stand_navigator/stand_navigator/ros_node.py) | profile numbers are nominal spec-sheet values; set by commissioning |
| 14 | Wall docking limits | host-tested (pytest, Monte Carlo); not started in the two-stand simulation passes | `max_linear_mps`, `max_angular_radps`, `tolerance_standoff_m`, `tolerance_yaw_rad`, `plane_max_age_s`, `wall_timeout_s`, `odom_timeout_s` in [`wall_docking_node.py`](../src/manipulation/stand_navigator/stand_navigator/wall_docking_node.py) | set by commissioning |

## Known gaps behind the table

- **Localisation and obstacle avoidance**: the base drives a straight line on simulator ground-truth odometry; there is no map, no localisation and no Nav2 ([navigation_and_perception.md](navigation_and_perception.md)). Status: untested on any real odometry.
- **Safety layer end to end**: no node publishes the monitor's speed scale or stop request, and nothing consumes it (item 10).
- **Real-sensor behaviour**: noise, thermal drift, cable forces and latency are not modelled in the simulators ([hardware_bringup.md](hardware_bringup.md)).
- **Wheel and floor physics**: wheels are fixed links in the simulators; slip and drive dynamics are untested ([ROADMAP.md](ROADMAP.md) section 9).

## Keeping this page honest

When an item is run on hardware, change its status here in the same change that records the result, and cite
the log or test. Do not promote an item to a stronger status than the evidence behind it.
