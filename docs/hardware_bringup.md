# Hardware bring-up: commissioning the force path

A procedure for taking the force path (F/T sensor, `ft_conditioner`, `ft_bridge_hardware`, JTAC, tool
profile) from the simulators to a real arm and a real wrist sensor. **Nothing in this repository has been
run on hardware.** The procedure is derived from the code and the simulation results; every threshold below
is a starting value to be lowered or tuned, and a step that fails on your robot is a finding, not a mistake
in the procedure. Hardware brings a hardware safety function (e-stop, speed and force limits of the robot
controller) that this software does not replace: no abort threshold here is tied to an e-stop path yet
([ROADMAP.md](ROADMAP.md) section 8).

Related: [ft_sensing.md](ft_sensing.md) (the conditioning maths and services),
[tools.md](tools.md) (tool profile fields), [sim_mujoco.md](sim_mujoco.md) and
[sim_gazebo.md](sim_gazebo.md) (what the simulations measured),
[`hardware_template.yaml`](../src/control/compliant_trajectory_controller/config/hardware_template.yaml)
(every value to set, with safe defaults marked).

## 0. Before powering anything

1. Select the driver for your arm (section 0a) and fill in the template: copy `hardware_template.yaml`, replace every `MUST SET` entry, leave `MEASURED`
   entries at zero until their step below. No launch file loads the template yet; pass the sections to the
   nodes with `--params-file` or merge them into your own yaml.
2. The chain and its names must agree: sensor driver topic = conditioner `input_topic`; conditioner
   `output_topic` = bridge `source_topic`; ros2_control `<sensor name>` = JTAC `ft_sensor.name`; JTAC
   `ft_sensor.frame.id` = `control.frame.id` = the frame the wrench is already in (the TCP when the
   conditioner runs). JTAC only rotates wrenches, it never moves them with a lever arm.
3. `injection_mode` of the bridge must be `false`.
4. Robot side: set the controller's own speed, force and workspace limits, and check the e-stop, with the
   tool mounted and the wall out of reach. Mount the tool, plate and adapter exactly as they will run:
   payload identification is only valid for that assembly.
5. Start with JTAC's axes all deselected (`selected_axes` all false: the shipped default), so the arm
   follows trajectories with no force obligation.

## 0a. Vendor drivers: `hardware:=real`

`hardware:=real` is the real-robot counterpart of `hardware:=mujoco|gazebo` ([simulation.md](simulation.md)):
the same description and controllers, with the ros2_control plugin of the arm vendor. The vendor drivers are
**not vendored and not a dependency** of this repository; install the one you need and source its workspace.
[`hardware_drivers.yaml`](../src/description/mobile_manipulator_description/config/hardware_drivers.yaml) maps
the profile's `arm` to the driver:

| Arm (`arm:`) | Driver key | Package to install | ros2_control plugin | Required `real_params` | Wrench source |
|---|---|---|---|---|---|
| `xarm5/6/7`, `lite6`, `uf850` | `ufactory` | `xarm_controller` (xarm_ros2) | `xarm_control/UFRobotSystemHardware` | `robot_ip` | wrist sensor topic `/xarm/uf_ftsensor_ext_states` through `ft_bridge_hardware`, sensor `uf_ft_sensor` |
| `ur3` ... `ur30` | `ur` | `ur_robot_driver` | `ur_robot_driver/URPositionHardwareInterface` | `robot_ip`, `script_filename`, `output_recipe_filename`, `input_recipe_filename` | the driver's own `tcp_fts_sensor` (controller's TCP wrench) |
| `panda`, `fr3` | `franka` | `franka_hardware` (franka_ros2) | `franka_hardware/FrankaHardwareInterface` | `robot_ip` | none declared: only an estimated external wrench exists |
| any (explicit `real_driver:=mock`) | `mock` | none | `mock_components/GenericSystem` | none | scripted (`injection_mode`) |

```bash
ros2 launch mobile_manipulator_description hardware_real.launch.py profile:=ur10e_telescopic \
    real_params:="robot_ip=192.168.1.10,script_filename=...,output_recipe_filename=...,input_recipe_filename=..." \
    real_lift_plugin:=my_lift/LiftSystem controllers_file:=my_controllers.yaml controller:=combined_arm_lift_controller
ros2 launch mobile_manipulator_description hardware_real.launch.py profile:=generic_6dof_telescopic real_driver:=mock
xacro urdf/mobile_manipulator.urdf.xacro arm:=generic_6dof lift:=none hardware:=real real_driver:=mock
```

- The launch checks the choice before starting anything and, on a problem, prints one message and shuts down:
  no driver listed for the arm, a missing required parameter (named, with the `real_params` to pass), a moving
  lift without `real_lift_plugin`, or a driver package that is not installed (it names the package).
  `real_driver:=mock` needs no package and no robot: it is the wiring test used by
  `test/test_hardware_real.py` (profile to driver table, argument validation, xacro expansion).
- The lift is not driven by an arm vendor driver. A profile with a moving lift needs `real_lift_plugin` (your
  lift's ros2_control plugin; this repository has none), so a real lift is never silently mocked.
- The description emits four blocks: `lift_system`, `arm_system` (driver plugin, arm joints from the profile,
  position and velocity command for JTAC), `ft_sensor_bridge` (only for a wrist-sensor topic) and
  `ft_conditioned_bridge` (sensor `ft_sensor_tcp`, as for `hardware:=mujoco`; all zero until the
  `ft_conditioner` runs). Which sensor JTAC reads (`ft_sensor.name`) must match the table's sensor name
  (section 0, step 2).
- **Unverified.** The plugin names, parameters, sensor names and command interfaces in the table come from
  the vendors' public documentation, not from an installed driver and not from a robot. Before the first
  power-up check, with the driver installed: the plugin class and parameter names against its version
  (`ros2 pkg prefix`, its README), that the joint order of the profile (`joints.arm`) is the driver's, that
  the driver exports the velocity command interface JTAC needs, and the frame of the wrench it reports.
- Not done: the vendor controller's own limits, safety and e-stop wiring (ROADMAP section 8); the UR and
  xArm driver launch files and controller yaml are not wrapped, only the ros2_control side.

Commissioning continues with the sensor and calibration steps below; every calibration is in
[calibration.md](calibration.md) and the order to run them is the table at the top of that page.

## 1. Sensor alive, nothing moving

- Echo the raw topic: rate, frame id, stamp. Confirm the stamp is current and the rate is steady.
- Record 10 s with the arm still and nothing touching the tool. Look at noise (sets
  `force_zero_threshold_n` and the filter, which must exceed the noise after biasing) and drift.
- The conditioner drops wrenches whose TF is older than `max_tf_age` (0.1 s) and counts them; make sure
  nothing is being dropped (`sensor_frame` and `tool_frame` must both be in TF).

## 2. Frames and lever arm

- `tool_frame` is the TCP: check `tcp_xyz` / `tcp_rpy` against a physical measurement of the tool (flange
  to contact point). A wrong TCP shows up later as a pose-dependent torque residual.
- With `tool_in_sensor_from_tf: true`, check the transform sensor frame -> TCP with `ros2 run tf2_ros
  tf2_echo`; otherwise measure it and set `tool_in_sensor_xyz` / `_rpy`.
- The z axis of `world_frame` must point up.

## 3. Sign check (before any calibration)

The sign cannot be inferred from the sensor datasheet alone; measure it.

1. Arm still, tool free. Hang or hold a known mass on the tool, or just note the sign of the weight
   reading: the sensor must show the tool's weight in the direction of gravity expressed in its own axes.
2. Press gently by hand on the tool tip, from the +Z side, towards the robot (the direction in which a wall
   would push back on the tool). The conditioned output must read negative `force_z` at the tool point: the
   convention is the wrench the environment applies to the tool, so pressing the tool onto a wall along its +Z
   gives about minus the target.
3. If the conditioned force has the wrong sign, set `sign: -1.0`. The calibration below also detects it
   indirectly: a negative fitted mass means the sign is wrong, repeat with `sign = -1`.
4. The simulators publish the sign convention `+1` and each measures its own sign in a test; that says
   nothing about your sensor.

## 4. Payload identification with the conditioner services

All with nothing touching the tool and the arm still. Low accelerations only: the fit ignores inertial terms.

1. Launch the conditioner with the best known starting values (`payload_mass_kg`, etc. may stay zero).
2. Move to a pose, wait for the arm to settle, call `ros2 service call /wrench_conditioner/capture_pose
   std_srvs/srv/Trigger`. It averages the last 0.5 s and refuses if the input is stale or too short, the
   force spread exceeds `capture_max_spread_n` (1 N) or the orientation drifts more than
   `capture_max_rotation_rad` (0.02 rad).
3. Repeat in at least four poses in which gravity points in clearly different directions in the sensor
   frame (at least 10 degrees apart; rotate the wrist about at least two non-parallel axes). Six poses is
   what the simulation checks used.
4. Call `/wrench_conditioner/identify`. It fits mass, centre of gravity (in the sensor frame) and the six
   bias values, uses them immediately, and returns a YAML snippet: write it into the template
   (`payload_mass_kg`, `payload_cog_xyz`, `bias`). `/wrench_conditioner/clear_samples` starts over. A
   degenerate set of poses is rejected with a reason.
5. Check the result: the identified mass should be close to the weighed tool + plate + adapter (the
   simulations recovered 0.5 and 0.8 kg to 0.2 percent; a real sensor is noisier). Move through poses that
   were not used in the fit: the conditioned force must stay near zero (within the noise from step 1) in
   every pose. A residual that depends on the pose means a wrong mass, centre of gravity, TCP or frame.
6. Bias drifts with temperature and time; the repository has no tracking for it. Repeat step 1-4 after a
   warm-up and whenever the tool is changed or re-mounted.

## 5. Free-space motion with JTAC, force not engaged

- Run a short trajectory far from any surface with all axes deselected. Confirm that the arm tracks it, and
  watch the conditioned wrench through the motion: deviations during acceleration are inertial terms that
  the conditioner does not model (there is no inertial compensation); keep speeds low enough that it is
  small compared with the contact threshold.

## 6. Stiffness and limits check

Do this before any contact, in the reverse order from what is tempting: limits first.

1. Set the tool profile abort thresholds low for commissioning, a fraction of the shipped values
   (`overload_abort_n` ships at 60 N, `tangential_torque_abort_nm` at 8 Nm; use something near the target
   force while commissioning). These values are checked by the force stage; they are not an e-stop.
2. Set `control.admittance.max_offset_m` (shipped 0.05 m) to the smallest value that the process needs.
   The compliance offset builds on every axis while the tool is blocked and releases slowly (about 3 s) after
   the reference retreats ([sim_mujoco.md](sim_mujoco.md)); a large value means a large travel into the wall.
3. Remember what the gains do: the admittance spring pulls toward the reference, so the steady force is
   `target + stiffness * (depth of the reference past the surface)`. Place references on the surface. With
   the shipped Z stiffness of 300 N/m, 5 mm of error is 1.5 N; a stiffer wall or a rigid tool makes the same
   reference error far worse (see the Gazebo notes: the rigid tool saw about 105 N at 35 mm past the wall).
4. The mass in the diagnostic gains carries an empirical 4x safety factor against discrete instability when
   the reference moves ([jtac_controller.yaml](../src/control/compliant_trajectory_controller/config/jtac_controller.yaml)).
   Do not lower the mass or raise stiffness without repeating a moving-reference test in free space with zero
   applied force: the failure signature is the desired velocity alternating in sign and growing each cycle.
5. Gains are not production values for any tool or robot. Per tool, change one thing at a time.

## 7. Slow first contact

1. Hold the arm 10 to 20 cm in front of the wall in the approach pose. Stroke speed and approach speed at a
   small fraction of the profile values (shipped 0.10 m/s stroke and 0.05 m/s approach): a first contact builds
   about `k * speed * dt` of force per control step on a stiff surface, so the speed bounds the first spike.
2. A target force of a few newtons, far below the production target, with the reference on the surface.
3. Engage the tool Z axis only and approach. Watch the conditioned force, the commanded offset and the
   abort state. Expect creep: the simulation's first contact built about 13 N for a 10 N target for 2 s before
   settling; a real tool and wall will differ.
4. Retreat and confirm the force goes to zero and the offset releases. Stop at once on a force above the
   abort threshold, a force that grows while the target is constant, or any oscillation.
5. Only after repeatable contact at the low target, raise the target in steps, then the speed, then slide along
   the wall (friction adds tangential force and torque at the TCP: watch the tangential torque abort).
6. Sign and frame under load: during contact the conditioned force must be along the wall normal, with only
   friction-sized tangential components. A large contact torque at the TCP at a purely normal press means the
   TCP or lever arm is wrong.

## 8. Abort thresholds

| Threshold | Where | Shipped | Note |
|---|---|---|---|
| `overload_abort_n` | `tool_profiles.yaml` `task:` (contact tool) | 60 N | instantaneous abort above this |
| `tangential_torque_abort_nm` | same | 8 Nm | abort above this torque at the TCP |
| `unexpected_contact_abort_n` | `task:` (non-contact tool) | 10 N | any force above this means a collision |
| `settle_timeout_s` | same | 5 s | force settle watchdog |
| `max_offset_m` | `control.admittance:` | 0.05 m | largest compliance offset |
| `deadband_n`, `clamp_n` | `control.ft:` | 0.3 N, 100 N | readings below are zeroed, above are clamped |

Opt-in command guard (`command_guard.enabled: true` on the JTAC controller): every command is checked for NaN and
against `joint_limits.<joint>.min_position` / `max_position` (include the lift) before it is written; a bad cycle holds
the last good command, `command_guard.debounce_cycles` (3) bad cycles in a row latch a fault, `update()` returns
ERROR (the controller deactivates) and a non-finite measured position counts as a dead axis, never healthy. A latch
clears only on re-activation. Core: `core_types/command_guard.hpp`. Not yet: a `fault` state interface, the hardware
valve forced off from a fault, drive-side watchdog.

All are starting points to tune on the real tool. They live in the software layer; the robot's own safety
limits and the e-stop have to be the last line, and the link between the two does not exist yet.

## 9. What the simulations do not model

The results in [sim_mujoco.md](sim_mujoco.md), [sim_gazebo.md](sim_gazebo.md) and [ft_sensing.md](ft_sensing.md)
(10.13 N held for a 10 N target, conditioner error below 0.005 N) hold for the models below, not for a robot.

| Real hardware has | The simulations |
|---|---|
| Sensor noise, bias, temperature drift, the sensor's own dynamics | Noise-free, no bias, no drift; the wrench is the exact rigid-body value each step, so the conditioner's errors measure only the model, not a real sensor |
| Sensor compliance, cable forces | A rigid sensor, no cable or hose dynamics |
| Signal delay, jitter, sample-and-hold of the driver | Simulated time advances in fixed steps with no sensor or network latency |
| Gearbox friction, backlash, motor and speed limits, thermal limits | Ideal torque-limited PD joints (MuJoCo) or ideal position/velocity interfaces (Gazebo); URDF velocity and effort limits are not enforced in Gazebo; MuJoCo has no speed or acceleration limits |
| A base on wheels with slip, floor flex, 4-wheel-steering kinematics | A kinematic planar-velocity base in MuJoCo (no floor, no wheels) and a static base in Gazebo |
| Tool compliance, wear, a wet or sticky surface, real wall stiffness and friction | MuJoCo: soft convex contact, stiffness set by `solref` and joint gains; Gazebo: a rigid tool and wall with a 5 N/mm tip spring added to make 10 N holdable, Coulomb friction only |
| Tool and payload inertia effects while accelerating | Static payload compensation only (no inertial term) in both the conditioner and the simulations |
| Sensor mounting that differs from the model (sign, orientation, offset) | Frames and signs are exact by construction; each backend only measures its own sign |
| Perception noise, localisation error, base-to-wall error | Wall poses are exact; no perception in the loop of the contact tests |
| The vendor controller's safety layer and e-stop | None |

Open items for hardware, from the roadmap: payload identification with safe excitation trajectories,
bias and temperature drift tracking, per-tool production gains and a stability analysis for moving
references, inertial compensation and filtering, an end-to-end safety layer, `hardware:=real` description
branch and vendor-driver selection, and the calibration of tool TCP, wall plane and base-to-wall in one place.

## 10. Record what you find

For each step, record the measured values (noise level, identified mass and centre of gravity, bias, sign,
first-contact peak, steady force against the target, abort-threshold behaviour) next to the configuration
that produced them. Update this guide and [ROADMAP.md](ROADMAP.md) with what failed; only tick roadmap items
that a hardware run actually verified.
