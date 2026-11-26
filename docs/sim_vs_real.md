# Simulation versus hardware

What the simulators model, what they leave out, and what to expect when a result moves to the robot. The numbers
in [sim_mujoco.md](sim_mujoco.md), [sim_gazebo.md](sim_gazebo.md) and [ft_sensing.md](ft_sensing.md) (10.13 N held for a
10 N target, conditioner error below 0.005 N) are best cases of an ideal sensor on a rigid-body model. Nothing here was
measured on hardware; the right-hand column is what the model implies or what is known from the sensor and driver
documentation, not a measurement. The commissioning side is [hardware_bringup.md](hardware_bringup.md) (section 9 is the
short form of this table); the overview of the backends is [simulation.md](simulation.md).

## Table

| Topic | MuJoCo | Gazebo | Real hardware, and what to do |
|---|---|---|---|
| Contact stiffness | Soft convex contact; the force at a given penetration is set by `solref` defaults and the joint gains, not by a measured wall (208 N for a 5 cm push with the position controller). | Rigid tool and wall plus a 5 N/mm spring in the tool tip, added so that 10 N is holdable | Plaster, concrete and paint differ by orders of magnitude, and the tool and sensor add their own compliance. Identify the stiffness in the slow first contact (hardware_bringup section 7), then set the JTAC gains. |
| Friction | Coulomb, MuJoCo defaults, no stick-slip or wet surface | Coulomb only | Surface, tool wear and moisture change it; a slip shows as a tangential force that the sim never produces. |
| Sensor noise | Off by default. Opt-in white noise density, quantisation (`sim_noise`, below) | Noise-free | Take the density, resolution and bandwidth from the sensor data sheet, then measure at rest ([ft_sensing.md](ft_sensing.md), hardware_bringup section 1). |
| Sensor bias and drift | Off by default; opt-in random-walk bias. Temperature is not modelled | None | Bias drifts with temperature and after a load cycle. Re-tare in free space before each contact phase; the conditioner services handle a bias, nothing tracks it online yet. |
| Sensor and driver latency | Off by default; opt-in fixed delay and first-order low-pass on the F/T, command delay on the joints. Simulated time is exact otherwise | None | Real chains add driver sample-and-hold, jitter and a network hop. Measure the loop delay and check the admittance gains against it. Jitter is not modelled (the delay is fixed). |
| Joint state | Exact; opt-in Gaussian noise on position and velocity | Exact | Encoder resolution, backlash, filtered velocity. |
| Actuator limits | Ideal torque-limited PD joints; no speed or acceleration limit; joint limits are soft | Ideal position and velocity interfaces; URDF velocity and effort limits not enforced | Vendor drivers clamp velocity, acceleration, and torque, add their own jerk limit, protective stops and thermal derating. The retiming layer reports limit violations, but the sims never trigger them. |
| Base | Kinematic planar velocity base (`base_motion:=true`); no floor, wheels or slip | Static base, or a planar-velocity plugin | Wheel slip, floor flex, 4-wheel-steering kinematics, odometry drift and a base that moves under reaction forces. Base-to-wall error is not simulated either. |
| Calibration | Frames, signs and the tool inertial are exact by construction | Same | Sensor sign and orientation, tool TCP, payload, wall plane, base-to-wall: [calibration.md](calibration.md). Each one is an error the sims do not contain. |
| Thermal drift | Not modelled | Not modelled | Sensor offset, link expansion and motor derating over a shift. Plan a periodic re-tare and a cool-down check. |
| Payload inertia | Static payload compensation only; inertial forces are in the raw wrench | Same | Fast motions with a heavy tool leak inertial force into the contact reading. |
| Perception | Exact wall pose in the contact tests; ray-cast depth with optional range noise; opt-in ray-cast 2D lidar (`lidar:=true`, optional range noise and dropout) | gz `depth_camera` with optional noise; opt-in `gpu_lidar` (`lidar:=true`, optional noise) | Real depth has multipath, holes and a bias; the wall plane error enters the contact reference. |
| Safety layer | Software obstacle stop in `stand_navigator` when it is given `/scan` (sim lidar, `run_lidar_stop.sh`); no e-stop | Same, `run_lidar_stop.sh` |  The vendor controller's limits and the e-stop are the last line; the software layer is not one. |

## Opt-in sensor non-idealities in the MuJoCo plugin

`sim_noise` (launch argument of `sim_mujoco.launch.py`, a string of `key=value` pairs separated by spaces or commas; empty means ideal and nothing
changes) switches on a per-signal model in `sim_mujoco/signal_model.hpp`, applied inside the plugin once per controller
period. Order per sample: signal + bias + white noise, first-order low-pass, fixed delay, quantisation.

| Key | Applies to | Unit |
|---|---|---|
| `ft_noise_density` | each of the six raw F/T channels | N/sqrt(Hz) (N m/sqrt(Hz) for torque); per-sample std is density / sqrt(2 dt) |
| `ft_bias_walk` | same | per sqrt(s); the bias starts at 0 and does a random walk |
| `ft_lowpass_hz` | same | corner of a first-order low-pass |
| `ft_delay_ms` | same | fixed delay, rounded to whole controller periods |
| `ft_quantum` | same | quantisation step |
| `joint_pos_noise`, `joint_vel_noise` | joint state read by the controllers | std per sample, rad and rad/s |
| `control_delay_ms` | joint position and velocity commands | fixed latency |
| `seed` | all | random seed, so a run is repeatable |

Noise is added to the raw F/T that feeds both `/sim_mujoco/ft_raw` and the ros2_control sensor interface, so the
conditioner sees it; `/sim_mujoco/ft_ground_truth` is never touched, which keeps the conditioner-versus-truth error
meaningful. The plugin logs a warning with the active values at start. Example:

```bash
ros2 launch sim_mujoco sim_mujoco.launch.py controller:=jtac conditioner:=true \
  sim_noise:="ft_noise_density=0.05 ft_bias_walk=0.01 ft_lowpass_hz=40 ft_delay_ms=8 ft_quantum=0.01 control_delay_ms=10"
```

The model is gtest-covered (`test_signal_model`: noise standard deviation against the density, bias growth with
the square root of time, low-pass step response, delay shift, quantisation, repeatability). The values are not
those of any sensor; they are a stress setting. Gazebo has no equivalent yet.

## Contact test with the models on

`CONTACT_LAUNCH_ARGS="sim_noise:=..." run_contact.sh --err-force-max 5 --err-torque-max 2` (the same scripted
test as in [sim_mujoco.md](sim_mujoco.md), 100 Hz controller, 10 N target, one run each, seed 3, MuJoCo 4.0.1).
Settings of run B: `ft_noise_density=0.02` (std 0.14 N per sample at 100 Hz), `ft_bias_walk=0.01`, `ft_lowpass_hz=50`,
`ft_delay_ms=10`, `ft_quantum=0.01`, `joint_pos_noise=0.0002`, `joint_vel_noise=0.002`, `control_delay_ms=10`.
Run A used `ft_noise_density=0.05` (std 0.35 N), the rest equal.

| | ideal sensor | run B (density 0.02) | run A (density 0.05) |
|---|---|---|---|
| steady force, ground truth, 10 N target, tolerance +-1.5 N | 10.13 N, std 0.03 | 10.18 N, std 0.23 (pass) | 10.23 N, std 0.30 (pass) |
| conditioned minus truth, force rms / max | <= 0.001 / 0.004 N | 0.41 N / 1.26 N (fz); fx, fy 0.19 N | fz 0.54 N, but fx 4.9 N (not calibrated) |
| conditioned minus truth, torque rms | <= 0.0005 N m | 0.16 N m | 0.85 N m (not calibrated) |
| payload identification | 0.5012 kg, bias < 2e-4 | 0.4982 kg, CoG error up to 13 mm, bias up to 0.10 N m | rejected: 5 of 6 poses refused |
| release after disengage | quiet after 4.1 s | quiet 0.6 s into the 4 s retreat | same |

Reading: the contact is held within the +-1.5 N band in both runs (truth force 10.18 N and 10.23 N), so JTAC does not
need a clean sensor for that, and the conditioner's own error is dominated by the sensor noise (rms 0.4 N on Fz,
close to the 0.14 N per-sample noise plus the 10 ms delay and the drifting bias during the hold). Two things the
ideal sensor hid: at density 0.05 the conditioner's `capture_pose` gate (`capture_max_spread_n` 1.0 N, not exposed
in the launch) rejected five of six calibration poses, so the payload was never removed and the conditioned
Fx carried a 4.9 N error while Fz still held; and with the noise on, the identified centre of gravity and bias are
off by millimetres and a tenth of a unit, which fails the test's own noise-free checks on those (the "FAIL" lines of
run B: centre of gravity, bias, and "free space after approach" at its tight tolerance). The overall script exit code is
therefore 1 for both runs by design of those checks; the lines above are the numbers. One run each, so no spread
over seeds.
