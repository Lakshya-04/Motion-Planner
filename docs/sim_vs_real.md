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
| Perception | Exact wall pose in the contact tests; ray-cast depth with optional range noise | gz `depth_camera` with optional noise | Real depth has multipath, holes and a bias; the wall plane error enters the contact reference. |
| Safety layer | None | None | The vendor controller's limits and the e-stop are the last line; the software layer is not one. |

## Opt-in sensor non-idealities in the MuJoCo plugin

`sim_noise` (launch argument of `sim_mujoco.launch.py`, a string of `key=value` pairs; empty means ideal and nothing
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

RESULT_PLACEHOLDER
