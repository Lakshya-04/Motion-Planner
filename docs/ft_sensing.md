# Force/torque sensing: from a raw sensor reading to the contact wrench

JTAC (the compliant trajectory controller) and the force layer work on one quantity: the wrench the
environment exerts on the tool, at the tool control point (`link_tcp` on the real robot description, `tcp_link` in the simulated mobile-manipulator description), in the tool axes
(`core_types::Wrench`; pressing along the tool's +Z gives `force_z` of about minus the target). A real
wrist sensor does not report that. It reports a wrench at its own origin, in its own axes, with the weight
of everything beyond it and an offset. The conversion is four steps, in this order.

| Step | What | Why it is needed |
|---|---|---|
| 1. sign | +1 / -1 for the sensor's measure direction | Sensors differ in whether they report the load on the tool side or the reaction on the robot side. Gazebo has a `measure_direction` setting for exactly this. |
| 2. bias | subtract the no-load offset | Zero drift, electronics, mounting stress. |
| 3. gravity | subtract the payload's weight for the sensor's orientation | The sensor feels the tool, the plate and the adapter. Pose-dependent leftover force means the mass, centre of gravity or frame is wrong. |
| 4. move | re-express at the tool origin, in the tool axes: `torque_T = R^T (torque_S - p x force_S)` | The tool point is offset from the sensor. A force at the tool tip produces a real torque at the sensor that is not there at the tip. |

`force_core/wrench_conditioning.hpp` implements the four steps (plain C++, no ROS):
`WrenchConditioner::condition(raw, R_world_sensor)`, with `moveWrench`, `gravityWrench` and the config
(sign, bias, payload, tool frame in the sensor frame, gravity).

**The lever-arm term matters.** Rotating a wrench into another frame is not the same as moving it. With
the tool 0.2 m beyond the sensor, 15 N of contact with a little friction reads as several newton-metres at
the sensor and none at the tool point; a rotate-only change of frame reports those as a tangential torque,
which is what the tool's tangential-torque abort watches. A test pins both: the conditioner recovers the
contact exactly, the rotate-only version does not.

## What JTAC does today

JTAC wraps the ros2_controllers admittance rule. Its `ft_sensor.frame.id`, `control.frame.id` and
`gravity_compensation` (`CoG.pos`, `CoG.force`) parameters exist, but reading the installed implementation:
the wrench is only **rotated** into the world and control frames, never moved with a lever arm, and the
admittance dynamics are evaluated at the sensor frame. So `ft_sensor.frame.id` has to be the frame the
wrench is already expressed at, which is why the real-robot config uses `link_tcp` and relies on the
UFACTORY controller, which reports the external force at the TCP with the payload already removed
(`CoG.force: 0.0` for that reason). A raw sensor (a simulator, or a different robot) needs the conditioner
in front of it.

## How it is solved elsewhere

- **Payload identification** from static poses by least squares, then compensation, is standard: see
  [Duan et al., Sensors 2022](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC8777628/) (excitation poses,
  least squares for load, gravity and inertial terms). Robot vendors ship a routine for it; an
  [ABB note](https://industrialmonitordirect.com/blogs/knowledgebase/identifying-abb-force-sensor-mass-and-inertia-data)
  (third-party summary, check against ABB's own documents) says to include the adapter plate and fasteners
  in the identified assembly.
- **ROS 2:** the [admittance controller](https://control.ros.org/iron/doc/ros2_controllers/admittance_controller/doc/userdoc.html)
  has the sensor frame, gravity frame and CoG parameters; it rotates, it does not apply the lever arm.
- **Simulators:** a sensor in Gazebo reports the wrench at the sensor origin, and its `frame` option only
  changes the axes the numbers are expressed in, not the point of application
  ([gz-sim `WrenchMeasured`](https://gazebosim.org/api/sim/9/WrenchMeasured_8hh.html)). MuJoCo's force
  sensor is reported in its site frame; a [forum thread](https://github.com/google-deepmind/mujoco/discussions/2006)
  reports a sign that differed from the docs, so each backend measures its own sign in a test.

## `identifyPayload`: calibrating mass, centre of gravity and bias

`identifyPayload(samples, gravity, sign)` fits mass, centre of gravity (in the sensor frame) and the six
bias values from static, contact-free readings, each with the sensor's orientation at that moment (from
forward kinematics). It needs at least four poses in which gravity, seen in the sensor frame, points in clearly
different directions: the largest angle between any two poses must be at least 10 degrees (a planar sweep
about one axis is enough; the fit recovers it exactly). A set below that is rejected with a message, not
guessed. A negative fitted mass is rejected too: it means the sensor sign is wrong, so repeat with `sign = -1`. It returns the fit residual. Static only: inertial terms are ignored, so use it, and the
compensation, at low accelerations.

## In simulation

The simulation backends ([simulation.md](simulation.md)) publish what a real sensor would: a raw wrench in
the sensor's own frame, with the payload's weight, plus a ground-truth contact wrench at `tcp_link` for
evaluation. The same conditioner then runs on simulated and real data, and the ground truth checks it.

MuJoCo, measured ([sim_mujoco.md](sim_mujoco.md), `scripts/contact_test.py`): the chain raw sensor ->
`ft_conditioner` -> `ft_bridge_hardware` (sensor `ft_sensor_tcp`) -> JTAC. Calibrating with six free-space
wrist poses through `capture_pose` / `identify` recovers the tool's 0.5 kg to 0.2 % and a centre of gravity
within 1 mm; after that the conditioned wrench is 0.000 N in free space. Pressing the wall with a 10 N
target through the bridge's `~/target_wrench` held 10.13 N (ground truth, std 0.03 N); over the 10 s of
contact the conditioned wrench differed from the ground truth by at most 0.004 N and 0.001 N m (rms
0.001 N, 0.0005 N m). Those are the limits of a noise-free rigid simulation, not of a real sensor; they
confirm the sign, the TF-derived orientation and lever arm, and the payload term, not noise, drift or
tool flex. The steady force is `target + K * d` for a reference `d` past the surface (the admittance spring
pulls toward the reference), so references belong on the surface.

## The ROS node: `ft_conditioner`

`src/control/ft_conditioner` wraps the conditioner: `wrench_conditioner_node` takes a raw
`geometry_msgs/WrenchStamped` in the sensor's frame and publishes the contact wrench in the tool frame
(`output_topic`, frame `tool_frame`), ready for `ft_bridge_hardware` (point its `source_topic` or
`injection_topic` at it).

| Parameter | Default | |
|---|---|---|
| `input_topic`, `output_topic` | `wrench_raw`, `wrench_conditioned` | |
| `sensor_frame` | empty: the message's `frame_id` | |
| `tool_frame` | `tcp_link` | the output frame |
| `world_frame` | `base_footprint` | any frame whose z axis is up: gravity's direction |
| `tool_in_sensor_from_tf` | `true` | `false`: use `tool_in_sensor_xyz` / `_rpy` (a simulator with no TF frame for its sensor) |
| `sign` | `1.0` | `-1.0` for a sensor that reports the reaction on the robot side |
| `payload_mass_kg`, `payload_cog_xyz`, `bias` | 0 | what calibration finds |

The sensor's orientation (for the payload's weight) comes from TF at the message's stamp, falling back to the
newest transform if that stamp is not there yet, but only if that transform is at most `max_tf_age` seconds
(default 0.1) older than the wrench; a static transform never ages. Beyond that the wrench is dropped, so a
stalled TF publisher stops the output instead of leaving it compensating with a frozen orientation. The
sensor-to-tool offset is looked up once (the sensor is rigid on the tool). A wrench with no usable TF is
dropped and counted, with a throttled warning, never guessed. `max_tf_age: 0` turns the limit off.

**Calibrating.** With the arm still and nothing touching the tool, move to a pose and call
`~/capture_pose` (it averages the raw wrenches of the last 0.5 s, and refuses with a message if the input is stale, too short, or the arm is not still: force spread or orientation drift above `capture_max_spread_n` / `capture_max_rotation_rad`); repeat in at least four poses that point gravity in clearly different directions in the
sensor frame (rotate the wrist about at least two non-parallel axes); then `~/identify`. The node fits mass,
centre of gravity and bias, starts using them immediately, and returns a YAML snippet to keep in the
parameters. `~/clear_samples` starts over. A degenerate set of poses is rejected with a reason.

Gazebo's wrench is `/force_torque_sensor_broadcaster/wrench` (frame `ft_sensor_link`) and MuJoCo's is
`/sim_mujoco/ft_raw`; both are raw readings of the form environment wrench plus weight, so `sign` stays
`1.0`. Launch: `ros2 launch ft_conditioner ft_conditioner.launch.py input_topic:=... payload_mass_kg:=0.8`.

## Not done yet

- Reading the sensor-to-tool offset and the payload from the robot profile / URDF, so no parameter has to
  be set by hand.
- Inertial compensation (acceleration terms), a low-pass filter ahead of the controller, temperature drift,
  and checking a real sensor's sign and frame against this convention on hardware.
