# MuJoCo simulation backend (`sim_mujoco`)

A headless physics backend for the mobile manipulator under ros2_control, built against the new
MuJoCo 4.x C SDK (tested with 4.0.1). It is `hardware:=mujoco` in the description's ros2_control
xacro, next to `mock` and `jtac_mock`. Package: [`src/simulation/sim_mujoco`](../src/simulation/sim_mujoco).

| Part | What it is |
|---|---|
| `sim_mujoco/MujocoSystemInterface` | C++ ros2_control `SystemInterface` plugin (pluginlib). Loads an MJCF, steps MuJoCo by the controller period in `write()`. |
| `MujocoSim` | ROS-free core under the plugin (joint commands in, joint state and wrenches out); gtest-covered. |
| `sim_mujoco.mjcf_generator` | URDF to MJCF (pure Python, standard library only). |
| `sim_mujoco.wall_world` | `wall_scene.json` / `scene_model` scene / cuRobo world to MuJoCo geoms. |
| `launch/sim_mujoco.launch.py` | profile -> xacro -> MJCF -> controller_manager + plugin + controllers. Headless. |
| `scripts/smoke_test.py`, `run_smoke.sh` | trajectory-following and wall-contact smoke test. |
| `scripts/contact_test.py`, `run_contact.sh` | contact-controlled test of the full force path (conditioner, bridge, JTAC). |

## Build

The SDK is not vendored. Download the tarball of the release and point `MUJOCO_ROOT` at it:

```bash
curl -L -o mujoco.tgz https://github.com/google-deepmind/mujoco/releases/download/4.0.1/mujoco-4.0.1-linux-x86_64.tar.gz
mkdir -p /opt/mujoco-sdk && tar xzf mujoco.tgz -C /opt/mujoco-sdk       # -> /opt/mujoco-sdk/mujoco-4.0.1
export MUJOCO_ROOT=/opt/mujoco-sdk/mujoco-4.0.1
colcon build --packages-select sim_mujoco mobile_manipulator_description
```

No SDK download is needed when the `sim` image's Python wheel is at hand: `mujoco-core` ships `libmujoco.so.4.0.1` and the
headers, so a `MUJOCO_ROOT` can be assembled from it (`include/` copied from `site-packages/mujoco/include`, `lib/libmujoco.so.4.0.1`
plus a `libmujoco.so` symlink). This is how the contact test was run on a machine with a slow link to GitHub.

Without `MUJOCO_ROOT` (or `/opt/mujoco`) CMake prints a warning and builds only the Python tools,
launch files and data, so the repo-wide `colcon build` of the `build` image stays green. With it,
`libmujoco.so.4.0.1` is installed next to the plugin (`$ORIGIN` rpath), so the install tree is
self-contained. No new Docker image: the `build` image (ROS Jazzy) builds and runs it with the
SDK mounted, e.g. `-v <sdk>/mujoco-4.0.1:/opt/mujoco:ro -e MUJOCO_ROOT=/opt/mujoco`. The pure-Python
tests also run in the `sim` image (it has the `mujoco-core` 4.0.1 Python package).

## Run

```bash
ros2 launch sim_mujoco sim_mujoco.launch.py profile:=ur10e_telescopic \
    scene:=$(ros2 pkg prefix sim_mujoco)/share/sim_mujoco/data/smoke_wall_scene.json
ros2 launch sim_mujoco sim_mujoco.launch.py controller:=jtac     # compliant controller
ros2 launch sim_mujoco sim_mujoco.launch.py controller:=jtac conditioner:=true   # + ft_conditioner, JTAC on the conditioned wrench
ros2 launch sim_mujoco sim_mujoco.launch.py viewer:=true         # optional, needs a display
ros2 run sim_mujoco generate_mjcf --profile ur10e_telescopic --out /tmp/mj --scene wall_scene.json \
    --scene-offset 0.6 0 0 0                                    # model only, no ROS graph
```

The launch file renders the URDF from the profile (`xacro_args`), generates the MJCF into a temp
dir (`out_dir:=` to keep it), exports its path as `SIM_MUJOCO_MJCF` (read by the xacro
`<param name="mjcf">$(optenv SIM_MUJOCO_MJCF)</param>`), renders `robot_description` with
`hardware:=mujoco` and starts `ros2_control_node`, `robot_state_publisher`, the
`joint_state_broadcaster` and a controller named `combined_arm_lift_controller` (the name MoveIt's
controller config uses). `controller:=jtc` is a `JointTrajectoryController` (position command);
`controller:=jtac` loads `compliant_trajectory_controller/JtacController` from the repo's own
`jtac_controller.yaml`, overriding the joint names, the kinematics chain (`frames.base` to
`frames.tcp`), the F/T sensor name and frames from the profile. Joint names always come from the profile.

What `controller:=jtac` compensates. The plugin's `ft_sensor` is the raw wrench at `tool_mount_link` in
the sensor axes, with the tool weight. The launch therefore sets the sensor frame and the gravity
compensation frame to `tool_mount_link` (the control frame stays the TCP) and fills JTAC's
`gravity_compensation.CoG` from the URDF: force = tool mass x 9.81, position = tool centre of gravity in
the mount frame. So the tool weight is removed. Not compensated: the sensor bias (use `ft_tare`) and the
lever arm from the sensor to the TCP, because JTAC only rotates the wrench into the control frame
(see [ft_sensing.md](ft_sensing.md)); a contact force at the TCP therefore shows a torque error of
`r x F` (r = 0.165 m). Engage force axes only. Torque axes need a conditioner in front of JTAC.

If several ROS 2 containers share a host, give the sim its own `ROS_DOMAIN_ID` (and
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`): containers on one Docker bridge discover each other,
and another stack's controller_manager will answer your spawners.

## Interfaces

Per joint (profile order: lift, then the six arm joints): command `position`, `velocity`; state
`position`, `velocity`, `effort` (applied actuator force after the force-range clamp). The mimic
lift stages are an MJCF `<equality><joint>` constraint, not commanded. `initial_value` of the
position state interface sets the start pose (the xacro's home pose). A position command of NaN
holds the last target, a NaN velocity counts as zero; on activation the commands are set to the
start pose.

Command law: one MuJoCo position actuator per joint with `kp = 10 * effort_limit` (N m/rad) for
revolute and `100 * effort_limit` (N/m) for prismatic joints, `kv = 0.1 kp`, force range = effort
limit. The plugin applies `ctrl = q* + kv/kp * v*`, which is exactly the PD law
`kp (q* - q) + kv (v* - qdot)`: velocity feed-forward without a second actuator. Arm links and the
lift are gravity compensated (`gravcomp`), so the PD holds poses; everything beyond the F/T site
is not (see below). Joint damping and armature are small defaults (2 N m s/rad plus
`0.0005 * effort`; lift 100 N s/m and 2 kg) chosen so the implicit-fast integrator stays stable at
2 ms and a 100 Hz controller.

## Tool force/torque

Modelled like a real wrist sensor, nothing is pre-compensated:

- MJCF site `ft_sensor` at the wrist (origin of `tool_mount_link`, between the arm flange and the
  tool) with a MuJoCo force and torque sensor. The tool is physically offset: the TCP is
  `tcp_xyz` (0.165 m) along z, and a thin contact pad sits at the TCP so a wall touch happens where
  coverage poses are defined.
- ros2_control sensor `ft_sensor` with state interfaces `force.x .. torque.z` (the shape
  `ft_bridge_hardware` exports and JTAC reads), in the **sensor frame**, and the same raw wrench as
  `geometry_msgs/WrenchStamped` on `/sim_mujoco/ft_raw`.
- **Measured sign convention.** MuJoCo's site force sensor is multiplied by -1 in the plugin. The
  result is the force the tool side puts on the sensor: `F = F_env + m g (+ inertia)` for
  everything beyond the sensor, torque about the sensor origin. A tool hanging at rest reads
  `(0, 0, -m g)` when the sensor z axis points up. `test_mujoco_sim.cpp` (`WeightAndPush`) measures
  both a known weight and a known push; the smoke test checks it on the UR10e (0.5 kg tool reads
  4.91 N). The weight of the tool is deliberately kept (bodies beyond the sensor are not gravity
  compensated); a downstream conditioner has to remove payload gravity, the lever arm to the TCP
  and the bias.
- **Ground truth (evaluation only):** `/sim_mujoco/ft_ground_truth`, `mj_contactForce` summed over
  contacts between the tool subtree and anything else, the wrench the environment exerts on the
  tool, referred to the TCP and expressed in the tcp frame (`force_z ~ -push` when pressing along
  tool +z). Control must not read it.
- Hardware parameters: `mjcf`, `ft_site`, `tcp_site`, `ft_frame_id`, `tcp_frame_id`, `ft_tare`.

## MJCF generator

`urdf_to_mjcf` converts the URDF of the profile's robot. Bodies, joints, origins, axes, limits and
inertials are carried over (full inertia tensors are rotated into the link frame; checked against an
independent hand-written FK). Collision geometry only: boxes, cylinders and spheres stay primitives,
STL/OBJ meshes are copied next to the MJCF (`meshes/`) and referenced as assets with `maxhullvert=64`.
MuJoCo collides meshes as their convex hull, so there are no separate hull files. Neighbouring links
within 2 joints of each other get `<contact><exclude>` pairs (the same rule the other collision
checkers use; welded links form one rigid group), because the UR meshes overlap at the joints.
Home pose becomes a `home` keyframe in MuJoCo's depth-first qpos order.

The wall world (`wall_world.py`) reads the files the coverage planner reads: `wall_scene.json`
(each wall becomes a box whose front face lies on the scanned plane and which extends 0.1 m behind
it; obstacles become oriented boxes), a `scene_model` native scene (box, sphere, cylinder, capsule,
roles) or a cuRobo world dict. Meshes and point clouds are skipped and reported. Walls and
`contact_surface` objects get friction 1.0. `--scene-offset x y z yaw` places the scene in the robot
base frame (scans are in the mapping frame, not the robot's).

## Smoke test

`scripts/run_smoke.sh` launches the UR10e + `smoke_wall_scene.json` (wall plane x = 0.98 m, tool axis
+x) and runs `smoke_test.py`: trajectory home -> pre-contact pose, `/joint_states` reach it and pass
through the midpoint; in free space ground truth is 0 and raw reads the 4.9 N payload; a 5 cm push
past the wall gives a negative tool-z force of a few hundred N at most, mostly along the tool axis;
the raw wrench equals ground truth plus payload plus lever arm (`tau_s = tau_tcp + r x F + tau_payload`,
`r = (0, 0, tcp_z)`) within 8 %; the force goes away on retreat. Last run with `jtc`: all checks pass
(Fz = -208 N, raw/truth difference 0.05 N, 0.00 N m). With `jtac` (no admittance axes selected, so
it passes the reference through) the same run passes too; the retreat check waits for the force to vanish
(quiet for 0.5 s, 8 s allowed) and prints how long that took: 0 s with `jtc`, about 3 s with `jtac` (see
"Retreat" below). The poses were solved with a damped-least-squares IK on the generated model; they are
UR10e-specific.

### Retreat: why JTAC leaves a residual for about 3 s

Earlier versions of the smoke test looked 1.5 s after the retreat trajectory ended and, with `jtac`, found
13.8 N still on the tool. It is not the simulator. Measured over the retreat (press 5 cm past the wall, then
back to a pose 3 cm short of it; wall plane x = 0.98 m, tool axis +x; the JTAC `~/debug_offset` topic gives
the reference and the commanded tool position):

| t after retreat start | reference x | commanded x (reference + compliance offset) | ground-truth Fz |
|---|---|---|---|
| 0 s | 1.030 | 1.127 | -47 N |
| 4 s (trajectory ends) | 0.950 | 1.022 | -22 N |
| 5 s | 0.950 | 1.008 | -14 N |
| 6 s | 0.950 | 0.996 | -12 N |
| 7 s | 0.950 | 0.983 | 0 |
| 8 s | 0.950 | 0.971 | 0 |

- The force is not simulator hysteresis: with `jtc` the same retreat gives 0 N at once, and the force goes
  to exactly zero (not a tail) the moment the commanded x crosses the wall plane (0.98 m, at about 7 s).
  There is no tare (`ft_tare` off), and the tool is not stuck: the joints follow the command.
- It is the JTAC compliance offset. The ros2_controllers admittance rule integrates a mass-spring-damper on
  the tracking error and only masks the *force* by `selected_axes`; the spring acts on every axis. While the
  wall blocked the tool the rule accumulated a 0.10 m offset (commanded 10 cm past the reference), which
  is also why the push is 50 N and not the 208 N of `jtc`. When the reference leaves, the offset relaxes
  with the over-damped time constant D/K (about 2 s for the Z gains of `jtac_controller.yaml`), so the
  commanded pose is still inside the wall about 3 s after the trajectory ended.
- Consequence for tests and for coverage sequencing: after a contact segment, wait for the offset to relax
  (about 3 time constants) or reset it before relying on "no contact", and do not treat a JTAC retreat as
  instantaneous. With a force target and the axis engaged (contact test below) the offset is small, and the
  force is released as the trajectory ends (4.1 s after its start).
- Not changed in the simulator: nothing in `sim_mujoco` was wrong. A fix would be in JTAC (reset the
  admittance state on disengage); left alone here.

## Contact-controlled test (full force path)

`scripts/run_contact.sh` launches `controller:=jtac conditioner:=true` with the smoke wall and runs
`scripts/contact_test.py`. The chain:

```
/sim_mujoco/ft_raw (sensor frame tool_mount_link, raw, payload weight included)
  -> ft_conditioner (sign +1, tool = tcp_link and sensor orientation from TF, payload from its own calibration)
  -> /ft_conditioner/wrench_conditioned (tcp_link, tool axes)
  -> ft_bridge_hardware, sensor `ft_sensor_tcp` (second ros2_control block `ft_conditioned_bridge` of the
     mujoco hardware; the name `ft_sensor` is the raw sim sensor), biased by ~/target_wrench
  -> JTAC (ft_sensor.name ft_sensor_tcp, frame tcp_link, control frame tcp_link)
```

What the test does: (1) calibrates the conditioner with its own services, six free-space wrist poses
(`capture_pose` each, `identify`) and compares the fit with the tool inertial of the URDF; (2) approaches the
wall (pose 3 cm short), publishes `fz = -10 N` on the bridge's `~/target_wrench`, engages the tool-Z axis
(`admittance.selected_axes` through the controller's parameters), and presses with the reference on the wall
surface; (3) holds 10 s and compares the steady force with `/sim_mujoco/ft_ground_truth` and the conditioned
wrench with the ground truth sample by sample (same stamp); (4) disengages (axes off, target 0) and retreats.
Last run (MuJoCo 4.0.1, 100 Hz controller, JTAC gains of `jtac_controller.yaml`):

| | result |
|---|---|
| identified payload | 0.5012 kg (known 0.5), centre of gravity (0, 0, 0.7 mm) (known 0), bias < 2e-4; fit residual 0 |
| conditioned wrench in free space, after calibration | 0.000 N, 0.000 N m (uncalibrated it reads the 4.91 N payload) |
| steady contact force, target 10 N | ground truth 10.13 N, std 0.03 N over 10 s (tolerance +-1.5 N); lateral 0.18 N, 0.05 N |
| conditioned minus ground truth, 1002 samples | force: mean 0.000, rms <= 0.001 N, max 0.004 N; torque: rms <= 0.0005 N m, max 0.001 N m (the script checks rms against a loose tolerance and the maxima against 0.05 N and 0.02 N m, `--err-force-max` / `--err-torque-max`) |
| release after disengage | measured from a trace sampled during the retreat: the last sample at or above 0.5 N is at the end of the 4 s trajectory (4.1 s after its start), then 0.000 N; the check needs 0.5 s of quiet after it, not a single low sample |

The steady force depends on where the reference is, because the admittance spring pulls toward it:
`F = target + K * d` with K = 300 N/m (Z) and `d` the reference depth past the surface. Measured for a 10 N target
(reference depths from a joint-space interpolation between the two poses, so approximate): 0.6 cm short of
the surface gave 8.3 N, on the surface 10.1 N, 1.2 cm past 13.2 N, 3.4 cm past 20.5 N, 5 cm past 25.5 N, a
slope of about 306 N/m. A reference
inside the wall therefore over-presses by K*d; coverage references sit on the surface.

The conditioner errors are near zero because the simulated sensor is noise-free and rigid and the
calibration is exact; the test pins the sign, the frame handling, the lever arm (TF `tool_mount_link` ->
`tcp_link`, 0.165 m) and the payload compensation. The contact here is almost purely along the tool axis,
so the lever-arm torque is small (0.165 m x 0.2 N); a lateral-load test would stress it more.

Tests: `colcon test --packages-select sim_mujoco` (gtests need the SDK; Python tests skip what they
lack), or `python -m pytest src/simulation/sim_mujoco/test` in the `sim` image.

## Base motion (`base_motion:=true`)

By default the base is welded to the world, as before (the MJCF is byte-identical, `run_wall_pass.sh` and
`run_manager_pass.sh` are unchanged). `base_motion:=true` adds a planar-velocity base:

```bash
ros2 launch sim_mujoco sim_mujoco.launch.py base_motion:=true
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

- **Model.** The generator puts three joints on the root body (`base_footprint`): slides `base_x_joint`,
  `base_y_joint` and hinge `base_yaw_joint`, each with a stiff position actuator (kp 2e4 N/m, 2e4 N/m,
  1e4 N m/rad; force limits 3000 N, 3000 N, 1500 N m), plus `<custom>` numerics `sim_mujoco_base_diff` and
  `sim_mujoco_base_max_vel`. `--base-motion` on `generate_mjcf` does the same. The wheels stay visual links.
- **Command.** `geometry_msgs/Twist` on `/cmd_vel`, body frame, read by the plugin (subscription spun in
  `read()`). The plugin integrates a world-frame set point from it, so a zero command holds the pose
  (a contact force pushes the base back by `F/kp`, 0.5 mm per 10 N), and the set point never runs more
  than 5 cm / 0.05 rad ahead of a blocked base. A command older than 0.5 s counts as zero
  (`cmd_vel_timeout`). Speeds are clamped to the profile's `max_linear_velocity_mps` and
  `max_angular_velocity_radps`. `drive_type: diff` and `skid` drop `vy`; `omni` and `4ws` are treated as omni
  (the 4-wheel-steering kinematics are not modelled).
- **Output.** The plugin publishes the base pose (`sim_mujoco/base_pose`, PoseStamped in `odom`) and body
  velocity (`sim_mujoco/base_twist`); the `base_odom.py` node (started by the launch file) turns them into
  `nav_msgs/Odometry` on `/odom` and TF `odom -> base_footprint`, from the simulated base pose (ground
  truth, no noise or drift); `odom` starts at the base's initial pose. `robot_state_publisher`
  continues the tree from `base_footprint`, so `odom -> tcp_link` resolves.
- **Why not `mujoco_ros2_control`.** Its `base_velocity` plugin needs a freejoint base, its own simulation
  node, and MuJoCo 3.12 through `mujoco_vendor`; ours runs inside our own `SystemInterface` against MuJoCo
  4.0.1. Adding the three joints in our generator and a velocity command to our plugin was less work than
  porting that stack, so it was not tried; the roadmap's reuse entry stays as the reference.
- **Test.** `scripts/run_base_motion.sh` (`base_motion_test.py`) drives 0.3 m forward and checks odom, TF,
  the held arm and lift joints, F/T, the hold and the command timeout. Generator tests are in
  `test/test_mjcf_generator.py`.
- **Stand navigation.** `scripts/run_stand_navigation.sh` adds the reachability node and `stand_navigator`: plan stands for
  the smoke wall, drive the arm base to the first one, check arrival, arm, F/T (`docs/stand_navigation.md`).

## Depth sensor (`depth_sensor:=true`)

A ray-cast depth sensor (`mj_multiRay` against the scene geoms) on the root link publishes
`sensor_msgs/PointCloud2` on `/depth_sensor/points` (frame `depth_sensor_link`, TF from the launch file); arguments
`depth_sensor_pose`, `depth_sensor_noise_m`, `depth_sensor_rate_hz`. Off by default and then the MJCF is unchanged.
Details, the wall detector that consumes it and `scripts/run_depth_wall.sh` are in [perception_live.md](perception_live.md).

## MuJoCo 4 notes

- 4.0 split the Python package into `mujoco-core` and `mujoco-studio` (the viewer lives with the
  studio package) and removed the `composite` element, the cable plugin and the `flexvert`
  equality. None of them is used here: the lift is `<joint>` equality, everything is rigid.
- The C SDK tarball has `include/mujoco/mujoco.h` and `lib/libmujoco.so.4.0.1`; the plugin only
  uses the stable C API (`mj_loadXML`, `mj_step`, `mj_contactForce`, name lookups).
- MuJoCo reads binary STL only (an ASCII STL fails with "number of faces ... perhaps this is an
  ASCII file"); `ur_description` collision meshes are binary.
- `fullinertia` cannot be combined with a `quat` on `<inertial>`: the generator rotates the tensor
  instead. Parent-child collision filtering does not apply when the parent is welded to the world,
  so the generator excludes those pairs explicitly.
- The apt packages `ros-jazzy-mujoco-vendor` 0.1.0 and `ros-jazzy-mujoco-ros2-control` 0.1.2 were
  built for older MuJoCo and were not used or installed here; this plugin is a small independent
  design (model and data owned by the plugin, stepped from `write()`, no separate simulation
  thread, no clock publisher).

## Not modelled / limits

- Wheels, wheel-ground contact and slip (`base_motion:=true` moves the base kinematically as a planar velocity
  base, there is no floor), 4-wheel-steering kinematics, tool compliance, cable and hose
  dynamics, sensor noise, bias drift, temperature and the sensor's own dynamics (the wrench is the
  exact rigid-body value at every step; add a noise and filter stage downstream).
- Actuator dynamics: ideal torque-limited PD joints, no gearbox backlash, friction beyond a small
  damping, no speed or acceleration limits (the URDF velocity limits are not enforced); joint
  limits are MuJoCo soft limits.
- Contact is MuJoCo's soft convex contact: stiffness and force levels depend on `solref` defaults
  and on the joint gains, so a push past the wall gives a stiff, position-controller-set force (208 N
  for 5 cm with `jtc`); tune with the commanded penetration. Meshes collide as convex hulls (concave
  UR links are over-approximated); pairs within 2 joints are not checked.
- Real time follows the controller manager: simulated time advances by the measured period, five
  2 ms steps per 10 ms cycle. On a loaded host (the benchmark GPU box) cycles overrun and the
  controller manager logs warnings, but simulated time stays consistent with ROS time.
- Single F/T sensor, one tool. No rendering sensors, no viewer inside the plugin: `viewer:=true`
  starts `viewer.py`, which only mirrors `/joint_states` (needs a display and the `mujoco` Python
  package; not run in the headless images).

## Recording a run and scoring it

`run_contact.sh --record /tmp/run.csv` (extra arguments go to `contact_test.py`) writes the whole run as a `score_run` CSV
(`t,x,y,z,force,active`: the tool point in `base_footprint` from TF, the ground-truth force along the tool axis) and
`/tmp/run.csv.wall.json` (the wall plane x = 0.98 m, u along y, v along z). Score it with
`ros2 run coverage_planner score_run --samples /tmp/run.csv --wall /tmp/run.csv.wall.json --target-force 10 --band 1.5 --swath 0.05
--min-coverage 0 --max-rms 2 --max-peak 15` (see `docs/coverage_patterns.md`). One recorded run (UR10e, smoke wall, 10 N target):
9188 samples, 6 strokes, force peak 12.3 N, force RMS error 1.25 N over the working samples (the approach and release ramps
count), 85 percent of them within 1.5 N of the target. The contact test presses at one point, so coverage is not meaningful
there (hence `--min-coverage 0`); a full wall pass would feed the same scorer.

## Scripted wall pass

`run_wall_pass.sh` launches the sim with `controller:=jtac conditioner:=true move_group:=true` (the conditioner payload is
passed as launch parameters, no calibration step), runs `wall_pass_test.py` and scores the recording. For each stroke y
(`--strokes-y`, default 0.07 and 0.19) MoveIt's `/compute_cartesian_path` plans an approach 3 cm in front of the wall, a
press 1.5 mm past the wall surface (JTAC's tool-Z admittance axis engaged with the 10 N target on JTAC's own
`~/target_force_n`) and a vertical stroke (z 1.28 to 1.02 at the smoke wall x = 0.98 m); JTAC executes them and the tool retreats. The
launch argument `move_group:=true` starts `move_group` for planning only. The script exits nonzero if a stroke does not
plan (fraction below 0.98) or execute, or coverage of the planned region is below 0.9.

Runs (UR10e, two strokes). At 0.05 velocity scale every stroke planned and executed and coverage of the region was 1.0
(806 of 806 cells), but the contact was poor: peak 21.5 N against the 10 N target, force RMS error 2.8 N, 34 percent of the
working samples within 1.5 N, 192 strokes in the scorer (the contact breaks and re-makes along the stroke, stick-slip on the
sliding wall). At 0.02 (the default now) the same pass gives peak 18.6 N, RMS error 1.5 N, 81 percent in band, 146 strokes.
The 18.6 N peak is not first contact: the recording has 36 spikes of exactly two samples (20 ms) at about 18 N spread along the
strokes (stick-slip micro-events on the sliding wall), while the mean is 10.4 N; a real F/T filter or a lower wall friction
should remove them, and the scorer's default 15 N peak limit therefore fails this run. The first-contact creep (about 13 N for
2 s, see the JTAC force-limit notes in the roadmap) is the second source. Still to improve: the
sliding friction in the sim, the admittance gains, the press depth. The manager and the coverage planner are not in this
loop yet, the strokes are scripted.

### Manager pass (the real pipeline)

`scripts/run_manager_pass.sh` runs the same wall with the pipeline instead of scripted strokes: the coverage planner
plans the contact region (`wall_pass_coverage_scene.json` and a synthetic `walls.pcd`, tool `contact_wiper`, tool roll
`tool_frame_yaw_deg` -90 to match the arm's ready pose), `world_model_loader_node` puts the wall into MoveIt's scene as an
octomap, the arm goes to the smoke test's ready pose (`ready_pose.py`), and `manipulation_manager` (wall at `[1, 0, 0]`,
`dock_at_stroke_start`) docks, presses, strokes and retracts per column through MoveIt and JTAC. The run is recorded
(`record_run.py`) and scored like the wall pass. Everything is installed with `sim_mujoco`; outputs and logs go to
`MANAGER_PASS_OUT` (default `/tmp/manager_pass`).

Status: the pass runs end to end and the scorer passes (exit 0): coverage 0.93 (866 of 930 cells), both columns execute
(dock, press, stroke, retract). Force: peak 20.5 N, RMS error 3.6 N against the 10 N target, in-band 0.48 (stroke 0.01 m/s; at 0.02 m/s: peak 27.7 N, RMS 6.0 N; the end-of-stroke spikes of 27-31 N, even 50 N with a position unload, came from disengaging the admittance with the 10 N target still set, so the manager now publishes a zero target and waits 1.5 s before disengaging)
(the scorer limits are the script's, see `MANAGER_PASS_SCORE_ARGS` and `MANAGER_PASS_MIN_COVERAGE`). What it took: the wall
in MoveIt's scene (without it OMPL plans through the wall), the tool roll (a 90 degree mismatch made the arm collide with
the lift on every contact path), docking at the first pose of the stroke, the allowed-collision matrix (links mounted on
the lift, `generate_acm.py`), a press-then-stroke sequence with a 2 s settle (`press_settle_s`), and planning for the sim
tool's swath (0.14 m, `MANAGER_PASS_SWATH`) instead of the profile's 0.20 m. Next: the contact force (slower stroke,
F/T filtering, JTAC gains).

### Spray pass (non-contact tool)

`scripts/run_spray_pass.sh` is the manager pass for a non-contact tool (the contact script is unchanged). The coverage
planner plans with `tool_name:=spray_gun` (standoff `SPRAY_PASS_STANDOFF`, default 0.30 m, cone half-angle 20 deg,
vertical strokes, 20 % overlap: swath 0.175 m, spacing 0.140 m), so the poses already sit 0.30 m in front of the wall; the
manager runs with `contact: false` (no admittance engage, no force target, no press, no wall contact; `docs/tools.md`). It
docks, takes up the first stroke pose with the valve closed, opens the valve (`std_msgs/Bool` on `/tool/valve`) for the
stroke only, closes it and retracts. `record_run.py` subscribes to `/tool/valve` and writes the state as the `active`
column; `score_run --no-contact --spray-standoff 0.30 --spray-half-angle 20` counts only the valve-open samples and
reports the thickness uniformity next to the covered fraction. Outputs go to `SPRAY_PASS_OUT` (default `/tmp/spray_pass`);
other knobs: `SPRAY_PASS_SPEED` (0.05 m/s), `SPRAY_PASS_MIN_COVERAGE`, `SPRAY_PASS_SCORE_ARGS` (for example
`--max-thickness-cov 0.6`).

Result (one run, UR10e, smoke wall, standoff 0.30 m): three columns, all `done` (valve ON/OFF once per stroke, six
waypoints each); 9087 samples, the valve open in 1561 (15.6 s of 90.9 s); peak force 0.00 N while spraying (the tool never touched the
wall); coverage 1.0 (930 of 930 cells, mean overlap 1.39); thickness mean 0.0185 (relative), CoV 0.52, min 0, max 0.059;
the scorer exits 0. The CoV is high because the wall is only 0.30 m wide: three strokes, and the cells at the wall edges
and at the ends of the strokes get the tail of the Gaussian only; it is a number to improve with spacing and standoff
optimisation (roadmap section 4), not a pass criterion (no limit is set by default). Regression: `run_manager_pass.sh` on
the same build gives coverage 0.931, peak 19.4 N, RMS error 3.58 N (the contact scorecard has no `thickness` section).
Over five runs the thickness CoV was 0.52, 0.61, 0.52, 0.53 and 0.52 and coverage 1.0 every time. No force limit is set by
default (`SPRAY_PASS_MAX_PEAK=0`): the whole-run peak force was 0, 237, 10, 0 and 0 N. The two spikes happened during valve-closed
free-space moves in the air (z 1.6-1.8 m, x 0.7 m, far from the wall) and never while spraying; they are transients of the fast
OMPL moves (or an arm-link touch), not wall contact, so a 5 N limit over the whole run would fail some runs for a reason that is
not the tool. Worth chasing before a force limit is turned on (the script prints the peak while spraying separately).
Limits: the standoff is open loop (no feedback on the wall distance), the valve timing is one open and one close per
stroke, and the sim has no spray: the deposit is computed by the scorer from the recorded path.

### Stand pass (plan stands, drive, then the manager pass)

`scripts/run_stand_pass.sh` closes the loop with `base_motion:=true`: the wall is moved (`STAND_PASS_WALL_DX/DY`, default
0 and +0.9 m, `shift_wall_scene.py`) so the first planned stand is not where the base starts; the coverage planner plans it
in odom; `stand_drive.py` asks reachability `plan_stands` (start = the base pose) and drives to stand 0 through
`navigate_to_stand`; the wall goes into MoveIt's scene (odom frame); the manager runs with `world_frame: odom`, which
transforms each column into `base_footprint` from TF (`docs/stand_navigation.md`); `record_run.py --frame odom` records the
tool point in odom. Outputs go to `STAND_PASS_OUT` (default `/tmp/stand_pass`); `STAND_MAP` reuses a reachability map.

Result (one run each, UR10e generic omni base): stand 0 at arm base (-0.02, 0.51), yaw 1.571 (side-on to the wall: the
planner prefers footprint clearance), 0.51 m from the start; arrival error 15 mm, 0.013 rad; the manager's stance check
agrees; three columns dock, press, stroke and retract; scorer passes with coverage 0.931, peak 18.8 N, RMS error 3.31 N,
in-band 0.45. The unchanged manager pass (base at the origin, `run_manager_pass.sh`) on the same build: coverage 0.931, peak
18.8 N, RMS 3.60 N. Flaky points: a dock from the turned base needs more OMPL tries (`dock_attempts`, 6 in the script; one
earlier run with 3 lost the first column, coverage 0.68), and the F/T peak was 34 N in another run (the scorer limit passed).
A wall needing several stands: see the multi-stand pass below.

### Multi-stand pass (the manager's `execute_coverage` action)

`scripts/run_multi_stand_pass.sh` makes the wall 0.9 m wide (`MULTI_STAND_WALL_WIDTH`, from y = `MULTI_STAND_WALL_DY` 0.3;
`shift_wall_scene.py` takes a width argument: the coverage scene, the sim wall with its 0.25 m margins and the `walls.pcd`
octomap grow with it) and starts the manager with `run_on_start: false`. `execute_coverage_client.py` sends one `multi_stand`
goal; the manager plans stands (`plan_stands`, columns asked for in groups of at most `stand_max_span_m` = 0.5 along the wall),
and per stand stows the arm, drives there (`navigate_to_stand`), re-injects the wall octomap (`stand_arrival_command`:
MoveIt keeps obstacles in the robot's root frame, so the wall must be put in again after the base moved) and runs the columns
in odom. `record_run.py --frame odom` records, `score_run` scores the whole wall. Details: `docs/stand_navigation.md`.

Result (first of two full runs, UR10e generic omni base, 9 columns at swath 0.14, speed 0.01 m/s; the repeat is below): stand 0 at arm base (0.18, 0.01) yaw 1.571
with columns 3-8, stand 1 at (0.18, 0.51) with columns 0-2 (arrival errors 4.1 and 19.4 mm); all 9 columns done, goal
`success`, feedback and result peak force 23.8 N (conditioned wrench magnitude, includes the press transient); scorer passes:
coverage 0.977 (2726 of 2790), peak 20.9 N, RMS error 3.51 N, in-band 0.22, 459 s. In-band is lower than the one-stand
passes (0.45 to 0.49) and was not investigated. `MULTI_STAND_CANCEL_AFTER_S=N` cancels instead and prints the arm's joint speed
afterwards; `MULTI_STAND_MANAGER_EXTRA` adds manager parameters (for example `stand_nav_timeout_s: 2.0` to watch a stand being
skipped).

The repeat run on the final build did **not** pass: column 8 (the last of stand 0) pressed with a spike, its contact stroke
aborted, the retract then failed in 11 ms of planning (the tool was left in collision with the wall) and so did the stow, so
stand 1 was reported "not reached (could not stow the arm for the drive)" and its three columns failed: 5 done, 4 failed, scorer
coverage 0.672, peak 97 N, RMS 9.3 N. The failure handling did what it should (recorded, went on, per-stand result) but the pass
is not reliable yet: roughly one stroke in a long run still ends with the tool jammed in the wall box, and nothing recovers the
arm from there (a back-off along the wall normal before the retract would be the first thing to try). Two runs, one pass.

What it took, honestly: without `stand_max_span_m` the planner returns **one** stand "reaching" 54 poses for the 0.9 m wall
(stand at y 0.11, 0.2 to 1.1 m of wall to its side), and the run from it failed: two of nine columns could not dock, the rest
pressed with 100 to 296 N spikes, the tool tunnelled into the 10 cm wall box and the scorer coverage was 0.16 (one run, 1090 s).
The RM4D map does not know the arm's self-collision or how awkward the pose is at the edge of reach, so a span cap is a
manager parameter. A first two-stand run also failed every column until the wall octomap was re-injected after the drive.

Checked in the same script (one run each): **cancel** 100 s in, during the stroke of the second column: the result is `CANCELED`
(column 3 done, column 4 failed, stand 1 never reached), the log shows the admittance off and the force target at 0, joint speed
over the next second 0.008 rad/s (the unload ramp makes the cancel take 1.65 s from request to result); **navigation failure**
(`stand_nav_timeout_s: 0.5`, `stand_nav_attempts: 1`): both stands skipped, result `0 done, 0 partial, 9 failed` with each stand
"not reached (navigation timed out)" and its columns listed as failed; with `stand_nav_timeout_s: 2.0` the first attempt timed out,
was cancelled, the retry arrived (3.9 mm) and the stand ran (that run was stopped early). Dock failures: in that run the first two
dock attempts of the first column ended in OMPL's 5 s search timeout and the third succeeded in 59 ms; the same pattern as the
flaky first dock noted for the stand pass. The cause was not found (it looks like a stale start state or scene right after the
arrival, not the goal: the same dock pose is trivial on a later try); `dock_attempts: 6` covers it.


#### Sensed wall (M4 exit test)

`scripts/run_sensed_pass.sh` is the same pass without any wall file: the sim runs with `depth_sensor:=true`,
`wall_detector` writes `wall_scene.json` + `walls.pcd` from `/depth_sensor/points`, `crop_wall_scene.py` crops the detected
scene to the contact region (y 0..0.3, z 1.0..1.3 in `base_footprint`, `SENSED_PASS_CROP_Y/Z`; the whole sensed face is 0.76 x 0.70 m
and the arm does not reach all of it), the coverage planner plans the crop, `world_model_loader_node` loads the detected
whole-wall `walls.pcd` (a patch-only octomap lets OMPL swing over the edge), then the manager, recording and scoring are as above.
The detector is stopped once the scene is written so the arm cannot change it mid-run. The shared steps live in
`scripts/manager_pass_common.sh`, sourced by both scripts. Output in `SENSED_PASS_OUT` (default `/tmp/sensed_pass`).

Result (3 mm range noise): detected normal (-1.0000, 0.0002, -0.0001), d 0.9801, rms 2.85 mm; crop 0.300 x 0.300 m, 4 columns
(the crop is centred on the detected centroid, not on the file scene's patch, so the column layout differs from the file pass);
coverage 1.00 (930 of 930), peak 23.6 N, RMS error 3.4 N, in-band 0.46, scorer exit 0. The detected wall schema needed nothing
added: the planner reads only the plane, extents and `plane_rms_mm` / `spread_p95_mm` / `spread_p99_mm`, and the observed mask
comes from `walls.pcd`. Single runs, not repeated.
