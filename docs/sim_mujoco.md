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

- Mobile base and its wheels (the base is welded to the world), tool compliance, cable and hose
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

Status (honest): all seven segments now execute (dock, six contact waypoints and retract for both columns), but the pass
still fails the scorer: coverage 0.73, peak force 27 N against 10 N, force RMS error 6.6 N, in-band 0.44. What it took:
the wall in MoveIt's scene (without it OMPL plans through the wall), the tool roll (a 90 degree roll mismatch made the arm
collide with the lift on every contact path), docking at the first pose of the stroke instead of the middle (otherwise the
contact path starts with an IK branch jump), and the allowed-collision matrix: the arm's root link and shoulder, bolted
to the carriage, were not excluded against the lift stages, so MoveIt reported a false self-collision that truncated
every dock and retract (`generate_acm.py` now excludes links mounted on the lift). Next: the contact force (the manager's
contact segments do not yet get the press-depth and speed tuning the scripted pass has) and the coverage shortfall.
