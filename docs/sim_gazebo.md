# Gazebo simulation backend (`sim_gazebo`)

Gazebo Harmonic backend for the mobile manipulator (base + lift + arm + tool), selected with
`hardware:=gazebo` in the description. Package: [`src/simulation/sim_gazebo`](../src/simulation/sim_gazebo).
Everything runs headless (`gz sim -s`, no GPU, no display), so it works in a container.

## Run

```bash
docker build -f docker/Dockerfile --target sim-gazebo -t motion-planner:sim-gazebo .
# The image holds only the system packages (Gazebo, ros_gz, gz_ros2_control), no workspace. Mount
# the sources, build them inside, then source the install space (mobile_manipulator_description,
# sim_gazebo, and for the compliant controller compliant_trajectory_controller + ft_bridge_hardware):
docker run --rm -it -v $PWD/src:/ws/src -w /ws motion-planner:sim-gazebo bash
colcon build --packages-up-to sim_gazebo && source install/setup.bash
ros2 launch sim_gazebo sim.launch.py                                          # headless, joint_trajectory_controller
ros2 launch sim_gazebo sim.launch.py wall_scene:=<wall_scene.json> wall_id:=0 standoff:=0.9
ros2 launch sim_gazebo sim.launch.py controller:=jtac                         # compliant JTAC controller
ros2 launch sim_gazebo sim.launch.py profile:=ur10e_telescopic gui:=true      # gz GUI needs a display
ros2 run sim_gazebo smoke_test.py [--controller jtc|jtac]                     # headless smoke test
ros2 launch sim_gazebo sim.launch.py controller:=jtac ft_conditioner:=true    # force path (see below)
ros2 run sim_gazebo contact_control_test.py                                   # contact test, about 3 minutes
ros2 run sim_gazebo wall_world_gen.py scene.json -o wall.sdf --standoff 0.9   # world only
bash $(ros2 pkg prefix sim_gazebo)/lib/sim_gazebo/run_manager_pass.sh         # coverage planner + manager + MoveIt + JTAC, scored (about 5 minutes)
```

Launch arguments: `profile` (name in `config/profiles` or a path; its `robot.xacro_args`, joint
names, frames and home pose drive everything), `world`, `wall_scene`/`wall_id`/`standoff`, `gui`,
`controller` (`jtc` | `jtac` | `none`), `ft_conditioner` (`true` starts the conditioner and points JTAC at
the sensor `ft_sensor_tcp`), `payload_mass_kg` (initial payload for the conditioner, 0: calibrate),
`ft_filter` (JTAC `ft_sensor.filter_coefficient`), `move_group` (also start MoveIt for planning, hardware `jtac_mock`),
`jtac_position_only` (JTAC claims only the position command interface, see the manager pass),
`base_motion` and `depth_sensor` (+ `depth_sensor_pose`, `depth_sensor_noise_m`, `depth_sensor_rate_hz`; both off by default, see
below). Several containers on one Docker bridge share DDS
multicast: give each simulation its own `ROS_DOMAIN_ID` (or `ROS_LOCALHOST_ONLY=1`), otherwise
another stack's `/robot_description` can reach the gz controller manager.

What starts: gz server, `robot_state_publisher`, the robot spawned from `hardware:=gazebo`,
`ros_gz_bridge` (`/clock`, raw wrench, contacts), `contact_ground_truth`, and spawners for
`joint_state_broadcaster`, `force_torque_sensor_broadcaster` and the chosen controller. The
controller manager lives inside the gz server (`gz_ros2_control`). `jtc` uses
`joint_trajectory_controller`; `jtac` loads `config/jtac_controller.yaml` of
`compliant_trajectory_controller` with joints and frames replaced from the profile and the sensor
`ft_sensor` in frame `ft_sensor_link`. No wrench axis is selected there, so it tracks like a JTC.

## The force path (`ft_conditioner:=true`)

```
gz force_torque sensor -> ft_sensor (gz system) -> force_torque_sensor_broadcaster
   /force_torque_sensor_broadcaster/wrench  (raw, ft_sensor_link, includes the payload weight)
 -> ft_conditioner   payload from ~/capture_pose + ~/identify, sign +1, tool offset from TF
   /ft_conditioner/wrench_conditioned  (contact wrench at tcp_link)
 -> ft_bridge_hardware, sensor `ft_sensor_tcp` (block `ft_conditioned_bridge`), biased by ~/target_wrench
 -> JTAC (ft_sensor.name ft_sensor_tcp, frame tcp_link, control frame tcp_link)
```

**How the bridge sits in Gazebo.** `gz_ros2_control`'s resource manager loads every `<ros2_control>` block
of the robot description through its own plugin loader with the base class
`gz_ros2_control::GazeboSimSystemInterface`. A second, non-Gazebo hardware component (the plain
`FtBridgeHardwareInterface`, a `SensorInterface`) is refused: `class ft_bridge_hardware/FtBridgeHardwareInterface
with base class type gz_ros2_control::GazeboSimSystemInterface does not exist`, its interfaces never appear and
JTAC cannot activate (`state interface 'ft_sensor_tcp/force.x' is not available`). Tried and rejected: a second
controller manager (JTAC has to live in the one that owns the joints). Arrangement used: a thin class
`ft_bridge_hardware/FtBridgeSimSystem` (package `ft_bridge_hardware`, built only when `gz_ros2_control` and
gz-sim8 are found) that derives from that base class and forwards lifecycle, `read()` and the state interfaces
to an embedded `FtBridgeHardwareInterface`, so the bridge itself is unchanged (`~/target_wrench` is
`/ft_bridge_hardware_ft_sensor_tcp/target_wrench`). In the description it is a second
`<ros2_control name="ft_conditioned_bridge" type="system">` block for `hardware:=gazebo` only, with the
sensor `ft_sensor_tcp` (the raw sim sensor stays `ft_sensor`) and `source_topic`
`/ft_conditioner/wrench_conditioned`. The block exists whether or not the conditioner runs; without it the
sensor reads minus the setpoint.

## What is modelled

- Joints: lift + six arm joints with the `jtac_mock` interfaces (position and velocity command,
  position, velocity and effort state). The base is pinned to the world through a `world` link
  (static base). Mimic lift stages follow through `robot_state_publisher`.
- F/T sensor: link `ft_sensor_link` sits between the arm flange and the tool, at the flange pose
  (kinematics equal the other backends). The tool hangs behind it, offset by `tcp_xyz` along the
  tool axis. gz `force_torque` sensor on `ft_sensor_joint`, exposed as ros2_control sensor
  `ft_sensor` (`force.*`, `torque.*`) and as `/force_torque_sensor_broadcaster/wrench`
  (WrenchStamped, frame `ft_sensor_link`). The raw gz topic is bridged to `/ft_sensor/wrench_gz`.
- Raw means raw: nothing is compensated or moved to the tcp.
  **Sign convention:** gz `frame=child`, `measure_direction=child_to_parent`: the reading is the
  force and torque the tool side exerts on the arm side, in the sensor axes, about the sensor
  origin. By statics `reading = f_env + m g`, i.e. at rest it is the weight of the sensor body and
  tool (7.85 N for 0.8 kg), in contact it adds the force the environment exerts on the tool. A
  conditioner removes `m g` (and `com x m g`), moves the torque to the tcp with the lever arm
  `tau_tcp = tau - r x f`, and rotates into `tcp_link`. Pressing along tool +z gives `fz < 0`.
- Ground truth: a gz contact sensor on the tool collision, summed by `contact_ground_truth.py`
  into `/ft_sensor/ground_truth_wrench` (WrenchStamped, frame `tcp_link`, the wrench the environment
  exerts on the tool at the tcp). For evaluation only; zero when nothing touches the tool.
  Only the tool collision is counted (contact on the wrist or arm links is not).
- Worlds: `wall_world_gen.py` / `sim_gazebo.wall_world` turn `wall_scene.json` (the coverage
  planner's input: wall slab behind the plane, obstacles as boxes `uv_on_wall` x
  `protrusion_max_m`) or a SceneModel YAML (`objects:` box, sphere, cylinder; `contact_surface`
  becomes the wall) into SDF with ground, physics and the force-torque and contact systems.
  `--standoff` yaws the scene so the wall's normal faces the robot, places the plane that far in
  front of the base and the wall's lowest edge on the floor. `--crop-u/--crop-v` limit the wall.
- Friction: mu 0.8 on tool, wall and ground. Inertials come from the description; the sensor link
  adds 0.3 kg with its centre of mass 2 cm out.

## Base motion (`base_motion:=true`)

Default `false`: the base is pinned to the world and the rendered description is identical to before (checked by rendering both).
With `true` the world joint is dropped and the model is driven by two gz-sim systems that already exist, no custom C++:
`VelocityControl` (a `geometry_msgs/Twist` on `/cmd_vel`, bridged by `ros_gz_bridge`) and `OdometryPublisher` (planar,
`nav_msgs/Odometry` on `/odom` and TF `odom -> base_footprint` on `/tf`, bridged). Same ROS interface as the MuJoCo sim.
The wheels stay visual: a flat 0.8 x 0.5 m skid with friction 0.02 under the base keeps it at the right height (the base box floats
5 cm above the floor) and upright. The arm, lift, JTC/JTAC, F/T sensor and the compliant tool tip are unchanged.

```bash
bash $(ros2 pkg prefix sim_gazebo)/lib/sim_gazebo/run_base_motion.sh     # about 1 minute
```

`scripts/base_motion_test.py` (mirror of the MuJoCo one) drives 0.2 m/s until `/odom` says 0.3 m and checks: odom starts at the
origin, frames `odom -> base_footprint`, travel 0.3 m +-0.03, lateral and yaw error below 2 cm / 0.02 rad, TF equals odom,
`odom -> tcp_link` resolves, arm and lift joints hold (< 0.02), raw F/T within 3 N of the resting reading, no drift at rest.
Result: travelled 0.294 m, lateral and yaw error 0.0000, joint deviation 0.0000, F/T change 0.17 N, drift 0.00 mm.
Differences from MuJoCo: `VelocityControl` has no command timeout (a stale `/cmd_vel` keeps driving; send zero), no velocity or
acceleration limits (use `stand_navigator`'s), omni only (the profile `drive_type` is not applied, `vy` is not dropped for
`diff`), and no odom noise. Not tried: the wall contact and the manager pass with a free base.

## Depth sensor (`depth_sensor:=true`)

Default `false`: nothing is added. With `true` the description gets `depth_sensor_link` (on `base_footprint` at
`depth_sensor_pose`, default `0.1 0 1.15 0 0 0`, x forward) with a gz-sim `depth_camera` (96 x 64, 60 degree horizontal field of view,
0.1 to 5 m, `depth_sensor_rate_hz` 5, optional Gaussian range noise `depth_sensor_noise_m`), the launch file adds the `Sensors` system
(ogre2) to the world, and `ros_gz_bridge` publishes the gz point cloud as `sensor_msgs/PointCloud2` on `/depth_sensor/points`
in frame `depth_sensor_link` (TF from `robot_state_publisher`). The cloud is organised (96 x 64) with NaN for rays that hit nothing.
This is a rendered depth image: it sees every visual in the world (wall, obstacles and the robot's own links), unlike the MuJoCo
ray cast against scene geoms. Headless rendering works in the `sim-gazebo` image with no display and no GPU: gz prints "Unable to open
display ... Trying to run in headless mode" and renders in software (about 6144 points per scan at 5 Hz, no problem seen).

```bash
bash $(ros2 pkg prefix sim_gazebo)/lib/sim_gazebo/run_depth_wall.sh      # needs wall_detector and coverage_planner too
```

`run_depth_wall.sh` starts the sim with the smoke wall world (front face `x = 0.915` in the world and `base_footprint`; the tcp
plane is 0.98), `wall_detector` and the coverage planner nodes, then `scripts/depth_wall_test.py` checks the cloud (frame, points,
median sensor-frame x equal to the wall distance), the fitted plane (normal within 1 degree, plane within 1 cm at the visible corners and
centre, extent), and that the coverage planner accepts the written scene. Result with 3 mm range noise: 6144 points,
median x 0.8150 m (wall at 0.8150), normal error 0.008 degrees, plane error 0.08 mm, rms 2.99 mm, extent 0.95 x 0.64 m, coverage plan 14 waypoints
and 1.02 m^2. At the home pose the fit is clean (the robot does not block the view); an arm raised in front of the sensor would appear in the cloud and is not filtered.

## Tests

```bash
colcon test --packages-select sim_gazebo     # generator (9) and hardware:=gazebo rendering (6, incl. base_motion / depth_sensor off by default), no gz needed
ros2 run sim_gazebo smoke_test.py            # gz server, spawn, trajectory, wall contact (about 2 minutes)
```

The smoke test (profile `generic_6dof_telescopic`): gz up and `/clock` advancing, home held under
gravity, a trajectory home to a pose with the tool pointing at the wall (checked against forward
kinematics of the rendered URDF) with `/joint_states` within 0.03 rad, a second trajectory 3.5 cm
into the wall, then the raw wrench is converted with plain math (payload weight, lever arm,
rotation) and compared with the ground truth: force within 3 N or 15 percent, torque within
0.5 Nm or 20 percent. Observed: force difference 0.05 N of 209 N, torque 0.00 of 9.3 Nm.

## Contact-controlled test

`ros2 run sim_gazebo contact_control_test.py` (starts the simulation itself, `--no-launch` to use a running one).
Steps and results (profile `generic_6dof_telescopic`, wall 0.9 m in front of the base, tool contact at tcp x = 0.965 m):

1. **Calibration** of the conditioner with six still, contact-free wrist poses (`~/capture_pose`, `~/identify`):
   mass 0.8003 kg (URDF inertials 0.8000), centre of gravity (0, 0, 0.0075) m (known 0.0075), bias zero, fit
   residual 0.0000 N. In free space the conditioned wrench is then 0.000 N / 0.0000 Nm.
2. **Conditioned vs ground truth in steady contact** (JTAC with no wrench axis selected and its default 0.1 m
   offset clamp, tool pressed 5 mm and 35 mm past the wall; with the rigid tool the ground truth was about 108 N
   along the tool at both depths (with the compliant tip of step 3: 8.0 N at 5 mm, 105.6 N at 35 mm, where the tip is on its stop), 120 N lateral
   from friction and a contact torque of about 11 Nm about the tcp, so the lever-arm and payload terms are both
   exercised): force error |dF| mean 0.001 N (max 0.002) at 5 mm and 0.005 N (max 0.011) at 35 mm; torque error
   |dT| mean 0.0002 Nm (max 0.0005) and 0.0010 Nm (max 0.003). The checks gate the mean with 1 N and 0.2 Nm; the
   maxima are printed, not checked.
3. **Force control with the tool-z admittance axis** (target 10 N through `~/target_wrench`, K 300 N/m, mass 8,
   damping ratio 6, offset clamp 3 mm, approach 2 mm/s, reference 5 mm past the wall): **held with the compliant tool
   tip, not with the rigid one.** Ground-truth force over the last 4 s of the 8 s hold (`--require-hold` makes it a
   check):

   | tool                                            | mean   | std    | max (last 4 s) | peak since approach | in contact |
   |-------------------------------------------------|--------|--------|----------------|---------------------|------------|
   | rigid (before)                                  | 163.9 N | 120.6 N | 466 N          | 680 N               | 84 percent |
   | spring tip 5 N/mm, reference at the wall        | 8.43 N | 0.00 N | 8.43 N         | 8.8 N               | 100 percent |
   | spring tip 5 N/mm, reference 2 mm past          | 8.93 N | 0.00 N | 8.93 N         | 9.1 N               | 100 percent |
   | spring tip 5 N/mm, reference 5 mm past (default)| 9.69 N | 0.00 N | 9.69 N         | 9.7 N               | 100 percent |
   | spring tip 20 N/mm, reference 5 mm past         | 10.94 N | 0.00 N | 10.94 N        | 11.1 N              | 100 percent |

   (one run each; the simulation is deterministic, so the standard deviation is the numerical resolution of a
   settled contact, not a measured noise level). MuJoCo holds 10.13 N, std 0.03.
   **Why it was not held:** the rigid tool on a servo joint meets the wall with an effectively infinite stiffness
   (about 108 N for a position hold 5 mm and 35 mm past the wall, 209 N with plain trajectory control), so the
   force loop sees either 0 N or hundreds of newtons within a fraction of a millimetre of travel, and the tool
   bounces about 3 mm at 2 Hz. Gains from K 100 to 300 N/m, mass 8 to 1e5 kg, damping up to 1e7 Ns/m, approach
   speeds, low-pass filters and offset clamps from 3 mm to 0.1 m had no effect (earlier sweeps, not in the
   committed script). With 5 N/mm of real compliance the same controller, with unchanged gains, settles.
   **Compliant tip** (`urdf/gazebo_tags.xacro`, parameters `tip_stiffness` 5000 N/m, `tip_damping` 100 Ns/m,
   `tip_travel` 20 mm): the tool's collision moves from `tool_mount_link` to `tool_tip_link`, a side branch on a
   prismatic joint along the tool axis (`tool_tip_joint`, `implicitSpringDamper`, spring reference at the rest
   position, the wall can only push it back). The tcp stays on the rigid `tool_mount_link`, so kinematics and the
   controller are unchanged; the generic tool takes `collision:=false` for `hardware:=gazebo`. The contact
   sensor and the ground-truth wrench now read the tip. In steady contact the force follows
   k_tip (F_t + K e)/(K + k_tip) where e is the reference penetration, as for the MuJoCo wall; the reference
   pose therefore sets how close to the target the force settles (8.4 N at the wall, 9.7 N at 5 mm past the
   wall). The residual (0.3 to 1.6 N) is the admittance spring acting on the reference offset; it was not tuned further.
   Overpressing past the tip travel (35 mm past the wall in step 2) still ends on the hard stop and gives about
   105 N, as before. With 20 N/mm the hold is at 10.94 N, but the conditioned-vs-truth check at 35 mm failed
   (107 N difference, the hard-stop contact; not investigated), so 5 N/mm is the default.
4. **Release:** target to zero, disengage, retreat to the standoff: ground-truth force 0.000 N after the retreat.


## Manager pass (the real pipeline)

`scripts/run_manager_pass.sh` is the Gazebo twin of `sim_mujoco/scripts/run_manager_pass.sh` (same steps, same ready
pose, same contact region): the sim (`profile:=ur10e_telescopic controller:=jtac jtac_position_only:=true
ft_conditioner:=true move_group:=true`, wall world from `test/data/smoke_wall_scene.json`) comes up headless, the coverage
planner plans the contact region (`test/data/wall_pass_coverage_scene.json` with `walls.pcd` beside it, tool
`contact_wiper`, `tool_frame_yaw_deg` -90, swath 0.14 m), `world_model_loader_node` puts the whole 2 m x 2 m wall into
MoveIt's scene as an octomap, the arm goes to the ready pose (`ready_pose.py`, the MuJoCo smoke test's FREE pose), and
`manipulation_manager` (`controller_name combined_arm_lift_controller`, wall `[1, 0, 0]`, `dock_at_stroke_start`, 10 N,
0.01 m/s, `hardware:=jtac_mock`, `profile:=ur10e_telescopic`, `use_sim_time` true: Gazebo has a `/clock`) docks, presses,
strokes and retracts per column. `record_run.py` records the tool point and the ground-truth force; `score_run` scores it.
Outputs and logs go to `MANAGER_PASS_OUT` (default `/tmp/manager_pass`); `MANAGER_PASS_SPEED/PRESS/TARGET/SWATH/TOOL_YAW`
and `MANAGER_PASS_LAUNCH_ARGS` override the defaults.

Geometry: the tool tip ends 6.5 cm short of the tcp, so the world wall is at x = 0.915 while the plane the coverage
planner, the octomap and the scorer use is where the tcp touches it, x = 0.98 (the same numbers as the MuJoCo pass).

Status: the pass runs end to end and the scorer passes (exit 0): all three columns dock, press, stroke and retract
(`done=3 failed=0`), coverage 0.93 (865 of 930 cells), peak force 16.8 N, RMS error 2.14 N against the 10 N target, in-band
(+-1.5 N) 0.85, about 190 s of sim time. Runs vary a little (three other runs of the final configuration: RMS 1.6 to 1.9 N,
in-band 0.86 to 0.91, peak 15.9 to 18.1 N, coverage 0.93 each time). Ground-truth force is the compliant tool tip's
(`/ft_sensor/ground_truth_wrench`), the tool point comes from TF.

What it took, beyond the MuJoCo lessons (wall octomap, tool roll, `dock_at_stroke_start`, `press_settle_s`, swath, stale
coverage file, zero force target before disengage):

- **The wall in MoveIt must be the whole wall.** With only the contact patch as an octomap, free-space moves (OMPL
  `prime_continuity`) swung over the patch edge (z up to 2.7 m) and hit the real wall top at 684 N.
- **`jtac_position_only`** (a launch argument, off by default). With both JTAC command interfaces (position and velocity),
  gz_ros2_control drives the joints from the velocity feed-forward and ignores the position command (open loop), so a
  moving reference was tracked far behind: the 0.01 m/s stroke ended 8 cm short, the dock 20 cm short, coverage 0.88.
  With the position interface only the same pass reaches coverage 0.93. The cost: gz_ros2_control's position loop has a
  gain of 0.1 /s (10 s time constant), so the arm settles slowly (the ready pose needs about 20 s; the calibration poses of
  `contact_control_test.py` do not settle in its 5 s moves and fail, so it keeps the default). A gain of 10
  (`position_proportional_gain`) makes the joints stiff and the contact bounces (8 kN peaks); the soft loop is part of
  why the contact holds. Not fixed: a properly tuned joint model.
- JTAC's Cartesian offset clamp (3 mm instead of the default 0.1 m) made no difference to this pass and is not used.
- The tool roll is -90 degrees because the UR10e ready pose has the same roll as in MuJoCo; the generic 6-DOF profile
  (smoke test pose, wrist roll -46.4 degrees) also ran end to end but its ACM only has adjacent pairs, and
  OMPL-chosen configurations then collided in the manager's Cartesian planner in some runs, so the pass uses the UR10e.

## Limits

- Contact is compliant only through the tool tip spring (5 N/mm, 20 mm travel). Past the travel the tool is
  rigid again: about 105 N at 35 mm past the wall. It settles 0.3 to 1.6 N below the target depending on the reference depth (9.69 N with the
  default 5 mm). The wall itself is still a rigid primitive. Spring and damper are plausible round numbers,
  not an identified tool.
- Joint dynamics are idealised: `gz_ros2_control` position/velocity interfaces, no gearbox
  friction, backlash, motor limits (URDF limits are not enforced) or sensor noise, bias and drift.
- The tool body and wall are rigid primitives with Coulomb friction; only the tip spring gives compliance, no
  wall material model, no wet or sticky effects.
- The gz sensor measures rigid-body joint loads including dynamic terms, so the check above is
  exact only quasi-statically.
- Base motion is a velocity-commanded planar body (see above), not wheel physics; lift mimic stages are visual only; collision uses the
  description's primitives.
- Gazebo needs the `sim-gazebo` image; it is not a rosdep dependency of the package, so the
  common `deps` image stays small.
