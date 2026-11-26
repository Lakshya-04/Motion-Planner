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
bash $(ros2 pkg prefix sim_gazebo)/lib/sim_gazebo/run_spray_pass.sh           # non-contact spray pass (about 3 minutes)
bash $(ros2 pkg prefix sim_gazebo)/lib/sim_gazebo/run_multi_stand_pass.sh     # base_motion, plan stands, drive, 9 columns (about 9 minutes)
```

Launch arguments: `profile` (name in `config/profiles` or a path; its `robot.xacro_args`, joint
names, frames and home pose drive everything), `world`, `wall_scene`/`wall_id`/`standoff`, `gui`,
`controller` (`jtc` | `jtac` | `none`), `ft_conditioner` (`true` starts the conditioner and points JTAC at
the sensor `ft_sensor_tcp`), `payload_mass_kg` (initial payload for the conditioner, 0: calibrate),
`ft_filter` (JTAC `ft_sensor.filter_coefficient`), `move_group` (also start MoveIt for planning, hardware `jtac_mock`),
`jtac_position_only` (JTAC claims only the position command interface, see the manager pass),
`base_motion`, `rgb_camera` (+ `rgb_camera_pose`, `rgb_camera_size`, `rgb_camera_rate_hz`), `rgb_camera_frame` (`base` | `world`), `arm_meshes` (default `true`) and `lidar` (+ `lidar_pose` and the `lidar_*` arguments), `depth_sensor` (+ `depth_sensor_pose`, `depth_sensor_noise_m`, `depth_sensor_rate_hz`; the sensors are off by default, see
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
  (static base). The telescopic lift stages (`lift_stage_N_joint`) are declared in the ros2_control block
  with `mimic` / `multiplier` (1/stages) / `offset` params, state interfaces only: gz-sim does not apply the URDF
  `<mimic>`, so without them the stages stayed at 0 while `lift_joint` carried the arm up and the arm base floated
  above the column. Scissor lift linkage joints are not declared yet (they still stay at 0 in Gazebo). Not run in
  Gazebo here; check: move the lift and read `/joint_states`, `lift_stage_*_joint` must equal `lift_joint / stages`.
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
`diff`), and no odom noise. The manager pass with a free base is the multi-stand pass below.

## Gravity check

`ros2 run sim_gazebo gravity_check.py` starts the sim, drives home and a few poses through the joint trajectory
controller, and compares the effort Gazebo reports for each joint at rest with the torque from
[`gravity_core`](../src/control/gravity_core/README.md) for the same URDF. Exit code 0 when every joint is within
`--tol` (default 2 N m or N). The telescopic stages are compared as outputs of their own, because Gazebo reports the
lift joint alone. Result and numbers: [gravity_compensation.md](gravity_compensation.md).

## Depth sensor (`depth_sensor:=true`)

Default `false`: nothing is added. With `true` the description gets `depth_sensor_link` (on `base_footprint` at
`depth_sensor_pose`, default `0.1 0 1.15 0 0 0`, x forward) with a gz-sim `depth_camera` (96 x 64, 60 degree horizontal field of view,
0.1 to 5 m, `depth_sensor_rate_hz` 5, optional Gaussian range noise `depth_sensor_noise_m`), the launch file adds the `Sensors` system
(ogre2) to the world, and `ros_gz_bridge` publishes the gz point cloud as `sensor_msgs/PointCloud2` on `/depth_sensor/points`
in frame `depth_sensor_link` (TF from `robot_state_publisher`). The cloud is organised (96 x 64) with NaN for rays that hit nothing.
This is a rendered depth image: it sees every visual in the world (wall, obstacles and the robot's own links), unlike the MuJoCo
ray cast against scene geoms. Headless rendering works in the `sim-gazebo` image with no display and no GPU: gz prints "Unable to open
display ... Trying to run in headless mode" and renders in software (about 6144 points per scan at 5 Hz, no problem seen).

## Lidar (`lidar:=true`)

Default `false`: nothing is added. With `true` the description gets `lidar_link` (on `base_footprint` at `lidar_pose`, default
`0.45 0 0.2 0 0 0`) with a gz-sim `gpu_lidar` (`lidar_angle_min_deg` -135, `lidar_angle_max_deg` 135, `lidar_angle_increment_deg` 0.5, so
541 samples, `lidar_range_min_m` 0.05, `lidar_range_max_m` 10, `lidar_rate_hz` 10, optional Gaussian `lidar_noise_m`), the launch file adds the
`Sensors` system to the world and `ros_gz_bridge` publishes `sensor_msgs/LaserScan` on `/scan` in frame `lidar_link` (TF from
`robot_state_publisher`). Like the depth camera it is rendered, so it sees every visual in the world. Headless works in the
`sim-gazebo` image with no display and no GPU (ogre2 in software): about 9.9 Hz, 541 rays, nothing else needed. In the empty world the
sensor on the front face of the base returns `inf` on every ray, so the robot does not hit itself at that mount. No dropout option
here, and gz prints a harmless warning that `gz_frame_id` is not in the SDF schema (the same as for the depth camera).

`platform:=<profile or base name>` fills the lidar and depth arguments and the two link / frame names (`gz_lidar_frame`,
`gz_depth_frame` in the xacro) from that platform's `sensors:` suite; arguments you pass still win. See
[base_catalog.md](base_catalog.md#feeding-a-suite-to-the-simulators).

`scripts/run_lidar_stop.sh` is the obstacle-stop check: empty world, a 0.3 x 0.6 x 0.6 m box with its face at x = 1.45 m, the base sent 3 m
ahead through `navigate_to_stand` with `obstacle_scan_topic:=/scan` (`LIDAR_STOP_LIDAR=0` leaves the lidar and the topic off).
Results are in the changelog. Without the lidar the base hits the box, gz pushes it back and the navigator ends with `NAVIGATION_STUCK`.

```bash
bash $(ros2 pkg prefix sim_gazebo)/lib/sim_gazebo/run_depth_wall.sh      # needs wall_detector and coverage_planner too
```

`run_depth_wall.sh` starts the sim with the smoke wall world (front face `x = 0.915` in the world and `base_footprint`; the tcp
plane is 0.98), `wall_detector` and the coverage planner nodes, then `scripts/depth_wall_test.py` checks the cloud (frame, points,
median sensor-frame x equal to the wall distance), the fitted plane (normal within 1 degree, plane within 1 cm at the visible corners and
centre, extent), and that the coverage planner accepts the written scene. Result with 3 mm range noise: 6144 points,
median x 0.8150 m (wall at 0.8150), normal error 0.008 degrees, plane error 0.08 mm, rms 2.99 mm, extent 0.95 x 0.64 m, coverage plan 14 waypoints
and 1.02 m^2. At the home pose the fit is clean (the robot does not block the view); an arm raised in front of the sensor would appear in the cloud and is not filtered.

## Colour camera (`rgb_camera:=true`)

Default `false`: nothing is added. With `true` the description gets `rgb_camera_link` (on `base_footprint` at `rgb_camera_pose`,
default `0.0 -2.4 1.6 0 0.11 1.38`: 2.4 m to the right of the base, looking at the wall in front of the arm), a gz-sim `camera`
(`rgb_camera_size` default 1280 x 720, 60 degrees, `rgb_camera_rate_hz` 2) and the launch file adds the `Sensors` system (ogre2,
software rendering, no display) and bridges `/rgb_camera/image` (`sensor_msgs/Image`, rgb8). The camera is fixed to the base, so
after a base move it moves with it (a multi-stand run with stills showed this: the base drives toward the wall and the camera ends up against it, every later frame is a blank wall) (`rgb_camera_frame:=world` below fixes this). `scripts/capture_still.py OUT_DIR --every S --count N` writes PNGs (no imaging library).
`GZ_PASS_STILLS=N` in the pass scripts turns the camera on and runs it.

`rgb_camera_frame:=world` (also `GZ_PASS_CAMERA_FRAME=world` in the pass scripts) makes the camera a static model in the world at
`rgb_camera_pose` (now a world pose), so it stays put while the base drives: the multi-stand pass can be filmed (the default `base`
keeps the old behaviour). The default pose `0 -2.4 1.6 0 0.11 1.38` is 2.4 m to the right of the spawn point; for a 0.9 m patch
starting at y = 0.3 a pose further back may be wanted. Rendering was checked only through the generated world (unit tests), not
filmed in a run.

**`arm_meshes` (default `true`).** gz resolves the description's `model://ur_description/...` URIs through `GZ_SIM_RESOURCE_PATH`;
the launch file now adds the `ur_description` share to it by default (a warning if the package is missing). Before, gz could not
find the UR meshes (an error per link), so the arm had NO visual and NO collision geometry (only the lift, the sensor link and the
tool existed) and every earlier Gazebo number in this document was measured that way. `arm_meshes:=false` (`GZ_PASS_MESHES=0`)
restores that. With the meshes the physics differ (arm collision, arm self-collision, real link shapes), see the spray and multi-stand
tables for the effect on the passes.

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
   **Bump stop** (`stop_stiffness` 50 N/mm, `stop_damping` 500 Ns/m, `stop_travel` 5 mm): `tool_tip_link` now hangs off a
   second, stiff and short prismatic stage (`tool_tip_stop_joint`, link `tool_tip_stop_link`, 10 g) in series behind the soft one,
   so beyond the 20 mm travel the force rises 50 N per mm instead of the joint limit stopping the tool at once (a joint limit alone
   is rigid). It adds 10 g to the payload (the conditioner identifies 0.8703 kg against 0.8500 in the URDF, check tolerance 0.02 kg).
   Effect measured in the multi-stand pass: none on the peaks (see there). Overpressing past the tip travel (35 mm past the wall in step 2)
   gives about 106 N, as before. With 20 N/mm the hold is at 10.94 N, but the conditioned-vs-truth check at 35 mm failed
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
  why the contact holds. The manager passes now set the gain to 2 (`position_gain:=`, see "Cause found" below); 0.1 left the arm behind the plan.
  Not fixed: a properly tuned joint model.
- JTAC's Cartesian offset clamp (3 mm instead of the default 0.1 m) made no difference to this pass and is not used.
- The tool roll is -90 degrees because the UR10e ready pose has the same roll as in MuJoCo; the generic 6-DOF profile
  (smoke test pose, wrist roll -46.4 degrees) also ran end to end but its ACM only has adjacent pairs, and
  OMPL-chosen configurations then collided in the manager's Cartesian planner in some runs, so the pass uses the UR10e.

## Spray pass and multi-stand pass (parity with MuJoCo)

Both reuse the MuJoCo flow with Gazebo helpers (`scripts/gz_pass_common.sh`, the twin of `manager_pass_common.sh`);
`shift_wall_scene.py` and `execute_coverage_client.py` are copies of the MuJoCo ones (nothing simulator specific in them).
All runs: headless, UR10e, `jtac_position_only`, one run at a time, `ROS_DOMAIN_ID` 191 to 197, the host loaded by other work
(load average 8 to 22). The geometry difference to MuJoCo that matters: the world wall is at x = 0.915 but every plane the
planner, the octomap and the scorer use is x = 0.98 (the spring-loaded tool tip ends 6.5 cm short of the tcp), so a spray nozzle
"0.30 m from the wall" is 0.365 m from the real wall in Gazebo.

**Spray** (`scripts/run_spray_pass.sh`, about 3 minutes). `ready_pose.py --joints ... --planned` is the MP_READY seed (a MoveIt-planned,
collision-checked move to the first stroke's arm configuration, velocity scale 0.3, same joints as MuJoCo); `ready_pose.py` got
`--joints/--planned/--scale/--settle` for it. `record_run.py` records the valve (`/tool/valve`) as the `active` column.
Seed, 20 degree cone, 0.30 m stand-off, 0.05 m/s, 20 percent overlap, `score_run --no-contact`:

| run | coverage | thickness CoV | peak force | notes |
|---|---|---|---|---|
| a | 1.000 (930/930) | 0.56 | 0.0 N | 3 columns done, 89 s |
| b | 1.000 | 0.53 | 0.0 N | 98 s |
| c | 1.000 | 0.55 | 0.0 N | 102 s |
| camera on, no meshes | 1.000 | 0.92 | 0.0 N | the seed ended at z 0.77 instead of 1.4 (the arm had not settled), the first stroke started from a worse arm configuration |
| arm meshes + camera (1) | 1.000 | 0.47 | 0.0 N | different physics (arm collision), see above |
| arm meshes + camera (2) | 0.581 (540/930) | 1.76 | 0.0 N | scorer fails; 125 s, 3 columns reported done; not investigated (the stills are from this run) |
| run killed | - | - | - | hung: the manager's first plan went on, JTAC reported "Cartesian offset diverged (0.5 m)" every few seconds and the goal never finished; seen once in 6 runs, the host was running a second simulation |

Pass rate: the 4 runs without meshes pass (3 plain, 1 with the camera on); with the meshes 1 of 2 passes; 1 further run hung (7 attempts). MuJoCo: coverage 1.0, CoV 0.52. Gazebo
matches in the three plain runs. The CoV is a property of the stroke spacing and the speed, the valve was open 346 to 594 samples.

**Re-measured with `arm_meshes` on (now the default), idle host, built from this branch (`fix/gazebo-build-tip`):**

| run | coverage | thickness CoV | result | what happened |
|---|---|---|---|---|
| 1 | 0.920 (856/930) | 0.80 | passes | 142 s |
| 2 | 0.573 | 1.16 | fails | the manager's first free-space plan (`prime_continuity`) was refused by MoveIt ("collision truncation" octomap vs `arm_wrist_2_link` / `arm_forearm_link`), JTAC logged "Cartesian offset diverged" 53 times |
| 3 | 0.000 | - | fails | same refused plan, 24 JTAC divergences, 73 s |
| (stray run, log lost) | 0.484 | 1.73 | fails | |

1 of 3 (1 of 4 counting the stray run) against 4 of 4 without meshes.

**Root cause and fix (this branch).** Not the manager, not the SRDF and not the wall octomap margin. Traced with the joint states and
link poses of a seed move in the sim: the seed is a MoveIt plan of 4.1 s (velocity scale 0.3), but gz_ros2_control's position loop
under `jtac_position_only` has a 10 s time constant (gain 0.1 /s). The joints therefore do not follow the planned path: all of them
converge exponentially on the goal, i.e. the arm goes along a joint-space straight line from the stowed pose, which MoveIt never
checked. That line takes the wrist (`arm_wrist_1/2_link`) over the wall at z 2.06 to 2.1 m, the wall is 2.0 m tall with its front at
x = 0.915; with the meshes the arm has real collision, so the wrist comes to rest on the wall top (wrist 1 at x 1.10, z 2.09) and
elbow / shoulder_lift / lift stay 0.5 to 1.2 rad off the seed (tcp (0.79, 0.27, 2.28) instead of (0.68, 0.25, 1.26)). The manager's
first plan (`prime_continuity`, a joint goal to the live state) then starts in collision with the wall octomap (plane x = 0.98,
z up to 2.0) and `CheckStartStateCollision` refuses it; whether the arm slides off the wall top or not decides pass or fail (hence
0.92 / 0.57 / 0.0 and the random JTAC "Cartesian offset diverged": the controller is told to hold a reference the arm cannot reach).
Without the meshes the arm passes through the wall top, which is why it never showed. The geometry mismatch (octomap at 0.98 against
the wall at 0.915) is not the cause: the converged arm is clear of both. The seed plan is now 27 s (`SPRAY_PASS_SEED_SCALE`, default
0.04, 270 points): the arm follows it, the seed is reached to 1 mrad (tcp 0.68, 0.25, 1.26) and the first plan is accepted.
Collision checking, the ACM and the scene are unchanged. Baseline with the old scale on the same build (3 more runs): coverage
0.988 (passes, 83 JTAC divergences), 0.805 (fails), and one failed diagnostic run, all with the refused first plan.

**Re-measured, meshes on, 27 s seed, idle host (4 runs, 4 containers in a row, `ROS_DOMAIN_ID` 71 to 74):**

| run | coverage | thickness CoV | result | notes |
|---|---|---|---|---|
| 1 | 0.919 (855/930) | 0.55 | passes | 202 s, 38 JTAC divergence warnings |
| 2 | 1.000 | 0.57 | passes | 147 s, 3 warnings |
| 3 | 1.000 | 0.62 | passes | 198 s, 54 warnings; 884 N ground-truth peak while the valve was closed (a retract touched the wall) |
| 4 | 0.963 | 0.65 | passes | 159 s, 26 warnings |

4 of 4 pass (the scorer's limit is coverage 0.9), against 1 of 3 before, no refused plan in any run (the old seed on the same build:
0.988 / 0.805 / failed). Run 1 and run 4 sit near the 0.9 limit, so the margin is thin. What is left: the JTAC "Cartesian
offset diverged" warnings still appear (they start during the seed move itself, the arm lags the reference by far more than the
controller's offset clamp, and none of the 4 runs failed on them), and the 0.9 coverage floor is hit with only 6 percent to spare once.
The MuJoCo twin is coverage 1.0.


**Multi-stand** (`scripts/run_multi_stand_pass.sh`, 0.9 m patch from y = 0.3, `stand_max_span_m` 0.5, two stands, 9 columns,
about 9 minutes, `base_motion:=true`, contact pass at 10 N). The reachability map is built once (`STAND_MAP`). The Gazebo world wall
is the stock 3 m wall centred on the robot (the generator always centres the wall), its one obstacle moved out of the patch.

Env knobs, the same names and defaults as the MuJoCo script (the header of each script lists them; `docker/tools/tests/test_multi_stand_script_parity.py`
checks that the header and the variables the script reads agree in both): `MULTI_STAND_DOUBLE_PASS` (default `false`, every strip once;
`true` = up and back down, passed to the coverage demo as `double_pass`), `MULTI_STAND_REACH_FRACTION` / `MULTI_STAND_ARM_REACH` /
`MULTI_STAND_LIFT_STROKE` (manager `dock_ik_reach_fraction` 0 = off, `dock_ik_reach_m` 1.3, `dock_ik_lift_max_m` 1.5; the fraction needs
`MULTI_STAND_DOCK_IK_CONTINUITY=true`), `MULTI_STAND_MIN_CLEARANCE` (reachability node `stand_min_clearance_m`, 0 = off),
`MULTI_STAND_COVERAGE_EXTRA`, `MULTI_STAND_TOOL_YAW`. Both scripts end with one line `multi_stand knobs: sim=... patch_z=... double_pass=...`
so a run's output says which knobs it used.

Tall wall: `MULTI_STAND_PATCH_Z="0.6,2.0"` moves the patch (coverage scene, `walls.pcd`, the manager's `coverage_z_min_m`/`_max_m`, the
recorder's wall band) and makes the sim wall `hi + 0.3` m high (`shift_wall_scene.py`, argument 6; the stock wall is 2 m and the world
generator puts its bottom on the floor). `MULTI_STAND_SCENARIO` (`make_scenario_scene.py`, obstacles and named walls) is MuJoCo only and
stops the Gazebo script with exit 2. NOT verified in Gazebo (no run): the 2.3 m wall in the generated world, the tool reaching z 2.0 with the
lift near its top (the tool sits 6.5 cm short of the tcp, see above), the MoveIt octomap, which is only the patch plus a 5 cm rim here (MuJoCo
pushes the whole sim wall as `walls_sim.pcd` and a curtain), and the reach settings that go with a tall wall (MuJoCo needed `MULTI_STAND_MIN_CLEARANCE`
about 0.42 and `MULTI_STAND_REACH_FRACTION` 0.8). Command:
`MULTI_STAND_PATCH_Z=0.6,2.0 MULTI_STAND_DOCK_IK_CONTINUITY=true MULTI_STAND_REACH_FRACTION=0.8 MULTI_STAND_MIN_CLEARANCE=0.42 bash $(ros2 pkg prefix sim_gazebo)/lib/sim_gazebo/run_multi_stand_pass.sh`.

| run | columns | coverage | peak (score) | RMS error | in band | result |
|---|---|---|---|---|---|---|
| a | 9 of 9 (stand 0: 6, stand 1: 3) | 0.997 (2782/2790) | 922 N | 13.9 N | 0.82 | scorer passes |
| b | 4 of 9 (column 3 press `failed_runtime`, then 5 columns failed on stand 0) | 0.716 | 3119 N | 321 N | 0.57 | fails |
| c | 7 of 9 (stand 1 columns 0 and 1 failed) | 0.763 | 3424 N | 147 N | 0.67 | fails |

Pass rate 1 of 3. Both stands were reached every time (arrival error 4 mm, 0.014 rad). MuJoCo's twin: 9 of 9 once and 5 of 9 once
(`docs/sim_mujoco.md`). The force numbers are far worse than the MuJoCo ones (20 N peaks): the manager's own `peak_force_n`
(conditioned wrench) was 236 to 380 N in runs that finished all columns, the ground truth peaks are single-sample impacts.

**Re-measured with the arm meshes on and the tip bump stop** (this branch, idle host, 3 runs in a row, same script, reachability map reused):

| run | columns done | coverage | peak (score) | RMS error | in band | samples above 350 N / 1000 N | result |
|---|---|---|---|---|---|---|---|
| 1 | 6 of 9 (columns 0 to 2: dock failed) | 0.749 | 1368 N | 75.9 N | 0.53 | 1198 / 252 of 210k | fails |
| 2 | 8 of 9 (column 0: dock failed) | 0.939 | 1248 N | 34.3 N | 0.59 | 1058 / 12 of 246k | scorer passes (558 s) |
| 3 | 7 of 9 (column 2 only on retry, others failed) | 0.889 | 1758 N | 35.3 N | 0.62 | 627 / 4 of 249k | fails |

1 of 3 again. The bump stop did NOT bound the peaks: they are the same order as before (922 to 3424 N without meshes, 1248 to 1758 N
now). A fourth run with a softer, more damped stop (30 N/mm, 10 mm, 2000 Ns/m) completed 9 of 9 columns but regulated worse (in band
0.04, peak 1710 N), so the stiffer 50 N/mm / 5 mm stop is kept. Why the stop is not the cause: in the quasi-static overpress
(`contact_control_test.py`, tip 35 mm past the wall) the ground truth is 105.9 N with or without the stop, i.e. the tip never goes
rigid at the arm's servo stiffness; the spikes are sub-millisecond contact impacts (hundreds of samples above 350 N, but only a
few above 1000 N) at the start of a press or on a jam, which the constraint solver produces with whichever stop is behind the tip. A
force-limiting guard in the controller path (the `force_core` `ForceLimiter` is not wired into JTAC) would be the real bound; not
done here, the controller belongs to another change.

**Multi-stand with `wait_arm_settled.py`** (this branch, `fix/gazebo-multistand-dock`; 10 columns, two stands, same script and map, 3 runs in a row,
the host was NOT idle: the load average rose to 21 on 12 cores during run 1 to 2 from an unrelated browser, run 3 ran at 13 to 18; every run
started after the loadavg gate of below 12; the start/end loadavg was only sampled from run 2 on, so run 1 has none):

| run | columns done | coverage | peak (score) | RMS error | in band | result | load at end | `settle.log` waits |
|---|---|---|---|---|---|---|---|---|
| 1 | 6 of 10 (stand 0: columns 4, 5, 6 dock failed; stand 1: column 3 dock failed) | 0.601 | 35.8 N | 2.4 N | 0.85 | fails | not sampled | 2.7 s, 20.3 s, 505 s |
| 2 | 10 of 10 | 0.941 | 54.0 N | 3.7 N | 0.85 | scorer passes | 16.7 | 2.7 s, 430 s |
| 3 | 10 of 10 | 1.000 | 3025 N (one impact, RMS 88 N) | 88.0 N | 0.55 | scorer passes | 17.7 | 2.7 s, 47.1 s |

2 of 3 pass against 1 of 3 and 1 of 2 before, but with 3 runs on a loaded host that is not a demonstrated improvement. Peaks are 35 to 54 N in the two
runs without a single-sample impact (before: 1248 to 3424 N); run 3 has one 3025 N sample. The refused-plan pattern of run 1 is the old one: the
OMPL `*_approach` segment completes (29 s) and the next Cartesian dock plan is truncated at the self-collision pair `arm_forearm_link` <->
`arm_wrist_3_link` / `tool_mount_link` and refused 6 times, then `recovery jog refused: the good state is too far away`. That fits a joint
state that still lags the plan (gz position loop, 10 s time constant) at the start of the dock plan. `wait_arm_settled.py` only runs in front
of the stand arrival world-model reload (the stow move before a stand), so it does not cover the lag after an in-stand approach segment: in run 1 the
failing docks were the first ones of stand 0, 37 s after the arrival, and the settle wait before them had returned after 2.7 s (the arm was at rest). Whether
the load (timing) or the approach lag caused the failures could not be separated in 3 runs; not ruled out. Not tested: `dock_approach_extra_m`,
the dock standoff margin, the wall octomap (0.98) vs the Gazebo wall (0.915), a slower approach (`retime_vel_scale`); no collision check was loosened.
The next step is a settle wait between the approach and the dock (a manager hook) or a slower approach retiming, measured on an idle host.

**Re-measured again on this branch (27 s spray seed; the multi-stand pass has no seed move, so the fix does not touch it), 2 runs, same script,
reachability map reused, `ROS_DOMAIN_ID` 81, 82:**

| run | columns done | coverage | peak (score) | RMS error | in band | result |
|---|---|---|---|---|---|---|
| 1 | 8 of 10 (stand 1 columns 2 and 3: dock failed, 6 attempts each, plus a retry) | 0.746 | 2560 N | 63.8 N | 0.58 | fails (1095 s) |
| 2 | 9 of 10 (column 0: dock failed on stand 1) | 0.990 | 1476 N | 96.1 N | 0.62 | scorer passes (1149 s) |

1 of 2 (with the two earlier sets, 3 of 8 over the meshed physics). Both stands are reached every time (arrival error 4 mm and 20 mm).
The columns lost are always on stand 1 (the second stand, after the base drove): the dock plan is refused (`wall_column_N_dock`
`planning_failed`, 12 to 24 times per failed column), and the manager logs `CartesianPlanner: collision truncation` for
`<octomap>` against `tool_mount_link` (138 times in run 1) and `arm_upper_arm_link` against `lift_carriage_link` (22). That is the tool
mount, in front of the wrist, meeting the wall octomap plane (x = 0.98) at the dock standoff of 0.04 m with a 0.1 m approach, not the
wrist-over-the-wall mechanism of the spray seed. Not root-caused here (the pass was only re-measured); the likely levers are the
`dock_approach_extra_m` / `dock_standoff_m` margins and the stand's arm base y, not the collision checking. Peaks are the same single-sample
impacts as before.

Three things this exposed, all fixed here:
- **Ground truth under `base_motion`.** `contact_ground_truth.py` looked the tcp up against the TF frame `world`, which does not
  exist when the world joint is dropped, so the contact force was silently 0 and the first multi-stand run scored coverage 0 although
  all 9 columns were done. The launch file now passes `world_frame: odom` (odom starts at the spawn pose, the world origin).
- **Goal before the manager's TF listener is up** aborts with "no TF odom -> arm_base_link"; the script waits 10 s first. (The race
  is in the manager and is left to its owner.)
- The recorder takes `--frame odom` and the wall in odom, like the MuJoCo one.

Not done: the stand 1 dock failures with the meshes; a controller-side peak-force guard; `run_stand_pass.sh` (single stand, `stand_drive.py`) has no Gazebo twin; the scorer's thin 0.9 limit on peak force is not
used (`--max-peak` default 0), so a pass with a 922 N spike still passes; a cancel run (`MULTI_STAND_CANCEL_AFTER_S`) was not tried;
the spray with a different wall x, and a run with a wall that the tool tip really sits 0.30 m from, were not done.

## Running a pass with cuRobo as the planner

Run twice on 2026-11-21 in the multi-stand pass: **no usable result** (see below). It makes cuRobo v0.9.0 the free-space planner of the manager in a
Gazebo pass; contact seams stay on MoveIt Cartesian. The selection file is
`manipulation_manager/config/planning_curobo.yaml` (`free_space: curobo`, everything else as the default `planning.yaml`;
each choice is explained in the file: IK and collision stay MoveIt because the manager's planners do not call the selected
plugins and cuRobo's collision input is joint-space only, no `scene` plugin because the manager does not feed it).

Two containers, one host: the cuRobo node runs in `motion-planner:full` (GPU, `--network host`), the sim and the manager in
the image you already use for the pass (`docs/sim_gazebo.md` Run). They find each other through DDS, so both need the same `ROS_DOMAIN_ID`
(and the same RMW, and the same IPC namespace if Fast DDS shared memory is on: add `--ipc host` to both, or set
`CUROBO_DOCKER_ARGS=--ipc host`). The manager container needs `curobo_planner` and `curobo_bridge_msgs` built (they are
in the workspace; `motion_planner_ros` loads the `curobo` plugin from `curobo_planner_plugins`); the node container needs
`curobo_bridge` and `curobo_bridge_msgs` in an install space.

1. Generate the 7-joint robot config (lift + UR10e, joint order of the manager's `lift_arm` group; GPU, a few minutes):

   ```bash
   docker/tools/generate_curobo_7dof.sh --out "$PWD/build/robot_assets/ur10e_telescopic"
   # -> build/robot_assets/ur10e_telescopic/lift_arm.yml  (absolute paths inside: do not move the directory)
   ```

2. Start the node (waits for `cuRobo ready` and the six services, then returns; the container keeps running):

   ```bash
   export ROS_DOMAIN_ID=7          # any value, the same in the sim container
   docker/tools/run_curobo_node.sh start --ws "$PWD" \
       --robot "$PWD/build/robot_assets/ur10e_telescopic/lift_arm.yml" --max-boxes 256
   ```

   Add `--esdf-dims 2,2,2 --esdf-voxel 0.04 --esdf-center 0.9,0,1.2` to reserve an ESDF grid (not used by the pass).
   `docker/tools/run_curobo_node.sh status|logs|stop` manage it.

3. Start the pass with the env var, in the sim container with the same `ROS_DOMAIN_ID`. The path must be readable
   there (the installed share or the repo mount):

   ```bash
   export ROS_DOMAIN_ID=7
   MANAGER_PASS_PLANNING_CONFIG=$(ros2 pkg prefix manipulation_manager)/share/manipulation_manager/config/planning_curobo.yaml \
       src/simulation/sim_gazebo/scripts/run_manager_pass.sh
   ```

   `run_multi_stand_pass.sh and run_spray_pass.sh` takes the same variable. The variable adds `planning_config: <path>` to the `manager.yaml` the script writes
   into its output directory (the launch's `params_file`), so with the variable unset nothing changes.

4. `docker/tools/run_curobo_node.sh stop` afterwards.

`gz_pass_common.sh` is unchanged: it does not write the manager parameters, each pass script does.

### What the first Gazebo runs showed (2026-11-21)

The multi-stand script now also pushes the wall to the node at every stand arrival (`MANAGER_PASS_CUROBO_WORLD=1`, `source_frame:=odom`) and sets
`seed_start_from_live_state` and the dock seed, as the MuJoCo script does. Three findings, none a pass:
- With the sim container on the default user (root in this image) and the node on uid 1000, every cuRobo call timed out; a second try with the sim as
  uid 1000 left the manager without a goal. Forcing UDP transport for both (`FASTDDS_BUILTIN_TRANSPORTS=UDPv4` in both containers) fixed discovery:
  the wall loaded in under a second.
- With cuRobo, the first approach "completed" through cuRobo but the tool stayed at the stow pose; the OMPL retry ended 0.83 m short of the approach pose.
- **The same pass without cuRobo fails too** (control run, default planning config, 0 of 10 columns, coverage 0): `move_group` reports
  `Invalid goal state` for the dock goal and `Start state out of bounds`, and 140 approach plans fail. Adding the MuJoCo dock seed did not change it
  (0 done, 6 failed within the first stand). So the Gazebo multi-stand pass is broken independently of cuRobo; the earlier runs recorded below completed
  at least the first stand. Not isolated (suspects: the whole-wall octomap and approach-pose check merged for MuJoCo, the lift joint a hair below its
  limit in the start state).

### Gazebo two-stand pass: findings and fixes of 2026-11-21

Same-session runs of `run_multi_stand_pass.sh` (two stands, ten columns, no cuRobo), one run each, host load 1 to 3:

| code | columns done / failed | score |
|---|---|---|
| commit before the second-stand fix (#110) | 9 / 1 | peak force 583.9 N in the feedback; the pass itself ended `SUCCEEDED` with 1 failed column |
| main with the seeded dock IK (no Gazebo changes) | 0 / 10 | coverage 0.0 |
| main, seeded dock IK off for Gazebo (`dock_ik_seeded: false`) | 4 / 6 | coverage 0.42, peak 39.7 N |
| main, seeded dock IK off and the live start state clamped into the joint bounds (this change) | 6 / 4 | coverage 0.66, peak force 68.6 N in the feedback, 1820 N in the scored contact trace |
| main after #157 (the Cartesian planner start state clamped too), same script | 1 / 9 | coverage 0.16, peak 41.6 N in the feedback, 1240 N in the scored trace; stand 1 not reached (the arm could not stow) |

- **Cause of the regression:** the seeded dock IK (a joint goal on the previous dock's branch, added for the second-stand fix in MuJoCo) gives a configuration
  MoveIt calls an invalid goal in Gazebo, and the arm ends in poses from which later plans fail; with it off the Gazebo script uses the planner's pose goal
  as before. MuJoCo keeps it on (`dock_ik_seeded` defaults to true; `MULTI_STAND_DOCK_IK_SEEDED=true` turns it back on in the Gazebo script).
- **Second cause:** `Joint 'lift_joint' from the starting state is outside bounds by -2e-14` (a rounding error, 0.00000000000002 m below the lift's lower limit): MoveIt's
  `CheckStartStateBounds` rejects such a start state and every plan then fails in milliseconds. `OmplPlanner` already clamped an explicit start state; it now also
  clamps the live state it takes when none is given.
- **Still failing:** 4 columns (the two stand-1 columns' approach and the stow among them), 19 start-out-of-bounds errors remain in the run with magnitudes of 1e-16 to
  2e-14 even with the clamp, so some request path still sends an unclamped start state (not found; the only caller of `MoveGroupInterface::plan` is `OmplPlanner`);
  force peaks of 1820 N in the scored trace (583.9 N before the second-stand fix) are an unrelated problem of the Gazebo contact model that this pass shows and
  does not explain. The Gazebo two-stand pass is therefore still failing; the numbers above are single runs.

- **Cause found (2026-11-26): the joint loop gain, not the planner, the tool or the contact model.** A baseline run on main (no options, headless, idle host)
  gave 3 of 10 columns, coverage 0.32 (884 of 2790 cells), peak 1886 N, wall gap 57 mm, 1592 s. Its `manager.log`: 77 `approach attempt ... ended 0.13 to 0.91 m from the
  approach pose` with `tool still moving after 4.0 s of settling` each time, 5 `dock failed`, and three columns that docked to 2 to 4 mm but
  `17 to 85 deg off the standoff pose`, so no press. gz_ros2_control's position loop (`jtac_position_only`) has a gain of 0.1 /s, a 10 s time constant,
  while a planned approach lasts 1 to 6 s: the joints are still converging when the manager looks, 4 s later. The 4 s settle check only watches the
  tool position, which does not move when wrist 3 (the tool axis) turns, so it passes while the roll is tens of degrees off. Every failed column starts from that
  half-converged arm (wrist joint winding to -4.4 rad in the log), which is what the earlier IK-branch notes above saw. The force peaks are the tool
  meeting the wall in these off-pose arms. MuJoCo's position actuators are far stiffer (kp 10 N m/rad per N m of effort limit, `mjcf_generator.py`) and follow the plans.
  **Fix:** launch argument `position_gain:=G` (xacro `gz_extra_params`, a second `<parameters>` file with `gz_ros_control.position_proportional_gain`),
  and `run_multi_stand_pass.sh` now defaults `GZ_POSITION_GAIN` to 2.0 (the plugin default stays 0.1 everywhere else; `GZ_POSITION_GAIN=0.1` gives the old behaviour).
  Same script, same map, headless, one run each unless noted (peak = scorer's ground truth; the feedback peak is the manager's conditioned wrench):

  | gain /s | columns done | coverage | peak (score) | wall gap max | duration | scorer |
  |---|---|---|---|---|---|---|
  | 0.1 (main) | 3 of 10 | 0.32 | 1886 N | 57 mm | 1592 s | fails |
  | 1 | 6 of 10 (10 `not pressing`, orientation) | 0.70 | 29.6 N | 22 mm | 421 s | fails |
  | 2 (4 runs, the last one on the new script default) | 10 of 10 each | 1.00 each | 58.7, 50.9, 44.6, 53.5 N | 15, 12, 7, 13 mm | 540 to 559 s | passes 4 of 4 |
  | 3 | 7 of 10 | 0.90 | 15 005 N | 56 mm | 534 s | fails |
  | 4 | 8 of 10 | 0.96 | 17 036 N | 58 mm | 583 s | passes (coverage only; in-band 0.05) |

  The usable window is narrow: at 3 /s and up the stiff joints bounce on the wall (the earlier 8 kN note under `jtac_position_only`), at 1 /s the
  roll is still off at the dock. 2 /s is a measured sweet spot, not a derived one. At 2 /s the peak is 45 to 59 N against about 20 N in MuJoCo and the
  in-band fraction is 0.45 to 0.53, so the force quality is still behind MuJoCo (why is not isolated; the soft tip and the 200 Hz controller are
  suspects, not tuned here). The single-stand `run_manager_pass.sh` (4 columns, 930 cells) shows the same thing: at 0.1 /s on main, run 1 docked
  nothing useful (coverage 0, peak 1131 N, 4 columns `not pressing` or `dock failed`), run 2 got coverage 1.0 with a 1100 N peak; at 2 /s 4 of 4 runs
  have coverage 1.0, peaks 31 to 59 N, RMS error about 3 N, but only 24 to 34 % of the contact samples in the force band (the older 0.85 was at 0.1 /s). It
  defaults to 2.0 now too (`GZ_POSITION_GAIN`). Not verified: other profiles, the spray pass (it keeps the plugin default, its 27 s seed move was
  built for the slow loop), a tall wall, a loaded host beyond the load average of 19 seen during 2 of the gain-2 runs. A better fix would be a settle check that
  includes the joints, or a properly tuned joint model; neither is done.

- **Run after #157 (2026-11-21):** no `Start state out of bounds` error at all (0 of the 19 earlier ones), so the unclamped path was the Cartesian planner. The pass is still
  failing and was worse in this single run (1 / 9; the earlier run was 6 / 4, so the pass varies a lot between runs). What the log shows: the first dock approach on stand 0
  ends in one IK branch (`q2 = -0.62`), the next attempt jumps to another (`q2 = +2.49`, shoulder folded back) and every column after it fails `dock failed` from that
  pose, with the wrist joint winding down to -4.6 rad on the one column that docks (column 9); the stow then fails 36 times (`OMPL plan failed`). That is the IK-branch and
  wrist-winding problem of `docs/development_plan.md` section 4 (`ColumnIkChain` is the first piece; it is not yet in the manager). Not caused by cuRobo (this is the MoveIt pass).

- **Opt-in dock continuity (`dock_ik_continuity`, default off; not built or run):** `manipulation_manager` can choose the dock joint goal among IK solutions from the previous dock, the live
  state and `dock_seed_joint_state` (`core/dock_ik_selector.hpp`, ROS-free): the closest to the previous dock wins (max |dq|, non-lift joints wrapped), a flip of the sign of a
  `dock_ik_branch_joints` joint (default `[1, 3]`, the shoulder pan and the elbow: in the Gazebo log the pan went -0.62 to +2.49 and the elbow +0.87 to -0.94) or a `dock_ik_wrist_joint` (default 6) outside `[dock_ik_wrist_min_rad, dock_ik_wrist_max_rad]` (default +-3.2) is dropped; none left means
  the old seeded IK. Turn it on with `MULTI_STAND_DOCK_IK_CONTINUITY=true` in this script (it works with `dock_ik_seeded: false`). The default joint indices are a guess for the
  lift-first UR10e group, check them against the `q2` of the log before relying on it. Gtested with a fake IK only; whether it fixes the pass is not measured.
- **Opt-in recovery on the same branch (`recovery_keeps_branch`, default off; not built or run):** the rerun showed `approach_short`, then a jump to another branch. `recoverArm` jogged toward `last_good_state_` of any branch and the next attempt used a planner-sampled pose goal. With the flag the jog goes only toward a configuration on the live branch (last good, else the last docked one, else the arm stays) and the attempts after a short approach keep a joint goal from `selectDockIk` with the live state as reference (`core/recovery_branch.hpp`, 6 gtests with fake IK, host g++). Uses the `dock_ik_*` branch and wrist limits. Whether it fixes the 9 failed columns is not measured.

### Log lines that show it

- Node container (`run_curobo_node.sh logs`): `cuRobo ready: 7 joints, tool ['tcp_link']`. If instead
  `cuRobo planner unavailable: ...` appears, the services answer `unavailable`.
- `manager.log` at start-up: `planning backends from <path>/planning_curobo.yaml: cartesian=moveit_cartesian free_space=curobo ik=moveit_ik collision=moveit_collision`
  (`manipulation_manager_node.cpp`, `buildPlanners`). A bad selection stops the node with
  `planning backend '<plugin>' (<role>) failed: [<code>] <message> (config <path>)`.
- Per segment: `segment_exec id=.. label=.. outcome=.. mode=.. planner=.. ...` (`MotionExecReporter`). **`planner=` is not
  proof of cuRobo**: `MotionPlanner` fills it with the legacy planner name (`ompl`) whatever plugin did the work
  (`motion_planner.cpp`, `planner_used = planner_for_segment`), so a cuRobo transition reads `planner=ompl` too.
  The same line now carries `fsp_backend=<name>` (`Planner::name()` of the free-space plugin that produced part of the segment: `curobo`, `moveit`, or `-` for a pure Cartesian segment); that field is the direct evidence. Authored and unit-tested, not yet seen in a sim run.
- Evidence that cuRobo planned: `nvidia-smi` shows the node's process and its utilisation rises during transitions; the
  trajectory of a transition should look like cuRobo's (dense, 0.025 s interpolation; not checked); and the negative control: stop the node
  (`run_curobo_node.sh stop`) and re-run: transitions the Cartesian attempt cannot do fail with
  `cuRobo planner service is not available` (`failure_reason_code: unavailable`, within `wait_for_service_sec` = 5 s each).
  The node logs nothing per call, so there is no call counter; if one is wanted, a log line in `ros_node.py` is a one-line change.

### Checklist for the run

- [ ] `cuRobo ready` in the node log; six services under `/curobo_planner` (`ros2 service list`) seen from the sim container.
- [ ] Manager start-up line names `free_space=curobo`; no `[unavailable]`/`[unknown_plugin]` error.
- [ ] Contact segments (`press`, `stroke`, `retract` labels) are `mode=cartesian`, completion as in the plain-MoveIt run.
- [ ] Transition segments (`mode=fsp`) succeed; compare time and path against the MoveIt run (same seed and scene).
- [ ] How often cuRobo was actually reached: transitions are tried as Cartesian first, so only those the Cartesian attempt
      falls short on (and jump or collision bridges inside contact segments) go to cuRobo. Expect few; if none, the
      pass proves nothing about cuRobo. The negative control above shows which ones.
- [ ] With the node stopped: those segments fail as `unavailable`, the manager does not hang (each call waits at most 5 s).
- [ ] Score of the pass unchanged (coverage, force) against the MoveIt-only run.
- [ ] Joint order: a plan from cuRobo has 7 positions in `lift_joint, arm_shoulder_pan_joint .. arm_wrist_3_joint` order.

### Risks found by reading the code

- **cuRobo plans in an empty world.** The scene plugin is not selected and the manager does not feed the `ScenePlugin`
  (`docs/backend_selection.md`, Limitations); nothing calls `setWorld` / `updateWorldGeometry` of the client. The wall in
  MoveIt's octomap and any later scene update are invisible to cuRobo, which also checks only against its own robot
  spheres (self-collision). A cuRobo transition can therefore pass through the wall. Keep transitions short and in free air,
  or feed the world by hand (`ros2 service call /curobo_planner/set_world ...`) before the run. Syncing it with the manager's
  scene updates needs a manager change (another PR) and is not authored here.
- **Frame.** The node works in the robot base frame of the generated URDF (`base_link`, `lift_arm` chain); pose goals are
  sent as given by the manager, whose `frame_id` is `base_footprint`. The client only checks a world update's frame when
  `Options::base_frame` is set (it is not set from the YAML), and rejects a differing frame (test `RejectsAWorldInAnotherFrame`).
  Whether `base_footprint` and `base_link` coincide for this profile was not checked; joint goals are not affected, pose goals are.
- **One planner per process.** One cuRobo node, one planner; the manager's plugin shares one client. Do not start a second
  node or run the GPU tests/benchmarks at the same time on a 4 GB card.
- **First-call compile.** CUDA/Warp kernels compile in the node's start-up warm-up (about 40 to 60 s cold, shorter with the
  `~/.cache/motion-planner-curobo` cache), before the services exist; `run_curobo_node.sh` waits for it. A plan call is not
  expected to pay it, but a first plan with new shapes may be slower than later ones; `timeout_sec` is 60 s against the
  manager's own segment timeouts (not checked against them).
- **Cartesian-first.** `free_space` keeps "Cartesian first, free-space planner on shortfall", and `curobo` also serves as the
  bridge inside contact segments; "contact seams stay on Cartesian" holds for the seam itself, not for a bridge around a jump.
- **Plan-only pose goals.** cuRobo takes a joint goal or exactly one pose goal and no Cartesian path; the lift is in its
  joint set (limits 0 to 1.5 m, lift velocity from the profile is not in the config: time scaling not checked).
- **Same network, two containers.** Mismatched `ROS_DOMAIN_ID`, RMW or SHM settings show up as `service is not available`.
- Not verified: everything above was read from source; nothing was built or run.

## Contact-force discrepancy (Gazebo 1240 to 1820 N scored, 40 to 70 N feedback, MuJoCo 22 N): hypotheses, not a diagnosis

Found by reading the code and config only. Nothing was built or run for this section (no colcon, no Docker, no Gazebo): the cause is **not found**;
the ranked list says what each suspect rests on and which run would confirm or kill it. Two traces are compared and they are not the same signal:

| | feedback peak (`peak_force_n`) | scored trace (`run.csv` -> `score_run`) |
|---|---|---|
| source | `/ft_conditioner/wrench_conditioned` (the gz `force_torque` sensor on the flange joint, payload removed), `|F|` of all three axes | `/ft_sensor/ground_truth_wrench`: the gz `contact` sensor on `tool_tip_link_collision`, `|Fz|` along the tool |
| window | only while `tracking_peak_` is set: the press and stroke segments of a column (`manipulation_manager_node.cpp`, `tracking_peak_ = true` before `_contact`, false after) | the whole run: dock, approach, retract, stow, and every stand change |
| sampling | 200 Hz sensor, latest sample at each message | 200 Hz contact sensor on a 1 kHz physics step: one physics step's contact force per sample, no averaging |

Ranked hypotheses:

1. **Different windows (the scored trace includes motion the feedback never watches).** The recorder keeps every ground-truth sample, the manager's peak ignores dock,
   approach, retract and stow. The same pass logs `dock failed` columns, IK branch flips and a wrist winding to -4.6 rad; a tool driven into the wall (or an obstacle) outside a
   contact segment is in the trace and not in the feedback. MuJoCo passes finish every column and never leave the contact segments in contact. Evidence: the code above, and
   the large scored peaks (583.9, 1240, 1820 N) all coming from runs that failed columns; the run with a 39.7 N feedback peak has no scored figure in this document. Does not by itself
   explain a 20 to 40 fold ratio unless the tool really is pushed past its travel.
2. **The tool is pushed past the compliant range and the contact is rigid there.** The tip is 5 N/mm over 20 mm, then 50 N/mm over 5 mm, then a joint limit (rigid). The wall,
   obstacles and floor are rigid primitives. docs/sim_gazebo.md measured 106 N quasi-static at 35 mm and "contact impacts of 1.2 to 1.8 kN for a sample" on this model; the spike
   is the constraint solver resolving a joint-limit plus contact impact inside one 1 ms step against a position-servoed arm (large effective mass, no compliance, joint effort limits
   not enforced in `gz_ros2_control`). The F/T sensor behind it reports the same load but conditioned and at a different instant, so it can read lower. Evidence: `gazebo_tags.xacro`
   (`stop_travel` 5 mm then rigid limit; damping 100 and 500 N s/m), the Limits section.
3. **Contact solver parameters are not what the SDF reads like.** The world sets only `mu`/`mu2` (and, since this change, explicit SDF-default `kp 1e12`, `kd 1`, `max_vel 0.01`,
   `min_depth 0`, restitution 0). gz-sim 8 uses dartsim by default (`<physics type="ignored">`, no engine chosen in `sim.launch.py`: `gz sim -r -s <world>`); to our knowledge (unverified) it
   solves contacts as hard LCP constraints and does not read `kp/kd/max_vel/min_depth`, so there is no soft-contact tuning knob in this world, and a contact force is whatever impulse
   the solver needs to stop the penetration within a step. MuJoCo uses a soft contact model (solref/solimp), which is the plausible reason it stays near 22 N.
4. **Step size and sampling of an impulsive force.** 1 ms step, force sampled for one step. The force of a hard impact is impulse / dt; a shorter step raises the peak, a longer
   one lowers it, and a 5 ms controller period (200 Hz) cannot see it. A peak that is one sample wide and absent in the F/T sensor is this, not a sustained load.
5. **Counting: several contact points summed, or points reported twice.** `contact_ground_truth.py` adds `body_N_wrench.force` over every point of every pair that involves the tool.
   A cylinder face on a box gives several points; if gz reports the whole-pair force on each point (instead of its share), the sum is N times too large. Evidence for: the docs'
   rigid-tool figure was 108 N for any penetration of 0.05 mm or more, i.e. independent of depth, as a constant per-point impulse would be; evidence against: none either way, the
   per-point values were never logged. This change makes it visible (see below).
6. **Other collisions on the tool link.** The sensor is on `tool_tip_link_collision` only, so the ground and other links are included only if they touch the tip: the tip against the
   obstacle boxes (the 0.04 m radius edge on a 0.1 m wall-front box), the floor (stow near the floor is not expected at z 1.0 to 1.3), or a robot link if self-collision were on (not
   checked; `self_collide` is not set). The include/exclude filter below and `contact_summary`'s `others` (not published) are there to rule it in or out.
7. **Frame and sign.** `abs(force.z)` in `tcp_link` throws away x and y; this can only make the trace lower than `|F|`, so it does not explain a larger value. Not a suspect for the size.

What this change adds (all opt-in or evaluation-only; the default sum is unchanged):

- `sim_gazebo/contact_summary.py` (ROS-free, unit-tested): the tool wrench sum with per-message statistics.
- `contact_ground_truth.py` publishes `/ft_sensor/ground_truth_info` (`[pairs, points, filtered_pairs, sum_point_norm, max_point_norm, duplicate_points]`) and takes
  `include_other` / `exclude_other` (substrings of the other collision's name; empty = every pair). Example: `ros2 run sim_gazebo contact_ground_truth.py --ros-args -p include_other:="['wall']"`
  (the launch starts the node itself: pass the parameter in `sim.launch.py` to change it for a pass).
- `record_run.py` writes `PATH.contacts.csv` beside `PATH` (`t,force_norm,pairs,points,sum_point_norm,max_point_norm,dup_points,others`), same time base as `run.csv`. `others` is the name of the collision the tool touches (several joined with `|`), from `/ft_sensor/ground_truth_others` (String); it says what was hit when a spike shows up.
- The ground and the generated wall carry an explicit `<surface>` (friction, restitution 0, contact ode with the SDF defaults), commented as believed ignored by dartsim; a test
  (`test_wall_world.py`) checks every collision of both worlds has positive friction and contact parameters, a step of at most 1 ms and the Contact system.

What to run to confirm or reject each (all in the `sim-gazebo` image; none of these has been run):

| hypothesis | run | observation that confirms it | observation that rejects it |
|---|---|---|---|
| 1 windows | a normal `run_multi_stand_pass.sh`; join `run.csv` with the manager's state/segment log by time (`segment_exec` lines carry labels) | the samples above 100 N fall in dock, approach, retract or stow segments | the largest samples are inside `press`/`stroke` segments |
| 2 past the travel | `ros2 topic echo /joint_states` during a pass and read `tool_tip_joint` and `tool_tip_stop_joint` (whether the broadcaster lists these uncontrolled joints was not checked; if not, `gz topic -e -t /world/<world>/model/<robot>/joint_state` does) | at the force peak `tool_tip_joint` is at -0.020 and `tool_tip_stop_joint` at about -0.005 (both limits) | the peak occurs with the tip joints mid-travel |
| 3 solver parameters | `gz sim -r -s --physics-engine gz-physics-bullet-plugin <world>` (or `gz-physics-bullet-featherstone-plugin` if installed) with the same pass, then again with explicit large `kp` and `kd` in the SDF | peak force changes by a large factor when the engine or `kp/kd` changes | identical trace for both engines and for different kp/kd (dartsim ignores them, as believed, and the engine is not the cause) |
| 4 step size | `max_step_size` 0.0005 and 0.002 in the world (`wall_world_gen.py` `step_size`, and `empty.sdf`), the same press script `contact_control_test.py` | the peak scales roughly with 1/step | the peak does not move with the step |
| 5 counting | the pass with the new recorder; look at `PATH.contacts.csv` | `points` > 1 with `max_point_norm` ~ `force_norm` and `dup_points` > 0, or `sum_point_norm` several times `force_norm` | `points` is 1 to 2, `force_norm` ~ `max_point_norm` at the peaks |
| 6 other collisions | the same pass with `include_other:=['wall']`, and again with `exclude_other:=['wall']` | the peak survives only without the wall filter, and `others` names something else (obstacle, ground, a robot link) | the filtered peak equals the unfiltered one |
| (sanity) quasi-static | `contact_control_test.py` as in the table above | 8.0 N at 5 mm, 105.6 N at 35 mm (unchanged by this change) | a different value means the SDF change altered the contact |

`others` (the names of the collisions summed) is returned by `contact_summary.tool_wrench_world` but is not published; to read it, call it from a short script on `/tool_contact/contacts`,
or run the include/exclude pair above. The cheapest first step is hypothesis 1 and 5: both need only a normal pass and the new `PATH.contacts.csv`.

## Limits

- Contact is compliant only through the tool tip spring (5 N/mm, 20 mm travel) and the bump stage behind it (50 N/mm, 5 mm). Past the travel the tool is
  rigid again; the quasi-static force is about 106 N at 35 mm past the wall, the contact impacts in a manager pass reach 1.2 to 1.8 kN for a sample. It settles 0.3 to 1.6 N below the target depending on the reference depth (9.69 N with the
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


### Two-stand pass: the obstacle in the world, and what remains (2026-11-22)

Runs on `fix/gazebo-two-stand` (main plus the recovery option, the dock-continuity option with its seed fix, the contact diagnostics and the lift-stage fix), one run each:

| run | columns done / failed | coverage | scored peak force | wall gap max | notes |
|---|---|---|---|---|---|
| stock wall with its obstacle left in the sim world | 0 / 10 | 0.0 | 1230.8 N | 81.7 mm | stand 1 not reached (the arm could not stow) |
| plain wall (no obstacle, as in the MuJoCo pass) | 2 / 8 | 0.24 | 18.1 N (feedback 23.5 N) | 6.8 mm | both stands reached; 0 start-state bound errors |

What the contact sidecar (`run.csv.contacts.csv`, 1.33 million samples over 1700 s) showed in the first run: no contact until sim time 76.6 s, then a 1400 N impact at 2 points, afterwards a constant 3 to 6 N for the remaining 1600 s (the tool stuck against something), 1.5 % of the samples above 100 N, per-sample point counts of 1 to 3 and a sum of per-point norms equal to the reported force within 1 % (so the points are not double counted). The Gazebo script kept the stock wall's obstacle in the simulated world (moved along the wall) while the planner's scene never contains it, and the MuJoCo pass uses the plain scenario: the arm drove into an obstacle the planner did not know about. The script now builds a plain wall (`MULTI_STAND_OBSTACLES=1` keeps the obstacle). This confirms hypothesis 6 of the contact analysis above (a collision other than the tool-wall pair) and rejects hypothesis 5 (counting); the 1400 N spike is a hard impact.

Still failing in the plain-wall run (8 of 10 columns): `MoveGroupInterface::plan() failed or timeout` 23 times with 62 `Failed to fetch current robot state` and 107 `No state update received within 100ms` from the current state monitor (the joint states reach MoveIt late or not as a full set; a possible link to the new state-only stage joints is not checked); 16 times `docked N m off the standoff pose, not pressing from there` (the Cartesian dock path ends short); collision truncation at `arm_forearm_link <-> arm_wrist_{2,3}_link`. The dock continuity selector rejected the first dock of each stand (its reference is the stow pose, a different branch by construction) and fell back to the seeded IK; the recovery option refused its jog ("too far") in some cases. Not run yet: the same pass with PR #184 (tool pad in the planner model, a contact exclusion that works).

### Two-stand plain wall: what the run log showed, and the fixes (2026-11-22)

Read from the plain-wall run's `manager.log` and `sim_launch.log` (10 columns, 2 of 10 done, peak 18 N):

- **The current-state-monitor timeouts and the plan failures behind them.** Every `Didn't receive robot state ... latest received state has time 0.000000`
  (62 of them, never any other value) is the `MoveGroupInterface`'s own state monitor on the backend node `motion_planner_backend`, which nothing spins:
  it never receives a `/joint_states` message. `move_group` then planned from its own state, and 20 of the 62 plan requests were refused by the
  `CheckStartStateBounds` adapter (`Joint 'lift_joint' ... outside bounds by -2.2e-14`, the lift read at its lower stop). The planner clamps the start
  into the joint bounds, but only on the path that got a state from that monitor. `OmplPlanner` now reads the live state from the planning scene monitor
  first (the one `CartesianPlanner` already uses, which has its own spinner), clamps it, and falls back to the `MoveGroupInterface` state only without
  it. The joint states themselves were never starved (`/joint_states` reaches `wait_arm_settled.py` and the manager's own subscription).
- **Docks ending off the standoff.** All 16 `docked N m off the standoff pose` warnings read 0.027 to 0.108 m with 0.0 to 0.7 deg, after a dock segment
  of 7 trajectory points over 0.52 s. The tool track in `run.csv` still moves 6 cm in the second after the check (x 0.87 to 0.94 m between 167.0 and
  168.0 s, check at about 167 s), so the arm trails its trajectory by more than the fixed 0.5 s the check waits. New manager parameter
  `tcp_settle_timeout_s` (default 0 = the old fixed wait; the Gazebo pass sets 4 s, `MULTI_STAND_TCP_SETTLE`): the approach and dock checks then wait
  until the tool position changes by less than `tcp_settle_tolerance_m` (2 mm) over 0.4 s, at most that long, before comparing.
  **Premature "settled" (gazebo-tall-fixed3, stand 0, 10 columns, 2 of 10 done).** Under `jtac_position_only` the joint loop is first order with a 10 s time
  constant, so a tool 5 cm from its goal moves only about 2 mm in 0.4 s: the 2 mm check declared it settled ("tool settled after 4.4 s") while it was still
  converging, and every dock was then judged 4 to 6 cm off the standoff (limit 2 cm) with the lift at 0 and shoulder/elbow 0.17 to 0.19 rad short of the MuJoCo
  run's joints. `MULTI_STAND_TCP_SETTLE_TOL` (`tcp_settle_tolerance_m`, default 0.002 unchanged) with `MULTI_STAND_TCP_SETTLE=60` waits for the arrival
  (0.5 mm per 0.4 s corresponds to about 1.2 cm left). Not run yet.
- **Forearm and wrist collision.** In the first rerun (the two fixes above only) every plan failed at the first dock: its joint goal (wrist 1 at -4.25 rad)
  had `arm_forearm_link` against `arm_wrist_2_link` in contact ("Unable to sample any valid states for goal tree"), and after a drive to another branch
  79 of 140 plan requests were refused by `CheckStartStateCollision` with the same pair (51 more with `arm_upper_arm_link` against `lift_carriage_link`),
  64 Cartesian truncations at that pair. The UR matrix (`srdf/acm_ur*.xacro`, from `scripts/generate_acm.py`, `arm_never_pairs`) now lists the forearm
  against wrist 2 and wrist 3 as `Never`: the wrist 1 link separates them on the real arm, and the description's cylinders overlap at angles the arm reaches.
  Not changed: `arm_upper_arm_link` against `lift_carriage_link` (a pair that can touch for real).

| run (plain wall, two stands, 10 columns, same script and options) | columns done / failed | coverage | scored peak | feedback peak | in band | result |
|---|---|---|---|---|---|---|
| before (`fix/gazebo-two-stand` tree) | 2 / 8 | 0.24 | 18.1 N | 23.5 N | n/a | fails |
| start state from the scene monitor + tool settle (no matrix change) | 0 / 10 | 0.0 | none | none | n/a | fails (start and goal states in forearm/wrist contact) |
| + forearm/wrist `Never` pairs | 10 / 0 (columns 3 and 4 retried once after docking 0.07 and 0.04 m off) | 1.000 (2790 of 2790 cells) | 872.5 N | 33.5 N | 0.876 | scorer passes (RMS error 9.7 N, wall gap max 18.9 mm) |

One run each. The 872.5 N is a single episode at sim time 399.9 to 401.2 s (32 samples above 350 N, 774 above 100 N of 282663), on stand 1 (y 0.48 to 0.59 m,
z 1.18 to 1.27 m); the feedback peak of the pass was 33.5 N. Not investigated, and one run does not show the pass is stable.

### Queued visible runs (maintainer tooling)

`docker/tools/gazebo-infra/` holds the wrappers used to run the two-stand pass visibly through a one-at-a-time job queue:
`gz_queue.sh <prio> <name> <worktree> [VAR=val ...]` writes a job, `gz_run.sh` builds in the `sim-gazebo` image (shared ccache, per-lane
build volumes), waits for the host's memory slots, runs `run_multi_stand_pass.sh` with `gui:=true` and the `MULTI_STAND_*` knobs from the
environment, and writes `<name>_out`. The host layout is set by `MP_INFRA` (default `$HOME/mp-infra`: `cp/` for outputs, `jobs/`, `cctool`), `MP_REPO` (default `$HOME/Motion-Planner`, for `docker/tools/hostwatch`) and `SIMQ_ADD` (the queue's add script); `gz_in.sh` is the in-container driver the run script mounts from `$MP_INFRA/cp`. Copy and adapt.

The sim container runs as `--user 1000:1000` with `--gpus all`, `NVIDIA_DRIVER_CAPABILITIES=all`, `__GLX_VENDOR_LIBRARY_NAME=nvidia` and
`__NV_PRIME_RENDER_OFFLOAD=1`, so the Gazebo GUI renders on the NVIDIA GPU of a hybrid-graphics laptop. `MULTI_STAND_TCP_SETTLE_TOL` (default 0.002 m) is the arm-arrival tolerance next to
`MULTI_STAND_TCP_SETTLE`: with the position loop's 10 s time constant a larger settle time and a tighter tolerance make the dock check
wait for the arm instead of rejecting a pose it is still approaching.

### Which phase the Gazebo force spikes are in (gain 2, one wall, 2026-11-26)

`w1-gz-onewall-gain2` (plain wall, position gain 2, 10/10 columns): 36 samples above 500 N, peak 4175 N. The contact points confirm a real impact (at the peak: 1 pair, 4 points, largest point 3320 N, sum 6529 N), and the tool did not move during the burst. Sim time was aligned to the manager's segment log through the `docked_wall_column_4` pose (tcp 0.998 0.748 1.009 at run.csv t=79.5). Samples above 300 N by phase: `dock` of column 4 (32 samples, the 4175 N peak: the dock move brings the tool to the wall at about 0.2 m/s and the tip arrives at x 0.993 against a wall at 0.98), `approach` of column 4 (105 samples, peak 1007 N at z 2.1) and `approach` of column 0 (475 samples, peak 756 N). Forces during an approach are a contact in free space, so something other than the wall is touched there; the new `others` column names it in the next run. Not yet explained: why the approach of column 0 touches anything.

