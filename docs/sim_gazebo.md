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
```

Launch arguments: `profile` (name in `config/profiles` or a path; its `robot.xacro_args`, joint
names, frames and home pose drive everything), `world`, `wall_scene`/`wall_id`/`standoff`, `gui`,
`controller` (`jtc` | `jtac` | `none`), `ft_conditioner` (`true` starts the conditioner and points JTAC at
the sensor `ft_sensor_tcp`), `payload_mass_kg` (initial payload for the conditioner, 0: calibrate),
`ft_filter` (JTAC `ft_sensor.filter_coefficient`). Several containers on one Docker bridge share DDS
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

## Tests

```bash
colcon test --packages-select sim_gazebo     # generator (9) and hardware:=gazebo rendering (3), no gz needed
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
   offset clamp, tool pressed 5 mm and 35 mm past the wall; ground truth about 108 N along the tool, 120 N lateral
   from friction and a contact torque of about 11 Nm about the tcp, so the lever-arm and payload terms are both
   exercised): force error |dF| mean 0.001 N (max 0.002) at 5 mm and 0.005 N (max 0.011) at 35 mm; torque error
   |dT| mean 0.0002 Nm (max 0.0005) and 0.0010 Nm (max 0.003). The checks gate the mean with 1 N and 0.2 Nm; the
   maxima are printed, not checked.
3. **Force control with the tool-z admittance axis** (target 10 N through `~/target_wrench`, K 300 N/m, mass 8,
   damping ratio 6, offset clamp 3 mm, approach 2 mm/s): **not reached.** Ground-truth force over the last 4 s:
   mean 160 N, standard deviation 121 N, peaks 470 to 580 N, in contact 84 percent of the samples; the tool
   bounces about 3 mm off the wall at about 2 Hz. The same happens with gains from K 100 to 300 N/m, mass 8 to 1e5 kg,
   damping up to D = 1e7 Ns/m, approach 1 to 5 mm/s, a 0.02 to 0.05 JTAC low-pass and offset clamps from 3 mm to
   the controller's 0.1 m default (those sweeps are earlier runs: the committed script runs one setting, set by its
   options).
   What the script itself measures: with JTAC and no axis, a position hold 5 mm and 35 mm past the wall gives
   about 108 N at both depths. The smoke test, which uses plain `joint_trajectory_controller`, reads 209 N at
   3.5 cm. So the push is not a property of the wall alone: it differs by a factor of two between the two
   controllers, and in MuJoCo the same JTAC compliance offset cut a 208 N push to 50 N
   ([sim_mujoco.md](sim_mujoco.md)). In free space the admittance converges to F/K (33 mm for K 300 and 10 N,
   by that arithmetic; no overshoot seen).
   **Why 10 N is not held is not established.** A force that stays near 108 N while the penetration grows by a
   factor of seven fits a very stiff contact driven by servo-controlled joints, but it fits a controller-side limit
   (the JTAC offset clamp or the velocity servo) at least as well. An earlier measurement, not recorded and not
   reproduced by the committed script, gave a constant 104 to 108 N for position holds from 0.05 mm to 10 mm past
   the wall and 0 N below that; if that holds, 10 N sits inside the first 0.05 mm and the loop would see either
   10 N (free) or 300 to 500 N impulses (contact). Tracing the penetration sweep and JTAC's `~/debug_offset`
   would separate the explanations; the script does not do it yet. By default the script only reports the hold
   result; `--require-hold` turns it into a failing check.
4. **Release:** target to zero, disengage, retreat to the standoff: ground-truth force 0.000 N after the retreat.

Things to try, depending on the cause: a contact with real compliance (a softer tool tip, e.g. a spring joint in
the tool, or contact parameters of a physics engine that supports them), or joint effort control instead of the
velocity servo. Neither is done, and neither is justified until the cause is established.

## Limits

- Contact is stiff and impulsive: JTAC with no wrench axis gives about 108 N at 5 mm and 35 mm past the wall
  (plain trajectory control read 209 N at 3.5 cm), and the compliant controller with a selected wrench axis
  does not hold 10 N. The cause is not established (see the contact-controlled test above).
- Joint dynamics are idealised: `gz_ros2_control` position/velocity interfaces, no gearbox
  friction, backlash, motor limits (URDF limits are not enforced) or sensor noise, bias and drift.
- The tool and wall are rigid primitives with Coulomb friction; no compliance of the tool, no
  wall material model, no wet or sticky effects.
- The gz sensor measures rigid-body joint loads including dynamic terms, so the check above is
  exact only quasi-statically.
- Base motion is not simulated (static base); lift mimic stages are visual only; collision uses the
  description's primitives.
- Gazebo needs the `sim-gazebo` image; it is not a rosdep dependency of the package, so the
  common `deps` image stays small.
