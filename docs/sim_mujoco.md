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

## Build

The SDK is not vendored. Download the tarball of the release and point `MUJOCO_ROOT` at it:

```bash
curl -L -o mujoco.tgz https://github.com/google-deepmind/mujoco/releases/download/4.0.1/mujoco-4.0.1-linux-x86_64.tar.gz
mkdir -p /opt/mujoco-sdk && tar xzf mujoco.tgz -C /opt/mujoco-sdk       # -> /opt/mujoco-sdk/mujoco-4.0.1
export MUJOCO_ROOT=/opt/mujoco-sdk/mujoco-4.0.1
colcon build --packages-select sim_mujoco mobile_manipulator_description
```

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
`jtac_controller.yaml`, overriding only the joint names, the kinematics chain and the F/T sensor
name from the profile. Joint names always come from the profile.

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
it passes the reference through) the same run passes except the retreat check (13.8 N residual after
1.5 s). The poses were solved with a damped-least-squares IK on the generated model; they are
UR10e-specific.

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
