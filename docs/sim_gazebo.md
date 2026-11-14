# Gazebo simulation backend (`sim_gazebo`)

Gazebo Harmonic backend for the mobile manipulator (base + lift + arm + tool), selected with
`hardware:=gazebo` in the description. Package: [`src/simulation/sim_gazebo`](../src/simulation/sim_gazebo).
Everything runs headless (`gz sim -s`, no GPU, no display), so it works in a container and in CI.

## Run

```bash
docker build -f docker/Dockerfile --target sim-gazebo -t motion-planner:sim-gazebo .
# inside the image, workspace built (mobile_manipulator_description, sim_gazebo, and for the
# compliant controller compliant_trajectory_controller + ft_bridge_hardware):
ros2 launch sim_gazebo sim.launch.py                                          # headless, joint_trajectory_controller
ros2 launch sim_gazebo sim.launch.py wall_scene:=<wall_scene.json> wall_id:=0 standoff:=0.9
ros2 launch sim_gazebo sim.launch.py controller:=jtac                         # compliant JTAC controller
ros2 launch sim_gazebo sim.launch.py profile:=ur10e_telescopic gui:=true      # gz GUI needs a display
ros2 run sim_gazebo smoke_test.py [--controller jtc|jtac]                     # headless smoke test
ros2 run sim_gazebo wall_world_gen.py scene.json -o wall.sdf --standoff 0.9   # world only
```

Launch arguments: `profile` (name in `config/profiles` or a path; its `robot.xacro_args`, joint
names, frames and home pose drive everything), `world`, `wall_scene`/`wall_id`/`standoff`, `gui`,
`controller` (`jtc` | `jtac` | `none`). Several containers on one Docker bridge share DDS
multicast: give each simulation its own `ROS_DOMAIN_ID` (or `ROS_LOCALHOST_ONLY=1`), otherwise
another stack's `/robot_description` can reach the gz controller manager.

What starts: gz server, `robot_state_publisher`, the robot spawned from `hardware:=gazebo`,
`ros_gz_bridge` (`/clock`, raw wrench, contacts), `contact_ground_truth`, and spawners for
`joint_state_broadcaster`, `force_torque_sensor_broadcaster` and the chosen controller. The
controller manager lives inside the gz server (`gz_ros2_control`). `jtc` uses
`joint_trajectory_controller`; `jtac` loads `config/jtac_controller.yaml` of
`compliant_trajectory_controller` with joints and frames replaced from the profile and the sensor
`ft_sensor` in frame `ft_sensor_link`. No wrench axis is selected there, so it tracks like a JTC.

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

## Limits

- Contact is stiff: position control into a wall gives about 200 N for 3.5 cm of commanded
  penetration, so force control tests should use the compliant controller with a selected wrench axis.
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
