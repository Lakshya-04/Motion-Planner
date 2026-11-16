# Simulation backends

The same controllers, planners and coverage code run against a simulator exactly as they do against
hardware: every backend sits behind **ros2_control**, so a backend is a `hardware:=<name>` choice in the
robot description and nothing above it changes. The robot comes from a profile
([robot_description.md](robot_description.md)), the world from the same wall scenes the coverage planner
uses ([scene_model.md](scene_model.md)).

| Backend | `hardware:=` | Package | Engine | Status |
|---|---|---|---|---|
| Mock | `mock`, `jtac_mock` | `mobile_manipulator_description` | none (ros2_control mock components, scripted F/T) | working; unit and demo level |
| Gazebo | `gazebo` | `sim_gazebo` | Gazebo Harmonic (gz-sim 8) with `gz_ros2_control` | not on this branch: arrives with the Gazebo PRs |
| MuJoCo | `mujoco` | `sim_mujoco` | MuJoCo 4.x through our own ros2_control `SystemInterface` | not on this branch: arrives with the MuJoCo PRs |
| Real robot | `real` | `mobile_manipulator_description` | vendor driver chosen by arm (xarm_ros2, ur_robot_driver, franka_ros2; not vendored), `real_driver:=mock` for CI | wiring and tests only, nothing run on hardware ([hardware_bringup.md](hardware_bringup.md) section 0a) |
| Isaac Sim | `isaac` | `sim_isaac` | NVIDIA Isaac Sim | **stub**: contract and plan only, nothing runs |

## What every backend provides (the contract)

A backend is done when all of this works headless, from one profile and one wall scene:

1. **Joints.** The actuated joints of the profile (lift when the lift is actuated, then the arm joints) as
   ros2_control interfaces: position and velocity command, position / velocity / effort state; names and
   order from the profile, not hard-coded.
2. **Clock.** `/clock`, with the controllers and nodes on `use_sim_time`.
3. **Tool wrench.** The tool force / torque at `tcp_link` in the shape `ft_bridge_hardware` and the JTAC
   compliant controller consume, with the sign convention checked against a wall push.
4. **World.** A wall (plane plus obstacles) generated from `wall_scene.json` / a `SceneModel`, with
   contact that is stiff enough to push against and soft enough not to explode.
5. **Base.** The wheels, steering and casters of the chosen base are fixed links, so the base is driven
   by `cmd_vel` and reports `odom`; the planning chain (lift + arm) is the same for every base.
   Backends may start with a static base and add base motion afterwards.
6. **Headless.** Runs without a display or GPU rendering, so it can be a test in a container.

## Selecting one

```bash
xacro urdf/mobile_manipulator.urdf.xacro arm:=ur10e base:=ranger_mini lift:=telescopic hardware:=jtac_mock
```

On this branch only `mock` and `jtac_mock` exist; any other `hardware:=` value (`gazebo`, `mujoco`,
`isaac`) fails at render time with "unknown hardware". Each backend PR adds its value and its launch file,
which does the above from a profile (`scripts/profile_xacro_args.py`) and starts the controllers with the
same configs the mock demos use, so a controller change is tested on every backend.

## Layout

```
src/simulation/
  sim_gazebo/   launch, worlds, bridge config, wall-world generator, smoke test   (planned: Gazebo PRs)
  sim_mujoco/   ros2_control SystemInterface on MuJoCo 4, MJCF + wall generators   (planned: MuJoCo PRs)
  sim_isaac/    stub: launch that says it is not implemented, README with the plan   (this branch)
```

Only `sim_isaac` exists on this branch. The `sim_gazebo` and `sim_mujoco` packages and their pages
(`docs/sim_gazebo.md`, `docs/sim_mujoco.md`) arrive with their own PRs.

Docker: the `sim` target holds MuJoCo 4 (`mujoco-core`, MJX, JAX) for Python work
([docker.md](docker.md)); a thin Gazebo target arrives with the Gazebo PRs. Isaac Sim is large and ships as its own NVIDIA
image: it will run as a separate compose service, not inside ours.

## Isaac Sim (stub, to be developed later)

Planned shape, so the stub is a real placeholder and not a blank: the robot URDF is imported to USD (the
Isaac Sim URDF importer, collision as convex hulls like the other backends); joints are driven through
ros2_control with a topic-based hardware interface that talks to the Isaac Sim ROS 2 bridge
(`topic_based_ros2_control`), which keeps `hardware:=isaac` identical to the others from the controllers'
side; the wall world is built from the same scene file; the tool wrench comes from a contact sensor or the
articulation's joint forces. Open questions to settle first: the Isaac Sim version and licence terms for the
deployment, how the F/T sign convention maps, and whether the GPU on the target machine (RTX 3050 4 GB
here) is enough: Isaac Sim needs far more, so this backend is for a bigger workstation or a cloud runner.

`sim_isaac` today contains a launch file that exits with a clear message and a test that pins that
behaviour, so nobody mistakes it for a working backend. Replacing it means implementing the contract
above; the checklist in `src/simulation/sim_isaac/README.md` tracks it.
