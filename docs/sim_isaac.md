# Isaac Sim add-on (`sim_isaac`)

An optional add-on. Nothing in the default build, the images, CI or the other docs depends on it; if you never
touch it, nothing changes. Package: [`src/simulation/sim_isaac`](../src/simulation/sim_isaac).

First slice only: Isaac Sim runs headless here, loads the UR10e-on-telescopic-lift robot, and talks ROS 2 with a
smoke test that passes. It is not a backend yet (`hardware:=isaac` does not exist).

## What runs where

Isaac Sim cannot go into our ROS image (NVIDIA's image, its own EULA, Python 3.11 and Kit runtime), so it is a sibling
container next to the normal one. Both use host networking and the same `ROS_DOMAIN_ID`, and talk through Isaac's
bundled ROS 2 bridge extension (`isaacsim.ros2.bridge`, Jazzy libraries inside the Isaac image).

```
isaac-sim container (GPU)                       our ROS image (motion-planner:sim-gazebo)
  run_isaac.py: robot USD, ground plane,          smoke_test.py (rclpy)
  OmniGraph with the ROS 2 bridge nodes    <--->  /clock, /joint_states, /isaac/joint_command
```

| Piece | Where | What it does |
|---|---|---|
| `scripts/make_resolved_urdf.py` | ROS image | profile to xacro to a URDF with `package://` meshes copied next to it |
| `scripts/urdf_to_usd.py` | Isaac container | Isaac's URDF importer, position drives, mimic fix (below) |
| `scripts/run_isaac.py` | Isaac container | headless runner with the bridge graph; `--usd`, `--profile`, `--steps` or `--seconds` |
| `scripts/isaac_common.py` | both | profile to joint list and order, graph description, arguments (no Isaac imports) |
| `scripts/smoke_test.py` | ROS image | the checks below |
| `scripts/run_smoke.sh` | host | builds the USD if missing, starts Isaac, runs the test, removes the container |
| `docker/isaac/Dockerfile` | host | `FROM` the Isaac image plus these scripts and the profiles, nothing else |
| `docker/compose.isaac.yaml` | host | service `isaac` under the compose profile `isaac`; nothing else starts it |

The graph publishes `/clock`, `/joint_states` and `/tf`, and subscribes to `sensor_msgs/JointState` position
commands on `/isaac/joint_command` (an Articulation Controller applies them; a command may name any subset of joints).
Joints and their order come from the profile (`joints.lift`, `joints.arm`); `/joint_states` also carries the lift's two
mimic stages, which the smoke test expects and checks.

## Run it

```bash
# Isaac image (15 GB) must already be pulled; the script does not pull. Setting ACCEPT_EULA=Y is you
# accepting NVIDIA's licence, PRIVACY_CONSENT=N declines telemetry.
src/simulation/sim_isaac/scripts/run_smoke.sh            # first run also builds the robot USD
```

Env: `PROFILE`, `ISAAC_IMAGE`, `ROS_IMAGE` (default `motion-planner:sim-gazebo`), `ROS_DOMAIN_ID`, `ISAAC_WORK`
(default `~/.cache/mp-isaac`: robot USD and the Isaac caches; the Isaac image runs as uid 1234, so files in
there are not yours to delete without `sudo` or a container). The ROS side needs `rclpy`, `sensor_msgs` and
`rosgraph_msgs`, which that image has; `colcon build --packages-select sim_isaac` works there without Isaac.

Packaged form: `docker build -f docker/isaac/Dockerfile -t motion-planner:isaac .` then
`docker compose -f docker/compose.isaac.yaml --profile isaac up isaac` (the USD has to exist under
`$ISAAC_WORK/assets/<profile>/`, `run_smoke.sh` leaves it there).

Smoke test output of the run on this machine (full output of the second of two passing runs):

```
[PASS] topics: /clock at 1.99 s, /joint_states with 9 joints
[PASS] joint names and order: profile joints True in order, extra ['lift_stage_1_joint', 'lift_stage_2_joint'] (profile mimic [...])
[PASS] arm holds start pose: max |q - home| = 0.0022, max drift 0.00000 over 3 s
[PASS] mimic stages follow the lift: max |stage - 0.5 * lift| = 0.0001
[PASS] command reaches target: arm_wrist_3_joint = 0.5022 for target 0.5000 after 0.19 s sim time
[PASS] target holds after the command stops: max |q - target| = 0.0000 over 2 s
[PASS] other joints stay put: max |q - home| = 0.0027
SMOKE PASS
```

## Measured (RTX 5080 16 GB, driver 580, Isaac Sim 5.1.0, headless, no display)

| What | Number |
|---|---|
| `SimulationApp(headless=True)` start, empty cache mounts | 11.5 s (whole container 15.3 s, 120 physics steps in 2.2 s) |
| same, second start with the caches filled | 10.1 s and 14.7 s; one run had the physics part take 65 s (host was busy with other containers, not reproduced) |
| `run_smoke.sh` end to end, USD already built | about 27 s |
| GPU memory of the Isaac process | 0.4 GB bare app, 3.2 GB peak with the ROS 2 bridge graph (`nvidia-smi`, sampled every 3 s) |
| `/clock` and `/joint_states` rate | 120 Hz (physics rate; `rendering_dt` equals `physics_dt`) |
| Hold under gravity (no command) | worst joint 0.0022 rad off the profile's home pose, no drift in 3 s |

Start was fast even with empty cache mounts (why: not investigated). Nothing blocked on the network: no asset-server
fetch was needed, the ground plane is made in code, and the robot comes from local files.

## Things that cost time

- **DDS shared memory.** Topics showed up in `ros2 topic list` from the other container but no data arrived. Fast DDS
  picks shared memory between two processes on one host and the two containers do not share `/dev/shm`. Both sides
  use `docker/isaac/fastdds_udp.xml` (UDP only) through `FASTRTPS_DEFAULT_PROFILES_FILE`; with it the data flows.
  Without a profile you would need `--ipc host` on both containers instead.
- **Image entrypoint.** The Isaac image's entrypoint is `runheadless.sh` (a streaming app), so a plain
  `docker run IMAGE ./python.sh x.py` just starts the streaming app and ignores the script. Use `--entrypoint /isaac-sim/python.sh`.
  The container user is uid 1234 with home `/isaac-sim`; the caches live under `/isaac-sim/.cache`, `.local/share/ov/data`,
  `.nv/ComputeCache` and `kit/cache`, not under `/root`.
- **Kit exits without flushing stdout.** The scripts use line-buffered output and `os._exit` on errors.

## Robot asset and the lift's mimic joints

The URDF importer (`isaacsim.asset.importer.urdf`, `parse_mimic=True`) turns the lift's `<mimic>` tags into PhysX mimic
joints, but with two faults for prismatic joints that `urdf_to_usd.py` repairs:

- the stage joints get limits of -0.15 to 0.9 instead of the URDF's 0 to 0.75, so under gravity they sat on the bogus
  lower limit. They are set back to the URDF limits.
- the mimic constraint is soft (25 Hz, damping ratio 0.005) and the stages sagged 4 cm. It is set to 400 Hz and damping
  ratio 1; stages then follow the lift to 0.1 mm.

The lift joint keeps the URDF's 0.2 m/s velocity limit, so a lift command ramps. Drives: position drives on the lift and
arm joints; the arm uses the importer's drive with stiffness 1e5 and damping 1e4, the lift 1e6 and 1e5. The
importer writes acceleration-type drives, so these are per unit mass, not N/m; they are untuned. Collision geometry is
the URDF's collision (UR10e STL meshes and primitives); the resolved URDF has no visuals, so the USD has none either.
Self-collision is off.

## Not done

- No ros2_control hardware interface and no `hardware:=isaac` (the planned route is a topic-based one on this bridge).
- No wall world, no F/T sensor, no cameras or depth sensor, no base `cmd_vel`/odom.
- The manager pass and the other scripted runs of the other backends were not tried on it.
- Not in CI and not in `docker/run_all_checks.sh` (it needs a GPU and the NVIDIA image). No `isaac-smoke` entry in a checks script.
- Drive gains and the mimic frequency are not tuned; joint-limit and effort behaviour under fast commands not examined.
- `/tf` is published but not compared against `robot_state_publisher`. Time is not synchronised with a ROS wall clock; use `/clock`.
- Only the `ur10e_telescopic` profile was converted and run. Other profiles should go through the same scripts but were not tried.
- The Isaac EULA and the deployment licence terms were not reviewed; setting `ACCEPT_EULA=Y` is up to the person running it.
