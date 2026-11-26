# sim_isaac

Optional NVIDIA Isaac Sim add-on. Not a backend yet: there is no `hardware:=isaac`, and `ros2 launch sim_isaac isaac.launch.py`
is still the stub that exits with a message. What exists is the plumbing: Isaac Sim runs in its own container next to
the ROS image and talks to it over Isaac's ROS 2 bridge.

Full description, measured numbers and the not-done list: [docs/sim_isaac.md](../../../docs/sim_isaac.md).

## Run the smoke test

```bash
src/simulation/sim_isaac/scripts/run_smoke.sh
```

Needs the NVIDIA container runtime, `nvcr.io/nvidia/isaac-sim:5.1.0` and `motion-planner:sim-gazebo` already pulled or built.
The script sets `ACCEPT_EULA=Y` (you accepting NVIDIA's licence) and `PRIVACY_CONSENT=N`.

## Scripts

- `run_isaac.py`: headless runner (robot USD, ground plane, bridge graph with `/clock`, `/joint_states`, `/tf`, commands on `/isaac/joint_command`).
- `urdf_to_usd.py`: URDF to USD with drives and a repaired lift mimic. `make_resolved_urdf.py`: profile to URDF with meshes next to it.
- `smoke_test.py`, `run_smoke.sh`: the ROS-side test and the wrapper that starts and stops everything.
- `isaac_common.py`: the pure-Python parts (profile to joint order, graph description, arguments); `test/` covers it without Isaac.

`colcon build --packages-select sim_isaac` installs the scripts and needs no Isaac. `docker/isaac/` and
`docker/compose.isaac.yaml` hold the optional image and compose service.

## Not done

No ros2_control hardware interface, no wall world, no F/T sensor, no cameras, no base motion, no manager pass, not in CI.
