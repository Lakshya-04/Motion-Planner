# sim_mujoco

MuJoCo 4.x simulation backend: a ros2_control `SystemInterface` (`hardware:=mujoco`) that steps a MuJoCo model, an MJCF generator from the robot URDF and profile, a wall-world generator from the coverage planner's scene files, headless launch files and the scripts that run and score the demo passes.

## Tier

Simulator (ROS 2 plugin, a ROS-free C++ core `MujocoSim`, pure-Python generators, launch files and scripts).

## Inputs and outputs

- Plugin `sim_mujoco/MujocoSystemInterface` (`sim_mujoco.xml`, library `sim_mujoco_hardware`): joint position and velocity commands in; position, velocity and effort state out; the tool force/torque as a sensor (raw wrist wrench, plus ground truth).
- Launch `launch/sim_mujoco.launch.py`; arguments include `profile`, `scene` and `scene_offset`, `controller` (`jtc` default or `jtac`), `viewer`, `conditioner`, `depth_sensor` (+ `depth_sensor_pose`), `base_motion`. With `depth_sensor` it publishes `/depth_sensor/points`; with `base_motion` it takes `/cmd_vel` and publishes `/odom` and TF; the raw wrench is on `/sim_mujoco/ft_raw`.
- Python modules `sim_mujoco.mjcf_generator` (URDF to MJCF, standard library only), `sim_mujoco.wall_world`, `sim_mujoco.sim_noise`; the `generate_mjcf` script.
- Scripts under `scripts/`: `run_smoke.sh`, `run_contact.sh`, `run_wall_pass.sh`, `run_manager_pass.sh`, `run_spray_pass.sh`, `run_multi_stand_pass.sh`, `run_scenarios.sh` and others, each with its Python test client; `render_run.py` and `plot_run.py` for media.

## Dependencies

- In this repo: `mobile_manipulator_description`, `mobile_manipulator_moveit_config`, `ft_bridge_hardware`, `ft_conditioner`, `coverage_planner`, `manipulation_manager`, `motion_planner_moveit`, `reachability`, `stand_navigator`, `wall_detector` and the message packages `coverage_planner_msgs`, `coverage_task_msgs`, `reachability_msgs`, `stand_navigator_msgs` (the scripts drive the whole stack).
- External: ros2_control, `controller_manager`, `joint_trajectory_controller`, MoveIt, and the MuJoCo 4.x SDK, which is not bundled: set `MUJOCO_ROOT` to the extracted SDK. Without it only the Python tools build.
- Needs ROS 2 for the plugin; the generators are plain Python. No GPU. It is the simulator.

## Use it on its own

```bash
ros2 launch sim_mujoco sim_mujoco.launch.py profile:=ur10e_telescopic
bash $(ros2 pkg prefix sim_mujoco)/lib/sim_mujoco/run_smoke.sh
```

## Tests

72 test cases in 10 files (counted from the sources): 21 for the MJCF generator, plus xacro, wall world, scenarios, contact scripts, noise and the C++ core and signal model. The C++ tests need the MuJoCo SDK.

```bash
colcon test --packages-select sim_mujoco
```

## Status and limits

- Passes are scored in simulation only; the numbers in the root README come from these scripts. The simulation does not model everything a real sensor and arm do (see the sim-versus-real doc). Nothing here has run on hardware.
- MuJoCo contacts and the force path are not a proof of behaviour on a real wall.
- The depth sensor is a ray cast against the scene, not the robot.

## Related docs

- [sim_mujoco.md](../../../docs/sim_mujoco.md)
- [simulation.md](../../../docs/simulation.md)
- [sim_vs_real.md](../../../docs/sim_vs_real.md)
- [mujoco_crosscheck.md](../../../docs/mujoco_crosscheck.md)
- [docker.md](../../../docs/docker.md)
