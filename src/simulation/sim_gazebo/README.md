# sim_gazebo

Gazebo Harmonic simulation backend (`hardware:=gazebo`): `gz_ros2_control` tags with an F/T sensor at the tool, launch files driven by the description profile YAML, a wall world generator fed by `wall_scene.json` or a `SceneModel` YAML, and a headless smoke test.

## Tier

Simulator (launch files, Python generators and scripts; no compiled code of its own).

## Inputs and outputs

- Launch `launch/sim.launch.py`; arguments include `profile`, `world`, `wall_scene` / `wall_id` / `standoff`, `gui`, `controller` (`jtc`, `jtac`, `none`), `ft_conditioner`, `base_motion`, `rgb_camera`, `depth_sensor`, `arm_meshes`. It starts the gz server, `robot_state_publisher`, the spawned robot, `ros_gz_bridge` and the controller spawners; the controller manager lives inside the gz server.
- Topics: `/clock`, the raw wrench (`/ft_sensor/wrench_gz`) and `/force_torque_sensor_broadcaster/wrench` (frame `ft_sensor_link`), ground-truth contact wrench `/ft_sensor/ground_truth_wrench`, `/depth_sensor/points` and `/rgb_camera/image` when enabled; with `base_motion` it takes `/cmd_vel` and publishes `/odom` and TF.
- Config: `config/controllers.yaml`, `urdf/gazebo_tags.xacro`, `worlds/empty.sdf`.
- Python: `sim_gazebo.wall_world`, `sim_gazebo.profile`, `sim_gazebo.fk`; scripts `wall_world_gen.py`, `smoke_test.py`, `contact_control_test.py`, and `run_manager_pass.sh`, `run_spray_pass.sh`, `run_multi_stand_pass.sh`.

## Dependencies

- In this repo: `mobile_manipulator_description`; the scripts also use `mobile_manipulator_moveit_config`, `coverage_planner_msgs` and `coverage_task_msgs`, and the passes need `compliant_trajectory_controller`, `ft_bridge_hardware`, `manipulation_manager` and the rest of the stack.
- External: Gazebo Harmonic, `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`, `controller_manager`, `force_torque_sensor_broadcaster`, `xacro`.
- Needs ROS 2 and Gazebo; runs headless (server only), no GPU needed. It is the simulator.

## Use it on its own

```bash
ros2 launch sim_gazebo sim.launch.py
ros2 run sim_gazebo smoke_test.py
ros2 run sim_gazebo wall_world_gen.py scene.json -o wall.sdf --standoff 0.9
```

The `sim-gazebo` Docker image holds the system packages; see the docker doc.

## Tests

19 test cases in 3 Python files (wall world, contact script, the Gazebo description), counted from the sources; `colcon test` registers the wall world and description files. The smoke and contact scripts need a running simulator.

```bash
colcon test --packages-select sim_gazebo
```

## Status and limits

- Passes run in simulation only; the parity with the MuJoCo passes is described in the Gazebo doc. Nothing here has run on hardware.
- Several simulations on one Docker bridge share DDS multicast: give each its own `ROS_DOMAIN_ID`.
- The depth and colour cameras are rendered in software and are off by default.

## Related docs

- [sim_gazebo.md](../../../docs/sim_gazebo.md)
- [simulation.md](../../../docs/simulation.md)
- [sim_vs_real.md](../../../docs/sim_vs_real.md)
- [docker.md](../../../docs/docker.md)
