# stand_navigator

Drives the mobile base to a stand chosen by the reachability planner, and servos it to a standoff and yaw against the fitted wall plane. A small stand-in until Nav2 is used; with a wall scene it can plan a collision-free base path around the scene's walls and obstacles.

## Tier

ROS node (Python, `rclpy`) with a ROS-free core (`core.py`, `wall_docking.py`).

## Inputs and outputs

- Node `stand_navigator_node`: action server `navigate_to_stand` (`stand_navigator_msgs/NavigateToStand`); subscribes `odom_topic` (default `odom`); publishes `geometry_msgs/Twist` on `cmd_vel_topic` (default `cmd_vel`). Parameters: `base_profile` (a profile YAML with a `base:` section, required), `tolerance_xy_m`, `tolerance_yaw_rad`, `timeout_s`, `rate_hz`, `odom_timeout_s`, and the planner ones: `use_path_planner`, `wall_scene_json`, `wall_scene_topic`, `planner_resolution_m`, `planner_margin_m`, `planner_lookahead_m`.
- Node `wall_docking_node`: action server `wall_dock` (`WallDock`); also subscribes the wall scene (`wall_scene_topic`, default `/wall_detector/wall_scene`, `std_msgs/String`, transient local). Parameters include `base_profile`, `ref_xy`, `facing_rad`, `tolerance_standoff_m`, `tolerance_yaw_rad`, `max_linear_mps`, `max_angular_radps`, `plane_max_age_s`.
- ROS-free API: `path_planner.py` (`scene_obstacles`, `straight_line_free`, `plan_path`, `time_parametrize`, `PathFollower`), `core.py` (`BaseProfile` incl. `footprint`, `load_profile`, `arm_to_footprint`, `footprint_to_arm`, `StandController`), `wall_docking.py` (`measure_wall`, `WallDockController`, optional lateral term `LateralConfig` with `PlannedStandLateral` / `WallEdgeLateral`, typed `DockOutcome`, `DockResult`, `plan_lateral_manoeuvre`), `dock_montecarlo.py` (noise Monte Carlo of the controller logic).
- Controller: speed and acceleration limited, for omni and differential bases (profile `base.drive_type`).

## Dependencies

- In this repo: `stand_navigator_msgs`.
- External: `rclpy`, `geometry_msgs`, `nav_msgs`, `python3-yaml`; `pytest` for tests.
- The core runs without ROS (plain Python). No GPU. No simulator needed (the simulations supply `/odom` and take `/cmd_vel`).

## Use it on its own

```bash
ros2 run stand_navigator stand_navigator_node --ros-args -p base_profile:=/path/profile.yaml
ros2 run stand_navigator wall_docking_node --ros-args -p base_profile:=/path/profile.yaml
```

`/path/profile.yaml` is any robot profile that has a `base:` section (see `mobile_manipulator_description/config/profiles`).

## Tests

`test/test_core.py`, `test/test_nav_safety.py`, `test/test_nav2_adapter.py`, `test/test_path_planner.py`, `test/test_wall_docking.py`, `test/test_wall_docking_lateral.py` and `test/test_dock_montecarlo.py` (127 tests in the last host run).

```bash
colcon test --packages-select stand_navigator
```

## Status and limits

- Open-loop on `/odom` with a software safety layer (stale odom, command gate, stuck, optional scan/cloud obstacle stop); no obstacle avoidance beyond stopping; the optional planner uses a static wall scene (grid A* plus shortcutting, host-tested only); the goal frame must be the odometry frame (no frame transform).
- Exercised in the MuJoCo simulation (`run_stand_navigation.sh`, `run_wall_docking.sh`); nothing here has run on hardware.
- Wheels are not modelled in the simulations; the base is a planar velocity-commanded body there.

## Path planning

Straight line stays the default. With `wall_scene_json` / `wall_scene_topic` set, the straight drive is checked against the scene's walls and obstacles with the profile footprint; when blocked (or with `use_path_planner:=true`) a grid A* path with line-of-sight smoothing is followed waypoint by waypoint, and the result message reports `planned path X m vs straight Y m`. Details, limits and the MuJoCo switches `MULTI_STAND_PATH_PLANNER=1` and `MULTI_STAND_WALL_DOCK=1`: [stand_navigation.md](../../../docs/stand_navigation.md) "Path planning".

## Safety layer and Nav2 backend

`nav_safety.py`: stale-odometry stop (`odom_timeout_s`), command gate (`safety_check_commands`), stuck abort (`stuck_window_s`,
`stuck_min_progress_m`) and an optional obstacle stop from `obstacle_scan_topic` (LaserScan) or `obstacle_cloud_topic` (PointCloud2)
with `obstacle_stop_m`, `obstacle_hold_s`, `obstacle_corridor_half_width_m`, `obstacle_data_timeout_s`, `obstacle_z_min_m`,
`obstacle_z_max_m`. The reason is the result `error_msg`; codes `NO_ODOM`, `BAD_COMMAND`, `NAVIGATION_STUCK`, `OBSTACLE`.
`backend:=nav2` (default `builtin`, `nav2_action_name`) forwards the goal to a Nav2 `NavigateToPose` server through `nav2_adapter.py`
(`NO_NAV2` if it is absent; `nav2_msgs` is imported only then). Both are host-tested with fakes (`test/test_nav_safety.py`,
`test/test_nav2_adapter.py`); nothing ran in colcon or against a real Nav2. See [stand_navigation.md](../../../docs/stand_navigation.md).

## Dock Monte Carlo

```bash
python3 -m stand_navigator.dock_montecarlo --runs 200 --seed 1 [--base diff] [--lateral off|planned|edge|all] [--sweep]
```

Runs the approach and the dock cores on a kinematic base with injected odometry error and plane noise, and prints the docked rate, failure reasons and the true final error distribution (mean, p50, p95, max). It checks the controller logic against synthetic noise, not a simulator or hardware; how to read it and a sample run are in [stand_navigation.md](../../../docs/stand_navigation.md). The lateral term and the typed outcomes are in the core only (the `WallDock` action does not carry them yet).

## Related docs

- [stand_navigation.md](../../../docs/stand_navigation.md)
- [reachability.md](../../../docs/reachability.md)
- [sim_mujoco.md](../../../docs/sim_mujoco.md)
