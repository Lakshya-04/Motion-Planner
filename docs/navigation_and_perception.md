# Navigation and perception: what is real today

One page that says, from reading the code and the pass scripts, what in the chain
perception -> scene -> coverage -> stands -> base navigation -> planning -> control runs on real algorithms,
what is a stand-in, and what the simulators hand the stack for free. Nothing here has run on hardware.
Details live in [perception_live.md](perception_live.md), [stand_navigation.md](stand_navigation.md),
[reachability.md](reachability.md), [sim_mujoco.md](sim_mujoco.md) and [sim_gazebo.md](sim_gazebo.md); this page links, it does not repeat.

## Short version

- The arm side is real: coverage planning, stand planning, MoveIt / cuRobo planning, JTAC force control run the same code in the
  simulators as they would on a robot.
- The base side is a stand-in. `stand_navigator` drives a straight line from the current odometry to the stand. It has no map,
  no costmap and no obstacle avoidance, and it does not localise: it trusts `/odom`.
- In both simulators `/odom` is the simulated base pose itself (no noise, no drift, no slip), and `/cmd_vel` is applied as a
  planar velocity. Navigation error in the sim passes is therefore much smaller than any real base will give.
- Perception is not in the loop of the two-stand passes. The wall the coverage planner plans on is a pre-baked JSON
  (`make_scenario_scene.py`), and the wall in MoveIt's scene is a baked `walls_sim.pcd` (MuJoCo) or `walls.pcd` (Gazebo)
  loaded by `world_model_loader_node`. `wall_detector` is not started. `wall_docking_node` is not started either.
- Where perception does run (`run_sensed_pass.sh`, `run_depth_wall.sh`, `run_live_octomap.sh`, `run_wall_docking.sh`) it is
  a single fixed base, a simulated ray-cast or depth cloud, one plane, no obstacles, no self hits.

## Component table

"Sim ground truth" means the simulator supplies the exact quantity and the stack never has to estimate it.

| Component | Status | MuJoCo | Gazebo | Hardware |
|---|---|---|---|---|
| Depth sensor | sim only | Ray-cast (`mj_multiRay`) on scene geoms only, optional Gaussian range noise; no robot self hits, no texture | gz `depth_camera`, optional noise; used in `run_depth_wall.sh` only | untested; no driver in the repo |
| 2D lidar | sim only | `lidar:=true`: `mj_multiRay` fan on scene geoms only, `LaserScan` on `/scan`; `run_lidar_stop.sh` | `lidar:=true`: gz `gpu_lidar`, `/scan`; `run_lidar_stop.sh` | untested; no driver in the repo |
| `robot_self_filter` | real (29 tests), not in any pass | not started (the MuJoCo cloud never contains the robot) | not started | untested |
| `wall_detector` (RANSAC + PCA plane) | real for one plane; sim input only | started only by `run_sensed_pass.sh`, `run_depth_wall.sh`, `run_live_octomap.sh`, `run_wall_docking.sh`; not by the two-stand pass | `run_depth_wall.sh` only; not in the two-stand pass | untested |
| `scene_model` (loaders, RANSAC, ESDF) | real library | used by the detector and the loaders | same | untested |
| Wall for coverage planning | pre-baked JSON in the passes | `make_scenario_scene.py` writes `wall_scene.json` and `walls_sim.pcd` (the sim wall world is generated from the same file) | `shift_wall_scene.py` shifts the stock scene and cloud | would come from `wall_detector` or a map; not wired |
| Wall in MoveIt / cuRobo scene | baked cloud loaded by `world_model_loader_node` (and `curobo_world_loader`) at stand arrival | `walls_sim.pcd` | `walls.pcd` | untested |
| Live octomap / ESDF | partial: `run_live_octomap.sh`, `octomap_esdf_node`; nothing feeds ESDF to cuRobo | single-stand check only | none | untested |
| Coverage planner | real | real | real | not run on a real wall |
| Stand planner (`plan_stands`, RM4D) | real, includes a grid `NavFeasibility` stand-in | real; obstacles are 2D from the scenario | real | not run on a real robot |
| `NavigateToStand` (`stand_navigator_node`) | stand-in: proportional controller, straight line, speed and acceleration limits per drive type | runs in the multi-stand, stand and navigation scripts | runs in the multi-stand pass | untested |
| Obstacle-aware base path | missing | none | none | none |
| `wall_docking_node` (`WallDock`) | real controller, tested in sim in isolation, not wired into the manager | only `run_wall_docking.sh` (single dock, wall plane from `wall_detector` on odom; 3 runs: 5 mm and 0.01 deg) | none | untested |
| Localisation | missing: the odom frame is the world frame, the base starts at its origin | `/odom` = simulated pose, no noise | `/odom` = `OdometryPublisher`, no noise | untested |
| Nav2 | not installed in any image, no adapter | none | none | none |
| Base safety (obstacle stop, e-stop on the base) | `nav_safety` obstacle stop from a `LaserScan` exists in `stand_navigator` (`obstacle_scan_topic`, off by default); not an e-stop, not in the manager passes | `run_lidar_stop.sh`: stops 0.30 m short of a floor box seen by the sim lidar; without the lidar the base drives into it | same, in an empty world | none |
| Manager `navigate_to_stand` call | real client; goal in the world (odom) frame, waits for the result, optional stand-check gate on arrival error (`stand_check`) | used | used | untested |
| MoveIt / cuRobo / VAMP planning | real | real, against the baked wall | real, against the baked wall | untested |
| JTAC + F/T | real controller; sensing is simulated | simulated F/T | simulated F/T | untested (see [hardware_bringup.md](hardware_bringup.md)) |

## What the simulators feed and take

- `/cmd_vel` (`geometry_msgs/Twist`): the MuJoCo plugin applies it as a kinematic planar velocity (no floor, wheels or slip;
  `base_motion:=true`). Gazebo uses `VelocityControl` (no command timeout: a stale command keeps driving; no wheel physics).
- `/odom` and TF `odom -> base_footprint`: MuJoCo `base_odom.py` republishes the simulated base pose; Gazebo's
  `OdometryPublisher` does the same. No noise, no drift, no latency model. `odom` starts at the spawn pose and is treated
  as the world frame by the manager (`world_frame: odom`).
- So `stand_navigator` is closed on the true pose. Arrival errors of 1 cm in [stand_navigation.md](stand_navigation.md)
  measure the controller, not odometry or localisation. [sim_vs_real.md](sim_vs_real.md) lists what a real base adds.

## How the manager reaches a stand

`manipulation_manager` (`navigateToStand`) sends a `NavigateToStand` goal in `world_frame` with the stand's arm-base x, y and
yaw, then waits for the result. Before that it stows the arm; afterwards the `stand_arrival_command` hook reloads the wall into
the planning scene, and the optional `stand_check` gate compares the arrival error with a tolerance from the reachability
margin. There is no wall-dock step in between, no replan of the path, and no choice of another stand when navigation fails.

## Gaps and the order to close them

1. **Obstacle-aware base path planner.** Replace the straight line by a planner over a footprint-inflated 2D map (in
   progress by another agent, branch `feat/stand-navigation`). Until it merges, a stand behind an obstacle is a collision.
2. **Perception in the loop.** Run `wall_detector` (and `robot_self_filter` on the live cloud) in the two-stand passes and
   plan from the detected scene instead of `make_scenario_scene.py` output; refresh the planning scene at each stand from a
   scan, not a baked cloud (in progress, branch `feat/perception-in-the-loop`). Multiple planes, obstacles and openings per
   scan are still open (`ROADMAP.md` section 1).
3. **Wall docking in the pass.** Start `wall_docking_node` and the detector with `target_frame:=odom` in both two-stand
   scripts, call `WallDock` after `NavigateToStand`, and feed the measured standoff and yaw error into the stand check. The
   lateral term and typed outcomes exist in the core only ([stand_navigation.md](stand_navigation.md)).
4. **Localisation.** Odometry plus a wall or map reference (`robot_localization` first); then add odom noise and drift to
   the simulators so the controllers meet it before hardware does.
5. **Nav2 adapter.** `NavigateToStand` is shaped like `NavigateToPose`; install Nav2 in an image, write a bridge, and use
   `opennav_docking` with a wall-plane dock plugin if it beats `wall_docking_node`. This comes after 1 to 4 because Nav2
   needs the map and localisation anyway.
6. **Safety.** A base-level obstacle stop and speed limit near the wall, a watchdog on stale odom and cloud, and the manager
   behaviour when the base fails (re-stand, abort). Needed before any run on hardware.

Then: hardware bring-up of the depth sensor, the base and the odometry, with the checks in
[hardware_bringup.md](hardware_bringup.md) and [calibration.md](calibration.md).
