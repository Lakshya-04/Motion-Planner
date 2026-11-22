# RViz profiles

`mp_rviz` (`src/common/mp_rviz`) holds one RViz profile per module and a launcher:

```bash
colcon build --packages-select mp_rviz && source install/setup.bash
ros2 launch mp_rviz view.launch.py profile:=<name> [use_sim_time:=true] [markers:=true] [scene:=true]
ros2 launch mp_rviz view.launch.py rviz_config:=/abs/path.rviz     # any other file
```

`markers:=true` also starts `mp_viz_markers.py`, which publishes `/mp_viz/mission_text` (state and force as text)
and `/mp_viz/tool_trace` (the tool path coloured by force). A wrong profile name lists the available ones.

`scene:=true` also starts `mp_viz_scene.py` (all topics transient_local; shown in `mission_overview`,
`manipulation_manager` and `reachability`):

- `/mp_viz/plan_columns`: the planned wall columns, one line per `MotionPlanSegment` of the
  `coverage_planner_msgs/CoveragePlan` on `/wall_coverage_service_node/coverage_plan` (parameter `coverage_topic`).
  Strips are coloured, connectors grey and thin.
- `/mp_viz/stands`: call `ros2 service call /mp_viz_scene/refresh std_srvs/srv/Trigger`. The node calls
  `reachability_msgs/PlanStands` on `/reachability_node/plan_stands` once, with the plan's path poses
  (at most `max_targets`) as `targets_world` and a wall plane from `wall_point`/`wall_normal` (default: first target and
  the opposite of its tool z axis), then draws each stand as an arrow, a footprint rectangle and `#index n tgt travel`.
- `/mp_viz/reach_slice`: nothing is published. `reachability_msgs` has only `IsReachable`, `GetBaseCandidates`,
  `PlanLiftBands` and `PlanStands`, no map or slice service, so no slice can be drawn without editing that package.

Unverified: none of this was run against ROS or RViz (only the pure logic is tested on the host). The footprint size is
the parameter `footprint_length`/`footprint_width`, not read from the response, which carries no reach radius, so no
reach circle is drawn. PlanStands without a configured base profile reports yaw 0 and no footprint clearance.

Start the module you want to look at first (the exact command is in each table), then open its profile.

| Area | Page | Profiles |
|---|---|---|
| Description and planning | [description_planning.md](description_planning.md) | robot_model, moveit_planning, cartesian_validation, fsp_benchmark, planner_backends, self_filter |
| Perception, coverage, reachability, stands, manager | [perception_coverage_reach.md](perception_coverage_reach.md) | wall_detector, scene_model, coverage_planner, reachability, reachability_moveit, stand_navigator, manipulation_manager, mission_overview |
| Control and simulators | [control_sim.md](control_sim.md) | jtac_compliance, ft_sensor, safety_teleop, sim_mujoco, sim_gazebo, sim_isaac, tool_trace |

## Honest limits

- Nothing was opened in RViz when these were written: the profiles are checked for well-formed YAML, a fixed frame and
  displays (`test/test_profiles.py`), and their topics were taken from the code, not from a running system.
- Several modules publish nothing RViz can draw: stand poses, the reachability map, manager state/markers, the wall
  detector's fitted plane, the cuRobo/VAMP planners' paths. The pages list what is missing per profile.
- `view.launch.py` does not pass MoveIt's semantic/kinematics parameters to RViz; for the MotionPlanning panel start
  `demo.launch.py` with `rviz:=true` instead.
- Fixed frames differ between modules (`base_footprint`, `map`, `world`, `link_base`); change it in RViz if a display is empty.
