# RViz profiles

`mp_rviz` (`src/common/mp_rviz`) holds one RViz profile per module and a launcher:

```bash
colcon build --packages-select mp_rviz && source install/setup.bash
ros2 launch mp_rviz view.launch.py profile:=<name> [use_sim_time:=true] [markers:=true] [scene:=true]
ros2 launch mp_rviz view.launch.py rviz_config:=/abs/path.rviz     # any other file
```

`markers:=true` also starts `mp_viz_markers.py`, which publishes `/mp_viz/mission_text` (state and force as text)
and `/mp_viz/tool_trace` (the tool path coloured by force). A wrong profile name lists the available ones.

`sim_overview` is the default profile of both `sim_gazebo` and `sim_mujoco` (fixed frame `odom`, window 1400x900): RobotModel,
TF (odom, base_footprint, tcp_link), the wall (`/mp_viz/wall`), the MoveIt planning scene, the plan columns, stands, coverage
paint, the motion plan (`/display_planned_path` from move_group for free-space segments, `/mp_viz/planned_trajectory` from
`manipulation_manager` for every segment sent to the controller, Cartesian included; both with a trail) and the tool trace.
The sims start `mp_viz_scene.py` for it (`rviz_scene`, `rviz_plan_file:=<coverage_output.json>`, `rviz_wall_scene`; the files
are re-read when they change) and pass `moveit_profile` so RViz gets the MoveIt parameters. The Gazebo sim runs RViz with
`use_sim_time:=true` (the TF is on `/clock`). The coverage_planner_node path is in frame `map` and the demo path in the wall
scene's `base_link`, so neither is drawn in `odom`; the plan columns are.

`scene:=true` also starts `mp_viz_scene.py` (all topics transient_local; shown in `mission_overview`,
`manipulation_manager` and `reachability`):

- `/mp_viz/plan_columns`: the planned wall columns, one line per `MotionPlanSegment` of the
  `coverage_planner_msgs/CoveragePlan` on `/wall_coverage_service_node/coverage_plan` (parameter `coverage_topic`).
  Strips are coloured, connectors grey and thin.
- `/mp_viz/stands`: call `ros2 service call /mp_viz_scene/refresh std_srvs/srv/Trigger`. The node calls
  `reachability_msgs/PlanStands` on `/reachability/plan_stands` once, with the plan's path poses
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

## Layers

`profiles/layers.rviz` (and the same wall and reachable displays in `mission_overview`) shows the plan as separate
layers, each its own display inside the group "Plan layers", so the checkboxes toggle them:

```bash
rviz2 -d $(ros2 pkg prefix mp_rviz)/share/mp_rviz/profiles/layers.rviz -f map     # -f = frame of your data (map, odom, world)
```

| Layer | Topic | Shows |
|---|---|---|
| 1 Real wall | `/mp_viz/wall` | the whole wall rectangle from `wall_scene_json` (`origin_xyz + u*basis_u + v*basis_v` over `u_range_m` x `v_range_m`): translucent quad, outline and a size label |
| 2 Full coverage plan | `/mp_viz/plan_columns` | every stroke of the plan (topic or `plan_file`), connectors grey |
| 3 Reachable | `/mp_viz/reachable` | one cube per sampled plan pose, green when `reachability_msgs/IsReachable` says the arm reaches it from the base pose `base_x/base_y/base_yaw`, red when not; label gives the count |
| 2b Transitions | `/mp_viz/transitions` | grey arrows from the end of each stroke to the start of the next (joins the plan itself has no connector segment for), and the visiting number at the start of each stroke group; drawn from `start_index`/`end_index` in execution order |
| 4 Stands | `/mp_viz/stands` | `PlanStands` result (arrow, footprint, `#n tgt travel`) |
| 5 Executed tool trace | `/mp_viz/tool_trace` | from `mp_viz_markers.py`, coloured by force |

Start the node with the parameters you need (frame_id is the frame of the wall and plan; the wall JSON's own
`frame_id` is the default for the wall):

```bash
ros2 run mp_rviz mp_viz_scene.py --ros-args -p wall_scene_json:=/path/wall_scene.json -p plan_file:=/path/coverage_output.json \
  -p frame_id:=map -p base_x:=1.0 -p base_y:=0.0 -p base_yaw:=0.0 -p arm_base_z:=1.0 -p max_reach_targets:=150
ros2 service call /mp_viz_scene/refresh std_srvs/srv/Trigger     # re-reads wall + plan file, redoes reachable and stands
```

Reachable details: the world pose is turned into the arm-base frame (`arm_base_z` is the world height of the arm base,
`arm_dx/arm_dy` its offset in the base frame) and sent as async `IsReachable` requests, one in flight at a time (150 queued at once were answered at about one per
second, sequentially in milliseconds), at most `max_reach_targets` evenly sampled poses after interpolating the strokes
every `reach_step` m (the service answers one pose per call, there is no bulk query). The service names default to
`/reachability/is_reachable` and `/reachability/plan_stands` (the node is called `reachability`). Moving to
another stand means setting `base_x/base_y/base_yaw` (`ros2 param set /mp_viz_scene base_x 1.5`) and calling refresh.

### A full-wall plan offline

The Gazebo multi-stand test wall is 3.0 x 2.0 m (`smoke_wall_scene.json`: u -1.5..1.5, v 0..2, plane x = 2.0, frame
`map`), but its plan used a cropped 0.9 x 0.3 m contact patch: about 0.5 m2 of strokes, about 4.5 % of the 6 m2 wall.
To see a plan over the whole wall without a simulator:

```bash
python3 src/common/mp_rviz/scripts/make_full_wall_fixture.py /tmp/fw          # wall_scene.json + walls.pcd (3 x 2 m, 2 cm grid)
ros2 launch coverage_planner wall_coverage_demo.launch.py wall_scene_json:=/tmp/fw/wall_scene.json tool_name:=contact_wiper show_rviz:=false
```

The coverage planner has no argument for the point cloud: it loads `walls.pcd` from the directory of the scene JSON
(PCL `x y z label`, label = wall id), which the generator writes beside the JSON, and the scene JSON must carry all
wall keys (`observed_area_m2`, `observed_fraction`, `plane_rms_mm`, `spread_p95_mm`, `spread_p99_mm`, `points`,
`merged_layers`, `layer_offset_spread_mm`), which it also writes.

Measured on the full 3 x 2 m fixture with `contact_wiper`: 20 vertical strips, 0.15 m apart, centres y -1.475 to 1.375
(each 0.2 m wide, so the whole 3.0 m width), two passes each (the numbers were recorded before one pass became the default), 80 poses; the planner reports 41.85 m of path and 8.37 m2
swept (overlap and the second pass counted), the unique strokes drawn are about 3.9 m2 of strip length x 0.2 m, and the
wall is 6.0 m2 (the observed mask from the synthetic cloud is 2400/2400 cells). The strips run bottom-to-top and the
planner joins one strip to the next with a straight diagonal (no connector segments unless `add_strip_connectors:=true`),
which the transitions layer draws. At base (1.2, 0, yaw 0) with the arm base at 1.0 m: 39 of 150 sampled poses reachable.

Robot model without a simulator: `xacro .../mobile_manipulator.urdf.xacro arm:=ur10e`, strip the XML comments and put
the text in a parameters file (a multi-line `-p robot_description:=` fails to parse), then run `robot_state_publisher`
and `joint_state_publisher` with it plus a static transform `map` to `base_footprint` at the base pose.

### Layer limits

- The point cloud is synthetic (a perfect grid), so the plan covers the whole rectangle with no scan holes.
- Reachable uses one arm-base height (`arm_base_z`); the lift is not searched, so poses another lift height would
  reach show red. It checks the arm only (RM4D map), not collisions, the base footprint or the wall standoff.
- The reachable layer needs a running `reachability_node` with a map; without it the layer stays empty and refresh says so.
- Only the sampled poses are tested; a layer of 150 cubes over a long plan is a sample, not every pose.

## Honest limits

- Nothing was opened in RViz when these were written: the profiles are checked for well-formed YAML, a fixed frame and
  displays (`test/test_profiles.py`), and their topics were taken from the code, not from a running system.
- Several modules publish nothing RViz can draw: stand poses, the reachability map, manager state/markers, the wall
  detector's fitted plane, the cuRobo/VAMP planners' paths. The pages list what is missing per profile.
- `view.launch.py` does not pass MoveIt's semantic/kinematics parameters to RViz; for the MotionPlanning panel start
  `demo.launch.py` with `rviz:=true` instead.
- Fixed frames differ between modules (`base_footprint`, `map`, `world`, `link_base`); change it in RViz if a display is empty.

## Plan-only playback

`src/common/mp_rviz/scripts/plan_playback.sh` shows the plan itself, animated: the manipulation manager and its real
planners run on ros2_control **mock hardware** (perfect tracking, `demo.launch.py`) instead of a simulator, so every
executed trajectory is the planned one, played back at the planned speed. Nothing else is simulated.

```bash
PB_OUT=/path/to/run_dir PB_SRC=$PWD/src PB_RVIZ_WS=/path/to/ws_with_mp_rviz_built \
  src/common/mp_rviz/scripts/plan_playback.sh      # containers plan_pb (stack) and plan_pb_rviz; both stay running
```

`PB_OUT` holds `coverage_output.json`, `data/walls.pcd` and `ur10e.rm4d` of an earlier run (reused, so the plan is
reproducible); missing ones are built. By default the plan covers the WHOLE 3.0 x 2.0 m wall (`make_full_wall_fixture.py`
in the odom frame, wall plane at `PB_PLANE_X`, strokes 0.2 m wide, 20 % overlap, one pass; `PB_DOUBLE_PASS=true` adds the reverse pass); `PB_FULL_WALL=0` plans the
0.9 m x 0.3 m patch of `run_multi_stand_pass.sh` instead. `data/smoke_wall_scene.json` there draws the wall in RViz.
`PB_BUILD_VOL` / `PB_INSTALL_VOL` (default `gzcols_b` / `gzcols_i`) are the colcon build and install of `PB_SRC`.
Every `PB_*` variable reaches the container. Settings: `PB_DOMAIN` (92), `PB_IMAGE`, `PB_SPEED` (tool speed along the
stroke, default 0.2 m/s), `PB_EXTRA_STANDOFF` (0.02 m), `PB_PLANE_X` (0.98 m), `PB_Z_MIN` / `PB_Z_MAX` (the manager's
`coverage_z_min_m` / `coverage_z_max_m`; 0.5 / 1.9 m by default), `PB_MAX_SPAN` (`stand_max_span_m`, 0.5 m),
`PB_SWATH`, `PB_OVERLAP`. The manager writes `verify.json` (column and geometric coverage). The script refuses to start
without a free node slot (`docker/tools/hostwatch/slots.sh`).

Reach limits found with the UR10e on the telescopic lift (see the run numbers below): the stand planner always puts the
arm base 0.60 m from the plane (the footprint clearance limit; `plan_stands` has no preferred distance and the manager
sets none), so `PB_PLANE_X` fixes only where the base ends up (arm base x = plane - 0.60), the 0.70 m of the standoff
advisor is not selectable through the manager. Strokes below about 0.5 m cannot be docked (no IK at that height with the
tool horizontal), hence the default `PB_Z_MIN=0.5` (the lowest 0.5 m of the wall is not planned); columns at the lateral edge of a stand's span often fail to dock (OMPL approach
or Cartesian dock `planning_failed`).

What runs: `demo.launch.py` (robot_state_publisher, mock ros2_control, plain joint_trajectory_controller, move_group), the
real `reachability_node` and `stand_navigator`, `manipulation_manager` (`contact: false`, controller `lift_arm_controller`)
with one multi-stand `ExecuteCoverage` goal, and `mp_fake_base.py`, which integrates `/cmd_vel` into `/odom` and the TF
odom to base_footprint so the base "drives" to each stand. RViz uses `layers.rviz` with fixed frame `odom`; the tool trace
is drawn from TF `tcp_link` in `odom`.

Differences from the simulator pass, on purpose:

- Whole wall, not all of it: with the defaults a run docked 15 of 19 columns (4 failed at stand edges) and the sampled tool path
  covered 4.05 m2 of the 6.0 m2 wall (the planned area is 4.85 m2: z 0.5 to 1.9). Run time about 19.5 min at 0.2 m/s, of which
  the OMPL approaches and failed-column retries take most.
- Perfect tracking: no controller lag, no settling, no recovery jogs caused by it. Execution time is the planned time.
- No contact: the mock has no force sensor, so the run uses the manager's non-contact mode (`contact: false`, the spray
  tool's flow): approach, dock at the stand-off, strokes 2 cm in front of the plan's plane, retract, per column. No press,
  no force engage, no unload, and the trace is not coloured by force.
- The mock plain controller rejects goals that end with a tiny nonzero velocity; the script sets
  `allow_nonzero_velocity_at_trajectory_end` on `lift_arm_controller` at run time, nothing in the packages is changed.
- The base is kinematic (no wheel slip, no odometry error), so the stand arrival is exact.

## Plan-only playback: the best known configuration

`scripts/plan_playback.sh` runs the real manager, reachability and stand navigator on mock hardware (no simulator)
and opens RViz with the plan layers. The settings below gave 17 of 19 columns and 806 s on the 3 x 2 m wall at 0.2 m/s
(the same wall with the old defaults: 15 of 19, 1179 s). They need a workspace with `serpentine_columns`,
`chain_columns` and `stand_merge_tol_m` in the manager (merged in #210, all opt-in; the `stand_merge_tol_m` default on main is 0 = off, so set 0.15 as below), built as an overlay if your workspace predates it.

```bash
PB_OUT=/path/to/run PB_OVERLAY=/path/to/overlay/install \
PB_COV_EXTRA="-p double_pass:=false" \
PB_MANAGER_EXTRA=$'    chain_columns: true\n    chain_max_shift_m: 0.4\n    serpentine_columns: true\n    stand_merge_tol_m: 0.15\n    stand_min_clearance_m: 0.40' \
PB_EXTRA_STANDOFF=0.05 PB_SPEED=0.2 PB_Z_MIN=0.5 PB_Z_MAX=1.9 PB_MAX_SPAN=0.5 PB_PLANE_X=0.98 \
bash src/common/mp_rviz/scripts/plan_playback.sh
```

Notes: `stand_min_clearance_m` is the chassis-to-wall clearance (arm base about 0.30 m further out), so 0.40 is an
arm base 0.70 m from the wall; 0.70 put the arm at its reach limit and the stow failed. `PB_EXTRA_STANDOFF` is
`spray_extra_standoff_m`: at 0.02 the tool mount grazed the wall model at the ends of a stand's span.
Columns still failing: the last waypoints of a strip at z 1.9 and the outer column of a stand.

## The green TCP line

`/mp_viz/tcp_executed` (the bright green line in `sim_overview.rviz`) draws the wall strokes only: the tool position while the
conditioned force is above `path_contact_force_n` (default 2 N), with a break between strokes. Approach, retract, stow and base
moves are free-space motion and are not drawn; `path_contact_only:=false` on `mp_viz_markers` draws everything again. Before this
the line included every arm move and looped across the scene. It is a polyline of 1 cm steps (`path_min_step_m`).
