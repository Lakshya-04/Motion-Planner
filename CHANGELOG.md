# Changelog

All notable changes are recorded here, in the style of [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
There are no tagged releases yet; every package is at version `0.1.0` (set together, nothing depends on an exact version). The status of each capability,
with evidence, is [docs/ROADMAP.md](docs/ROADMAP.md).

## [Unreleased]

The stack as it stands, covering the merged pull requests #5 to #61, summarised by area (the first block is
#5 to #22 and the documentation on top of it; "Since #22" below it covers #23 to #61). Not included: a launch that runs perception to execution,
anything verified on hardware.

### Added

- Backend routing test and cross-backend planner benchmark: `manipulation_manager` `backend_routing_test` (configs with `free_space: vamp` / `curobo` and `segments`, built through `PlanningPipeline` over the pluginlib registry, fakes where a real plugin is unavailable, MoveIt always faked); `motion_planner_moveit` `planner_benchmark` (OMPL RRT-Connect on a MoveIt PlanningScene) and `scripts/planner_benchmark.py` (shared stand / stow / dock-like joint-space problems for MoveIt/OMPL, VAMP and cuRobo, path length and clearance judged by one reference model); results and what could not run in `docs/planner_benchmark.md`.
- Time parameterisation by segment role: `motion_planner/core/time_parameterization.hpp` (`TimeParameterizer` port, `parameterizeSegment`: validation with typed `TimingError`, near-duplicate removal, split at reversals over 150 degrees, constant-speed stamps, limit-violation report), `MoveItTimeParameterizer` in `motion_planner_moveit` (MoveIt TOTG for transitions; constant tool speed through FK plus Ruckig smoothing for seams), and `RetimingExecutionPort` in the manager, opt-in with `retime:=true` (`retime_accel`, `retime_seam_jerk`, `retime_transition_jerk`, `retime_vel_scale`, `retime_tcp_link`); a seam over a limit fails as `time_limit_violation`. `ExecutionRequest::stroke_speed` carries the role. Default behaviour unchanged; gtests only.
- Set-cover stand selection: `reachability` `StandStrategy::SetCover` (`core/stand_cover.cpp`; `planStands(..., strategy, SetCoverOptions)`): every search-grid station scored once from the RM4D map, weighted greedy cover with a drive-type travel term, bounded exact search for fewer stands, nearest-next + 2-opt order, stands moved to the cheapest station that still covers what only they reach. Opt-in: `stand_strategy` parameter of `reachability_node` (`~/plan_stands` unchanged) and `rm4d_tool --strategy set_cover`; default stays greedy. gtests on synthetic walls: designed wall 2 stands / 4.2 s against greedy 3 / 7.0 s, never worse on the others. Docs in `docs/reachability.md`. Not confirmed with IK, no sim run.
- Tool actuation executor and command guard: `manipulation_manager/core/tool_actuation.hpp` (schedule on trajectory time with `tool_off_lead_s`, pump speed gate, echo verification with resend, `TOOL_FAULT` / `TOOL_NOT_RESPONSIVE`, forced off on cancel, fault and end, latency) with `tool_actuation_test`, and opt-in `tool_actuation_node`; `core_types/command_guard.hpp` (NaN and joint-limit check, hold, debounced fault latch, dead-axis detection) with `test_command_guard`, wired opt-in into `JtacController` (`command_guard.enabled`, `command_guard.debounce_cycles`, `joint_limits.<j>.min_position/max_position`; a latch makes `update()` return ERROR). Docs in `docs/tools.md` and `docs/hardware_bringup.md`. Not run against hardware; the JTAC wiring is compile-checked only.

- Failure handling in the manager loop: `column_retries` (retry a failed column once, then skip it and go on), recovery jog (`core/recovery_jog.hpp`: joint-space move back toward the last good configuration when a dock, retract or stow is refused for a start state in self-collision or a lift outside its bounds; `recovery_jog`, `recovery_jog_max_rad`, `recovery_jog_speed`, `recovery_joint_lower/upper`), and `column_order` (`input` default, `nearest`, `nearest_2opt`; `core/column_ordering.hpp`), with gtests. Numbers in `docs/sim_mujoco.md`.
- Dock and stow reliability in the multi-stand pass: `manipulation_manager` `group_joint_names` (the live arm state was the all-zero pose because the joint names did not match the robot; a missing joint is now an error), `travel_lift_margin_m`, `dock_approach_extra_m`, `wall_clear_command` (retract and stow retried with the wall out of the scene), `world_model_loader_node -p clear:=true`, `WallDirection::approachPose`, `core/joint_names.hpp`, `core/travel_pose.hpp` with tests; `OmplPlanner` clamps an explicit start state into the joint bounds. Cause and numbers in `docs/sim_mujoco.md`.
- Non-contact tool end to end: `coverage_planner` `spray_deposition.hpp` (Gaussian footprint over the cone swath), `CoverageVerifier::addSprayStroke` (thickness raster from valve-open samples; mean, CoV, min, max), `score_run --no-contact --spray-standoff`; `manipulation_manager` `contact:=false` (no force engage or press, stroke at the plan's standoff, `std_msgs/Bool` valve on `/tool/valve` for the stroke only); `record_run.py` records the valve as the `active` column; `sim_mujoco/scripts/run_spray_pass.sh`.
- Manager task interface: `coverage_task_msgs` (`ExecuteCoverage` action), `manipulation_manager` serves `execute_coverage` (`run_on_start:=false`; the default keeps the one-shot run) with feedback, cancel and, with `multi_stand`, the stand loop (plan stands, stow, `navigate_to_stand`, columns in the world frame, per-stand result; `stand_max_span_m`, `stand_nav_attempts`, `stand_arrival_command`); `core/coverage_task.hpp` with tests; `sim_mujoco/scripts/run_multi_stand_pass.sh`, `execute_coverage_client.py`, `shift_wall_scene.py` width argument.
- `manipulation_manager` `world_frame` (default off): coverage poses in a fixed frame, transformed into `base_footprint` per column from TF; `dock_attempts`; `base_check` in the world frame. `sim_mujoco/scripts/run_stand_pass.sh`: plan stands, drive to stand 0, manager pass from there, scored.

- `sim_mujoco/scripts/run_sensed_pass.sh`: the manager pass on a wall sensed by the simulated depth camera (wall_detector -> cropped scene -> coverage -> whole-wall octomap -> manager); `manager_pass_common.sh` shared with `run_manager_pass.sh`, `crop_wall_scene.py`.

**Perception and scene**
- `scene_model`: backend-neutral scene (primitives, meshes, point clouds, voxel grids, ESDF), loaders for
  PCD, PLY, OBJ, STL and native / cuRobo-style YAML scenes, CPU voxelisation and exact ESDF, RANSAC plane
  fit, ROS converters for `PointCloud2`, `CollisionObject` and `PlanningScene`.
- `scene_adapters`: one scene to the geometry every planner backend takes (`ScenePort`), with a report of
  every approximation.
- `robot_self_filter`: geometry-containment self filter (library) and a ROS node that filters a
  `PointCloud2` using TF at the cloud's time stamp.
- `coverage_planner`: boustrophedon and sweepline grid planners, wall patterns (vertical, horizontal,
  zigzag with panels and a bottom pass), scan-to-plan library, tool-aware pass expansion, and three ROS
  nodes (grid service, wall service, demo).

**Robot description and profiles**
- `mobile_manipulator_description`: base + lift + N-DOF arm + tool; arms UR10e, generic 6-DOF and a
  7-DOF Panda-like arm; lifts telescopic, scissor, fixed and none; nine base models; 13 profiles.
- `mobile_manipulator_profile` (header-only loader) and profile-driven group, joint and frame names in the nodes.
- `mobile_manipulator_moveit_config`: MoveIt 2 config built from a profile, selectable IK solver (TRAC-IK
  default, KDL, pick_ik), a JTAC controller variant, mock demo and smoke test.
- One command from a profile to the per-robot assets (spheres, cuRobo config, VAMP header, ACM, reachability
  map, manifest) in `docker/tools`, and a MuJoCo / MJX cross-check of kinematics and self-collision.

**Planning**
- `motion_planner`: `Planner`, `IkPort`, `CollisionPort`, `ScenePort` and the `MotionPlanner` orchestration;
  `SeedPreferringIk`.
- `motion_planner_moveit`: Cartesian planner with jump and wall-penetration checks, OMPL free-space planner,
  contact and non-contact segments.
- `vamp_planner`: VAMP joint-space planner and collision checker over compiled-in robots.
- `curobo_bridge`, `curobo_bridge_msgs`, `curobo_planner`: cuRobo v2 behind a ROS node (plan, IK, validity,
  world update, ESDF) and a C++ client implementing `Planner` and `ScenePort`.
- `reachability`, `reachability_msgs`: RM4D reachability map, builder and CLI, base-aware stationing,
  multi-stand planning and a ROS shell with four services.
- `manipulation_manager`: runs coverage, planning and execution from a coverage file.

**Control and force**
- `force_core`: contact state machine and wrench conditioning (sign, bias, payload gravity, lever arm,
  payload identification).
- `compliant_trajectory_controller` (JTAC), `ft_bridge_hardware` (with a Gazebo system variant) and
  `ft_conditioner`.

**Simulation**
- Backend contract (`hardware:=mock|jtac_mock|mujoco|gazebo`), `sim_mujoco` (MuJoCo 4 `SystemInterface`, MJCF
  and wall generators, raw and ground-truth F/T, contact test), `sim_gazebo` (Harmonic, wall worlds, raw and
  ground-truth F/T, contact test), `sim_isaac` stub.

**Build and docs**
- Docker environment with nine targets (`base`, `deps`, `dev`, `build`, `gpu`, `sim`, `sim-gazebo`,
  `vamp-build`, `full`) and `docker/build.sh`.
- Documentation set under `docs/`, the roadmap with a reuse map, `THIRD_PARTY.md`, `CONTRIBUTING.md`, a
  root README with an overview.
- Sphere-set planning benchmark (partial: cuRobo on a 110-problem subset) and the FOAM versus cuRobo sphere comparison.

### Since #22 (pull requests #23 to #61)

**Added**
- Backend selection by YAML through pluginlib (#29, see `docs/backend_selection.md`); weighted IK.
- Tool profiles: `coverage_planner/config/tool_profiles.yaml` (contact wiper, spray gun), profile loading with
  overrides, a parametric spray swath model, `tool_profiles_file` / `tool_name` on the coverage launch files,
  `docs/tools.md` (#30, #31).
- Coverage verification and scoring: `CoverageVerifier` (per-cell raster of what executed strokes covered),
  `scoreRun` scorecard, `score_run` CLI with CSV/JSON formats (#36, #37, #39).
- `force_core::ForceLimiter`, a control-barrier bound on the velocity into the wall with an online
  stiffness estimate; library only, not wired into JTAC (#38).
- MuJoCo: `contact_test --record`, a scripted wall pass with scoring (`run_wall_pass.sh`), the manager in the
  loop (`run_manager_pass.sh`), a planar-velocity base (`base_motion:=true`) and a stand-navigation check (#40,
  #43, #47, #56, #57).
- `stand_navigator`: `NavigateToStand` action and a ROS-free controller (omni/diff, limits) (#57).
- Manager: `wall_direction`, `controller_name`, `base_check` (off|warn|enforce) parameters (#35, #46, #54);
  press-then-stroke with a settle and force release before disengage (#49, #51).
- Gazebo: compliant tool tip (5 N/mm spring-damper) so JTAC holds 10 N, and a manager pass
  (`sim_gazebo/scripts/run_manager_pass.sh`) (#55, #59).
- `plan_stands` `max_stands` request field (#34).

**Changed**
- MuJoCo wall pass default stroke speed 0.02 m/s (in-band fraction 34 to 81 percent, force RMS 2.8 to 1.5 N);
  pass stroke speed 0.01 m/s in the manager pass (#44, #51).
- Octomap: link the same copy MoveIt links (a second copy in the process aborted at exit) (#32).
- The ACM keeps links bolted to the lift from colliding with the lift stages (#48); profiles carry the base in
  `xacro_args` and `arm_mount_xy` where needed (#52).
- Docs refreshed several times to match the code (#23 to #26, #45, #50, #53, #60, #61); launch files that
  referenced packages that do not exist were fixed or removed (#27).

**Documentation**
- Hardware parameter template, hardware bring-up guide, release checklist and a single "run all checks" section
  in `docs/docker.md`.

### Changed (first block)
- TRAC-IK with `Distance` is the default MoveIt IK solver.
- Docs that said "planned" or "not done" for things that now exist were brought in line with the code
  (architecture status table, reachability, cuRobo, MoveIt config, scene model, self filter, coverage planner).

### Known gaps (see the roadmap)
- No launch that runs perception through execution; the simulator passes start from a coverage file.
- No live sensor ingest, no time parameterisation.
- Both simulators hold a 10 N contact (MuJoCo 10.13 N; Gazebo 9.69 N with the compliant tip).
- Nothing verified on hardware.
