# Changelog

All notable changes are recorded here, in the style of [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
There are no tagged releases yet; every package is at version `0.1.0` (set together, nothing depends on an exact version). The status of each capability,
with evidence, is [docs/ROADMAP.md](docs/ROADMAP.md).

## [Unreleased]

The stack as it stands, covering the merged pull requests #5 to #61, summarised by area (the first block is
#5 to #22 and the documentation on top of it; "Since #22" below it covers #23 to #61). Not included: a launch that runs perception to execution,
anything verified on hardware.

### Fixed

- VAMP robot model rejected most benchmark endpoints (60 of 72 in the empty scene, every `stow_side` pair): FOAM's 6 spheres on the 0.04 m lift carriage plate bulged into the upper arm. `docker/tools/generate_vamp_robot.sh` now defaults `FOAM_ARGS` to `--lift_carriage_link=4` (23 plate spheres, 50 in total); on the same 158 problems VAMP goes from 10/158 to 158/158. Numbers in `docs/planner_benchmark.md`.

### Added

- Gazebo fixes (`sim_gazebo`, `force_core`, `reachability`): `force_core` and `reachability` now `ament_export_dependencies(... Threads)`, so packages that link them find `Threads::Threads`; `arm_meshes` defaults to `true` (the UR meshes resolve, the arm has visual and collision; `GZ_PASS_MESHES=0` restores the old behaviour); a stiff bump-stop stage behind the tool tip (`tool_tip_stop_joint`; it does not bound the contact impacts, measured); `rgb_camera_frame:=world` (`GZ_PASS_CAMERA_FRAME=world`) for a world-fixed camera. Numbers in `docs/sim_gazebo.md`.
- Gazebo spray pass with the arm meshes: the first MoveIt plan was refused (start state in collision with the wall octomap) because the 4 s seed plan outruns gz's 10 s joint loop, so the arm followed a joint-space line and came to rest on the wall top; `SPRAY_PASS_SEED_SCALE` now defaults to 0.04 (27 s). Spray 4 of 4 pass (coverage 0.92 to 1.00), multi-stand 1 of 2 (the stand 1 dock failures are not fixed) (`docs/sim_gazebo.md`).
- Opt-in sensor non-idealities in the MuJoCo plugin (`sim_noise` launch argument, default empty = unchanged): `sim_mujoco/signal_model.hpp` (white noise by density, bias random walk, first-order low-pass, fixed delay, quantisation) on the raw F/T, Gaussian joint-state noise and command latency, environment variables read by the plugin; `test_signal_model` gtests and `test_sim_noise` pytest. `docs/sim_vs_real.md` (what the sims model and what differs from hardware), linked from `simulation.md` and `hardware_bringup.md`. Not calibrated against a real sensor; no Gazebo equivalent.
- Contact-area model for contact tools (`core_types/contact_area.hpp`, `coverage.contact_area` in the tool profile): effective pad width from target force, tilt and surface curvature; `toolSwath` / `toolLineSpacing` derive swath and spacing from it. Optional `edge_zone_m` / `edge_spacing_scale` and `contactStripOffsets` for tighter strips near edges. Off by default, shipped profiles unchanged; YAML loader checks. gtests in `test_contact_spacing`, `test_tool_profile`, `tool_profile_yaml_loader_test`. Edge spacing is not yet applied inside the boustrophedon/sweepline planners. No sim run. Docs in `docs/tools.md`.
- Payload identification and bias drift (`force_core`): `payload_excitation.hpp` (`regressorConditionNumber`; `generateExcitationPoses` picks well-spread wrist poses inside joint limits with margin, velocity and acceleration bounds and a caller safety proxy checked along every move, rest-to-rest quintics; `sampleExcitationPlan`), `identifyPayload` now reports and limits the condition number (default 1000), `bias_tracker.hpp` (per-axis Kalman bias tracker updated only when free of contact and still, innovation gate, total-correction cap, recalibration flag). The `identify` service prints the condition number. 12 gtests; with 0.1 N noise, 10 poses, 50 trials: worst mass error 0.0096 kg, centre of gravity 0.68 mm, bias 0.09 N; 0.1 N drift tracked to 0.02 N. Not done: a service that runs the plan, the tracker in the node, hardware. Docs in `docs/calibration.md`.
- Coverage on walls with obstacles (`coverage_planner`): `components_in_sweep_order`, `fill_gaps` + `min_stroke_length_m` (short strokes up to an obstacle's keep-out edge) and `decomposition: cells` (boustrophedon cell decomposition) in `BoustrophedonPlanner`; `test_coverage_components` (rib, window, protrusion rasters, `CoverageVerifier` above 0.95). Docs in `docs/coverage_patterns.md`. No sim run.
- Mission ledger and resume: `manipulation_manager` `core/mission_ledger.hpp` (per column state PENDING, RUNNING, DONE, SKIPPED with typed reason `NAV_FAILED` / `DOCK_FAILED` / `PLAN_FAILED` / `CONTACT_OVERLOAD`, FAILED; atomic JSON file, corrupted file moved aside) and `core/mission_fsm.hpp` (IDLE, ACTIVE, PAUSED, STOPPED, RECOVERY). Parameters `ledger_path` and `contact_overload_n`, topic `mission_command` (`pause` / `resume` at column boundaries), new field `resume` in `ExecuteCoverage` (default false, backward compatible) and `--resume` in `execute_coverage_client.py`. gtests in `mission_ledger_test`. No sim kill-and-restart run, no hardware.
- Live octomap: `live_octomap:=true` (sim_mujoco launch, `build_moveit_config(live_octomap=True)`, `demo.launch.py`) adds `config/sensors_3d.yaml` so move_group builds its octomap from `/depth_sensor/points` with MoveIt's `PointCloudOctomapUpdater` (self filter by its shape mask); `moveit_ros_perception` added to `docker/apt-extra.txt` and the config package. `octomap_esdf_node` + `core/octree_esdf` in `wall_detector` (octomap -> `scene_model` ESDF on `~/esdf`, 1 gtest). `sim_mujoco/scripts/run_live_octomap.sh` + `live_octomap_test.py`. Default off; sim only.
- `hardware:=real`: `mobile_manipulator_description` `config/hardware_drivers.yaml` (arm -> vendor driver: UFACTORY xarm_ros2, UR `ur_robot_driver`, Franka `franka_ros2`, plus `mock`; package, plugin, required parameters, command interfaces, F/T source; nothing vendored), the `real` branch of `mobile_manipulator.ros2_control.xacro` (`real_driver`, `real_params`, `real_lift_plugin`; blocks `lift_system`, `arm_system`, `ft_sensor_bridge`, `ft_conditioned_bridge`; the arm-joint list is now a shared macro, other backends unchanged), `hardware_drivers.py` and `launch/hardware_real.launch.py` (one clear message and shutdown for an unknown arm, a missing parameter, a lift without a plugin or a driver package that is not installed), `test_hardware_real.py`. Docs in `docs/hardware_bringup.md` section 0a, cross-linked with `docs/calibration.md`. Driver names come from vendor documentation, not from an installed driver; nothing run on hardware.
- Planar patches: `coverage_planner` `core/surface_patch.hpp` (`SurfacePatch` for wall, floor, ceiling and column faces: `project`, `point`, `contains`, `liftPatchPath` with tool axis along the patch normal, `patchObstacles`, `planPatchCoverage`), and `planCoverageOnWall` split out of `planCoverageFromScan` (wall behaviour unchanged). gtests in `test_surface_patch`. Docs in `docs/coverage_patterns.md` and `docs/architecture.md`. Stand selection for horizontal surfaces not done; no sim run.
- Calibration procedures in one place (`docs/calibration.md`: F/T sign and bias, tool TCP, payload services, wall plane, base-to-wall) and `core_types/pivot_calibration.hpp` (ROS-free least-squares tool-TCP calibration by pivoting, degenerate-pose and too-few-pose rejection, 5 gtests including simulated noise). No ROS node for it; nothing run on hardware.
- Backend routing test and cross-backend planner benchmark: `manipulation_manager` `backend_routing_test` (configs with `free_space: vamp` / `curobo` and `segments`, built through `PlanningPipeline` over the pluginlib registry, fakes where a real plugin is unavailable, MoveIt always faked); `motion_planner_moveit` `planner_benchmark` (OMPL RRT-Connect on a MoveIt PlanningScene) and `scripts/planner_benchmark.py` (shared stand / stow / dock-like joint-space problems for MoveIt/OMPL, VAMP and cuRobo, path length and clearance judged by one reference model); results and what could not run in `docs/planner_benchmark.md`.
- Wall docking servo: `stand_navigator` `wall_docking.py` (ROS-free: plane to standoff and yaw, proportional control with velocity and acceleration limits, tolerances, timeout, omni and diff behaviour, typed status) and `wall_docking_node` serving the new `WallDock` action (`stand_navigator_msgs`), 21 pytest tests; `sim_mujoco/scripts/run_wall_docking.sh` (3 runs from 0.4 m and 15 degrees off: standoff error 4.8-5.2 mm, yaw 0.01 degrees). Docs in `docs/stand_navigation.md` and `docs/perception_live.md`. Sim base is omni; diff drive is gtest-only.
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

### Fixed

- `coverage_planner_node` defaulted `plan_all_components` to false, so a wall cut by a rib (`two_panels`) was planned only for its largest panel; now true (yaml too), and the node enables gap strokes and sweep-ordered components.
- `JtacControllerCompliancePort`: the destructor could hang in `join()` when it ran before the spin thread entered `spin()` (the executor `cancel()` was lost). The thread now polls a stop flag between short `spin_once()` calls. The 300 ms settle workaround in `test_jtac_compliance_port` is gone and a test constructs and destroys the port 200 times in a row under a watchdog.

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
