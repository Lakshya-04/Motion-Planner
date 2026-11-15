# Changelog

All notable changes are recorded here, in the style of [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
There are no tagged releases yet; every package is at version `0.1.0` (set together, nothing depends on an exact version). The status of each capability,
with evidence, is [docs/ROADMAP.md](docs/ROADMAP.md).

## [Unreleased]

The stack as it stands, covering the merged pull requests #5 to #61, summarised by area (the first block is
#5 to #22 and the documentation on top of it; "Since #22" below it covers #23 to #61). Not included: a launch that runs perception to execution,
anything verified on hardware.

### Added

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
