# Changelog

All notable changes are recorded here, in the style of [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
There are no tagged releases yet and every package version is still `0.0.0`. The status of each capability,
with evidence, is [docs/ROADMAP.md](docs/ROADMAP.md).

## [Unreleased]

First entry: the stack as it stands, covering the pull-request stack (#5 to #22 and the
documentation on top of it), summarised by area. Not included: a launch that runs perception to execution,
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

### Changed
- TRAC-IK with `Distance` is the default MoveIt IK solver.
- Docs that said "planned" or "not done" for things that now exist were brought in line with the code
  (architecture status table, reachability, cuRobo, MoveIt config, scene model, self filter, coverage planner).

### Known gaps (see the roadmap)
- No end-to-end run; the manager hard-wires MoveIt Cartesian + OMPL and has no runtime backend selection.
- No live sensor ingest, no tool profiles shipped, no time parameterisation, base motion not simulated.
- Gazebo cannot hold a 10 N contact; MuJoCo can (10.13 N for a 10 N target).
- Nothing verified on hardware.
