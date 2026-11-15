# Roadmap and status

The target: open-source, plug-and-play, **any mobile manipulator** (any arm, lift, base) covering **any
surface** with **any tool** (contact like a mop, non-contact like a spray gun), from sensor data to executed
tool motion, run in **MuJoCo and Gazebo** now (Isaac Sim is a stub) and on hardware with parameters only.

```
sensors -> perception -> self filter -> scene -> surface -> coverage (tool-aware) -> base stands
        -> navigation / docking -> motion planning -> execution (JTAC + F/T) -> monitoring
```

Last verified: 2026-11-15, after merging the stack #5 to #29. Legend: `[x]` done, `[~]` partial (what is
missing is stated), `[ ]` not done (some are design-only). Each item names its evidence. Tick items as they land
and update the date. **Policy:** reuse maintained open source before writing our own (section 17); ROS 2 distro
support is not a reason to reject a candidate (we port); prefer the newest method that fits, including papers
without code, when we can implement it.

**Headline:** the building blocks exist; **nothing runs them together** (section 11).

## 1. Perception input (self and external data)

- [x] File loaders: PCD, PLY, OBJ, STL, native and cuRobo-style YAML scenes (`scene_model/src/core/io_*.cpp`)
- [x] Voxelise and ESDF on CPU (`scene_model/src/core/voxel.cpp`)
- [x] ROS converters: PointCloud2, CollisionObject, PlanningScene (`scene_model/src/ros/conversions.cpp`)
- [x] Scene to planner geometry: `ScenePort` for VAMP, cuRobo, MoveIt (`scene_adapters`, 12 tests)
- [~] Robot self filter: geometry containment node, 29 tests (`robot_self_filter/src/ros/self_filter_node.cpp`); no live
      sensor wired, not a lifecycle node, no sphere-model / GPU / learned variants
- [ ] **Live ingest**: PointCloud2 or depth -> `SceneModel` (+ octomap for MoveIt). Only file loaders exist. *Reuse: MoveIt's `pointcloud_octomap_updater` and `point_containment_filter` (always built; BSD-3), `octomap_server` (`ros2` branch); MoveIt's depth updater and mesh-filter shape mask need OpenGL*
- [ ] Simulated sensors (depth camera, lidar) in MuJoCo and Gazebo. *Reuse: Gazebo `depth_camera` / `rgbd_camera` / `gpu_lidar` through `ros_gz_bridge` (Apache-2.0); MuJoCo: `mujoco_ros2_control` camera and `mujoco_3d_lidar` plugins, or `MuJoCo-LiDAR` (MIT); depth rendering in the headless `mujoco-core` wheel is unconfirmed, test it*
- [ ] Wall detection from a live scan (`planCoverageFromScan` is a library; no ROS node calls it). *Reuse: PCL multi-plane segmentation behind our plane port*
- [ ] External data: prior map / BIM / merging scans (nothing beyond the nvblox design note)
- [ ] Live mapping (TSDF/ESDF): *reuse cuRobo's Mapper or nvblox (GPU, both Apache-2.0); CPU: VDBFusion (MIT) or a ROS 2 fork of wavemap (upstream stale)*; design only today (`docs/nvblox_notes.md`)
- [ ] Live ESDF to the planners (cuRobo `set_esdf` exists; nothing produces a field)
- [ ] Doors, windows, openings (only as generic obstacles today)

## 2. Coverage planning (planar surfaces)

- [x] Grid planners: boustrophedon, sweepline (`coverage_planner/core`)
- [x] Wall patterns: vertical, horizontal, zigzag, with panels (one base stand per panel) and a bottom pass
- [x] `planCoverageFromScan`: plane fit, rasterise, plan, lift with a tool profile per part (library; 69 tests in the package)
- [x] Obstacles projected onto the wall and inflated per tool (`wall_rasterizer.cpp`)
- [~] ROS nodes: `coverage_planner_node` (OccupancyGrid -> path), `wall_coverage_service_node` (from `wall_scene.json` + PCD), demo node;
      no action interface, the `CoveragePlan` topic has no subscriber in the repo, nodes have only a conversions test
- [~] Spiral is listed in the service but is a stub (the README now says so)
- [ ] Cellular decomposition (BCD) for large cluttered walls (`docs/coverage_patterns.md`)
- [ ] Coverage feed over ROS (the manager reads `coverage_output.json`; a ROS `CoverageSource` is planned)
- [ ] **Coverage verification**: estimate what was actually covered from executed tool poses (a per-cell deposition/contact raster);
      the highest-leverage new piece, also the evaluator for the spray model (section 4)

## 3. Non-planar surfaces (floors, ceilings, columns, curved walls, meshes)

- [ ] `SurfaceModel` interface (project, lift with normal and tangent frame, contains, obstacles): design only (`docs/architecture.md`)
- [ ] Piecewise-planar rooms: one planar patch per face (floor, ceiling, columns) reusing today's planners
- [ ] Mesh / freeform surfaces. *Reuse: Noether (Apache-2.0; ROS-free `noether_tpp` core; plane-slice raster planners with `line_spacing`; offset and approach/departure path modifiers; robot-independent output of poses with normals; no swath or standoff awareness: swath maps to line spacing, standoff to the offset modifier); else parameterise-then-sweep (geometry-central or pmp-library, MIT)*
- [ ] Lift the vertical-wall assumptions: `chooseStation` rejects a horizontal plane; `loadWallScene` enforces wall flatness
- [ ] Learned / non-grid coverage planners as further implementations of the same port (design only; recent RL work is not ready for a deterministic task)

## 4. Tools (contact and non-contact)

- [x] `ToolProfile` with geometry, task, coverage and control facets, and a YAML loader (`core_types`, `ros_adapters`)
- [x] Coverage uses swath, overlap, inflation, standoff, yaw, tilt, contact flag (`ToolCoverageProfile`)
- [x] Tool offset from the arm TCP: kinematics via `tcp_xyz/rpy` in the profile; force via the conditioner's lever arm
- [ ] **Ship tool profiles**: no `tool_profiles.yaml` exists; nodes take parameters and default to "mop"
- [ ] Connect the YAML loader to the coverage planner (not wired end to end)
- [~] Spray model: `sprayerEffectiveSwath` is a placeholder (slope 1, intercept 0); swath is not derived from standoff;
      no overspray/overlap model, no valve/pump timing along the path. *Implement: parametric swath w(d), Gaussian/beta profile,
      thickness-accumulation raster, choose spacing and standoff by minimising thickness variance (about 16-18% overlap is a published optimum for an elliptical pattern)*
- [ ] Non-contact segments end to end: hold standoff, no force engage, trigger the valve along the path (contact approach/retreat exist)
- [ ] Contact tools: adaptive line spacing from a contact-area model (a cheap published idea, 2-3 days on top of a planner)
- [ ] "How to add a tool" guide

## 5. Motion planning (any backend, contact and non-contact)

- [x] Ports `Planner`, `IkPort`, `CollisionPort`, `ScenePort` (`motion_planner/core`)
- [x] Cartesian planner: jump detection, reachability-tilt retry, wall-penetration check (`motion_planner_moveit`)
- [x] OMPL free-space planner through MoveIt; routing Cartesian first then OMPL (`MotionPlanner::planSegment`)
- [x] Contact / non-contact: ACM exclusion scoped to the wall octomap; approach, contact and retreat as separate segments
- [x] IK: KDL, trac_ik (default), pick_ik (opt-in, untuned), `SeedPreferringIk`, cuRobo IK
- [x] cuRobo v2 backend (plan, IK, scene, ESDF) and VAMP backend (joint-goal plan, collision, scene)
- [x] **Runtime backend selection**: pluginlib plugins for planner, IK, collision and scene (MoveIt, cuRobo, VAMP); the manager builds them from a `planning_config` YAML (PR #29)
- [ ] VAMP and cuRobo used by the manager for free-space transitions (the architecture example: IK cuRobo, free-space VAMP)
- [~] VAMP: no pose goal, no IK, robots compiled in, not a contact planner
- [~] cuRobo: pose-based collision unsupported, no wall-contact equivalent, one planner per process
- [~] Joint-space goals honoured by OMPL only
- [ ] Time parameterisation. *Reuse: MoveIt TOTG and its Ruckig filter first (`moveit_core` links Ruckig, MIT); toppra (MIT) for coverage passes; cuRobo v2 has a TimeOpt module*
- [ ] Segment ordering by a time metric / TSP and failure-aware resequencing (input order only)
- [ ] cuRobo v2 for moves between stands: v0.8.0 (April 2026, the open-source cuRoboV2) and v0.9.0 (2026-11-05, cuRoboV2-Fast: faster planning, Cartesian waypoint planning with moving starts, C++ motion planner, TimeOpt); pin v0.9.0 or later, not v0.7.8; mobile-base support and VRAM on 4 GB are unverified
- [ ] Recovery proposer (design only: `docs/learned_recovery.md`) and the policy slot
- [ ] Shared planner benchmark across backends (success, time, path, clearance); only the sphere-set benchmark exists

## 6. Robot generality (any arm, lift, base)

- [x] Profiles as the single source of truth (13 profiles), Python and C++ loaders, profile-driven group and joint names
- [x] Arms: ur10e, generic_6dof, 7-DOF Panda-like; N-DOF contract
- [x] Lifts: telescopic, scissor, fixed, none; recipe for a column lift
- [x] Bases: ranger_mini, scout, scout_mini, husky, mpo_500, mir100, two turtlebot3, generic (omni / diff / skid / 4ws)
- [x] One command from a profile to every per-robot asset (spheres, cuRobo config, VAMP header, ACM, reach map, manifest)
- [~] Still hard-coded: controller name `lift_arm_controller`, `/combined_arm_lift_controller/target_force_n`, `z_offset_m` 0.31, default group
      `lift_arm`, three legacy diagnostic nodes with a `joint1..6` chain
- [ ] VAMP / cuRobo robot models built from the profile at run time (today: the asset command, compile-time for VAMP)
- [ ] MoveIt swap verified for the Panda arm in docs (tests exist; no recorded evidence)

## 7. Base placement and docking

- [x] RM4D reachability map, builder, CLI; forward and inverse queries; lift bands (`reachability`, 56 tests)
- [x] `chooseStation` (free side of the wall, footprint clearance, yaw), `planStands`, `orderStands`, drive-type travel time
- [x] ROS service `~/plan_stands` with a base profile (`reachability_node`)
- [~] **Reachability -> manager**: verify-only stance check (`base_check:=warn|enforce`, `docs/reachability.md`); the manager still never moves the base or asks for a different stand
- [ ] Stand -> navigation goal (`NavigateToPose`, `NavigateThroughPoses`) with the arm-base vs `base_footprint` frame conversion. *Reuse: Nav2*
- [ ] Docking approach to the wall. *Reuse: Nav2 `opennav_docking` (Jazzy branch, v1.3.14) with a custom `NonChargingDock` plugin: `configure` / `cleanup` / `activate` / `deactivate`, `getStagingPose(pose, frame)`, `getRefinedPose(pose, id)` (where the plane detector overwrites the dock pose) and `isDocked()`*
- [ ] Wall-plane standoff and yaw servoing for the last metres from the fitted plane (about 1-2 weeks; no off-the-shelf package, published
      wall-painting robots do exactly this)
- [ ] **Global vs local placement** (decision 1). Global exists as an offline plan from static wall targets; local refinement from live data does not
- [ ] Better global cost: set cover over waypoint-to-stand feasibility with a TSP term for base travel; RM4D as prefilter, batched cuRobo IK to confirm
      (about 2 weeks); then whole-path base refinement from the papers, neither of which has code: arXiv 2603.07003 (inverse-reachability graph search + L-BFGS; a path follower with fixed yaw; the paper's CC BY-NC-SA licence means only the idea is reusable) and B* arXiv 2504.12719 (fixed-base arms), 2-3 weeks each
- [ ] Compare against ros-industrial `reach` (Apache-2.0, last push 2025-03) on the same walls as an independent check; `reuleaux` declares no licence, so do not reuse its code
- [ ] Orientation-constrained wall queries for contact work; yaw preference; a validation harness against brute-force 6D; a collision-aware `ArmModel`

## 8. Execution and control (JTAC, force)

- [x] JTAC (`compliant_trajectory_controller`): engage/disengage live, contact FSM with nine states and sequencer (44 `force_core` tests)
- [x] F/T path: raw sensor -> `ft_conditioner` (sign, bias, payload gravity, lever arm, calibration services) -> `ft_bridge_hardware` -> JTAC
- [x] Verified in MuJoCo: 10.13 N for a 10 N target (std 0.03); conditioner error vs ground truth under 0.005 N in both simulators
- [~] JTAC compliance offset accumulates on every axis while blocked and releases over about 3 s after the reference retreats (`docs/sim_mujoco.md`)
- [ ] **Force-CBF limiter plus offset clamp on JTAC** (arXiv 2405.04859 "Guarding Force": no code; a small QP that clamps the joint-rate output of a compliant controller using an estimated environment stiffness and rest position, tested on a UR3e; 1-2 weeks) and an energy tank around the slow release (about 1 week):
      fixes the windup/release finding and gives a hard force bound, the top priority for safe contact
- [ ] **Hardware parameter template**: gains, sensor name, source topic, payload and bias live in yaml comments marked "diagnostic"
- [ ] Payload identification with safe excitation trajectories and bounds (arXiv 2504.21486, about 1.5 weeks; its code, `roahmlab/OnlineSafeSysID`, has no licence file, so implement from the paper) and bias/temperature drift tracking (3-5 days);
      optionally replace the gravity step with the `control_toolbox` gravity-compensation filter (Apache-2.0; gravity and centre-of-gravity torque only, so sign, bias and the lever arm stay ours)
- [ ] Gain auto-tuning by Bayesian optimisation on the MuJoCo twin (cost: force RMS + overshoot + windup; method reference arXiv 2512.03772: SAASBO tuning of a torque-level MPC on a UR10e with a digital twin, not admittance gains, no code; 1-2 weeks)
- [ ] Production gains per tool and robot; a stability analysis for moving references (today an empirical 4x mass factor)
- [ ] Evaluate FZI `cartesian_controllers` (BSD-3: force, compliance controllers; CI covers Humble, Iron and Jazzy, last `ros2` commit 2024-10, so low activity) as an alternative to wrapping the admittance rule
- [ ] Inertial compensation, filtering; sign and frame of a real sensor checked on hardware
- [ ] End-to-end safety layer (abort thresholds exist per tool; nothing ties them to an e-stop path)
- [ ] Learned skills / residual policy (ACP-style outputs: pose, force target, stiffness, under the classical safety layer); deferred, design only

## 9. Mobile base execution

- [ ] Base motion in the simulators (wheels are fixed links; both sims hold the base static or welded). *First version: planar-velocity base
      (`mujoco_ros2_control` `base_velocity` plugin in MuJoCo, which needs the base body to have a freejoint, so our MJCF generator must emit one;
      a planar/velocity plugin in Gazebo), wheels later*
- [ ] `ros2_control` drive controllers per base profile. *Reuse: `diff_drive_controller`, `mecanum_drive_controller` (Apache-2.0; the mecanum one takes a `TwistStamped` on `<name>/reference` and one velocity command per wheel); the 4-wheel-steering
      Ranger needs a small custom controller or the vendor `ranger_ros2` driver (BSD-3)*
- [ ] Nav2 integration: costmap from the base footprint, goals from the stands, `odom`. *Reuse: Nav2 (Smac lattice/Hybrid-A*, MPPI omni/diff/Ackermann)*
- [ ] Localisation against the building and the wall. *Reuse: `robot_localization` (Apache-2.0); Nav2 AMCL and slam_toolbox are LGPL-2.1 (separate ROS nodes, licence review);
      KISS-ICP, KISS-SLAM, GLIM, PIN-SLAM (MIT) if a 3D lidar is added; avoid FAST-LIO2, FAST-LIVO2 and iG-LIO (GPL-2.0)*

## 10. Simulation (MuJoCo and Gazebo; Isaac Sim is a stub)

- [x] Backend contract and `hardware:=mujoco|gazebo` selection; both run headless with smoke tests (`docs/simulation.md`)
- [x] MuJoCo 4: own `SystemInterface`, MJCF and wall generators, raw + ground-truth F/T, 10 N contact control
- [x] Gazebo Harmonic: package, wall world, raw + ground-truth F/T, `FtBridgeSimSystem`, conditioned path validated
- [~] Gazebo does not hold a 10 N contact: JTAC gave about 108 N at two depths and the cause is not established (rigid contact is one hypothesis; `docs/sim_gazebo.md`). *Fix: a compliant tool tip (a spring-damper joint)
      or effort control; physical stiffness of 5-50 N/mm, physics 5-10x faster than the controller*
- [ ] Evaluate `mujoco_ros2_control` (Apache-2.0, v0.1.4; Humble, Jazzy, Kilted, Lyrical; plugins `base_velocity`, `camera`, `mujoco_3d_lidar`,
      `external_wrench`, `fts_grav_comp`, `free_joint_state_publisher`) against our own plugin. Its `mujoco_vendor` pins MuJoCo 3.12.0 and no MuJoCo 4 work was
      found, while ours builds against 4.0.1: try building it against 4.0.1, otherwise borrow its plugin designs
- [ ] MuJoCo SDK pinned in Docker (today a mounted tarball; the plugin is skipped without `MUJOCO_ROOT`)
- [ ] Calibrated soft contact and noise / bias / delay models in the sims (the near-zero conditioner error is the noise-free best case). MuJoCo direct
      `solref` (-stiffness, -damping): 10 N at 0.5 mm penetration is 20 kN/m (arithmetic from the docs, not yet run)
- [ ] MuJoCo Warp (Apache-2.0, active) for parallel gain sweeps (released with MuJoCo 3.5; small batches on a 4 GB GPU, not verified with MuJoCo 4); avoid Isaac Lab / Genesis on this machine
- [ ] Contact and smoke scripts run by `colcon test` or one documented command (today: scripts the author ran)
- [ ] Sim-vs-real differences document
- [ ] `sim_isaac`: stub only; deferred (Isaac Sim does not run on this machine)

## 11. The end-to-end gap (the main remaining work)

- [ ] **One launch per simulator**: sim + move_group + controllers + conditioner + coverage + manager, executing a wall pass
- [ ] **A scorecard** for that run: coverage achieved (from the verification raster), contact force RMS and peaks, collisions, time; automated, reproducible
- [ ] Manager interface: an action or service to start, stop and monitor a coverage task (it runs once from `main` today). *Reuse: BehaviorTree.CPP (MIT) for the
      task flow, the way Nav2 does; keep the force FSM*
- [ ] Scenario set: empty wall, wall with obstacles, panels needing several stands, each robot profile
- [ ] Failure handling in the loop: replan, re-stand, recovery, abort (the FSM aborts; nothing resequences)

## 12. Hardware readiness

- [~] UFACTORY path: `ft_bridge` default topic, JTAC yaml for `uf_ft_sensor`, a real-robot branch in the compliance demo; nothing verified on hardware
- [ ] Hardware bring-up guide and commissioning tests (slow first contact, sign checks, payload identification, stiffness check, limits)
- [ ] `hardware:=real` description branch and vendor-driver selection
- [ ] Calibration procedures in one place: payload (services exist), tool TCP, wall plane, base-to-wall

## 13. Open source and plug-and-play

- [x] Apache-2.0 licence; docs index; architecture, robot, reachability, simulation, F/T docs; Docker with nine targets
- [~] Root README: overview, workflow, package map, Docker build and test, simulators; no one-command quickstart (none exists)
- [ ] **Getting started**: one command to a running simulated wall pass
- [x] `THIRD_PARTY.md` (licences, use, nothing vendored); revisit as adopted software lands (a few licences are marked unverified there; cddlib GPL-2.0 in the VAMP stage: reviewed 2026-11-15, source and recipes only, no binaries shipped, see THIRD_PARTY.md)
- [~] CHANGELOG and CONTRIBUTING written; real package versions still all `0.0.0`
- [x] `docs/docker.md` covers the `sim` and `sim-gazebo` targets
- [ ] CI: none by decision; instead one documented "run all checks" command (`docker/build.sh test` + sim smoke tests)
- [ ] **Public-readiness cleanup**
  - [ ] 63 references in 41 files to 9 documents that are not in the repo (`force_control_backend_admittance_vs_diffik.md`,
        `implementation_plan_manipulation_architecture.md`, ...): strip them or distil into `docs/`
  - [~] Earlier-project base and application names are removed from code, READMEs and comments; "mop" as the default tool name remains until tool profiles ship (section 4)
  - [ ] Four launch files that look up packages that do not exist (`description`, `moveit_config`):
        `jtac_compliance_demo`, `jtac_controller_test`, `jtac_gazebo_controllers`, `manipulation_manager.launch.py`
  - [ ] `.claude/` is not in `.gitignore`
  - [ ] Decide about the maintainer's email, which is in 97 files (70 `@author` tags plus `package.xml` maintainer entries; their own address: keep or drop)
  - [ ] Final scan before making the repo public (names, paths, secrets); the visibility change itself is your command

## 14. Docs that no longer match the code

- [x] `architecture.md` status table (reachability, self filter, TRAC-IK, VAMP / cuRobo collision, sim execution; the pluginlib wording is left to the backend-selection work)
- [x] `reachability.md` "Not done"
- [x] `curobo_v2_notes.md` and `moveit_config.md` "Not done"
- [x] `scene_model.md` and `robot_self_filter.md` (the node is described; test counts 53 and 29)
- [x] `coverage_planner` README rewritten
- [x] `package_layout.md` pluginlib selection implemented (PR #29)

## 15. Tests and quality

- [x] 736 colcon tests in 26 packages, 0 failures; Python tests for description, profiles, sim generators, tools
- [ ] Tests for the coverage ROS nodes, more for JTAC (3 files) and the manager node
- [ ] Sim end-to-end checks as a test target (sections 10 and 11)
- [ ] Planner-level benchmark on shared scenarios (section 5)

## 16. The PR stack

#5 `feat/scene-adapters` -> main, then #6 bases, #7 base-aware stationing, #8 lifts, #9 asset command, #10 MuJoCo cross-check,
#11 multi-stand, #12 `~/plan_stands`, #13 simulation contract, #14 F/T conditioning, #15 MuJoCo backend, #16 arm profile, #17 arm profile
nodes, #18 Gazebo backend, #19 sphere benchmark, #20 `ft_conditioner`, #21 MuJoCo contact test, #22 Gazebo contact test. Pending: cleanup of
stale `CLAUDE.md` references (committed locally; push waits for GitHub), this roadmap. #5 merges cleanly into `main`; merging it first shrinks the chain.

## 17. Reuse map: adopt, implement from a paper, avoid

Licences, versions and last-push dates were read from each repository's own files on 2026-11-15 (second pass, after GitHub came back) unless marked
"unverified". Porting effort is for the target ROS 2 distro where it is not already there. "Paper" items have no mature code that we know of.

| Gap | Adopt (reuse) | Implement (paper / ours) | Avoid |
|---|---|---|---|
| Live ingest for planning | MoveIt `moveit_ros_perception` (BSD-3, v2.15.2): `pointcloud_octomap_updater` and `point_containment_filter` always build; the depth updater and `mesh_filter` shape mask need OpenGL. `octomap_server` (`ros2` branch) | | octomap_server2 (stale) |
| Live mapping | cuRobo v2 Mapper (Apache-2.0, GPU TSDF to ESDF; standalone use, ROS interface and 4 GB VRAM are unverified; ROS wrappers exist, e.g. `Lab-CORO/curobo_ros`, Apache-2.0); nvblox (**Apache-2.0**, CUDA, last push 2026-10; `isaac_ros_nvblox` Apache-2.0); CPU: VDBFusion (MIT, 2025-01), wavemap ROS 2 forks (BSD-3: `Forestry-Robotics-UC/wavemap2`, pushed 2026-11; upstream `ethz-asl/wavemap` is ROS 1 only and stale since 2024-12) | | Gaussian-splat maps (high risk); Isaac ROS container route unless NVIDIA's stack is wanted |
| Wall / plane detection | PCL multi-plane segmentation (BSD) behind our plane port | MAGSAC++ variants if RANSAC is not enough | learned plane transformers for now (ZeroPlane MIT, PLANA3R Apache-2.0, OVO MIT exist); Zoo3D (CC BY-NC) |
| Self filter | ours; `robot_body_filter` (BSD-3, `ros2` branch active since 2026-04 but not in the Jazzy or Kilted rosdistro: source build); `leggedrobotics/robot_self_filter` (BSD-3, ROS 2, 2026-10) | sphere-based filter with inflation margin (2-3 days) | learned filters |
| Odometry / SLAM / localisation | `robot_localization` (Apache-2.0); KISS-ICP, KISS-SLAM, GLIM, PIN-SLAM (all MIT; the last three need a 3D lidar and, for GLIM, an IMU is usual); Nav2 AMCL and slam_toolbox are LGPL-2.1, run as separate nodes after a licence review | | FAST-LIO2, FAST-LIVO2, iG-LIO (GPL-2.0); Cartographer (barely maintained) |
| Last metres to the wall | Nav2 `opennav_docking` (Apache-2.0, Jazzy branch v1.3.14) with a custom `NonChargingDock` plugin (the plane detector feeds `getRefinedPose`) | plane-distance and yaw servo (1-2 weeks) | |
| Navigation | Nav2 (Apache-2.0): Smac lattice / Hybrid-A*, MPPI (omni, diff, Ackermann), costmap footprint | stand -> goal bridge, footprint per base | |
| Base motion in sim | `mujoco_ros2_control` (Apache-2.0, v0.1.4: `base_velocity`, camera, 3D lidar; needs MuJoCo 3.12 through `mujoco_vendor`, MuJoCo 4 untested); ros2_controllers `diff_drive` and `mecanum_drive` (Apache-2.0); Ranger driver `ranger_ros2` (BSD-3) over `ugv_sdk` (Apache-2.0) | small 4-wheel-steering controller | |
| Planar coverage | ours; Fields2Cover (BSD-3) and `opennav_coverage` (Apache-2.0, `jazzy-v2` branch, 2D only) as references | cellular decomposition | |
| Mesh coverage | Noether (Apache-2.0, last commit 2026-01, tag 0.16.0, no GitHub release; `noether_ros2` separate, Apache-2.0, tag 0.2.0; `noether_tpp` is ROS-free; plane-slice and cross-hatch raster planners, mesh modifiers incl. RANSAC primitive fits, offset / approach / lead-in path modifiers). `industrial_reconstruction` (Apache-2.0, ROS 2, Open3D) builds a mesh from RGB-D plus TF; `scan_n_plan_workshop` (Apache-2.0, now under `ros-industrial-consortium`) is the reference pipeline | parameterise-then-sweep with adaptive spacing (geometry-central MIT or pmp-library MIT, 2-3 weeks; the DLR freeform-finishing paper, Schneyer et al., RA-L 2023, has no code); spray and standoff on top of Noether (line spacing from swath, offset modifier for standoff) | CGAL (GPL/LGPL); libigl unless per-file licences are checked (MPL-2.0 core, GPL modules); Tesseract / Descartes as a replacement (heavy; Tesseract 0.35.0 mixed Apache / BSD, Jazzy unconfirmed); RL coverage |
| Spray / non-contact | no open-source deposition model found (GitHub topic searches) | swath model + thickness raster + spacing optimisation (1-2 weeks) | |
| Coverage verification | | per-cell deposition/contact raster from executed poses (1-2 weeks) | |
| Base placement | RM4D (ours) as prefilter; `reach` (Apache-2.0) as a comparison | set cover + TSP over stands with batched IK; whole-path refinement from arXiv 2603.07003 and 2504.12719 (no code; the first paper's licence is CC BY-NC-SA) | `reuleaux` (no declared licence); learned world-model embodiment selection |
| Motion planning | cuRobo v2 (Apache-2.0; v0.9.0 2026-11-05); VAMP (Apache-2.0, v0.6.4); MoveIt Pilz (BSD-3, Jazzy branch) and MoveIt Task Constructor (BSD-3, last commit 2026-09) for approach / retreat sequences | | cuMotion (Isaac ROS); M3Bench (rearrangement only) |
| Time parameterisation | MoveIt TOTG and `moveit_ruckig_filter` (Ruckig MIT); toppra (MIT, Jazzy package unconfirmed); cuRobo TimeOpt | | |
| Backend selection | pluginlib | | a hand-rolled factory |
| Orchestration | BehaviorTree.CPP (MIT); alternatives SMACC2 (Apache-2.0), FlexBE (BSD-3), py_trees (BSD-3) | | LLM planners, TAMP (no benefit for a deterministic task) |
| Force control | ros2_controllers admittance (Apache-2.0, in use); `control_toolbox` gravity filter (Apache-2.0: gravity and CoG torque only); FZI `cartesian_controllers` (BSD-3, last `ros2` commit 2024-10) to evaluate; `force_feedback_mpc` (BSD-3) as a later option | force-CBF limiter + offset clamp (arXiv 2405.04859, no code); energy tank; excitation-based payload ID (arXiv 2504.21486; its code has no licence file); BO tuning (method reference arXiv 2512.03772, torque-level MPC) | learned F/T replacement; learned skills until the safety layer exists |
| Simulation | core MuJoCo 4.0.1 (Apache-2.0) with calibrated soft contact; MuJoCo Warp (Apache-2.0) for sweeps; Newton (Apache-2.0, active) to watch; Gazebo for integration | compliant tool tip for Gazebo | Genesis, Isaac Lab on a 4 GB GPU |

Corrected from the first draft: nvblox is Apache-2.0 (not NVIDIA terms); `nav2_amcl` is LGPL-2.1 and `robot_localization` Apache-2.0; wavemap upstream is stale
(last push 2024-12-30); the Noether swath/standoff awareness is ours to add; `mujoco_ros2_control` pins MuJoCo 3.12, not 4; arXiv 2604.01414 is a vision-torque
diffusion policy (not UMI-FT) and 2512.03772 tunes torque-level MPC (not impedance gains); the cuRoboV2-Fast paper is arXiv 2611.06913.

Still unverified: whether cuRobo's Mapper runs standalone and its VRAM on 4 GB; cuRobo support for a planar mobile base; Noether's mesh preconditions;
`mujoco_ros2_control` against MuJoCo 4.0.1; headless depth rendering in `mujoco-core`; the ROS 2 wavemap forks' quality; the toppra and Tesseract Jazzy
packages; the licences of paper code beyond those named; all effort figures (researcher estimates).

## 18. Ideas adopted from an earlier construction-robot stack

Ideas only: that stack is proprietary, so none of its code is reused. Each item was checked against this repo with grep (2026-11-15); items already done
here were dropped. Placement is the milestone that should carry it.

**Safety and execution (M1, M2; hardware wiring M6)**
- [ ] Fault abort in ros2_control: a `fault` state interface and typed codes; a lift or arm fault makes `read()/write()` return ERROR so every controller claiming the component deactivates; mark devices safety-critical or not (tool I/O is not); debounce about 3 cycles
- [ ] Never report a dead axis healthy: hold the last position, raise `fault`, return ERROR past a threshold (no "degraded mode")
- [ ] NaN and limit guard on every command interface; per-joint trajectory tolerance including the lift
- [ ] Spray valve forced off on any fault and confirmed by polling; drive-side watchdog and bring-up ordering (hardware only)
- [ ] **Tool actuation executor**: nothing executes `tool_command` or `tool_off_lead_s` today. Typed commands verified against echoed state, scheduled on trajectory time (a pause freezes the tool), tool off on every exit path, `TOOL_FAULT` / `TOOL_NOT_RESPONSIVE` results, on/off latency as profile data
- [ ] Segment-level intent in the motion layer: force stage, target force, standoff, normal, speed, tool command, lift height (unknown stage degrades to NONE)

**Manager and task flow (M1)**
- [ ] Manager action contract: goal kind plus params, one job at a time, per-app cancel; BehaviorTree.CPP with BehaviorTree.ROS2 action nodes, not hand-rolled clients
- [ ] Always-report tree convention (`Fallback(work, ForceFailure(ReportStatus))`), halt and skip guards, mission FSM (IDLE/ACTIVE/PAUSED/STOPPED/RECOVERY) with one typed command service
- [ ] Durable ledger keyed (mission, stand, segment) and resume; acceptance test: kill mid-wall, restart, only unfinished segments rerun
- [ ] Per-station failure policy: typed SKIPPED reasons, hardware-origin codes, no blind retry on a contact overload; station queue skips a blocked stand with a reason and slides it in small steps
- [ ] Controller switching only at segment boundaries

**Planning (M1, M2)**
- [ ] Time parameterisation by segment role: constant-speed jerk-limited for contact seams, time-optimal for transitions, split at 180-degree reversals, limit violations on a seam are failures
- [ ] Lift-aware planning: time metric max|dq|/vmax for ordering, strips by ascending lift height, in-band passes with the lift held, lift-travel KPI
- [ ] Rate-feasible-by-construction IK; failure-aware resequencing (forbidden arcs, bounded rounds, committed prefix frozen)
- [ ] Contact strips must plan to at least 98 percent or split (manager default `min_path_fraction` is 0.5 today)
- [x] **Weighted IK on the 7-DOF group**: prismatic joints weighted (1 m of lift about 6 rad) and TRAC-IK Speed for `lift_arm` (PR #5 fixes)

**Scorecard and verification (M2)**
- [ ] Numeric pass/fail thresholds per mode (for example any contact above 5 N fails a non-contact run; force, standoff, coverage, no controller switch under load)
- [ ] One verify-result contract (passed, coverage percent, failed segment ids, report path): v1 from execution telemetry, v2 from perception
- [ ] Post-work verification from perception: severity raster on the wall plane, 0-100 score, initial vs final pair, targeted re-work from the score map (ship end to end or not at all)
- [ ] Observability bundle: per-goal trajectory CSV, force trace, BT log, per-mission rosbag; test-only wrench-inject topic
- [ ] Golden exact-waypoint tests for the coverage planner before any algorithm change; table of mode-derived coverage parameters (the content of the shipped tool profiles); tool-profile loader invariants as hard errors

**Stationing (M3)**
- [ ] Re-perceive at the station and use the achieved pose (never the commanded one) as the work frame; log arrival error; reject degenerate planes (mean residual above 0.35 m); reach-derived station pitch and a tolerance-budget doc

**Simulation and tooling (M0, M2, M6)**
- [ ] Sim environment registry run by `colcon test`; launch readiness chaining instead of sleeps; ground-truth odometry toggle
- [ ] Local quality tooling without CI: `docker/build.sh check`, pre-commit config, `pyproject.toml`, pinned clang-format
- [ ] Container lessons: base / cacher / dev stages copying only `package.xml`, BuildKit secrets, large shm for DDS, no `-march=native` in distributed images
- [ ] Contributor guidance for users and coding agents (`AGENTS.md`, `docs/contributing/`)

**Pitfalls to cover with tests**
- [ ] Frozen lift feedback with a loose tolerance must still abort; a tool-I/O fault must not
- [ ] Mixed-unit jump detection (0.06 m of lift vs 0.2 rad of arm), including against the start state
- [ ] Mimic and stage joints reach `/joint_states`; every runtime velocity limit equals the profile value
- [ ] Dead config: a file never loaded, padding at the wrong namespace, `allowed_start_tolerance` handling (not set anywhere in `src` today)
- [ ] Redundant-group pose goals are random-goal samplers; a failed IK call can leave a partial result
- [ ] Retimer hazards: reversals, near-duplicate waypoints, mixed joint types, zero acceleration limit give a valid trajectory or a typed error
- [ ] Force-offset leakage: the tracking-error threshold must budget the admittance offset; offset zero when force is off; settle threshold above the sensor deadband
- [ ] NaN or out-of-range commands clamped on the real write path; navigation arrival error of 5-9 cm and 3 degrees must not corrupt the work frame

Not adopted: the supervisor launch system, a site database with authoring GUI, a QP DiffIK controller (JTAC covers it), learned dirt detection, wrist hand-eye
calibration, Isaac bring-up (stub here), vendor-specific tool bridges.

---

## Proposed order (each milestone has an exit test)

- **M0 Housekeeping** (days): push the cleanup, `.gitignore`, fix or delete the four broken launch files, reconcile section 14, strip the dangling
  references and earlier-project names. *Exit:* the section 13 scan finds nothing; docs match the code.
- **M1 Pipeline core, by reuse**: pluginlib backend selection, BehaviorTree.CPP task flow behind a manager action, ship tool profiles and wire them to
  coverage, reachability `plan_stands` called by the manager, ROS coverage source. *Exit:* swap MoveIt / cuRobo / VAMP and the tool by config, in a test.
- **M2 First full run in sim, base static** (what you can start testing with): one launch per simulator, a scenario set, the scorecard **with the coverage
  verification raster**, plus the force-CBF limiter and a compliant tip for Gazebo. MuJoCo first. *Exit:* an automated wall pass in MuJoCo with the scorecard
  recorded on two robot profiles; Gazebo holds 10 N.
- **M3 Base motion and docking, by reuse**: planar-velocity bases, then ros2_control drive controllers; Nav2 with stands as goals; `opennav_docking` with a
  wall-dock plugin; global stand selection by set cover + TSP; then local refinement. *Exit:* the base drives to a stand in sim and the pass succeeds from where it stopped.
- **M4 Live perception, by reuse**: MoveIt octomap + `octomap_server`, PCL wall detection, our self filter on the live cloud, simulated depth/lidar, then a
  mapper (cuRobo Mapper or wavemap) and AMCL / KISS-ICP localisation. *Exit:* M2's scenario runs from a sensed wall, not a file.
- **M5 Tools and surfaces**: spray model and the non-contact path; floors and ceilings as planar patches; Noether for meshes. *Exit:* a spray pass over a wall
  and a ceiling in sim with the verification raster showing even coverage.
- **M6 Hardware and release**: hardware template and bring-up guide, payload identification with safe excitation, BO gain tuning, third-party notices,
  quickstart, changelog, final scan, release.

## Decisions I need from you

1. **Local base placement**: a lightweight verify-and-nudge at each stand (re-check reachability on the live scan, shift within a small neighbourhood) or a
   full local optimiser? I recommend the lightweight one first; global placement exists.
2. **Backend selection**: pluginlib, as the docs prescribe and as you asked for reuse, rather than my earlier idea of a hand-rolled factory. (I now recommend pluginlib.)
3. **Gazebo contact**: add a compliant tool tip so Gazebo can hold 10 N, or accept MuJoCo as the contact simulator and use Gazebo for integration only?
4. **MuJoCo plugin**: `mujoco_ros2_control` pins MuJoCo 3.12 (no MuJoCo 4 support found), ours builds against 4.0.1. Try building the upstream plugins (base velocity,
   camera, 3D lidar) against 4.0.1 and keep our own `SystemInterface` for the joints and F/T if that fails? I recommend trying it for base motion and sensors first.
5. **Base motion**: planar-velocity bases first (recommended), real wheel physics later?
6. **Tools to ship first**: mop (contact) and spray gun (non-contact), anything else?
7. **JTAC**: add the force-CBF limiter with an offset clamp (a controller change) instead of only documenting the slow release? I recommend yes.
