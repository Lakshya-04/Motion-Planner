# Roadmap and status

The target: open-source, plug-and-play, **any mobile manipulator** (any arm, lift, base) covering **any
surface** with **any tool** (contact like a mop, non-contact like a spray gun), from sensor data to executed
tool motion, run in **MuJoCo and Gazebo** now (Isaac Sim is a stub) and on hardware with parameters only.

```
sensors -> perception -> self filter -> scene -> surface -> coverage (tool-aware) -> base stands
        -> navigation / docking -> motion planning -> execution (JTAC + F/T) -> monitoring
```

Last verified: 2026-11-17 on main at 56712bb (PRs up to #94): simulator-free checks green (`docker/run_all_checks.sh --skip-sims`: 54 packages built, 1259 colcon tests, 0 failures, 229 skipped; 5 pytest directories passed; public-readiness scan clean). Simulator, GPU and hardware items below were not re-run for that date; their evidence is the earlier run each line cites. Legend: `[x]` done, `[~]` partial (what is
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
- [~] **Live ingest**: PointCloud2 -> MoveIt octomap through MoveIt's `PointCloudOctomapUpdater` (`live_octomap:=true`, `config/sensors_3d.yaml`; `run_live_octomap.sh` in the MuJoCo sim: wall in the planning scene's octomap from the live cloud, 13.7 kB octomap at 3 cm, a collision-checked Cartesian path through the wall stops at fraction 0.62, 1.00 unchecked). Earlier: PointCloud2 -> wall plane -> `wall_scene.json` + a MoveIt collision box (`wall_detector`, `docs/perception_live.md`), checked in the MuJoCo sim. Not done: PointCloud2 -> `SceneModel` update, octomap for MoveIt, obstacles, scan merging. *Reuse: MoveIt's `pointcloud_octomap_updater` and `point_containment_filter` (always built; BSD-3), `octomap_server` (`ros2` branch); MoveIt's depth updater and mesh-filter shape mask need OpenGL*
- [~] Simulated sensors (depth camera, lidar) in MuJoCo and Gazebo. MuJoCo: ray-cast depth in the plugin (`depth_sensor:=true`, `mj_multiRay`, PointCloud2 + TF, optional range noise, scene geoms only; `docs/perception_live.md`). Gazebo: `depth_sensor:=true` (gz `depth_camera` through `ros_gz_bridge`, rendered headless in software, `/depth_sensor/points`; `run_depth_wall.sh` fits the wall within 0.01 degrees / 0.1 mm through `wall_detector`; `docs/sim_gazebo.md`). RGB: `rgb_camera:=true` (gz `camera`, `/rgb_camera/image`, `capture_still.py`, stills in `docs/media`). Not done: lidar model, a Gazebo cloud without the robot's own links filtered. *Reuse: Gazebo `depth_camera` / `rgbd_camera` / `gpu_lidar` through `ros_gz_bridge` (Apache-2.0); MuJoCo: `mujoco_ros2_control` camera and `mujoco_3d_lidar` plugins, or `MuJoCo-LiDAR` (MIT); depth rendering in the headless `mujoco-core` wheel is unconfirmed, test it*
- [~] Wall detection from a live scan (drives the sensed manager pass, `run_sensed_pass.sh`, `docs/sim_mujoco.md`): `wall_detector_node` (scene_model RANSAC + PCA, TF to the target frame, writes `wall_scene.json` + `walls.pcd`, 7 tests; sim check: normal 0.012 deg, plane 0.13 mm, coverage planner accepts it). One plane per scan, no obstacles, not PCL; `planCoverageFromScan` is still not called by a node. *Reuse: PCL multi-plane segmentation behind our plane port*
- [ ] External data: prior map / BIM / merging scans (nothing beyond the nvblox design note)
- [ ] Live mapping (TSDF/ESDF): *reuse cuRobo's Mapper or nvblox (GPU, both Apache-2.0); CPU: VDBFusion (MIT) or a ROS 2 fork of wavemap (upstream stale)*; design in `docs/nvblox_notes.md`; cuRobo's Mapper measured standalone on the 4 GB GPU on synthetic depth (0.2-0.3 GB at 4 cm, ESDF into `set_esdf` with a conversion, `docs/curobo_v2_notes.md`), not on real depth, not in a ROS node, not together with a planner
- [~] Live ESDF to the planners: `octomap_esdf_node` (wall_detector) turns the planning-scene octomap into `scene_model` `computeEsdf` on a 1.6 m window, published on `~/esdf` + `~/esdf_info` in the `SetEsdf` layout; checked in the sim (wall voxel at x 0.975, 0.30 and 0.60 m in front read 0.300 / 0.600, 32^3 at 5 cm in 41-56 ms). Not done: nothing feeds it to cuRobo's `set_esdf` yet, fixed window, no unknown-space handling
- [ ] Doors, windows, openings (only as generic obstacles today)

## 2. Coverage planning (planar surfaces)

- [x] Grid planners: boustrophedon, sweepline (`coverage_planner/core`)
- [x] Wall patterns: vertical, horizontal, zigzag, with panels (one base stand per panel) and a bottom pass
- [x] `planCoverageFromScan`: plane fit, rasterise, plan, lift with a tool profile per part (library; 69 tests in the package)
- [x] Obstacles projected onto the wall and inflated per tool (`wall_rasterizer.cpp`)
- [~] ROS nodes: `coverage_planner_node` (OccupancyGrid -> path), `wall_coverage_service_node` (from `wall_scene.json` + PCD), demo node;
      no action interface, the `CoveragePlan` topic has no subscriber in the repo, nodes have only a conversions test
- [~] Spiral is listed in the service but is a stub (the README now says so)
- [x] All free components planned and ordered (rib = two panels; node default `plan_all_components` was false), short gap strokes beside obstacle keep-outs (`fill_gaps`, `min_stroke_length_m`), optional cell decomposition `decomposition: cells` (`docs/coverage_patterns.md`; gtests on synthetic rasters, covered fraction above 0.95; no sim rerun yet; exact cell ordering by a router not done)
- [ ] Coverage feed over ROS (the manager reads `coverage_output.json`; a ROS `CoverageSource` is planned)
- [~] **Coverage verification**: `CoverageVerifier` raster (covered fraction, overlap, overspray, missed cells; `docs/coverage_patterns.md`). Scores recorded runs through `score_run` (contact and, with the valve column, spray: section 4)

## 3. Non-planar surfaces (floors, ceilings, columns, curved walls, meshes)

- [~] `SurfaceModel` interface (project, lift with normal and tangent frame, contains, obstacles): planar implementation `SurfacePatch` in `coverage_planner` core; no common abstract interface yet, no mesh implementation
- [~] Piecewise-planar rooms: one planar patch per face (floor, ceiling, columns) reusing today's planners (`SurfacePatch`, `planPatchCoverage`; gtests on floor, ceiling, column face). Not done: stand selection and reach for horizontal surfaces, a room-level ordering of patches, any sim or hardware run
- [ ] Mesh / freeform surfaces. *Reuse: Noether (Apache-2.0; ROS-free `noether_tpp` core; plane-slice raster planners with `line_spacing`; offset and approach/departure path modifiers; robot-independent output of poses with normals; no swath or standoff awareness: swath maps to line spacing, standoff to the offset modifier); else parameterise-then-sweep (geometry-central or pmp-library, MIT)*
- [~] Lift the vertical-wall assumptions: the coverage chain (`planCoverageOnWall`) and the lift work for any plane orientation; still open: `chooseStation` rejects a horizontal plane; `loadWallScene` enforces wall flatness
- [ ] Learned / non-grid coverage planners as further implementations of the same port (design only; recent RL work is not ready for a deterministic task)

## 4. Tools (contact and non-contact)

- [x] `ToolProfile` with geometry, task, coverage and control facets, and a YAML loader (`core_types`, `ros_adapters`)
- [x] Coverage uses swath, overlap, inflation, standoff, yaw, tilt, contact flag (`ToolCoverageProfile`)
- [x] Tool offset from the arm TCP: kinematics via `tcp_xyz/rpy` in the profile; force via the conditioner's lever arm
- [x] **Ship tool profiles**: `coverage_planner/config/tool_profiles.yaml` ships `contact_wiper` and `spray_gun` (`docs/tools.md`)
- [x] The YAML loader is wired into the coverage launch files and the demo node (`tool_profiles_file`, `tool_name`, per-field overrides; the manager pass uses it)
- [~] Spray model: parametric swath `w(d) = f * 2 d tan(theta)` derived from the standoff (`sprayerEffectiveSwath`, used by `toolSwath`),
      Gaussian deposition footprint (`spray_deposition.hpp`) and a thickness raster in `CoverageVerifier::addSprayStroke` that counts
      only valve-open samples and reports thickness mean / CoV / min / max next to the covered fraction (`score_run --no-contact
      --spray-standoff`). Not done: spacing and standoff chosen by minimising thickness variance (about 16-18% overlap is a published
      optimum for an elliptical pattern), overspray at strip ends and around obstacles, speed/pressure/wall-angle terms; the width and
      the Gaussian shape are guesses until fitted to a real nozzle
- [~] Non-contact segments end to end: `manipulation_manager contact:=false` docks, moves to the first pose at the plan's standoff,
      opens the valve (`std_msgs/Bool` on `/tool/valve`) for the stroke only, closes it, retracts; no force engage, no press, no wall-contact
      allowance. Proven in the MuJoCo sim (`run_spray_pass.sh`: 3 columns, valve recorded and scored). Not done: a valve driver on real
      hardware, `tool_off_lead_s` / `tool_on_at_speed_ratio` timing, valve forced off on a fault, the standoff held by feedback (it is open loop)
- [~] Contact tools: adaptive line spacing from a contact-area model. Done: `coverage.contact_area` (width from force, tilt, curvature) drives `toolSwath` / `toolLineSpacing`; `contactStripOffsets` with edge zones; verified on a `CoverageVerifier` raster (`test_contact_spacing`). Not done: the boustrophedon/sweepline planners still use one uniform step, so edge spacing is not applied to a generated path; no hardware fit of the gain (`docs/tools.md`)
- [x] "How to add a tool" guide (`docs/tools.md`)

## 5. Motion planning (any backend, contact and non-contact)

- [x] Ports `Planner`, `IkPort`, `CollisionPort`, `ScenePort` (`motion_planner/core`)
- [x] Cartesian planner: jump detection, reachability-tilt retry, wall-penetration check (`motion_planner_moveit`)
- [x] OMPL free-space planner through MoveIt; routing Cartesian first then OMPL (`MotionPlanner::planSegment`)
- [x] Contact / non-contact: ACM exclusion scoped to the wall octomap; approach, contact and retreat as separate segments
- [x] IK: KDL, trac_ik (default), pick_ik (opt-in, untuned), `SeedPreferringIk`, cuRobo IK
- [x] cuRobo v2 backend (plan, IK, scene, ESDF) and VAMP backend (joint-goal plan, collision, scene)
- [x] **Runtime backend selection**: pluginlib plugins for planner, IK, collision and scene (MoveIt, cuRobo, VAMP); the manager builds them from a `planning_config` YAML (PR #29)
- [~] VAMP and cuRobo used by the manager for free-space transitions (the architecture example: IK cuRobo, free-space VAMP): routing from `free_space: vamp|curobo` configs is tested (`manipulation_manager/test/backend_routing_test.cpp`, fakes where the real plugin is not there); not run in the manager on a robot or in the sim, and VAMP still needs an IK step in front of it
- [~] VAMP: no pose goal, no IK, robots compiled in, not a contact planner
- [x] VAMP UR10e model accepts the benchmark endpoints (carriage plate spheres refined, `--lift_carriage_link=4` default in `generate_vamp_robot.sh`): 158/158 on the empty / wall / wall_box set, was 10/158 (`docs/planner_benchmark.md`)
- [~] cuRobo: pose-based collision unsupported, no wall-contact equivalent, one planner per process
- [~] Joint-space goals honoured by OMPL only
- [~] Time parameterisation: `TimeParameterizer` port and role-based `parameterizeSegment` in `motion_planner` core (near-duplicate drop, split at reversals above 150 degrees, limit violations reported), MoveIt adapter `MoveItTimeParameterizer` (TOTG for transitions, constant tool speed plus Ruckig smoothing for seams), opt-in in the manager as `retime:=true` (`RetimingExecutionPort`); gtests only, not run in the sim; the planners' own timing is unchanged and toppra / cuRobo TimeOpt are not wired. *Reuse: MoveIt TOTG and its Ruckig filter first (`moveit_core` links Ruckig, MIT); toppra (MIT) for coverage passes; cuRobo v2 has a TimeOpt module*
- [~] Segment ordering by a time metric: `column_order: nearest | nearest_2opt` orders the columns of a stand by travel time (`core/column_ordering.hpp`, gtests); default is the input order, not run in the sim. Failure-aware resequencing (replan a failed column from another stand) is not done
- [ ] cuRobo v2 for moves between stands: v0.8.0 (April 2026, the open-source cuRoboV2) and v0.9.0 (2026-11-05, cuRoboV2-Fast: faster planning, Cartesian waypoint planning with moving starts, C++ motion planner, TimeOpt); pin v0.9.0 or later, not v0.7.8; mobile-base support is unverified (the Mapper's VRAM on 4 GB is measured, see `docs/curobo_v2_notes.md`)
- [ ] Recovery proposer (design only: `docs/learned_recovery.md`) and the policy slot
- [~] Shared planner benchmark across backends (success, time, path, clearance): `motion_planner_moveit/scripts/planner_benchmark.py`, joint-space transitions between stand, stow and dock-like UR10e configurations, results in `docs/planner_benchmark.md`; cuRobo did not run (see the doc), pose-goal and contact planning are not benchmarked

## 6. Robot generality (any arm, lift, base)

- [x] Profiles as the single source of truth (13 profiles), Python and C++ loaders, profile-driven group and joint names
- [x] Arms: ur10e, generic_6dof, 7-DOF Panda-like; N-DOF contract
- [x] Lifts: telescopic, scissor, fixed, none; recipe for a column lift
- [x] Bases: ranger_mini, scout, scout_mini, husky, mpo_500, mir100, two turtlebot3, generic (omni / diff / skid / 4ws)
- [x] One command from a profile to every per-robot asset (spheres, cuRobo config, VAMP header, ACM, reach map, manifest)
- [~] Still hard-coded (the manager takes `controller_name` now): controller name `lift_arm_controller`, `/combined_arm_lift_controller/target_force_n`, `z_offset_m` 0.31, default group
      `lift_arm`, three legacy diagnostic nodes with a `joint1..6` chain
- [ ] VAMP / cuRobo robot models built from the profile at run time (today: the asset command, compile-time for VAMP)
- [ ] MoveIt swap verified for the Panda arm in docs (tests exist; no recorded evidence)

## 7. Base placement and docking

- [x] RM4D reachability map, builder, CLI; forward and inverse queries; lift bands (`reachability`, 56 tests)
- [x] `chooseStation` (free side of the wall, footprint clearance, yaw), `planStands`, `orderStands`, drive-type travel time
- [x] ROS service `~/plan_stands` with a base profile (`reachability_node`)
- [~] **Reachability -> manager**: stance check (`base_check:=warn|enforce`, `docs/reachability.md`, also in the world frame); `scripts/run_stand_pass.sh` (MuJoCo) plans stands, drives to stand 0 and runs the manager pass from there with `world_frame: odom` (columns transformed into `base_footprint` from TF; `docs/stand_navigation.md`): scorer passes, coverage 0.931, peak 18.8 N, RMS 3.31 N, stand 0.51 m and 90 degrees from the start, one run. `run_multi_stand_pass.sh` runs a 0.9 m wall from two stands in a loop (below). Not done: the manager asking for a different stand on its own when one fails
- [~] Stand -> navigation goal with the arm-base vs `base_footprint` frame conversion: `stand_navigator` (`NavigateToStand` action, `NavigateToPose`-shaped; ROS-free core, unit-tested; `docs/stand_navigation.md`).
      Done in MuJoCo: `plan_stands` -> first stand -> arrival within 1 cm. Not done: `NavigateThroughPoses`, obstacle avoidance, a Nav2 bridge. *Reuse: Nav2 (not installed in any image; the node is the stand-in)*
- [~] Docking approach to the wall (the last-metres servo above covers the final standoff and yaw only). *Reuse: Nav2 `opennav_docking` (Jazzy branch, v1.3.14) with a custom `NonChargingDock` plugin: `configure` / `cleanup` / `activate` / `deactivate`, `getStagingPose(pose, frame)`, `getRefinedPose(pose, id)` (where the plane detector overwrites the dock pose) and `isDocked()`*
- [~] Wall-plane standoff and yaw servoing for the last metres from the fitted plane: `stand_navigator` `wall_docking` (ROS-free core, 21 tests: convergence on synthetic walls, limits, diff turn-first / no strafe, omni) and `wall_docking_node` (`WallDock` action, typed result), `run_wall_docking.sh` (MuJoCo, 0.4 m and 15 degrees off: 4.8-5.2 mm and 0.01 degrees, 3 runs; `docs/stand_navigation.md`). Not done: lateral position along the wall, obstacles in front, diff drive in the sim, hardware, Nav2 `opennav_docking` integration
- [ ] **Global vs local placement** (decision 1). Global exists as an offline plan from static wall targets; local refinement from live data does not
- [~] Better global cost: `StandStrategy::SetCover` (`planStands`, `stand_strategy:=set_cover` on `reachability_node`, `rm4d_tool --strategy set_cover`): station-to-target feasibility from the RM4D map, weighted greedy cover with a drive-type travel term, bounded exact search for fewer stands, nearest-next + 2-opt order; on a designed wall 2 stands / 4.2 s against greedy 3 / 7.0 s, never worse on the other test walls (synthetic, `docs/reachability.md`). Default stays greedy. Not done: batched cuRobo IK confirmation, then whole-path base refinement from the papers, neither of which has code: arXiv 2603.07003 (inverse-reachability graph search + L-BFGS; a path follower with fixed yaw; the paper's CC BY-NC-SA licence means only the idea is reusable) and B* arXiv 2504.12719 (fixed-base arms), 2-3 weeks each
- [ ] Compare against ros-industrial `reach` (Apache-2.0, last push 2025-03) on the same walls as an independent check; `reuleaux` declares no licence, so do not reuse its code
- [ ] Orientation-constrained wall queries for contact work; yaw preference; a validation harness against brute-force 6D; a collision-aware `ArmModel`

## 8. Execution and control (JTAC, force)

- [x] JTAC (`compliant_trajectory_controller`): engage/disengage live, contact FSM with nine states and sequencer (44 `force_core` tests)
- [x] F/T path: raw sensor -> `ft_conditioner` (sign, bias, payload gravity, lever arm, calibration services) -> `ft_bridge_hardware` -> JTAC
- [x] Verified in MuJoCo: 10.13 N for a 10 N target (std 0.03); conditioner error vs ground truth under 0.005 N in both simulators
- [~] JTAC compliance offset accumulates on every axis while blocked and releases over about 3 s after the reference retreats (`docs/sim_mujoco.md`)
- [~] **Force-CBF limiter on JTAC** (arXiv 2405.04859 idea, no code used): opt-in `force_limiter.enabled` in JTAC runs `force_core::NormalAxisLimiter` (offset cap, barrier `alpha (F_max - F) / k` with the stiffness identified from the commanded position, back-calculation anti-windup, admittance reset on retreat). Spring-wall tests at 100 Hz keep the peak at 15 N against 30 to 488 N without it on a step, a ramp and a noisy ramp (stiff and soft arm), and a 10 N hold is unchanged (`docs/ft_sensing.md`). MuJoCo contact check (10 N target, limit 11 N): hold 10.11 N and release unchanged, but the ground-truth peak is 12.27 N with and without it (the stiffness estimate stays at the prior on the soft arm, so the barrier retreats at about 0.14 N/s; tune `k_max_n_per_m` near the arm stiffness, not run). Not done: that tuned run, the first-contact impulse `k v dt` is only bounded by the approach speed (26 N peak at 5 cm/s on a 20 kN/m wall, so the 100 Hz problem of #41 is not solved for fast approaches), no energy tank
- [~] **Hardware parameter template**: `compliant_trajectory_controller/config/hardware_template.yaml` lists every value to set (conditioner, bridge, JTAC, tool profile) with safe defaults marked; documented only, no launch file loads it yet and nothing is tested on hardware
- [~] Payload identification with safe excitation trajectories and bounds (arXiv 2504.21486; its code, `roahmlab/OnlineSafeSysID`, has no licence file, so implemented from the paper) and bias/temperature drift tracking: ROS-free `force_core` cores with simulated-noise gtests (`identifyPayload` condition-number check, `generateExcitationPoses`, `BiasTracker`, `docs/calibration.md` sections 3 and 3b). Wired into `ft_conditioner` (opt-in `bias_tracking.enabled`: tracked bias topic, optional apply, `recalibrate_needed`, reset service; `~/plan_excitation` plans from joint-space parameters; node tests with a fake wrench stream). Not done: executing the plan on the arm, a publisher of the `contact_free` flag, any hardware run, the paper's online set-based safety;
      optionally replace the gravity step with the `control_toolbox` gravity-compensation filter (Apache-2.0; gravity and centre-of-gravity torque only, so sign, bias and the lever arm stay ours)
- [ ] Gain auto-tuning by Bayesian optimisation on the MuJoCo twin (cost: force RMS + overshoot + windup; method reference arXiv 2512.03772: SAASBO tuning of a torque-level MPC on a UR10e with a digital twin, not admittance gains, no code; 1-2 weeks)
- [ ] Production gains per tool and robot; a stability analysis for moving references (today an empirical 4x mass factor)
- [ ] Evaluate FZI `cartesian_controllers` (BSD-3: force, compliance controllers; CI covers Humble, Iron and Jazzy, last `ros2` commit 2024-10, so low activity) as an alternative to wrapping the admittance rule
- [ ] Inertial compensation, filtering; sign and frame of a real sensor checked on hardware
- [ ] End-to-end safety layer (abort thresholds exist per tool; nothing ties them to an e-stop path)
- [ ] Learned skills / residual policy (ACP-style outputs: pose, force target, stiffness, under the classical safety layer); deferred, design only

## 9. Mobile base execution

- [~] Base motion in the simulators (wheels are fixed links; both sims hold the base static or welded). *MuJoCo done as the first version:
      `base_motion:=true` adds a planar-velocity base (x, y, yaw joints with stiff position hold) driven by `/cmd_vel`, `omni` or `diff`
      per profile `drive_type`, with `/odom` and TF `odom -> base_footprint`; arm, lift and F/T unaffected, tested with
      `scripts/run_base_motion.sh` (`docs/sim_mujoco.md`). Implemented in our own plugin, not `mujoco_ros2_control` (freejoint, MuJoCo 3.12
      pin). Gazebo done the same way with existing gz-sim systems (`base_motion:=true`: `VelocityControl` + `OdometryPublisher` through `ros_gz_bridge`, free base on a low-friction skid,
      `/cmd_vel`, `/odom`, TF `odom -> base_footprint`; `scripts/run_base_motion.sh`: 0.294 m of 0.3 m, arm and F/T unchanged; `docs/sim_gazebo.md`). Not done: wheel physics and floor contact, 4-wheel-steering kinematics, odom noise, a
      command timeout and velocity limits in Gazebo, a Gazebo wall contact or manager pass with a free base*
- [ ] `ros2_control` drive controllers per base profile. *Reuse: `diff_drive_controller`, `mecanum_drive_controller` (Apache-2.0; the mecanum one takes a `TwistStamped` on `<name>/reference` and one velocity command per wheel); the 4-wheel-steering
      Ranger needs a small custom controller or the vendor `ranger_ros2` driver (BSD-3)*
- [~] Nav2 integration: costmap from the base footprint, goals from the stands, `odom`. Goals from the stands and `odom` are done without Nav2 (`stand_navigator`, a straight-line proportional controller with
      velocity and acceleration limits per drive type, no costmap, no planning around obstacles); Nav2 (`nav2_msgs`, `nav2_bringup`) is not in the docker images and was not added. Swap path in `docs/stand_navigation.md`. *Reuse: Nav2 (Smac lattice/Hybrid-A*, MPPI omni/diff/Ackermann)*
- [ ] Localisation against the building and the wall. *Reuse: `robot_localization` (Apache-2.0); Nav2 AMCL and slam_toolbox are LGPL-2.1 (separate ROS nodes, licence review);
      KISS-ICP, KISS-SLAM, GLIM, PIN-SLAM (MIT) if a 3D lidar is added; avoid FAST-LIO2, FAST-LIVO2 and iG-LIO (GPL-2.0)*

## 10. Simulation (MuJoCo and Gazebo; Isaac Sim is a stub)

- [x] Backend contract and `hardware:=mujoco|gazebo` selection; both run headless with smoke tests (`docs/simulation.md`)
- [x] MuJoCo 4: own `SystemInterface`, MJCF and wall generators, raw + ground-truth F/T, 10 N contact control
- [x] Gazebo Harmonic: package, wall world, raw + ground-truth F/T, `FtBridgeSimSystem`, conditioned path validated
- [x] Gazebo holds a 10 N contact with a compliant tool tip (prismatic spring-damper joint, 5 N/mm, `docs/sim_gazebo.md`): mean 9.69 N, std 0.00 N over the last 4 s, was 163.9 N, std 120.6 N (rigid tool). Not yet: the 35 mm overpress gives about 106 N, the residual of 0.3 to 1.6 N depends on the reference depth, MuJoCo still holds 10.13 N.
- [ ] Evaluate `mujoco_ros2_control` (Apache-2.0, v0.1.4; Humble, Jazzy, Kilted, Lyrical; plugins `base_velocity`, `camera`, `mujoco_3d_lidar`,
      `external_wrench`, `fts_grav_comp`, `free_joint_state_publisher`) against our own plugin. Its `mujoco_vendor` pins MuJoCo 3.12.0 and no MuJoCo 4 work was
      found, while ours builds against 4.0.1: try building it against 4.0.1, otherwise borrow its plugin designs
- [ ] MuJoCo SDK pinned in Docker (today a mounted tarball; the plugin is skipped without `MUJOCO_ROOT`)
- [~] Calibrated soft contact and noise / bias / delay models in the sims (the near-zero conditioner error is the noise-free best case). Done: MuJoCo `sim_noise` (opt-in F/T white noise, bias random walk, low-pass, delay, quantisation, joint-state noise, command latency; `signal_model.hpp`, gtests, noisy contact run in `docs/sim_vs_real.md`: 10.18 N held for a 10 N target with density 0.02). Not done: calibration of any parameter against a real sensor, Gazebo equivalent, soft contact calibration. MuJoCo direct
      `solref` (-stiffness, -damping): 10 N at 0.5 mm penetration is 20 kN/m (arithmetic from the docs, not yet run)
- [ ] MuJoCo Warp (Apache-2.0, active) for parallel gain sweeps (released with MuJoCo 3.5; small batches on a 4 GB GPU, not verified with MuJoCo 4); avoid Isaac Lab / Genesis on this machine
- [ ] Contact and smoke scripts run by `colcon test` or one documented command (today: scripts the author ran)
- [x] Sim-vs-real differences document (`docs/sim_vs_real.md`; a table, not a measurement against hardware)
- [ ] `sim_isaac`: stub only; deferred (Isaac Sim does not run on this machine)

## 11. The end-to-end gap (the main remaining work)

- [~] **One launch per simulator**: MuJoCo has `run_wall_pass.sh` (scripted strokes planned by MoveIt, recorded and scored) and `run_manager_pass.sh` (coverage planner + manager + MoveIt + JTAC: both columns dock, press, stroke and retract; scorer passes with coverage 0.93, peak 20.5 N, RMS 3.6 N, in-band 0.48; `docs/sim_mujoco.md`). Gazebo has `run_manager_pass.sh` too (UR10e, three columns dock, press, stroke and retract; scorer passes with coverage 0.93, peak 16.8 N, RMS 2.1 N, in-band 0.85; needs `jtac_position_only`, `docs/sim_gazebo.md`); there is no scripted Gazebo wall pass, and force along the manager strokes is still looser than the MuJoCo scripted pass (RMS 1.5 N, 81 percent in band)
- [~] **A scorecard** for that run: `scoreRun` / `score_run` compute coverage, force RMS and peak, in-band fraction and time with pass/fail limits (`docs/coverage_patterns.md`); `run_contact.sh --record` feeds it a MuJoCo contact run (`docs/sim_mujoco.md`); `run_manager_pass.sh` records and scores a full manager pass; obstacle contacts are counted (tool point only); `run_stand_pass.sh` adds the base move (plan stands, drive, pass from the stand, scored in odom)
- [~] Manager interface: `manipulation_manager` serves the `execute_coverage` action (`coverage_task_msgs/ExecuteCoverage`: goal path / `multi_stand` / tool mode / column subset, feedback state + stand + column + counts + peak force, result per column and per stand) with cancel (trajectory and navigation cancelled, admittance off, force target 0, valve closed); `run_on_start:=true` (default) keeps the old one-shot run (`run_manager_pass.sh` unchanged: coverage 0.931, peak 19.97 N, RMS 3.61 N). Task flow is a small tested state machine (`core/coverage_task.hpp`), not BehaviorTree.CPP (swap path in `docs/stand_navigation.md`). Not done: pause/resume, a service for status outside a goal, cancel while MoveIt is planning is only heard when the planner returns, one goal at a time
- [~] **Multi-stand loop**: `multi_stand` goals plan stands (`plan_stands`), give each column to a stand, and per stand stow the arm, `navigate_to_stand`, run its columns in the world frame; a stand never reached or a failed column is recorded and the loop goes on; the result lists columns per stand. `scripts/run_multi_stand_pass.sh` (MuJoCo, 0.9 m wall, `stand_max_span_m: 0.5`): two stands: one run 9 of 9 columns, scorer passes (coverage 0.977, peak 20.9 N, RMS 3.51 N); the repeat failed (a jammed tool after column 8, then the stow failed and stand 1 was skipped: 5 of 9, coverage 0.67), so not reliable yet. Recovery of the arm after a jam is built (joint jog; see the failure-handling section in `docs/sim_mujoco.md`, not yet reliable in the sim). Not done: replanning a failed column from another stand, a wall that needs the planner's own stand count (see the span note in `docs/stand_navigation.md`), obstacles, the wall octomap is re-injected by a shell command after each arrival (`stand_arrival_command`)
- [~] **Gazebo spray and multi-stand parity** (`sim_gazebo/scripts/run_spray_pass.sh`, `run_multi_stand_pass.sh`, `gz_pass_common.sh`): `arm_meshes` is now on by default (gz finds the UR meshes through `GZ_SIM_RESOURCE_PATH`, so the arm has real collision), a tool-tip bump stop is in, `rgb_camera_frame:=world` gives a static camera for multi-stand footage. Re-measured with the meshes: spray was 1 of 3 (coverage 0.92 / 0.57 / 0.00), root cause found (the 4 s seed plan against gz's 10 s joint loop drags the wrist onto the wall top, the first MoveIt plan then starts in collision with the octomap), seed plan now 27 s, spray 4 of 4 (coverage 0.92 / 1.00 / 1.00 / 0.96); multi-stand 1 of 3 (6, 8, 7 of 9 columns), then 1 of 2 (8 and 9 of 10, coverage 0.75 / 0.99), the lost columns are stand 1 dock plans refused (`tool_mount_link` against the octomap). The bump stop does not bound the contact impacts. Not done: the stand 1 dock refusals with meshes, JTAC divergence warnings during the seed move, a controller-side peak-force guard (`ForceLimiter` is not wired into JTAC), a Gazebo `run_stand_pass.sh`, a peak-force limit on the scorer (`docs/sim_gazebo.md`) Update (`fix/gazebo-multistand-dock`): with `wait_arm_settled.py` before the stand arrival the Gazebo multi-stand pass (10 columns) measured 6/10, 10/10, 10/10 columns, coverage 0.60 / 0.94 / 1.00, peaks 36 / 54 / 3025 N, on a loaded host (loadavg 13 to 21); 3 runs do not prove an improvement, and the dock refusals after an in-stand approach segment remain unexplained (`docs/sim_gazebo.md`).
- [~] Scenario set: `run_scenarios.sh` (MuJoCo, UR10e): plain, plain_wide, window hole, protrusion, two panels with a rib; scorer counts obstacle contacts and scores the reachable area (`docs/sim_mujoco.md`, scenario set, with the table). Not done: other robot profiles, Gazebo, arm-link collisions with the obstacle, wiping up to the obstacle's keep-out
- [~] Failure handling in the loop: a failed column is retried once, then skipped with the loop going on (`column_retries`); a start state in self-collision or outside the lift's bounds is jogged back toward the last good configuration before the dock, retract or stow is repeated (`recovery_jog`, `core/recovery_jog.hpp`, gtests); see `docs/sim_mujoco.md`. Not done: replan from another stand, re-stand, abort policy

## 12. Hardware readiness

- [~] UFACTORY path: `ft_bridge` default topic, JTAC yaml for `uf_ft_sensor`, a real-robot branch in the compliance demo; nothing verified on hardware
- [~] Hardware bring-up guide: `docs/hardware_bringup.md` (procedure, abort thresholds, sim-vs-real differences); written from the code and simulations, no commissioning test exists and nothing was run on hardware
- [~] `hardware:=real` description branch and vendor-driver selection: `config/hardware_drivers.yaml` (arm -> driver package, plugin, required params, F/T source), the `real` branch of the ros2_control xacro (`real_driver`, `real_params`, `real_lift_plugin`), `hardware_real.launch.py` (fails with a message when the driver package is not installed), `real_driver:=mock` for the wiring; 28 pytest tests; the plugin and parameter names are from vendor documentation, no driver installed, no robot, no vendor controller wrapping, no lift driver
- [~] Calibration procedures in one place: `docs/calibration.md` (F/T sign and bias, tool TCP, payload services, wall plane, base-to-wall); tool-TCP pivot calibration is a ROS-free core (`core_types/pivot_calibration.hpp`, gtests with simulated noise), no ROS node for it and nothing run on hardware

## 13. Open source and plug-and-play

- [x] Apache-2.0 licence; docs index; architecture, robot, reachability, simulation, F/T docs; Docker with nine targets
- [x] Root README: overview, workflow, package map, Docker build and test, simulators, quickstart pointing at the one-command sim scripts
- [x] **Getting started**: one command to a simulated wall pass (`sim_mujoco/scripts/run_wall_pass.sh`, `run_manager_pass.sh`; `sim_gazebo/scripts/run_manager_pass.sh`)
- [x] `THIRD_PARTY.md` (licences, use, nothing vendored); revisit as adopted software lands (a few licences are marked unverified there; cddlib GPL-2.0 in the VAMP stage: reviewed 2026-11-15, source and recipes only, no binaries shipped, see THIRD_PARTY.md)
- [x] CHANGELOG (#5 to #61) and CONTRIBUTING written; every package is at `0.1.0` (checked: nothing depends on an exact version; `colcon list` over all 28 packages works)
- [x] `docs/docker.md` covers the `sim` and `sim-gazebo` targets
- [x] CI: none by decision; one documented "run all checks" section in `docs/docker.md` (`docker/build.sh test`, tool tests, public-readiness scan, sim scripts); the sim scripts are still not in `colcon test`
- [ ] **Public-readiness cleanup**
  - [x] References to documents that are not in the repo: none left in code or docs (checked 2026-11-16 with a script over every `*.md` mention in tracked files; `docker/tools/check_public_readiness.py` keeps checking)
  - [x] Earlier-project base and application names are removed (no `serum` anywhere; "mop" is a real tool in `coverage_planner/config/tool_profiles.yaml`, not a leftover)
  - [x] Launch files that looked up packages that do not exist are fixed or removed (every remaining `*.launch.py` uses `mobile_manipulator_description` / `mobile_manipulator_moveit_config`)
  - [x] `.claude/` is in `.gitignore`
  - [ ] Decide about the maintainer's email, which is in 97 files (70 `@author` tags plus `package.xml` maintainer entries; their own address: keep or drop)
  - [ ] Final scan before making the repo public (names, paths, secrets): the procedure is `docs/release_checklist.md`, not yet run; the visibility change itself is your command

## 14. Docs that no longer match the code

- [x] `architecture.md` status table (reachability, self filter, TRAC-IK, VAMP / cuRobo collision, sim execution; the pluginlib wording is left to the backend-selection work)
- [x] `reachability.md` "Not done"
- [x] `curobo_v2_notes.md` and `moveit_config.md` "Not done"
- [x] `scene_model.md` and `robot_self_filter.md` (the node is described; test counts 53 and 29)
- [x] `coverage_planner` README rewritten
- [x] `package_layout.md` pluginlib selection implemented (PR #29)

## 15. Tests and quality

- [x] 736 colcon tests in 26 packages, 0 failures; Python tests for description, profiles, sim generators, tools
- [x] Tests for the coverage ROS nodes (`coverage_planner_node`, `wall_coverage_service_node`), the JTAC compliance port and the manager run reporter; `docker/run_all_checks.sh` runs everything that needs no simulator
- [ ] Sim end-to-end checks as a test target (sections 10 and 11)
- [~] Planner-level benchmark on shared scenarios (section 5): see `docs/planner_benchmark.md`

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
| Live mapping | cuRobo v2 Mapper (Apache-2.0, GPU TSDF to ESDF; standalone use and VRAM on 4 GB measured on synthetic depth, see `docs/curobo_v2_notes.md`; ROS interface, real depth and running next to a planner are unverified; ROS wrappers exist, e.g. `Lab-CORO/curobo_ros`, Apache-2.0); nvblox (**Apache-2.0**, CUDA, last push 2026-10; `isaac_ros_nvblox` Apache-2.0); CPU: VDBFusion (MIT, 2025-01), wavemap ROS 2 forks (BSD-3: `Forestry-Robotics-UC/wavemap2`, pushed 2026-11; upstream `ethz-asl/wavemap` is ROS 1 only and stale since 2024-12) | | Gaussian-splat maps (high risk); Isaac ROS container route unless NVIDIA's stack is wanted |
| Wall / plane detection | PCL multi-plane segmentation (BSD) behind our plane port | MAGSAC++ variants if RANSAC is not enough | learned plane transformers for now (ZeroPlane MIT, PLANA3R Apache-2.0, OVO MIT exist); Zoo3D (CC BY-NC) |
| Self filter | ours; `robot_body_filter` (BSD-3, `ros2` branch active since 2026-04 but not in the Jazzy or Kilted rosdistro: source build); `leggedrobotics/robot_self_filter` (BSD-3, ROS 2, 2026-10) | sphere-based filter with inflation margin (2-3 days) | learned filters |
| Odometry / SLAM / localisation | `robot_localization` (Apache-2.0); KISS-ICP, KISS-SLAM, GLIM, PIN-SLAM (all MIT; the last three need a 3D lidar and, for GLIM, an IMU is usual); Nav2 AMCL and slam_toolbox are LGPL-2.1, run as separate nodes after a licence review | | FAST-LIO2, FAST-LIVO2, iG-LIO (GPL-2.0); Cartographer (barely maintained) |
| Last metres to the wall | Nav2 `opennav_docking` (Apache-2.0, Jazzy branch v1.3.14) with a custom `NonChargingDock` plugin (the plane detector feeds `getRefinedPose`) | plane-distance and yaw servo (1-2 weeks) | |
| Navigation | Nav2 (Apache-2.0): Smac lattice / Hybrid-A*, MPPI (omni, diff, Ackermann), costmap footprint | stand -> goal bridge, footprint per base (`stand_navigator` is the interim version, `docs/stand_navigation.md`) | |
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
| Simulation | core MuJoCo 4.0.1 (Apache-2.0) with calibrated soft contact; MuJoCo Warp (Apache-2.0) for sweeps; Newton (Apache-2.0, active) to watch; Gazebo for integration | (compliant tool tip for Gazebo: done) | Genesis, Isaac Lab on a 4 GB GPU |

Corrected from the first draft: nvblox is Apache-2.0 (not NVIDIA terms); `nav2_amcl` is LGPL-2.1 and `robot_localization` Apache-2.0; wavemap upstream is stale
(last push 2024-12-30); the Noether swath/standoff awareness is ours to add; `mujoco_ros2_control` pins MuJoCo 3.12, not 4; arXiv 2604.01414 is a vision-torque
diffusion policy (not UMI-FT) and 2512.03772 tunes torque-level MPC (not impedance gains); the cuRoboV2-Fast paper is arXiv 2611.06913.

Still unverified: cuRobo's Mapper on real depth, in a ROS node and in one process with a planner (standalone use and its VRAM on 4 GB are measured on synthetic frames, 2026-11-18); cuRobo support for a planar mobile base; Noether's mesh preconditions;
`mujoco_ros2_control` against MuJoCo 4.0.1; headless depth rendering in `mujoco-core`; the ROS 2 wavemap forks' quality; the toppra and Tesseract Jazzy
packages; the licences of paper code beyond those named; all effort figures (researcher estimates).

## 18. Ideas adopted from an earlier construction-robot stack

Ideas only: that stack is proprietary, so none of its code is reused. Each item was checked against this repo with grep (2026-11-15); items already done
here were dropped. Placement is the milestone that should carry it.

**Safety and execution (M1, M2; hardware wiring M6)**
- [ ] Fault abort in ros2_control: a `fault` state interface and typed codes; a lift or arm fault makes `read()/write()` return ERROR so every controller claiming the component deactivates; mark devices safety-critical or not (tool I/O is not); debounce about 3 cycles
- [~] Never report a dead axis healthy: hold the last position, raise `fault`, return ERROR past a threshold (no "degraded mode") (`core_types/command_guard.hpp` with gtests, wired opt-in into JTAC `update()`: hold, debounced latch, ERROR; a non-finite measurement is a fault; no `fault` state interface, no hardware run)
- [~] NaN and limit guard on every command interface; per-joint trajectory tolerance including the lift (guard done for JTAC, opt-in `command_guard.enabled`; not on the sim system interfaces; no per-joint trajectory tolerance)
- [~] Spray valve forced off on any fault and confirmed by polling; drive-side watchdog and bring-up ordering (hardware only) (forced off and polled until the echo confirms: `ToolActuation`, tested; not wired to the guard fault; watchdog and ordering not done)
- [~] **Tool actuation executor**: `manipulation_manager/core/tool_actuation.hpp` (ROS-free, 14 gtests) and opt-in `tool_actuation_node`: `tool_command` / `tool_off_lead_s` / `tool_on_at_speed_ratio` on trajectory time (a pause drives the tool off), commands verified against the echo with resend, tool off on cancel, fault and end, `TOOL_FAULT` / `TOOL_NOT_RESPONSIVE`, on/off latency measured. Not done: the manager does not feed it a window or the trajectory time (it still opens the valve itself), no echo in the sim, no hardware valve
- [ ] Segment-level intent in the motion layer: force stage, target force, standoff, normal, speed, tool command, lift height (unknown stage degrades to NONE)

**Manager and task flow (M1)**
- [~] Manager action contract: goal kind plus params, one job at a time, per-app cancel; BehaviorTree.CPP with BehaviorTree.ROS2 action nodes, not hand-rolled clients (the action, one job at a time and cancel are done with a hand-written state machine and plain action clients; the BT.CPP version is not)
- [~] Always-report tree convention (`Fallback(work, ForceFailure(ReportStatus))`), halt and skip guards, mission FSM (IDLE/ACTIVE/PAUSED/STOPPED/RECOVERY) with one typed command service (`core/mission_fsm.hpp` with gtests; pause and resume at column boundaries through a `mission_command` String topic, not a typed service; no behaviour tree, the FSM state is not in the feedback)
- [~] Durable ledger keyed (mission, stand, segment) and resume; acceptance test: kill mid-wall, restart, only unfinished segments rerun (`core/mission_ledger.hpp`, atomic JSON, `ledger_path` parameter, `ExecuteCoverage.resume`, a segment is a wall column, gtests including the kill-and-restart sequence; the sim kill-and-restart run is not done, no hardware)
- [~] Per-station failure policy: typed SKIPPED reasons, hardware-origin codes, no blind retry on a contact overload; station queue skips a blocked stand with a reason and slides it in small steps (reasons `NAV_FAILED`, `DOCK_FAILED`, `PLAN_FAILED`, `CONTACT_OVERLOAD` in the ledger, overload limit `contact_overload_n` stops the retry; no hardware-origin codes, no sliding of a blocked stand)
- [ ] Controller switching only at segment boundaries

**Planning (M1, M2)**
- [~] Time parameterisation by segment role: constant-speed jerk-limited for contact seams, time-optimal for transitions, split at reversals (150 degrees, joint-space direction), limit violations on a seam are failures (`time_limit_violation`); done in core and adapter with gtests, opt-in `retime:=true`, not run in the sim; reversal angle is measured in joint space with prismatic and revolute joints unweighted
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
- [ ] Contributor guidance for users and coding agents (an agent guide and a contributing folder)

**Pitfalls to cover with tests**
- [ ] Frozen lift feedback with a loose tolerance must still abort; a tool-I/O fault must not
- [ ] Mixed-unit jump detection (0.06 m of lift vs 0.2 rad of arm), including against the start state
- [ ] Mimic and stage joints reach `/joint_states`; every runtime velocity limit equals the profile value
- [ ] Dead config: a file never loaded, padding at the wrong namespace, `allowed_start_tolerance` handling (not set anywhere in `src` today)
- [ ] Redundant-group pose goals are random-goal samplers; a failed IK call can leave a partial result
- [~] Retimer hazards: reversals, near-duplicate waypoints, mixed joint types (lift plus revolute in the test), zero acceleration limit, non-finite input, joint mismatch give a valid trajectory or a typed `TimingError` (gtests); the planners' own retiming paths in `CartesianPlanner` are not yet routed through it
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
  recorded on two robot profiles; Gazebo holds 10 N (done with the compliant tip).
- **M3 Base motion and docking, by reuse**: planar-velocity bases, then ros2_control drive controllers; Nav2 with stands as goals; `opennav_docking` with a
  wall-dock plugin; global stand selection by set cover + TSP; then local refinement. *Exit:* the base drives to a stand in sim (done, `run_stand_navigation.sh`) and the pass succeeds from where it stopped (done for one stand in MuJoCo, `run_stand_pass.sh`: the manager's `world_frame` transforms each column from TF; several stands in a loop in `run_multi_stand_pass.sh`, see `docs/stand_navigation.md`).
- **M4 Live perception, by reuse**: MoveIt octomap + `octomap_server`, PCL wall detection, our self filter on the live cloud, simulated depth/lidar, then a
  mapper (cuRobo Mapper or wavemap) and AMCL / KISS-ICP localisation. *Exit:* M2's scenario runs from a sensed wall, not a file. **Met in the MuJoCo sim** (`run_sensed_pass.sh`: detected wall -> coverage -> octomap -> manager, coverage 1.00, peak 23.6 N, RMS 3.4 N); not on hardware, one plane, no obstacles, no octomap from the live cloud.
- **M5 Tools and surfaces**: spray model and the non-contact path; floors and ceilings as planar patches; Noether for meshes. *Exit:* a spray pass over a wall
  and a ceiling in sim with the verification raster showing even coverage.
- **M6 Hardware and release**: hardware template and bring-up guide, payload identification with safe excitation, BO gain tuning, third-party notices,
  quickstart, changelog, final scan, release.

## Decisions I need from you

1. **Local base placement**: a lightweight verify-and-nudge at each stand (re-check reachability on the live scan, shift within a small neighbourhood) or a
   full local optimiser? I recommend the lightweight one first; global placement exists.
2. **Backend selection**: pluginlib, as the docs prescribe and as you asked for reuse, rather than my earlier idea of a hand-rolled factory. (I now recommend pluginlib.)
3. **Gazebo contact**: done: the compliant tool tip holds 10 N (9.69 N). Remaining: an identified tip stiffness and a smaller reference-depth residual, if Gazebo is to be a contact simulator rather than integration only.
4. **MuJoCo plugin**: `mujoco_ros2_control` pins MuJoCo 3.12 (no MuJoCo 4 support found), ours builds against 4.0.1. Try building the upstream plugins (base velocity,
   camera, 3D lidar) against 4.0.1 and keep our own `SystemInterface` for the joints and F/T if that fails? I recommend trying it for base motion and sensors first.
5. **Base motion**: planar-velocity bases first (recommended), real wheel physics later?
6. **Tools to ship first**: mop (contact) and spray gun (non-contact), anything else?
7. **JTAC**: add the force-CBF limiter with an offset clamp (a controller change) instead of only documenting the slow release? Done as an opt-in (`force_limiter.enabled`); needs a MuJoCo contact run before it is on by default.
