# Documentation

| Document | What it covers |
|---|---|
| [ROADMAP.md](ROADMAP.md) | Status checklist for the whole workflow (done / partial / not done), the reuse map of open source and recent papers, the milestone order and the open decisions |
| [development_plan.md](development_plan.md) | What is planned next, in order, with the design of each theme: scoring, ceiling, tools, splits, singularities, base placement, docking, control, cobot safety, diff-IK, tool correction |
| [architecture.md](architecture.md) | The modular pipeline, every port, selection by config, surface and policy slots |
| [modules.md](modules.md) | Module map: every package with its tier and what it needs (ROS, GPU, simulator), the dependency layers, and recipes for lifting parts out |
| [package_layout.md](package_layout.md) | `core/` + `ros/` split per module, how modules connect (pluginlib in-process, ROS interfaces across processes) |
| [backend_selection.md](backend_selection.md) | Choosing planning backends (MoveIt, cuRobo, VAMP) by YAML through pluginlib: config schema, plugin names, writing and registering a plugin, what each backend supports |
| [design_principles.md](design_principles.md) | Rules and lessons learned (frames, the slow lift axis, timing, safety) |
| [curobo_v2_notes.md](curobo_v2_notes.md) | cuRobo v2 API, scene/perception inputs, how it maps onto our ports |
| [vamp_notes.md](vamp_notes.md) | VAMP: what it does, upstream state, robot generation, our wrapper layout |
| [spherization_comparison.md](spherization_comparison.md) | Collision spheres compared: FOAM vs cuRobo v2 fit (plus earlier reference models): coverage, over-approximation, gaps, recommendation per backend |
| [mujoco_crosscheck.md](mujoco_crosscheck.md) | MuJoCo/MJX cross-check of the UR10e and generic arms: FK vs URDF, analytic DH and cuRobo; self-collision verdicts of MuJoCo meshes vs FOAM and cuRobo spheres; vectorised MJX variant, honest limits |
| [spherization_benchmark.md](spherization_benchmark.md) | Sphere sets (FOAM, cuRobo fit, reference) vs planning success, time and path length, per backend |
| [coverage_patterns.md](coverage_patterns.md) | Grid planners (boustrophedon, sweepline), wall patterns (vertical, horizontal, zigzag), scan pipeline, non-planar plan |
| [tools.md](tools.md) | Tool profiles (contact wiper, spray gun): `ToolProfile` fields, how contact and non-contact tools differ in planning, the spray swath model and its limits, how to add a tool |
| [scene_model.md](scene_model.md) | Scene model: inputs (PCD, PLY, OBJ, STL, YAML, voxels, ESDF), processing, backend outputs |
| [nvblox_notes.md](nvblox_notes.md) | nvblox (latest releases only): facts, design ideas taken, how it fits next to the cuRobo mapper |
| [robot_description.md](robot_description.md) | Generic base + lift + N-DOF arm + tool description, swap contracts, profile YAML (group/joint names, DOF), UR10e / generic 6-DOF / Panda-like 7-DOF arms |
| [moveit_config.md](moveit_config.md) | MoveIt config: groups, controller, collision matrix, demo, smoke test, findings |
| [reachability.md](reachability.md) | RM4D reachability / inverse reachability module: base stationing, lift bands, feasibility pre-filter |
| [robot_self_filter.md](robot_self_filter.md) | Generic module that removes the robot's own body from sensor data: port, implementations, contact-mode and time-alignment rules, evaluation |
| [learned_recovery.md](learned_recovery.md) | Research on recovery to a known state: RL, imitation learning, MPC, safety filters; recommended staged architecture |
| [behavior_tree_evaluation.md](behavior_tree_evaluation.md) | BehaviorTree.CPP for the manager: availability in Jazzy, state map, target tree, prototype results, recommendation (adopt later) |
| [robot_assets.md](robot_assets.md) | One command from a robot profile to every per-robot artefact (URDF, SRDF, spheres, VAMP header, cuRobo config, reachability map, manifest); stages, images, skipping |
| [ft_sensing.md](ft_sensing.md) | Force/torque sensing: sign, bias, payload gravity and the lever-arm move from a raw sensor wrench to the contact wrench at the tool; what JTAC does today; how others solve it |
| [simulation.md](simulation.md) | Simulation backends behind ros2_control (`hardware:=` choice; `mock` and `mujoco` exist so far): the contract each must meet, layout, status; Isaac Sim is a stub |
| [perception_live.md](perception_live.md) | Live perception slice: simulated ray-cast depth sensor in MuJoCo, `wall_detector` (point cloud to `wall_scene.json` and a MoveIt box), the check script and its limits |
| [how_to_read_a_run.md](how_to_read_a_run.md) | The files a MuJoCo manager pass writes (`manager.log` segment lines, `run.csv`, the score card, the film npz), what each field means and where the code writes it, with the two-stand pass as the worked example |
| [sim_mujoco.md](sim_mujoco.md) | MuJoCo 4 simulation backend: ros2_control plugin (hardware:=mujoco), MJCF and wall world generators, raw wrist F/T and ground truth, smoke test, limits |
| [sim_gazebo.md](sim_gazebo.md) | Gazebo Harmonic backend (`hardware:=gazebo`): headless launch, wall worlds from wall_scene.json, raw F/T sensor and ground-truth contact wrench, smoke test, limits |
| [sim_overlay.md](sim_overlay.md) | Coverage plan columns and stand poses drawn inside Gazebo (gz-sim /marker) and MuJoCo (viewer user_scn); host-tested only, never run in a simulator |
| [hardware_bringup.md](hardware_bringup.md) | Commissioning procedure for the force path on a real arm and sensor (sign and frame checks, payload identification, limits, slow first contact), abort thresholds, what the simulations do not model; nothing verified on hardware. Parameter template: [`hardware_template.yaml`](../src/control/compliant_trajectory_controller/config/hardware_template.yaml) |
| [lift.md](lift.md) | Driving the one lift joint: `lift_hardware` (home on connect, stroke clamp, velocity and acceleration limits, fault hold, reconnect backoff), the `LiftTransport` interface, the simulated and scripted transports, how a device transport plugs in (the device protocol is not public); the reference lift profile |
| [calibration.md](calibration.md) | Calibration procedures in one place: F/T sign and bias, tool TCP (pivot calibration), payload services, wall plane, base-to-wall; nothing verified on hardware |
| [release_checklist.md](release_checklist.md) | Public scan (names, paths, secrets), licence and version checks before a release; the maintainer decisions left open |
| [teleop.md](teleop.md) | Teleoperation groundwork: AUTO / TELEOP / HOLD modes, jog shaping and limiting around MoveIt Servo and JTAC; status |
| [docker.md](docker.md) | Reproducible Docker environment: targets (including `sim` and `sim-gazebo`), dependency lists, GPU, VAMP SIMD build |
| [manager_parameters.md](manager_parameters.md) | `manipulation_manager` parameters that no feature page covers: coverage resampling, dock IK selection, recovery jog, time parameterisation, verify result, column IK check; checked against the code by `docker/tools/check_params.py` |
| [adr/](adr/) | Decision records (modular pipeline, upstream backends, core/ros split, C++ standard) |

Repository root files: [README.md](../README.md) (overview), [THIRD_PARTY.md](../THIRD_PARTY.md) (third-party software and licences), [CONTRIBUTING.md](../CONTRIBUTING.md) (layout, checks, commit rules), [CHANGELOG.md](../CHANGELOG.md). Package READMEs: every package under `src/` has one; the map is [modules.md](modules.md).
- [learnings/](learnings/README.md) — how this repo was built with AI coding agents, and engineering lessons from it
