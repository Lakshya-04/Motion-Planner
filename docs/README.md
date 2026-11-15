# Documentation

| Document | What it covers |
|---|---|
| [ROADMAP.md](ROADMAP.md) | Status checklist for the whole workflow (done / partial / not done), the reuse map of open source and recent papers, the milestone order and the open decisions |
| [architecture.md](architecture.md) | The modular pipeline, every port, selection by config, surface and policy slots |
| [package_layout.md](package_layout.md) | `core/` + `ros/` split per module, how modules connect (pluginlib in-process, ROS interfaces across processes) |
| [backend_selection.md](backend_selection.md) | Choosing planning backends (MoveIt, cuRobo, VAMP) by YAML through pluginlib: config schema, plugin names, writing and registering a plugin, what each backend supports |
| [design_principles.md](design_principles.md) | Rules and lessons learned (frames, the slow lift axis, timing, safety) |
| [curobo_v2_notes.md](curobo_v2_notes.md) | cuRobo v2 API, scene/perception inputs, how it maps onto our ports |
| [vamp_notes.md](vamp_notes.md) | VAMP: what it does, upstream state, robot generation, our wrapper layout |
| [spherization_comparison.md](spherization_comparison.md) | Collision spheres compared: FOAM vs cuRobo v2 fit (plus earlier reference models): coverage, over-approximation, gaps, recommendation per backend |
| [mujoco_crosscheck.md](mujoco_crosscheck.md) | MuJoCo/MJX cross-check of the UR10e and generic arms: FK vs URDF, analytic DH and cuRobo; self-collision verdicts of MuJoCo meshes vs FOAM and cuRobo spheres; vectorised MJX variant, honest limits |
| [spherization_benchmark.md](spherization_benchmark.md) | Sphere sets (FOAM, cuRobo fit, reference) vs planning success, time and path length, per backend |
| [coverage_patterns.md](coverage_patterns.md) | Grid planners (boustrophedon, sweepline), wall patterns (vertical, horizontal, zigzag), scan pipeline, non-planar plan |
| [scene_model.md](scene_model.md) | Scene model: inputs (PCD, PLY, OBJ, STL, YAML, voxels, ESDF), processing, backend outputs |
| [nvblox_notes.md](nvblox_notes.md) | nvblox (latest releases only): facts, design ideas taken, how it fits next to the cuRobo mapper |
| [robot_description.md](robot_description.md) | Generic base + lift + N-DOF arm + tool description, swap contracts, profile YAML (group/joint names, DOF), UR10e / generic 6-DOF / Panda-like 7-DOF arms |
| [moveit_config.md](moveit_config.md) | MoveIt config: groups, controller, collision matrix, demo, smoke test, findings |
| [reachability.md](reachability.md) | RM4D reachability / inverse reachability module: base stationing, lift bands, feasibility pre-filter |
| [robot_self_filter.md](robot_self_filter.md) | Generic module that removes the robot's own body from sensor data: port, implementations, contact-mode and time-alignment rules, evaluation |
| [learned_recovery.md](learned_recovery.md) | Research on recovery to a known state: RL, imitation learning, MPC, safety filters; recommended staged architecture |
| [robot_assets.md](robot_assets.md) | One command from a robot profile to every per-robot artefact (URDF, SRDF, spheres, VAMP header, cuRobo config, reachability map, manifest); stages, images, skipping |
| [ft_sensing.md](ft_sensing.md) | Force/torque sensing: sign, bias, payload gravity and the lever-arm move from a raw sensor wrench to the contact wrench at the tool; what JTAC does today; how others solve it |
| [simulation.md](simulation.md) | Simulation backends behind ros2_control (`hardware:=` choice; `mock` and `mujoco` exist so far): the contract each must meet, layout, status; Isaac Sim is a stub |
| [sim_mujoco.md](sim_mujoco.md) | MuJoCo 4 simulation backend: ros2_control plugin (hardware:=mujoco), MJCF and wall world generators, raw wrist F/T and ground truth, smoke test, limits |
| [sim_gazebo.md](sim_gazebo.md) | Gazebo Harmonic backend (`hardware:=gazebo`): headless launch, wall worlds from wall_scene.json, raw F/T sensor and ground-truth contact wrench, smoke test, limits |
| [docker.md](docker.md) | Reproducible Docker environment: targets (including `sim` and `sim-gazebo`), dependency lists, GPU, VAMP SIMD build |
| [adr/](adr/) | Decision records (modular pipeline, upstream backends, core/ros split, C++ standard) |

Repository root files: [README.md](../README.md) (overview), [THIRD_PARTY.md](../THIRD_PARTY.md) (third-party software and licences), [CONTRIBUTING.md](../CONTRIBUTING.md) (layout, checks, commit rules), [CHANGELOG.md](../CHANGELOG.md). Package READMEs: [coverage_planner](../src/perception/coverage_planner/README.md).
- [learnings/](learnings/README.md) — how this repo was built with AI coding agents, and engineering lessons from it
