# Architecture: a pipeline of swappable modules

The stack turns sensor data into executed tool motion over a surface. Every stage is a
**module**: one interface (port), any number of implementations (adapters), chosen at
runtime by config. Nothing downstream knows which implementation produced its input.

```
 sensors ──► Perception ──► Self filter ──► World/Scene ──► Surface ──► Coverage ──► Sequencing
                                                                        │
   ┌────────────────────────────────────────────────────────────────────┘
   ▼
 Motion planning (IK · collision · free-space · Cartesian · timing) ──► Execution ──► Control
                                                                                     (JTAC, admittance, …)
                     ▲                                                     │
                     └──────────── Policy slot (RL / learned) ◄─────────────┘
```

Target scope: **any task, any robot, any surface**. Today the surface model is planar
(walls); the contracts are written so non-planar surfaces drop in without changing
coverage, planning or control interfaces.

## Rules every module follows

1. **Every module has a `core/` and a `ros/` half.** `core/` holds the algorithms, the
   interfaces (ports) and the data types in plain C++17 with no ROS, so it never changes
   with the ROS version. `ros/` is the thin ROS 2 shell (node, parameters, topics, message
   conversions, plugin registration). See [package_layout.md](package_layout.md).
2. **Two ways to connect modules, as in Nav2 / MoveIt / ros2_control.** Inside one process,
   through the C++ interface with the implementation picked by pluginlib. Across processes,
   through ROS 2 messages, services and actions. Each core data type that crosses a module
   boundary has a matching message, so a module can run in-process or as its own node, and
   its traffic can be recorded with `ros2 bag` and replayed.
3. **Structured results.** Every port returns success + failure code + failure stage +
   message. No exceptions across module boundaries, no log-string parsing.
4. **Selected by config.** Each module family is a pluginlib plugin type;
   a YAML profile names the implementation and its parameters. Per-segment overrides are
   allowed (e.g. contact seam → Cartesian planner, transition → VAMP).
5. **Inject dependencies.** Loggers, clocks, ROS nodes and other ports are passed in.
   Adapters never spin hidden nodes or read global state.
6. **Upstream stays unmodified.** Third-party libraries (VAMP, cuRobo, MoveIt, TOPP-RA)
   are consumed pinned and unpatched; all glue is ours. Updating = bumping a pin.
7. **Same benchmark for every implementation.** A module is only "better" if it wins on the
   shared scenario set (success, time, path quality, clearance).

## Module families

Status: **exists** = port in the code today; **planned** = to be added.

| Stage | Port (status) | Implementations (now / planned) |
|---|---|---|
| Robot model | `mobile_manipulator_description` (exists) — see [robot_description.md](robot_description.md) | telescopic lift + UR10e (now); any lift / 6-DOF arm via contracts |
| Reachability | `ReachabilityMap` (planned) — see [reachability.md](reachability.md) | RM4D map: forward + inverse queries, base stationing, lift bands |
| Robot self filter | `RobotSelfFilter` (planned) — see [robot_self_filter.md](robot_self_filter.md) | geometry containment, collision-sphere model, GPU depth mask (cuRobo-style), learned segmentation later |
| Perception | `PerceptionSource` (planned) | point-cloud topic, depth + camera (cuRobo `Mapper`), file replay |
| World / scene | `SceneModel` (exists, `scene_model`, see [scene_model.md](scene_model.md)); `ScenePort` (exists, `motion_planner/core/scene_port.hpp`) | loaders: `.pcd`/`.ply`, meshes, primitives YAML/JSON, voxel/ESDF, MoveIt `PlanningScene`, depth; consumers: MoveIt, cuRobo v2, VAMP |
| Surface | `SurfaceModel` (planned) | planar fit from scene (`WallPlane` today); later: mesh / parametric / height-map surfaces |
| Coverage planner | `CoveragePlannerBase` + `CoveragePlannerFactory` (exists) | boustrophedon, sweepline (now); cell decomposition (BCD), spiral, learned (planned) |
| Coverage pattern | `planWallPattern` (exists) — see [coverage_patterns.md](coverage_patterns.md) | vertical, horizontal, zigzag (now), over any coverage planner; non-planar surfaces later |
| Coverage feed | `CoverageSource` (exists, `manipulation_manager`) | JSON source (now); ROS service source |
| Sequencing | `SegmentOrderingPolicy` (exists) | input order (now); TSP with time metric `max_i \|Δq_i\|/v_max_i`, failure-aware resequencing (planned) |
| IK | `IkPort` (exists) | MoveIt/KDL via `motion_planner_moveit` (now); TRAC-IK, cuRobo IK, analytic (planned) |
| Collision | `CollisionPort` (exists) | MoveIt/FCL (now); VAMP, cuRobo (planned) |
| Planning | `Planner` (exists: plan + ik + collision) | MoveIt Cartesian + OMPL (now); VAMP free-space, cuRobo v2 trajopt, velocity-constrained IK seam planner (planned) |
| Timing | `TimeParameterizer` (planned) | S-curve Cartesian for contact seams, TOPP-RA for transitions |
| Execution | `ExecutionPort` (exists) | JTAC / `FollowJointTrajectory` (now); mock, sim (planned) |
| Control | `CompliancePort` (exists, `core_types`) | JTAC admittance (now); other force controllers (planned) |
| Recovery | `RecoveryProposer` (planned) | retract along normal, path retrace, global plan to nearest safe target set; learned proposer later — see [learned_recovery.md](learned_recovery.md) |
| Policy | `Policy` (planned) | none yet — see below |
| Logging | `Logger` (exists) | rclcpp, null |

## Surface abstraction (planar now, general later)

Coverage and planning should depend on a surface *interface*, not on "a plane":

- `SurfaceModel`: `frame`, `project(point) -> (u, v)`, `lift(u, v, offset) -> Pose`
  (position + outward normal + tangent frame), `contains(u, v)`, `obstacles() -> regions`.
- Planar implementation: today's `WallPlane` + `wall_basis` / `wall_pose_lift`.
- Non-planar implementations later: triangle mesh (geodesic / per-face frames), parametric
  patch, height map. Coverage keeps working in `(u, v)`; only `lift()` changes.
- Keep the contact surface and obstacles as separate scene objects, always.

## Policy slot (RL / learned components)

Learned components plug in through the same ports instead of a side channel:

- **As a module:** a learned coverage planner, IK seed proposer, or planner is just another
  implementation of the existing port, chosen by config and judged on the shared benchmark.
- **As a residual / reactive layer:** a `Policy` port between planning and control —
  `act(observation) -> command delta` with a declared rate, bounds and a safety envelope
  (classical controller remains the fallback and the safety filter).
- **For training:** every contract has a serializable form, so episodes (scene, surface,
  plan, executed trajectory, forces, outcome) can be logged and replayed as datasets; sim
  backends implement `ExecutionPort`, which is the environment step.

## Backend selection example

```yaml
pipeline:
  scene:      { inputs: [pointcloud_topic, primitives_yaml] }
  surface:    { type: planar_fit }
  coverage:   { type: boustrophedon, tool_profile: wide_brush }
  sequencing: { type: tsp_time_metric }
  planning:
    ik:          curobo
    free_space:  vamp
    cartesian:   velocity_constrained_ik
    collision:   vamp
    timing:      { seam: scurve_cartesian, transition: toppra }
  execution:  { type: follow_joint_trajectory, controller: jtac }
```

## Related documents

- [Package layout: core/ + ros/](package_layout.md)
- [Coverage planners and patterns](coverage_patterns.md)
- [Scene model](scene_model.md)
- [nvblox notes](nvblox_notes.md)
- [Robot description](robot_description.md)
- [Reachability (RM4D)](reachability.md)
- [Robot self filter module](robot_self_filter.md)
- [Learned recovery research (RL / IL / MPC)](learned_recovery.md)
- [Design principles and lessons learned](design_principles.md)
- [cuRobo v2 integration notes](curobo_v2_notes.md)
- [VAMP integration notes](vamp_notes.md)
- [Decision records](adr/)
