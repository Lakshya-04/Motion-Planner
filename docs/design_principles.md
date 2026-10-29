# Design principles and lessons learned

Lessons carried over from earlier wall-coverage manipulation stacks (a 6-DOF arm on a
prismatic lift, and a comparable lift + UR-class arm system). They shape how the packages
here are meant to evolve. Each item is a rule, the reason behind it, and where it applies.

## 1. Architecture

- **Backends are adapters behind ports.** `Planner`, `ScenePort`, IK, collision, time
  parameterization and execution are pure C++ interfaces in `core_types` /
  `motion_planner`. MoveIt, cuRobo v2 (see [curobo_v2_notes.md](curobo_v2_notes.md)) and any
  future planner are interchangeable implementations, selectable by config. A stack that
  hard-wires its IK, collision and timing libraries turns every backend swap into a rewrite.
- **Planner is a library, not a node.** The manager links the planning pipeline and calls it
  in-process; ROS lives only at the edges (adapters, action servers). Inject the ROS node into
  adapters; never let an adapter spin its own hidden node.
- **The manager owns plan *and* execute.** Planning results carry the trajectory, the manager
  hands it to an `ExecutionPort`. Never book planning time as execution time.
- **Typed, serializable contracts** between coverage, manager and planner (segments with
  mode, planner hint, contact/retraction flags). In-process structs only are untestable from
  outside.
- **Structured failures** (`failure_reason_code` + stage) on every port result, not log strings.

## 2. World / scene input

- **One scene model, many inputs, many consumers.** Point clouds (`.pcd`/`.ply`, `PointCloud2`),
  meshes, primitives, voxel/occupancy grids, ESDFs, MoveIt planning scenes and depth images all
  normalize into one internal scene model; each backend adapter converts from it.
- **Keep geometry classes distinct.** The contact surface (the wall the tool may touch) and
  discrete obstacles (never touchable) must stay separate objects so a contact exclusion can
  never silently disable a real obstacle.
- **Live clouds need a pipeline:** TF into the planning frame -> clip to a capsule/box around
  the arm -> voxel downsample -> sanity guard -> backend. Do not double-filter (it degrades
  spatial indices). Refuse to plan with no world loaded when the task needs one.
- **Remove the robot from its own sensor data** before integrating depth, or the arm becomes
  an obstacle to itself.

## 3. Kinematics and frames

- **Plan to the TCP, explicitly.** Always pass the tool frame by name. A planning group whose
  tip defaults to the flange put pose goals 165 mm past the real tool point, which alone
  produced wild joint-space plans, Cartesian failures, slow planning and slow execution.
- **One source of truth for joint names, limits and conventions** (URDF / one profile file),
  never copy-pasted joint lists or hard-coded lift ranges.
- **Mimic joints must reach `/joint_states`** (or be resolved by the planner); otherwise
  collision geometry on mimic links sits at the wrong place.

## 4. The slow redundant axis (lift)

- **Time parameterization cannot fix redundancy.** TOPP-RA / TOTG only time a fixed path; if
  the path moves the lift, the lift's velocity limit slows *every* joint on the shared clock.
  The lift-vs-arm split must be decided upstream (IK / planner).
- **Prefer decoupling:** band the wall, index the lift once per band, plan in-band work with
  the lift locked (6-DOF problem). Industrial controllers treat this as the default mode.
- **When the lift must move during work, use velocity-constrained IK:** narrow each joint's
  IK bounds to what it can reach in the available `dt`; split the segment if no solution exists.
- **Order work by lift travel** (sort bands ascending, or sequence with a
  `max_i |dq_i| / v_max_i` time metric). Log lift travel on seams vs transitions as a KPI.
- **Units differ per joint type.** Jump / discontinuity thresholds are per joint type
  (e.g. 0.3 rad revolute, 0.05 m prismatic); unweighted joint-space distances on a mixed chain
  treat 1 m of lift as 1 rad of shoulder. Bake this into the trajectory types.
- Weighted IK tricks are a weaker version of velocity limits; keep hard limits as constraints,
  not weights (large weight ratios destabilize solvers).

## 5. Timing and execution

- **Dispatch the time parameterizer by segment role:** contact/seam segments get a Cartesian
  jerk-limited (S-curve) profile at the tool's EE speed; free-space transitions get TOPP-RA.
- **TOTG is wrong for on-surface passes:** it blends corners (tool leaves the commanded path),
  is ill-defined for mixed prismatic/revolute chains, errors on 180 deg reversals (every
  boustrophedon row end) and assumes zero start velocity. Split trajectories at reversals.
- **Set velocity/acceleration scaling explicitly**; defaults of 0.1 silently run everything
  at 10% speed. Supply acceleration limits (URDF has none).
- **Pipelining = overlap planning, not execution.** Pre-plan segment N+1 from N's *planned*
  end state while N executes; discard the whole look-ahead queue on any failure or mismatch.
  `joint_trajectory_controller` replaces goals rather than blending them.
- **Trigger tool actions (spray, pump) from trajectory time,** not from "the planner moved on".
- Timestamps at segment seams must strictly increase.

## 6. Planning robustness

- **Pose goals on a redundant arm are samplers:** seed IK from the start state and check the
  IK return value, or results are nondeterministic run to run.
- **Cartesian failures should be diagnosed** (jump vs collision vs IK) by relaxing one
  constraint at a time, and recovered by bridge-resume (free-space hop past the failure, then
  continue the Cartesian pass) instead of aborting the whole segment.
- **Straight-line probe first:** if the direct joint-space interpolation is collision-free,
  skip the sampling planner.
- **Failure-aware resequencing:** on a failed transition, re-run ordering with that arc
  forbidden (bounded rounds, keep the best), instead of a single blind retry.

## 7. Safety and hardware

- A fault on any device (lift, arm) must **abort the active trajectory**, not just log.
- Never report a dead or disconnected axis as healthy; stale position feedback with loose
  trajectory tolerances means nothing trips.
- Home the arm to a safe posture **before** sweeping the lift.

## 8. Method

- **Check config is live before reasoning about it:** trace the launch file. Dead config
  files look authoritative.
- **A scoped search returning nothing is not proof of absence.** Search the whole repo.
- Dump every executed trajectory (timestamped CSV) for field debugging.
