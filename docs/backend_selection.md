# Backend selection

The planning backends (IK, collision, scene, Cartesian planner, free-space planner) are chosen
at runtime by a YAML file, not compiled into the manager. Each backend is a **pluginlib plugin**
of one of four base classes that live in the ROS-free core of `motion_planner`. This is the
mechanism behind rule 4 of [architecture.md](architecture.md) ("selected by config").

```
 planning.yaml ──► loadPlanningSelection ──► PlanningPipeline::build ──► MotionPlanner
                                                  │  creates + initialize()s each plugin
                                                  ▼
                              PluginRegistry (pluginlib ClassLoaders, or FactoryRegistry in tests)
```

| Piece | Where |
|---|---|
| Plugin base classes `PlannerPlugin`, `IkPlugin`, `CollisionPlugin`, `ScenePlugin`, `PluginContext`, `PluginRegistry`, `FactoryRegistry` | `motion_planner/core/plugins.hpp` (no ROS) |
| `PlanningSelection`, `PlanningPipeline` (builds the plugins, makes the `MotionPlanner`) | `motion_planner/core/planning_pipeline.hpp` (no ROS) |
| YAML loader, pluginlib-backed registry | `motion_planner/ros/planning_config.hpp`, library `motion_planner_ros` |
| MoveIt plugins | `motion_planner_moveit` (`plugins.xml`, library `motion_planner_moveit_plugins`) |
| cuRobo plugins | `curobo_planner` (`plugins.xml`, library `curobo_planner_plugins`) |
| VAMP plugins | `vamp_planner` (`plugins.xml`, library `vamp_planner_plugins`; only built where VAMP is installed) |
| Shipped default config | `manipulation_manager/config/planning.yaml` |

## Config schema

```yaml
planning:
  ik:          moveit_ik            # IkPlugin
  free_space:  moveit_ompl          # PlannerPlugin, used for "fsp" segments and as fallback
  free_space_fallback: ''           # optional second free-space plugin, tried when free_space fails
  cartesian:   moveit_cartesian     # PlannerPlugin, used for "cartesian" segments, tried first for fsp
  collision:   moveit_collision     # CollisionPlugin
  scene:       moveit_scene         # ScenePlugin, optional (empty = none)
  segments:                         # optional per-segment overrides
    contact:    cartesian           # allow_target_contact segments (the contact seam)
    transition: free_space          # every other segment
  plugin_params:                    # per plugin name, passed to initialize() as strings
    curobo: {node_name: /curobo_planner, timeout_sec: 45}
    vamp:   {robot: ur5}
```

- Every key is optional; a missing key keeps the default shown above (the MoveIt backend,
  Cartesian first and OMPL as fallback, exactly what the manager did before). Unknown keys are an
  error. The same map is accepted under a `pipeline:` key, as in the architecture example.
- `free_space_fallback` (optional, empty = none) names a second planner plugin that plans a free-space request when `free_space`
  does not succeed (cuRobo unavailable, timed out, or no path): the request goes to it unchanged, a failed fallback keeps the
  primary's failure reason, and the `fsp_backend=` log field names the backend that produced the plan. `planning_curobo.yaml` sets
  it to `moveit_ompl`. Unit-tested with fakes; not yet seen in a sim pass.
- `segments` maps a segment kind to a **role** (`cartesian` or `free_space`), and beats the
  segment's own `mode_hint` and label routing. `contact` is a request with `allow_target_contact`,
  `transition` is any other. `free_space` keeps the existing rule that Cartesian is tried first
  and the free-space planner takes over only if the Cartesian attempt falls short.
- The manager reads the file named by its `planning_config` parameter; empty means the packaged
  default.

### Plugin names

| Name | Base class | Package | Notes |
|---|---|---|---|
| `moveit_cartesian` | Planner | motion_planner_moveit | jump detection, reachability-tilt retry, wall-contact ACM |
| `moveit_ompl` | Planner | motion_planner_moveit | pose and joint goals; params `free_space_direct_first` (default false), `direct_revolute_step` (0.05 rad), `direct_prismatic_step` (0.01 m), `direct_default_acceleration` (2.0, used only by the direct path retiming); optimal mode: `free_space_optimize` (false), `free_space_time_budget_s` (20), `free_space_num_candidates` (5), `free_space_objective` (`length`\|`duration`), `free_space_planners`, `free_space_shortcut_iterations` (300), `free_space_smoothing_rounds` (4) |
| `moveit_ik` | Ik | motion_planner_moveit | params `ik_extra_seeds`, `ik_seed_perturbation` |
| `moveit_collision` | Collision | motion_planner_moveit | |
| `moveit_scene` | Scene | motion_planner_moveit | param `octomap_resolution` |
| `curobo` | Planner | curobo_planner | needs the cuRobo node (GPU); params `node_name`, `wait_for_service_sec`, `timeout_sec` (default 45 s), `base_frame`, joint limit params (below) |
| `curobo_ik`, `curobo_collision`, `curobo_scene` | Ik, Collision, Scene | curobo_planner | share the client of `curobo` |
| `vamp` | Planner | vamp_planner | params `robot`, `max_iterations`, `range`, `point_radius`, `nominal_joint_speed`, `simplify`, `require_world`, `base_frame` |
| `vamp_collision`, `vamp_scene` | Collision, Scene | vamp_planner | share the planner of `vamp` for the same robot (a `base_frame` param on any of them applies to the shared planner) |

### Direct joint-space fast path (`moveit_ompl`, opt-in)

With `plugin_params: {moveit_ompl: {free_space_direct_first: true}}` a free-space segment first tries the straight line in joint space
from the start to the goal (a joint target as given, a pose goal solved by IK seeded from the start). The line is densified (at most
`direct_revolute_step` per revolute joint and `direct_prismatic_step` per prismatic joint between samples), every sample is checked
with `PlanningScene::isStateValid` (collision and joint bounds), and the result is retimed with the MoveIt TOTG time parameterizer
(`MoveItTimeParameterizer`), so the duration is set by the slowest joint instead of by a wandering path. One invalid sample, a failed
IK or a retiming failure falls back to the normal OMPL plan unchanged. Log lines: `direct joint path: N points, X s` and
`direct path invalid at sample K, falling back to OMPL`. Pure logic: `motion_planner/core/direct_joint_path.hpp`.
Why: the OMPL joint path was measured up to 16x longer than the straight-line joint distance (a stow took 37 s).

### Optimal free-space mode (`moveit_ompl`, opt-in)

Why: a free-space plan from the default OMPL planner (RRTConnect) was measured with a joint path 1.7x to 16x the straight-line joint
distance for simple transitions (stow: 375 points, 37 s; "approach" between adjacent columns: 70 to 108 points, 7 to 11 s). Executing a
long path costs wall time that no planning time can win back, so the planning time is spent on finding the shortest path.

`plugin_params: {moveit_ompl: {free_space_optimize: true}}` replaces the single OMPL call of a free-space segment by:

1. the direct joint-space line (same code as the fast path above, whether or not `free_space_direct_first` is set) as candidate 1;
2. `free_space_num_candidates - 1` runs (at least 1) of asymptotically optimal planners, round robin over `free_space_planners`
   (default `BITstarkConfigDefault,RRTstarkConfigDefault,AITstarkConfigDefault,InformedRRTstarkConfigDefault`, defined in
   `mobile_manipulator_moveit_config/config/ompl_planning.yaml` with `optimization_objective: PathLengthOptimizationObjective`).
   `free_space_time_budget_s` (default 20) is the total OMPL time, split equally; an asymptotically optimal planner keeps improving
   until its slice is over. MoveIt's OMPL interface here (2.12) accepts the objectives PathLength, Minimax, StateCostIntegral,
   MechanicalWork and MaximizeMinClearance; path length is used;
3. per OMPL result: randomised shortcutting (`free_space_shortcut_iterations`, 300) and corner-cutting smoothing
   (`free_space_smoothing_rounds`, 4, a repeated quadratic B-spline corner cut), every replacement segment densified and checked with
   `PlanningScene::isStateValid`, so the result is valid and never longer; then the MoveIt TOTG time parameterisation;
4. `selectBestCandidate`: `free_space_objective: length` picks the smallest joint path length weighted by 1/(joint range) (the lift's
   metres and the arm's radians become comparable), `duration` the smallest TOTG duration. Ties keep the earlier candidate, so a clear
   straight line (the direct path) always wins.

One line per plan: `free-space plan: candidates N, best length=X (baseline straight Y, efficiency Z), <planner> used` (for `duration`
the values are seconds). Baseline = straight joint line between the ends of the chosen path in the same unit (for `duration`, the
slowest-joint rest-to-rest time); efficiency = baseline / value, 1 = straight. A pose goal gets a different IK solution per OMPL run;
the direct candidate exists only when IK from the start succeeds. When no candidate is found the plan fails as before. Default off.
The planning time of one free-space segment becomes about `free_space_time_budget_s` seconds (plus the direct path and the post-processing).
Pure logic: `motion_planner/core/path_quality.hpp` (weighted length, duration estimate, baseline, efficiency, selection) and
`motion_planner/core/path_shortcut.hpp` (shortcut and smoothing), both with host gtests (`path_quality_test`, `path_shortcut_test`).
Status: host gtests pass; the MoveIt part is syntax-checked only, not yet run in a simulator.

Path shortcutting after OMPL in the default mode: the MoveIt OMPL interface already simplifies each solution (`simplify_solutions`, default true, bounded
by the planning time) and `ompl_planning.yaml` does not switch it off, so no extra shortcut stage was added.

### cuRobo: joint limits

cuRobo times its trajectories with the limits of its own robot yml, which are not the MoveIt `joint_limits.yaml` the controller side
uses (in one sim pass they ran about 14x faster than the same move retimed by MoveIt). With limits configured, the `curobo` plugin
checks each returned trajectory (`curobo_planner/joint_limit_guard.hpp`) and either slows it down or refuses it. Without any of
these params the trajectory is returned as cuRobo timed it (unchanged behaviour; `planning_curobo.yaml` sets them).

| Param | Meaning |
|---|---|
| `joint_limits_file` | MoveIt `joint_limits.yaml` (reads `max_velocity` / `max_acceleration` where `has_*_limits` is true), a path or `package://<pkg>/<path>` |
| `joint_limits` | inline `name:max_velocity:max_acceleration,...`; adds to or overrides the file's entries |
| `limit_safety_factor` | fraction of each limit that may be used, in (0, 1], default 0.9 |
| `limit_mode` | `retime` (default: scale the time axis) or `reject` |
| `limit_use_file_scaling` | multiply the file's limits by its `default_velocity/acceleration_scaling_factor`, as MoveIt does (default false) |

Failure codes: `limit_violation` (reject mode), `limit_missing` (a trajectory joint has no limit: an error, not "unlimited"),
`limits_invalid`, `invalid_trajectory`; all at stage `planning`. A bad param (unreadable file, unknown mode, malformed entry) makes
the plugin's `initialize` fail with `invalid_param`. Limits apply to the `curobo` planner plugin only. Velocity and acceleration only
(no jerk); speed is taken from finite differences of the positions, so a coarse trajectory under-reads its peaks. Gtests with
hand-made trajectories and a stub node; not run against the real node or in a sim.

### cuRobo: base frame, world size and timeouts

- `base_frame` (cuRobo and VAMP plugins): without it an update in another frame is not rejected (nothing to compare with); with it, a
  `ScenePort` update whose `frame_id` differs is refused before it reaches the backend. The plugins of one backend share one client / planner,
  so the first plugin to initialise builds it and a later plugin's `base_frame` is applied to the shared object.
- The cuRobo node's obstacle cache is `max_boxes` (default 256) cubes for the whole world; the scene adapter caps `max_obstacle_boxes`
  (default 2000) per object, so a cloud or mesh obstacle (or several objects together) can exceed it and the node refuses the update, keeping its
  previous world. The client then treats the world as stale: `plan`, `ik` and `collision` fail with `world_stale` until an update succeeds
  or the world is cleared. Raise the node's `max_boxes` or lower `max_obstacle_boxes` (the adapter default is not lowered: it is per object, so
  it cannot guarantee the total, and VAMP / MoveIt do not need the limit).
- `timeout_sec` default is 45 s: one `plan_joint` on the node can take 6 to 33 s (20 attempts) plus the seed fallback (docs/planner_benchmark.md).
  The node is single-threaded: after a client timeout it still finishes the abandoned plan and later calls queue behind it. A plan result
  with `status == "success_graph_seed"` is the validated graph path that was not optimised (its `message` says so).

## How to switch backends

Copy `manipulation_manager/config/planning.yaml`, change the names, and pass it:

```bash
ros2 run manipulation_manager manipulation_manager_node --ros-args -p planning_config:=/path/to/planning.yaml
```

An example that keeps MoveIt for contact seams and sends everything else to VAMP:

```yaml
planning:
  free_space: vamp
  segments: {contact: cartesian, transition: free_space}
  plugin_params: {vamp: {robot: ur5}}
```

A selection that cannot be built stops the node at start-up with one structured error, logged as
`[code] role 'plugin': message`. Codes: `unknown_plugin` (message lists the available names),
`load_failed`, `missing_dependency` (e.g. no node in the context), `unavailable` (the backend is
not there: cuRobo node not running, VAMP robot not compiled in), `invalid_selection`.

## How to write and register a plugin

1. Derive from the right base class (`motion_planner::PlannerPlugin`, `IkPlugin`,
   `CollisionPlugin` or `ScenePlugin`). The class needs a default constructor; everything it needs
   arrives in `initialize(PluginContext &, const PluginParams &)`, which returns a
   `PluginStatus` (`PluginStatus::fail("unavailable", "...")` when its backend is not there).
2. Take dependencies from the context, never from globals and never by creating a node:
   `context.get<rclcpp::Node>("node")` (a node the composition root spins in an executor thread),
   `context.logger`, or an object shared with sibling plugins via
   `context.getOrCreate<T>("key", factory)`. The pipeline also publishes the selected
   `"ik_port"`, `"collision_port"` and `"scene_port"` before it builds the planners.
3. Export it from a shared library and describe it:

```cpp
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(my_pkg::MyPlanner, motion_planner::PlannerPlugin)
```

```xml
<!-- plugins.xml -->
<library path="my_pkg_plugins">
  <class name="my_planner" type="my_pkg::MyPlanner" base_class_type="motion_planner::PlannerPlugin">
    <description>...</description>
  </class>
</library>
```

```cmake
add_library(my_pkg_plugins SHARED src/plugins.cpp)
target_link_libraries(my_pkg_plugins PUBLIC motion_planner::motion_planner_core pluginlib::pluginlib)
pluginlib_export_plugin_description_file(motion_planner plugins.xml)  # note: package is motion_planner
install(TARGETS my_pkg_plugins LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
```

The `name` in the XML is what the config uses. Without ROS (unit tests, other hosts) register
factories in a `FactoryRegistry` and call `PlanningPipeline::build` directly; see
`motion_planner/test/core/planning_pipeline_test.cpp`.

## Limitations

What each backend can do today (see also the table in [ROADMAP.md](ROADMAP.md) section 5):

| | Pose path (Cartesian) | Pose goal | Joint goal | IK | Collision input | Scene |
|---|---|---|---|---|---|---|
| MoveIt Cartesian | yes, with contact handling | n/a | no | yes | pose and joint | yes |
| MoveIt OMPL | no | yes | yes | yes | pose and joint | yes |
| cuRobo | no | single pose | yes | yes | joint-space only | boxes and points |
| VAMP | no | no | yes | no (`unsupported`) | joint-space only | points and boxes |

- A plugin is selected for a role, not checked against the request: choosing `vamp` for
  `free_space` makes the planner return `unsupported`/invalid-request failures for pose-only
  segments. Using VAMP from the manager needs an IK step in front of it, which is not built.
- The selected `ik` and `collision` plugins are built, initialized and published to the context
  (and available from `PlanningPipeline::ik()` / `collision()`), but `MotionPlanner` does not call
  them: each planner still answers `ik()` and `collision()` itself. Selecting another IK therefore
  changes what the pipeline exposes, not what the MoveIt planners use internally.
- The `scene` plugin is built but the manager does not feed it yet; scene feeding stays on the
  existing paths.
- The MoveIt plugins still create their own helper node inside `MoveItPlanningBackend` (an
  existing behaviour) instead of taking one from the context.
- cuRobo: one planner per process; the node from the context must be spun by another thread (the
  manager creates and spins a dedicated `planning_plugins` node, only when a plugin asks for it).
  VAMP robots are compiled in; if `vamp_planner` was built without VAMP, its plugins do not exist
  and the name is reported as `unknown_plugin`.
- Plugin libraries are never unloaded (the ClassLoaders live for the whole process).
- The `timing` entry of the architecture example (S-curve, TOPP-RA) is not part of this schema
  yet; an unknown key is rejected rather than ignored.
