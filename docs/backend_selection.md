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
| `moveit_ompl` | Planner | motion_planner_moveit | pose and joint goals |
| `moveit_ik` | Ik | motion_planner_moveit | params `ik_extra_seeds`, `ik_seed_perturbation` |
| `moveit_collision` | Collision | motion_planner_moveit | |
| `moveit_scene` | Scene | motion_planner_moveit | param `octomap_resolution` |
| `curobo` | Planner | curobo_planner | needs the cuRobo node (GPU); params `node_name`, `wait_for_service_sec`, `timeout_sec` (default 45 s), `base_frame` |
| `curobo_ik`, `curobo_collision`, `curobo_scene` | Ik, Collision, Scene | curobo_planner | share the client of `curobo` |
| `vamp` | Planner | vamp_planner | params `robot`, `max_iterations`, `range`, `point_radius`, `nominal_joint_speed`, `simplify`, `require_world`, `base_frame` |
| `vamp_collision`, `vamp_scene` | Collision, Scene | vamp_planner | share the planner of `vamp` for the same robot (a `base_frame` param on any of them applies to the shared planner) |

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
