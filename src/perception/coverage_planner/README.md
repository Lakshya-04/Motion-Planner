---
type: Reference
title: coverage_planner
description: Package README for the service-based boustrophedon coverage path planner that turns a sensed-wall occupancy grid into an arm waypoint path.
tags: [coverage-planning, motion-planning, package-readme, wall-cleaning]
generated: { by: claude-code/fable-5, at: 2026-07-25T06:27:42Z }
status: stable
---

# coverage_planner

Service-based coverage path planner: turns a sensed-wall `nav_msgs/OccupancyGrid` into a
`nav_msgs/Path` of waypoints for the arm to sweep. One pattern is implemented today --
`BoustrophedonPlanner` (grid-lawnmower sweeps) -- selected through a small factory
(`CoveragePlannerFactory::createPlanner`) so additional patterns (`spiral`, `zigzag` are
stubbed-but-not-implemented in `src/coverage_planner.cpp`) can be dropped in later without
touching the node.

## Package scope

This package owns the **geometry**: reading an occupancy grid, inflating obstacles, finding
connected free-space components, and sweeping them into an ordered `nav_msgs::msg::Path`.
It deliberately does **not** compute tool orientation (`gridToWorld()` in
`src/coverage_planner.cpp` always returns identity orientation -- see the comment block there)
and it does **not** yet emit typed motion-segment metadata (see below). Both of those are left
to the application/orchestration layer downstream (e.g. `wall_cleaning_action_server`,
`manipulation_manager`'s `ManipulationManager`) that has access to wall-normal and
mode-selection information the planner itself doesn't have.

## Algorithm: `BoustrophedonPlanner`

`src/planners/boustrophedon_planner.cpp` implements `BoustrophedonPlanner::generatePath()`
(declared in `include/coverage_planner/boustrophedon_planner.hpp`), the only pattern
registered in `CoveragePlannerFactory::createPlanner()` (`src/coverage_planner.cpp`):

1. **Inflate obstacles** -- `CoveragePlannerBase::inflateObstacles()` (in
   `src/coverage_planner.cpp`) marks any cell within `obstacle_inflation_radius` of an occupied
   cell (`grid_data > 50`, or unknown cells when `allow_unknown == false`) as blocked, using a
   square-window Euclidean-radius check.
2. **Connected-component extraction** -- `extractConnectedComponents()` (anonymous namespace,
   `boustrophedon_planner.cpp`) 4-connects free cells with a BFS flood fill, drops components
   smaller than `min_component_cells`, and sorts by size descending. `plan_all_components=false`
   (the node default) plans only the largest component; `true` plans every component above the
   size threshold.
3. **Optional polygon clipping** -- if `params.use_region` is set and the request carries a
   `geometry_msgs/PolygonStamped` in the same frame as the map, `pointInPolygon2D()` masks out
   free cells outside that polygon before sweeping.
4. **Sweep generation** -- per selected component, `determineWorldAwareSweepDirection()`
   (`coverage_planner.cpp`) picks grid-Y ("vertical") vs grid-X ("horizontal") sweep lines from
   the `sweep_direction` param (`"horizontal"`, `"vertical"`, or `"auto"`, which picks whichever
   axis has fewer lines). `generateHorizontalSweeps()` / `generateVerticalSweeps()`
   (`boustrophedon_planner.cpp`) then walk each row/column at `step` spacing
   (`step_cells = step / resolution`), alternating sweep direction each line (true boustrophedon
   /lawnmower back-and-forth), splitting a line into multiple free-space segments around
   obstacles, and emitting a `(start, end)` pose pair per segment via
   `CoveragePlannerBase::gridToWorld()`.
5. **Optional strip connectors** -- when `add_strip_connectors=true`,
   `addStripConnectorWaypoints()` Bresenham-walks from one strip's end cell to the next strip's
   first free cell and appends the intermediate free cells as waypoints; if any intermediate
   cell is blocked or out of bounds the whole connector is rejected (no partial connector is
   ever appended -- pinned by the `BlockedConnectorIsRejectedWithoutPartialAppend` test).
6. **Optional boundary pass** -- `generateBoundaryPass()` walks the component's rectangular
   perimeter (inset by `boundary_offset`) bottom -> right -> top -> left and closes the loop,
   prepended to the sweep path when `enable_boundary_pass=true`.
7. **Optional component ordering** -- with `optimize_strip_order=true` and multiple components,
   a greedy nearest-neighbor pass (`generatePath()`, main loop) chains component sub-paths by
   whichever endpoint (start or reversed-end) is closest to the current path tail, to cut
   deadhead travel between disconnected wall regions.
8. Path statistics (`total_distance`, `num_waypoints`) come from
   `CoveragePlannerBase::calculatePathStats()`; `coverage_area` is estimated as
   `swath * total_distance`.

`gridToWorld()` transforms grid cell centers into the map's frame using the occupancy grid's
`info.origin` (position + quaternion), but always sets orientation to identity -- by design, per
the comments in `coverage_planner.cpp`, since the planner has no wall-normal information.

## Node interface

`coverage_planner_node` (`src/coverage_planner_node.cpp`, class `CoveragePlannerNode`) exposes:

- **Service** `~/generate_coverage_path` (relative to node name, e.g.
  `/coverage_planner_node/generate_coverage_path`), type
  `coverage_planner_msgs/srv/GenerateCoveragePath` (defined in `srv/GenerateCoveragePath.srv`).
  No action server and no dedicated planning topic -- this is a synchronous service call.
  Request carries an optional embedded `nav_msgs/OccupancyGrid map` (falls back to the
  latest subscribed map if empty), `pattern`, `swath`, `step`, `sweep_direction`,
  `start_corner`, `pose_mode`, `allow_unknown`, `obstacle_inflation_radius`,
  `enable_boundary_pass`, `boundary_offset`, `add_strip_connectors`, and an optional
  `geometry_msgs/PolygonStamped region`.
- **Subscription** `map_topic` (param, default `/map`) of type `nav_msgs::msg::OccupancyGrid`
  -- cached in `latest_map_` under `map_mutex_`; used only when the service request doesn't
  embed its own map (service-request map takes priority, per `generateCoveragePathCallback()`).
- **Publishers** (for visualization/monitoring, not used for control):
  - `~/coverage_path` (`nav_msgs::msg::Path`)
  - `~/coverage_markers` (`visualization_msgs::msg::MarkerArray`, green sphere per waypoint,
    via `publishWaypointMarkers()`)
  - `~/coverage_diagnostics` (`std_msgs::msg::String`, a one-line summary of
    pattern/map size/waypoints/strips/components/step/distance/planning_time, via
    `publishDiagnostics()`)

Only one pattern is registered in the factory today (`"boustrophedon"`); requesting any other
`pattern` string returns `success=false` with the list of available planners in `message`.

## MotionPlanSegment metadata: not populated

`GenerateCoveragePath.srv`'s response declares `interfaces/MotionPlanSegment[] segments`
and `string[] segment_labels` ("Segment metadata aligned with MotionMode/Planner handoff").
`MotionPlanSegment` (`platform/interfaces/msg/MotionPlanSegment.msg`) carries
`label`, `start_index`, `end_index`, `frame_id`, `mode`, `planner`, `requires_contact`,
`requires_retraction`, `priority`, and `tags`.

As of this branch (`feat/motion-planner-segment-bridge`), **the node never fills these fields
in** -- `CoveragePlannerNode::generateCoveragePathCallback()` explicitly clears both, once at
entry and again right before returning:

```cpp
// src/coverage_planner_node.cpp:225-227
// Keep segment metadata explicit until planner emits typed segments.
response->segments.clear();
response->segment_labels.clear();
...
// src/coverage_planner_node.cpp:320-321
response->segments.clear();
response->segment_labels.clear();
```

`CoveragePlannerResult` (`coverage_planner_base.hpp`) has no segment field at all -- the
boustrophedon planner tracks segment boundaries internally (`segment_count`,
`segment_start`/`end_x`/`end_y` in `boustrophedon_planner.cpp`) purely for logging
(`RCLCPP_DEBUG`/`RCLCPP_INFO` sweep summaries) and never surfaces them as
`MotionPlanSegment` entries. The wire fields exist so downstream consumers
(`manipulation_manager`'s segmented plan+execute+retry path) can already declare their
message dependency, but this package is not yet the producer -- the "segment bridge" this
branch is building has a placeholder on the interface side only.

## Launch usage

Two launch files:

**`launch/coverage_planner.launch.py`** -- standalone node bring-up, args:
- `config_file` (default `config/coverage_planner.yaml`)
- `map_topic` (default `/map`)
- `obstacle_inflation_radius` (default `0.01`)
- `planning_cpu_set` (default `7-8`, passed to `taskset -c` to pin the node to specific CPU
  cores)

```bash
ros2 launch coverage_planner coverage_planner.launch.py map_topic:=/map
```

**`launch/wall_coverage_demo.launch.py`** -- standalone, dataset-driven demo: brings up
`coverage_planner_node` plus `wall_coverage_demo_node` against a real `wall_scene.json`/`walls.pcd`
export (no other `*` package needed), with RViz showing the coverage plan alongside
ground-truth point cloud and 2D/3D obstacle markers. Key args: `wall_scene_json` (defaults to this
package's own `test/data/` fixture), `wall_id`, `resolution`, `add_strip_connectors`,
`allow_unknown`, `output_json` (dump the full run for later review), `orientation_format`
(`quaternion`/`rpy`/`both`), plus the tool coverage/orientation args (`tool_name`, `contact`,
`standoff_m`, `swath_m`, `overlap_pct`, `sweep_direction`, ...). RViz's Fixed Frame is derived
automatically from `wall_scene_json`'s own `frame_id` at launch time.

```bash
ros2 launch coverage_planner wall_coverage_demo.launch.py wall_scene_json:=/path/to/wall_scene.json show_rviz:=true
```

`config/coverage_planner.yaml` sets the node defaults actually shipped: `coverage_pattern:
boustrophedon`, `swath: 0.5`, `step: 0.4`, `sweep_direction: vertical`, `start_corner:
bottom_left`, `add_strip_connectors: false`, `obstacle_inflation_radius: 0.0`,
`plan_all_components: false`, `min_component_cells: 25`, `optimize_strip_order: false`,
`map_topic: /navigational/map`.

`scripts/example_client.py` and `scripts/publish_3m_2m_wall.py` are installed executables
(`ament` `install(PROGRAMS ...)` in `CMakeLists.txt`) demonstrating/exercising the service;
`scripts/sample_map_publisher.py` publishes a generic test occupancy grid.

## Test coverage

`test/boustrophedon_planner_test.cpp` (gtest target `test_boustrophedon_planner`, built
directly against `BoustrophedonPlanner` -- no ROS node, no service) pins down six behaviors:

1. `HorizontalSweepStopsAtCurrentFreeCellBeforeObstacle` -- a sweep line terminates its segment
   at the last free cell before an obstacle, not past it.
2. `HorizontalSweepLookaheadDoesNotWrapAcrossRows` -- confirms sweep rows stay independent (no
   bleed of the end-of-row lookahead into the next row).
3. `BlockedConnectorIsRejectedWithoutPartialAppend` -- when `add_strip_connectors=true` and the
   connector line crosses a blocked cell, the connector is rejected wholesale (path keeps only
   strip start/end poses, no partial connector waypoints leak in).
4. `VerticalSweepStopsAtCurrentFreeCellBeforeObstacle` -- same before-obstacle termination check
   as (1), for `sweep_direction="vertical"`.
5. `ConnectorInterpolationHandlesNegativeXDirection` -- a connector spanning a negative-x,
   positive-y diagonal correctly interpolates and includes the intermediate free cell.
6. `ConnectorSkippedWhenNextSweepHasNoFreeCell` -- if the next sweep line has no free cell at
   all, no connector is attempted and no extra waypoints appear.

There is no test coverage for the service callback itself (`coverage_planner_node.cpp`), the
factory's unknown-pattern error path, obstacle inflation, connected-component
selection/ordering, boundary-pass generation, or the (currently inert) `segments`/
`segment_labels` response fields.
