---
type: Reference
title: coverage_planner
description: Package README for the wall coverage planner: grid planners (boustrophedon, sweepline), wall patterns, the scan-to-plan library and the ROS nodes.
tags: [coverage-planning, motion-planning, package-readme, wall-cleaning]
status: stable
---

# coverage_planner

Plans the passes a tool makes over a wall. Everything that decides geometry lives in a ROS-free core
(`include/coverage_planner/core`, `src/core`); three ROS nodes wrap parts of it. The wider design
(grid planner vs wall pattern, non-planar surfaces) is in [docs/coverage_patterns.md](../../../docs/coverage_patterns.md);
status of the whole workflow is in [docs/ROADMAP.md](../../../docs/ROADMAP.md) section 2.

## What exists

| Piece | Where | State |
|---|---|---|
| `boustrophedon` grid planner | `src/core/planners/boustrophedon_planner.cpp` | Implemented: strip sweeps over each connected free component, optional connectors, boundary pass, component ordering, polygon region |
| `sweepline` grid planner | `src/core/planners/sweepline_planner.cpp` | Implemented: slices the free space, chains free spans of consecutive slices into serpentine passes around obstacles |
| `spiral`, `zigzag` as **grid planners** | commented out in `src/core/coverage_planner.cpp` | **Stubs, not implemented.** `CoveragePlannerFactory::createPlanner` returns `nullptr` and `getAvailablePlanners()` lists only `boustrophedon` and `sweepline`. The service request text still mentions `spiral` and `zigzag`; they fail with the list of available planners. Cell decomposition is an option of the boustrophedon planner (`decomposition: cells`, see `docs/coverage_patterns.md`) |
| Wall patterns `vertical`, `horizontal`, `zigzag` | `planWallPattern` (`coverage_patterns.hpp`) | Implemented over either grid planner (default `sweepline`): panels (one base stand each), a bottom pass, tool turn. `zigzag` here is a wall pattern (every row the same way, diagonal swing back), not a grid planner |
| Scan to plan | `planCoverageFromScan` (`coverage_from_scan.hpp`) | Library: plane fit (`scene_model::fitPlane`), rasterise with obstacles inflated per tool, plan a pattern, lift with the tool profile per part |
| Pass expansion and lift | `coverage_pass_expander`, `wall_pose_lift`, `wall_rasterizer` | Implemented: standoff, tool orientation, multi-pass tools (double pass) |
| Wall scene loader | `wall_scene_loader`, `wall_observed_mask_pcd` (needs PCL) | Reads `wall_scene.json` and `walls.pcd` |

The wall patterns and `planCoverageFromScan` are **library only**: no ROS node calls them. The nodes
below use `boustrophedon` through the grid service.

## Nodes

- **`coverage_planner_node`** (`src/ros/coverage_planner_node.cpp`): synchronous service
  `~/generate_coverage_path` (`coverage_planner_msgs/srv/GenerateCoveragePath`) turning an
  `OccupancyGrid` (embedded in the request, else the last one on `map_topic`) into a `nav_msgs/Path`.
  The response carries `MotionPlanSegment`s and labels (strips are cartesian, connectors free-space;
  `requires_contact` follows the request). Publishes `~/coverage_path`, `~/coverage_markers`,
  `~/coverage_diagnostics`. Parameters and defaults: `config/coverage_planner.yaml` (pattern
  `boustrophedon`, swath 0.5, step 0.4, vertical sweeps). The request's `pose_mode` selects the waypoint orientation convention;
  the tool-specific orientation and standoff are applied by the pass expander and lift.
- **`wall_coverage_service_node`**: `~/start` and `~/stop` (`std_srvs/Trigger`). On start it loads a
  `wall_scene.json` and `walls.pcd` (node parameters), builds the coverage grid for the tool, calls
  the grid service with pattern `boustrophedon`, expands passes and lifts poses onto the wall, and
  publishes a `coverage_planner_msgs/CoveragePlan` on `~/coverage_plan`. Nothing in the repo
  subscribes to that topic yet (the manager reads a JSON file).
- **`wall_coverage_demo_node`**: one-shot demo of the same flow with RViz topics; optional JSON dump.

Not there: an action interface, tests of the node callbacks (only `test/ros/conversions_test.cpp`), a
ROS `CoverageSource` for the manager.

## Launch

```bash
ros2 launch coverage_planner coverage_planner.launch.py map_topic:=/map
ros2 launch coverage_planner wall_coverage_service.launch.py     # defaults to the packaged test data
ros2 launch coverage_planner wall_coverage_demo.launch.py wall_scene_json:=/path/to/wall_scene.json show_rviz:=true
```

Tools come from `config/tool_profiles.yaml` (`tool_profiles_file`, `tool_name`, default `contact_wiper`; per-field overrides allowed); see docs/tools.md. A tool's fixed tilt (`geometry.tilt_deg` / `tilt_axis`) is applied by the lift before the edge tilt and projected into the swath; both default to off.

`scripts/example_client.py`, `publish_3m_2m_wall.py` and `sample_map_publisher.py` exercise the service
and publish test grids.

## Boustrophedon in short

Inflate obstacles (`obstacle_inflation_radius`), 4-connect free cells into components (drop those under
`min_component_cells`; the largest only unless `plan_all_components`), optionally clip to a polygon,
sweep each component along `sweep_direction` (`horizontal`, `vertical`, `auto`) at `step`, alternating
direction and splitting a line around obstacles. Options: strip connectors (rejected whole if they would
cross a blocked cell), a rectangular boundary pass, nearest-neighbour component ordering. `sweepline`
honours the same parameters and adds `ensure_coverage_within_boundary` (a last pass flush with the far
edge).

## Tests

69 gtest cases (counted from the `TEST` macros): `boustrophedon_planner_test` (15),
`coverage_patterns_test` (12, parameterised over both grid planners), `wall_scene_test` (11),
`sweepline_planner_test` (10), `wall_pose_lift_test` (8), `coverage_from_scan_test` (7),
`conversions_test` (6). Not covered: the node callbacks and component ordering.

```bash
colcon test --packages-select coverage_planner && colcon test-result --verbose
```
