# coverage_planner_msgs

Coverage-planning interfaces: wall planes, obstacle regions, motion-plan segments and the coverage-path service. Split from `coverage_planner` so a consumer can use the types without building the planner.

## Tier

Message package (`rosidl` interfaces only, no code).

## Inputs and outputs

- Service `GenerateCoveragePath`: every request field is an optional override of the node default: `map` (an `OccupancyGrid`), `pattern`, `swath`, `step`, `sweep_direction`, `start_corner`, `pose_mode`, `allow_unknown`, `obstacle_inflation_radius`, `enable_boundary_pass`, `boundary_offset`, `add_strip_connectors`, `requires_contact`, `edge_zone_m`, `edge_spacing_scale`, `region`. The reply has `success`, `message`, a `nav_msgs/Path`, `segments`, `segment_labels`, `total_distance`, `coverage_area`, `num_waypoints`, `planning_time` and `pattern_used`.
- Messages: `CoveragePlan` (a whole wall's plan, the wire twin of the service reply), `MotionPlanSegment` (label, index ranges, `mode`, `planner`, `requires_contact`, `requires_retraction`, `priority`, `tags`), `ObstacleRegion`, `WallPlane` (pose, extents and plane-fit spread values).

## Dependencies

- In this repo: none.
- External: `rosidl_default_generators`, `rosidl_default_runtime`, `geometry_msgs`, `nav_msgs`.
- Needs ROS 2 to generate the interfaces. No GPU. No simulator.

## Use it on its own

```xml
<depend>coverage_planner_msgs</depend>
```

```bash
colcon build --packages-select coverage_planner_msgs
ros2 interface show coverage_planner_msgs/srv/GenerateCoveragePath
```

## Tests

No tests: the package only holds interface definitions. Servers and clients that use it carry the tests (`coverage_planner` (server)).

## Status and limits

- The pattern names a request may use are those the planner offers (see `coverage_planner`).
- Nothing here has run on hardware.

## Related docs

- [coverage_patterns.md](../../../docs/coverage_patterns.md)
- [architecture.md](../../../docs/architecture.md)
