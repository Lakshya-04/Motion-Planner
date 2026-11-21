# scene_adapters

Converts a `scene_model::SceneModel` into the world geometry the planner backends consume (`motion_planner::WorldGeometryUpdate`: obstacle boxes plus contact-surface points).

## Tier

ROS-free core library (C++17).

## Inputs and outputs

- `scene_adapters::toWorldGeometry(scene, options, report)` in `include/scene_adapters/world_geometry.hpp`.
- The split follows the object's role, not its shape: contact-surface objects become points (which a contact segment may exclude from collision checking), obstacles become boxes (never excludable).
- `WorldGeometryOptions`: `surface_spacing` (0.05 m), `obstacle_voxel` (0.05 m), `max_obstacle_boxes` (2000), `unknown_voxels_occupied`, `frame_to_scene` (poses of other frames; an object in another frame without an entry is skipped and reported).
- `ConversionReport`: counts of boxes and points, and lists of approximated, skipped and coarsened objects. Approximations only grow an object.

## Dependencies

- In this repo: `scene_model`, `motion_planner`, `core_types`.
- External: `Eigen3`.
- Builds and runs without ROS sourced, without a GPU, without a simulator.

## Use it on its own

```cmake
find_package(scene_adapters REQUIRED)
target_link_libraries(my_target scene_adapters::scene_adapters)
```

```cpp
auto update = scene_adapters::toWorldGeometry(scene);   // then hand it to any ScenePort
```

## Tests

16 test cases in `test/world_geometry_test.cpp` (counted from the source).

```bash
colcon test --packages-select scene_adapters
```

## Status and limits

- An ESDF grid cannot be expressed as boxes or points and is reported as skipped.
- Cubes are axis-aligned approximations: a conservative over-estimate, not an exact shape.
- Verified by unit tests; nothing here has run on hardware.

## Related docs

- [scene_model.md](../../../docs/scene_model.md)
- [architecture.md](../../../docs/architecture.md)
- [backend_selection.md](../../../docs/backend_selection.md)
