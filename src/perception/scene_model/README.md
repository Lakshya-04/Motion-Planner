# scene_model

Backend-neutral scene container: point clouds, meshes, primitives, voxel grids and signed distance fields in one `SceneModel`, with loaders, voxelisation with an exact ESDF, plane fitting for coverage planning, and exporters for planning backends.

## Tier

ROS-free core library (`scene_model_core`, C++17) plus an optional ROS conversion library (`scene_model_ros`).

## Inputs and outputs

Headers under `include/scene_model/core/`, namespace `scene_model`:

- `types.hpp`: `Box`, `Sphere`, `Cylinder`, `Capsule`, `Mesh`, `PointCloud`, `VoxelGrid` (free, occupied, unknown), `EsdfGrid`, `SceneObject`, `Aabb`.
- `scene_model.hpp`: `SceneModel`.
- `io.hpp`: `loadPcd`, `loadPly`, `loadPointCloud`, `loadObj`, `loadStl`, `loadMesh`, `savePcdAscii`, `loadSceneFile`, `loadSceneYaml`.
- `voxel.hpp`: `voxelize`, `occupiedCentres`, `computeEsdf` (with an unobserved-cell policy).
- `plane_fit.hpp`: `fitPlane`, `fitPlaneForLabel` (RANSAC plus PCA), `PlaneFit`.
- `export.hpp`: `toCuroboWorldYaml`.
- `include/scene_model/ros/conversions.hpp`: conversions to and from ROS and MoveIt messages; built only when `sensor_msgs`, `shape_msgs`, `geometry_msgs` and `moveit_msgs` are found.

## Dependencies

- In this repo: none.
- External: `Eigen3`, `yaml-cpp`; the ROS conversions need the four message packages above.
- Builds and tests with ROS not sourced (the conversion library is then skipped with a warning). No GPU. No simulator.

## Use it on its own

```cmake
find_package(scene_model REQUIRED)
target_link_libraries(my_target scene_model::scene_model_core)
```

```cpp
auto scene = scene_model::loadSceneFile("scene.yaml");
```

## Tests

53 test cases in 6 files (counted from the sources): scene container, I/O, voxel and ESDF, plane fit, export, ROS conversions.

```bash
colcon test --packages-select scene_model
```

## Status and limits

- Unit-tested on synthetic and small sample data; large real scans were not part of the tests.
- Non-box primitives are approximated when exported to box-based backends (see `scene_adapters`).
- The cuRobo export reports skipped objects instead of failing. Nothing here has run on hardware.

## Related docs

- [scene_model.md](../../../docs/scene_model.md)
- [perception_live.md](../../../docs/perception_live.md)
- [nvblox_notes.md](../../../docs/nvblox_notes.md)
