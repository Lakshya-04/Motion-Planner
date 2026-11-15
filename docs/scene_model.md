# Scene model (`scene_model`)

Backend-neutral container for everything known about the world, so any input can be given to
the coverage planner or to any motion-planning backend. ROS-free (C++17 + Eigen + yaml-cpp).
Package: [`src/perception/scene_model`](../src/perception/scene_model). 53 tests.

## Types

| Geometry | Notes |
|---|---|
| `Box`, `Sphere`, `Cylinder`, `Capsule` | primitives, local frame + pose |
| `Mesh` | triangles; remembers its source file |
| `PointCloud` | points + optional labels (e.g. wall id) + colors |
| `VoxelGrid` | tri-state occupancy: free / occupied / **unknown** |
| `EsdfGrid` | signed distance in metres, negative inside obstacles |

Every `SceneObject` has an id, a frame, a pose, a **role** (`kObstacle` or `kContactSurface`;
kept separate so a contact exclusion can never disable a real obstacle) and an `is_static` flag.
`SceneModel` adds / upserts / removes by id, filters by role, drops dynamic objects, computes
bounds and merges scenes.

## Inputs

| Input | Loader |
|---|---|
| Point cloud | `loadPcd` (ascii, binary; x y z, rgb, label), `loadPly` (ascii, binary little-endian). No PCL needed. NaN points are dropped; `binary_compressed` is rejected with a clear message |
| Mesh | `loadObj` (polygons triangulated, negative indices), `loadStl` (ascii and binary) |
| Static collision objects, planning scenes | `loadSceneFile` / `loadSceneYaml`: the native layout (`objects:` list with id, type, size, pose, role) **or** a cuRobo-style world dict (`cuboid:` / `sphere:` / ... with `[x y z qw qx qy qz]` poses). Mesh and cloud files resolve relative to the description |
| Voxel / depth-fused maps | `VoxelGrid` / `EsdfGrid` filled by a perception source (cuRobo mapper, nvblox), see [nvblox_notes.md](nvblox_notes.md) |
| MoveIt `PlanningScene` / `CollisionObject`, `PointCloud2` | ROS converters in `src/ros/conversions.cpp` (`scene_model::ros_conv::fromRos` / `toRos`); `octomap` input is not converted |

## Processing

- `voxelize(cloud, voxel_size, min_points)` bins a cloud into an occupancy grid (with a guard
  against absurd grid sizes); `occupiedCentres` goes back to points.
- `computeEsdf(grid, policy)`: exact Euclidean signed distance (separable O(n) transform),
  verified against brute force. `UnobservedPolicy` decides how unknown voxels count:
  `kFree`, `kOccupied` or `kMarkUnobserved` (NaN output), mirroring nvblox's policy.
- `fitPlane` / `fitPlaneForLabel`: RANSAC + PCA plane fit that returns the wall frame the
  coverage planner works in (u along the wall and horizontal, v up, normal), extents, RMS and
  p95 / p99 spread, and inlier indices. This is how a raw scan becomes coverage-planner input
  without a hand-written wall description.

## Outputs to backends

| Backend | Converter |
|---|---|
| cuRobo v2 | `toCuroboWorldYaml(scene)`: primitives and meshes as a world dict; clouds / voxels / ESDF are listed as skipped so they go through the mapper or ESDF path |
| MoveIt, VAMP, cuRobo client | `scene_adapters::toWorldGeometry` (below); an octomap built from points is not done |
| Coverage planner | `fitPlane` feeds `planCoverageFromScan` in the library; a scene-to-coverage adapter is not done |

## Not yet done

A ROS node around the model (topics, services, a live `PointCloud2` -> `SceneModel` ingest; only the
converters exist, and `robot_self_filter` is their one caller), TSDF fusion and mesh extraction on CPU,
ESDF slicing, sliding-window ESDF, and the coverage adapter. The MoveIt, VAMP and cuRobo adapters exist
(`scene_adapters`, below). See [architecture.md](architecture.md).

## Feeding the planners: `scene_adapters`

`scene_adapters::toWorldGeometry(scene, options, &report)` turns a `SceneModel` into the
`WorldGeometryUpdate` every backend consumes through `ScenePort` (MoveIt, VAMP, cuRobo client).
The split follows the object's **role**, because a contact segment may exclude only one blob from
collision checking:

- `kContactSurface` -> `points` (any shape: clouds as is, meshes sampled at `surface_spacing`,
  occupied voxels as centres, boxes / primitives sampled on their faces);
- `kObstacle` -> `boxes`, never excludable: boxes keep pose and size; spheres / cylinders / capsules
  become their bounding box; clouds, meshes and voxel grids become occupied cubes (`obstacle_voxel`,
  doubled until at most `max_obstacle_boxes`, so the obstacle only ever grows).
- Every approximation is listed in the `ConversionReport` (`approximated`, `coarsened`); an
  `EsdfGrid` has no box / point form and is `skipped` (it goes through the planner's ESDF channel, which a later world update keeps).

- Frames: an object in another frame is moved into the scene frame with `options.frame_to_scene`
  (frame id -> pose in the scene frame); without an entry it is `skipped` in the report, never placed
  as if it were in the scene frame. Cube boxes are named `<id>#c<n>` and every box id is unique.
- Obstacle meshes are sampled no coarser than `obstacle_voxel`, and `max_obstacle_boxes` is at least 8.

`VampPlanner` and `CuroboClientPlanner` implement `ScenePort` as well, so the same scene feeds any
backend. They cannot transform: set `Options::base_frame` and an update whose `frame_id` differs is
rejected (with it unset, the update's frame is trusted to be the base frame).
