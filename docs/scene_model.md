# Scene model (`scene_model`)

Backend-neutral container for everything known about the world, so any input can be given to
the coverage planner or to any motion-planning backend. ROS-free (C++17 + Eigen + yaml-cpp).
Package: [`src/perception/scene_model`](../src/perception/scene_model). 41 tests.

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
| MoveIt `PlanningScene`, `PointCloud2`, `octomap` | planned: ROS shell converters (`scene_model/ros/`) |

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
| MoveIt, VAMP | planned adapters next to each backend (boxes, meshes, octomap from points) |
| Coverage planner | wall plane + obstacle regions from `fitPlane` plus the scene's obstacles (planned adapter) |

## Not yet done

ROS shell (topics, services, `PlanningScene` / `PointCloud2` conversion), TSDF fusion and mesh
extraction on CPU, ESDF slicing, sliding-window ESDF, and the MoveIt / VAMP / coverage
adapters. See [architecture.md](architecture.md).
