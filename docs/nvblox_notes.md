# nvblox: what to take, and how we would use it

Evidence marks: [V] checked this session (GitHub API / pages), [M] from memory, not re-checked.

## Facts

- **nvblox** (`nvidia-isaac/nvblox`): "a GPU-accelerated TSDF and ESDF library for robots
  equipped with RGB-D cameras" [V]; C++ core with Python and PyTorch bindings (`python/`,
  `nvblox_torch/`) [V]. **Latest release: v0.0.10 (2026-05-07)** [V]; the `main` branch has
  moved since (pushed 2026-10-23) [V]. **License: Apache-2.0** (its licence file; the GitHub API
  reports "NOASSERTION" only because the file is Markdown) [V].
- v0.0.10 adds a Vulkan `nvblox_renderer` and a configurable `unobserved_esdf_policy`
  (`kIgnore`, `kFree`, `kOccupied`, plus `add_negative_truncation_band_sites`) [V].
- **Isaac ROS nvblox** (`NVIDIA-ISAAC-ROS/isaac_ros_nvblox`): the ROS 2 integration. **Latest
  release v5.0-0 (2026-09-22)**, Apache-2.0 [V]. Works with depth cameras and / or 3D lidar;
  its person mode uses a segmentation mask to split humans into a separate reconstruction [V,
  from the Isaac ROS docs]. No robot self-filtering was found in it; use the
  [self filter](robot_self_filter.md) before it [V].
- Layered map design [M]: TSDF fused from depth / lidar, ESDF computed from it, plus color,
  occupancy and mesh outputs; block-sparse storage in GPU hash tables; 2D ESDF slices for
  navigation costmaps; separate static and dynamic reconstructions.
- cuRobo v2 has its **own** GPU TSDF / ESDF mapper (`curobo.perception.Mapper`), so the two
  overlap. The cuRoboV2 preprint reports its ESDF computation at 1.15 ms against 11.93 ms for
  nvblox at 10 mm resolution with full coverage (preprint, unreviewed; see
  [curobo_v2_notes.md](curobo_v2_notes.md)).

## Version policy

Use **released** versions only, never `main`, and bump deliberately: nvblox **v0.0.10**, Isaac
ROS nvblox **v5.0-0**, cuRobo pinned in `docker/requirements-gpu.txt`. Before depending on
Isaac ROS nvblox, verify it supports our ROS distro and how it is distributed (NVIDIA's Isaac
ROS images / apt repository); not checked yet.

## What we take as design inspiration

| Idea | How it shows up here |
|---|---|
| **Unobserved space is a first-class state** | `scene_model` voxel grids carry `kVoxelUnknown`, and `computeEsdf` takes an `UnobservedPolicy` (`kFree`, `kOccupied`, `kMarkUnobserved`) with the same meaning as nvblox's policy, so maps from either source behave alike. Next to a wall, unknown space behind the surface must not look like free space |
| **Layers behind one API** | one `SceneModel` holds clouds, meshes, voxels and ESDFs side by side; consumers ask for the layer they need |
| **Static vs dynamic separation** | `is_static` on every object; `removeDynamic()` clears the re-sensed part. Masks from a person detector route into the dynamic part |
| **ESDF slice for 2D use** | planned: a horizontal slice of the ESDF for base navigation and station selection ([reachability.md](reachability.md)) |
| **Mesh extraction** | planned: surface mesh from the voxel / TSDF layer for visualization and for backends that take meshes |
| **Sliding window around the robot** | the ESDF only needs to cover the workspace near the arm; bounded memory and fast updates |
| **Block-sparse GPU storage** | stays inside the GPU mappers (cuRobo, nvblox); our CPU `VoxelGrid` is dense and meant for static scenes |

## How it fits the modular pipeline

nvblox is one **implementation of the perception-source module**, not a new core dependency:

| Source | Role |
|---|---|
| cuRobo v2 `Mapper` | default for the planner's world model (same process and GPU as the cuRobo planner; depth and lidar observations) |
| Isaac ROS nvblox (v5.0-0) | alternative or additional source through ROS topics (ESDF, mesh, costmap slice), useful for navigation and for visualization |
| `scene_model` CPU voxelizer | static scans and offline scenes, no GPU |

Measured 2026-11-18 ([curobo_v2_notes.md](curobo_v2_notes.md), `docker/tools/eval_curobo_mapper.py`): the cuRobo `Mapper` runs standalone on the
4 GB RTX 3050 (0.17-0.34 GB for 2x2x2 and 4x4x2 m at 2-8 cm voxels, 1.4 GB for 6x6x2 m at 1 cm, about 1-1.7 ms per depth frame, ESDF 0.2-6 ms),
its ESDF has our sign (positive outside) and layout, and it needs a float32 CPU copy, a clamp and an even cell count to go into `set_esdf`. Use
0.04 m or finer; the default `gather` seeding is about one voxel conservative, `scatter` is exact. nvblox itself was not run.

All of them produce the same `scene_model` types (`VoxelGrid` / `EsdfGrid`), and the planning
backends consume those. The self filter runs in front of every source.

## Open questions

- Isaac ROS nvblox on our ROS distro, and the install path (container vs apt).
- Whether two GPU mappers (cuRobo's and nvblox) are worth running together, or one is enough
  for both planning and navigation. Decide with a benchmark on the same recordings.
- Interop format between the sources and `scene_model` (ROS messages for nvblox's outputs vs
  in-process tensors for cuRobo).
