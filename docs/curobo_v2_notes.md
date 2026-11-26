# cuRobo v2 integration notes

Research notes for using [cuRobo v2](https://github.com/NVlabs/curobo) as the
planning backend and as the perception/world-model source. Written from the
upstream source at commit `78fd485` (2026-09-10; the pin is now v0.9.0, see the last section) and the
[cuRoboV2 paper](https://arxiv.org/abs/2603.05493). Re-check upstream before
relying on any API below; v2 is a rewrite of v1 and still moving.

- Docs: <https://nvlabs.github.io/curobo> - License: Apache-2.0
- v1 API is **not** compatible; v1 is pinned at tag `v0.7.8`.
- Stack: Python >= 3.10, PyTorch >= 2.5 (CUDA 12.x) or >= 2.9 (CUDA 13), Warp. Needs an NVIDIA GPU.
- Public API is the top-level `curobo.*` modules; `curobo._src.*` is internal.

## Public modules

| Module | What it gives us |
|---|---|
| `curobo` (`MotionPlanner`, `MotionPlannerCfg`) | IK + trajopt + graph planner |
| `curobo.scene` | `Scene`, `SceneData`, `Cuboid`, `Sphere`, `Capsule`, `Cylinder`, `Mesh`, `VoxelGrid` |
| `curobo.perception` | `Mapper`, `MapperCfg`, `FilterDepth`, `RobotSegmenter`, pose detectors |
| `curobo.types` | `JointState`, `Pose` (wxyz quaternion), `GoalToolPose`, `CameraObservation`, `LidarObservation`, `DeviceCfg` |
| `curobo.collision_checking` | `RobotCollisionChecker` for custom pipelines |
| `curobo.kinematics`, `inverse_kinematics`, `trajectory_optimizer`, `model_predictive_control` | standalone solvers |

## Planning API

```python
from curobo import MotionPlanner, MotionPlannerCfg
cfg = MotionPlannerCfg.create(
    robot="robot.yml",            # path / dict / RobotCfg
    scene_model=scene_dict,       # optional initial world
    collision_cache={"cuboid": 16, "mesh": 16, "voxel": 1},  # preallocate obstacle slots
    num_ik_seeds=..., num_trajopt_seeds=...,
    position_tolerance=0.005, orientation_tolerance=0.05,
    max_goalset=1, interpolation_dt=0.025,
)
planner = MotionPlanner(cfg)
planner.warmup()                                   # CUDA graphs; do once
res = planner.plan_pose(goal_tool_poses, current_state)   # GoalToolPose, JointState(2D)
planner.plan_cspace(goal_state, current_state)
planner.plan_grasp(...)                            # approach/grasp/lift
planner.update_world(scene_cfg)                    # swap world at runtime
planner.clear_scene_cache()
planner.enable_link_collision([...]) / disable_link_collision([...])
planner.attachment_manager                         # attach objects to links
```

- `plan_pose` returns `TrajOptSolverResult`: `.success` (tensor), `.js_solution`,
  `.interpolated_trajectory` (JointState at `interpolation_dt`), `.interpolated_last_tstep`,
  `.solve_time`, `.total_time`. Goalset is auto-detected from `GoalToolPose.num_goalset`.
- Batch variants: `batch_motion_planner`. Reactive: `model_predictive_control`.
- **Preallocate `collision_cache`** sizes large enough for every obstacle you will ever
  `update_world` with; CUDA graphs are captured against those buffer sizes.
- Keep CUDA graphs on in production (`curobo.runtime`), off only for debugging.
- cuRobo plans in joint space for the robot YAML's `cspace.joint_names`; the mobile base/lift
  must be either fixed joints or part of that chain.

## World / scene representation

`Scene` (alias of `SceneCfg`) holds lists: `cuboid, sphere, capsule, cylinder, mesh, voxel`
(+ generic `objects`). All obstacles take `name` and `pose = [x, y, z, qw, qx, qy, qz]`
(**wxyz**, unlike ROS xyzw - convert at the boundary).

| Type | Fields |
|---|---|
| `Cuboid` | `dims [x,y,z]` |
| `Sphere` | `radius`, `position` |
| `Capsule` | `radius`, `base`, `tip` |
| `Cylinder` | `radius`, `height` |
| `Mesh` | `file_path` \| `file_string` \| `urdf_path` \| `vertices`+`faces`, optional colors/normals/uvs, `scale` |
| `PointCloud` | `points` (N,3) - converted to a mesh via `Mesh.from_pointcloud`; **not** a native collision primitive |
| `VoxelGrid` | `dims`, `voxel_size`, `feature_tensor` (ESDF, fp16/fp32, shape (nx,ny,nz)), `xyzr_tensor` |

- Scene YAML/dict form: `{cuboid: {name: {dims, pose}}, mesh: {name: {file_path, pose}}, ...}`;
  `Scene.create(dict)` parses it.
- `VoxelGrid` **must contain an ESDF**: negative inside obstacles, positive outside, metres.
  Voxel center `world = center + (i - (n-1)/2) * voxel_size` (pose is the grid center).
- So a raw `.pcd` is not directly collidable: either voxelize/TSDF it through `Mapper`,
  or turn it into a mesh (slow/lossy for big clouds). **Meshes and ESDF grids are the
  first-class citizens.**
- Runtime path from perception into the planner: `Mapper.compute_esdf()` returns a
  `VoxelGrid`; wrap it in `Scene(voxel=[grid])` and `planner.update_world(scene)`.

## Perception (`curobo.perception`)

- `CameraObservation`: `depth_image` (B,H,W; `depth_to_meter` default 0.001),
  `rgb_image` (BxHxWxC), `intrinsics` (B,3,3), `pose` (`Pose`, camera in world),
  optional `image_segmentation`, `feature_grid`. Also `get_pointcloud()`.
  `LidarObservation` is supported by the mapper too.
- `FilterDepth(image_shape)` - denoise depth **before** integration.
- `Mapper(MapperCfg(extent_meters_xyz=..., voxel_size=0.005, esdf_voxel_size=0.05,
  truncation_distance=0.04, depth_minimum_distance=0.1, depth_maximum_distance=10.0,
  decay_factor, frustum_decay_factor, block_size=8, grid_center=...))`
  - `integrate(camera_observation=..., lidar_observation=...)` fuse a frame into block-sparse TSDF
  - `compute_esdf(esdf_origin=None, esdf_voxel_size=None) -> VoxelGrid` (GPU, PBA+; ESDF is
    coarser than TSDF by design; `esdf_origin` supports a sliding window)
  - `update_static_obstacles(SceneData)` stamp known cuboids/meshes (no decay, `min()`-combined
    with the depth channel)
  - `clear_region(min, max)`, `reset()`, `save_blocks()/load_blocks()/import_blocks()`
  - `extract_mesh()`, `extract_textured_mesh()`, `extract_occupied_voxels()`, `render_*`
- `RobotSegmenter` removes the robot's own pixels from depth (needs robot YAML + `JointState`).
  Do this before integration or the arm becomes an obstacle to itself.
- Decay lets old observations fade in dynamic scenes.

## Robot description

- Robot YAML (`robot_cfg.kinematics`): `base_link`, `asset_root_path`, `collision_link_names`,
  `collision_sphere_buffer`, `cspace{joint_names, default_joint_position, max_acceleration,
  max_jerk, ...}`, tool frames, link-sphere approximations. XRDF import is supported
  (`xrdf_util.convert_xrdf_to_curobo`); `curobo.robot_builder` / `robot_parser` build from URDF.
- Collision spheres are fit per link (`curobo.sphere_fit`); quality of these drives both
  speed and conservativeness.

## Mapping onto this repo

| This repo | cuRobo v2 |
|---|---|
| `ScenePort::updateWorldGeometry` (`points` + `boxes`) | `Scene(cuboid=..., mesh=..., voxel=...)` + `planner.update_world` |
| `WorldGeometryBox` | `Cuboid` (xyzw -> wxyz, full extents = `dims`) |
| `WorldGeometryPoint` cloud | voxelize via `Mapper`/ESDF, **not** raw points |
| wall-contact ACM exclusion (`kWallOctomapCollisionObjectId`) | no ACM; use `disable_link_collision` / `attachment_manager`, or keep the wall out of the collision world and rely on the compliant controller |
| `Planner`/`CartesianPlanner` result | `TrajOptSolverResult.interpolated_trajectory` -> `trajectory.hpp` |
| MoveIt `PlanningScene` input | convert: primitives -> `Cuboid/Sphere/...`, meshes -> `Mesh(vertices, faces)`, octomap -> voxelize |

## Integration shape (decision)

Backends are **pluggable, not exclusive**. MoveIt (OMPL / Cartesian), cuRobo v2 and any
future backend (e.g. VAMP) all implement the same ports (`ScenePort`, `Planner`, IK,
collision, time parameterization); the active one is chosen by config per request or per
segment role. cuRobo is Python/CUDA, so its adapter is a small Python node
(`curobo_bridge`) exposing ROS services/actions that mirror those ports; the C++ side
never links cuRobo directly.

## Open questions

- Exact v2 docs for point-cloud-native input were not found; treat `PointCloud` as
  mesh-convertible only and verify against the current docs before depending on it.
- Mobile base + lift + arm as one kinematic chain vs. arm-only planning with base fixed.
- Contact (wall touching) semantics under cuRobo's collision cost.

## Status: `curobo_bridge` (first adapter, verified on an RTX 3050 4 GB)

`src/manipulation/curobo_bridge` is a ROS-free Python core over cuRobo v2 (`78fd485`):
`CuroboPlanner.plan_joint` (`plan_cspace`), `plan_pose` (`plan_pose`), `set_world` / `clear_world`,
structured `PlanResult` (success, failure code, failure stage, message, joint names, rows, dt) in
the same shape as the motion_planner ports. A ROS node and a C++ `Planner` client go on top of it.
Tested in the `gpu` image (`docker run --gpus all ... pytest`); the GPU tests skip without CUDA.

Facts learned integrating it:

- cuRobo v2 needs no nvcc: it uses NVIDIA Warp kernels, JIT-compiled on first use (the first
  `warmup()` took ~40 s, cached afterwards in `~/.cache/warp`).
- The obstacle cache is allocated when the planner is built: pass `collision_cache={"cuboid": N}`
  to `MotionPlannerCfg.create` or `update_world` fails with `'NoneType' ... load_collision_model`.
  We expose it as `max_boxes`; explicit boxes beyond it raise.
- `SceneCfg` (exported as `curobo.scene.Scene`) has no raw point-cloud field. Sparse, already
  filtered points are voxelized into cubes that only ever grow when capped (conservative). Dense
  sensing should use cuRobo's own ESDF mapper (`Mapper`) instead.
- Interpolated plans include locked joints (Franka's two finger joints, 9 columns for a 7-DOF arm);
  the bridge returns only the active joints, in planner order.
- Poses and quaternions are `[x y z]` + `[w x y z]`; results come back as `(1, 1, T, dof)` tensors.

Done since this was written (see the sections below): the robot config for our arm
(`docker/tools/generate_curobo_robot.py`), the ESDF world path (`curobo_bridge/planner.py`), the ROS
node and services (`curobo_bridge`, `curobo_bridge_msgs`) and the C++ client (`curobo_planner`, a
`Planner` + `ScenePort` with IK and state / motion validity).

Runtime selection of the backend is done (pluginlib plugins chosen by the manager's
`planning.yaml`, MoveIt by default; see [backend_selection.md](backend_selection.md)).

Not done: pose-based collision checks, a wall-contact equivalent, more than one planner per
process, and live mapping (nothing produces an ESDF from a sensor yet).

## Our own robot in cuRobo (verified)

```
docker run --rm --gpus all -v "$PWD/robots:/robots" -v "$PWD/docker/tools:/tools:ro" motion-planner:gpu \
  /opt/venv/bin/python /tools/generate_curobo_robot.py --urdf /robots/meshed/robot.urdf \
  --tool-frame tcp_link --out /robots/mm_arm.yml
```

- Same mesh-converted URDF as the VAMP pipeline (`docker/tools/primitives_to_meshes.py`): cuRobo's
  `RobotBuilder` fits spheres (MorphIt) and samples the collision matrix; 396 spheres on 12 links,
  ~80 s on the 4 GB GPU, 38.5% of sampled poses collision-free (VAMP's Cricket model: 31.6%).
- The config stores **absolute** paths to the URDF and mesh root, so mount them at the same path
  when planning (`/robots`). `CuroboPlanner("/robots/mm_arm.yml")` then plans (1.5 s joint move) and
  correctly fails with `no_path` for a slab through the arm.
- The `Start or End state in collision` lines cuRobo prints during warmup are its own sampling of
  invalid seeds, not errors.

## ROS shell: `curobo_planner_node`

`curobo_bridge` ships `curobo_planner_node` (entry point `curobo_bridge.ros_node:main`) with
interfaces in `curobo_bridge_msgs`: `~/plan_joint`, `~/plan_pose` (ROS xyzw quaternion in, cuRobo
wxyz out), `~/set_world` (boxes + an optional PointCloud2 voxelized into the obstacle cache; an
empty request clears the world). Parameters: `robot` (bundled name or absolute path to a generated
config), `max_boxes`. If the planner cannot start (no GPU, bad config) the node stays up and the
services answer `failure_reason_code: unavailable` instead of hanging callers. Tested in the `gpu`
image: logic tests with a stand-in planner (no GPU needed) and a real-planner test through the
services.

## C++ client: `curobo_planner`

`curobo_planner::CuroboClientPlanner` implements the `motion_planner::Planner` port by calling the
node's services: `plan()` (joint goal, or exactly one pose goal), `setWorld()` / `clearWorld()`
(boxes + points with `points_origin` applied), `ik()` (pose as xyz + quaternion or xyz + rpy, optional
seed) and `collision()` (joint values and/or a trajectory; pose-based inputs are `unsupported`). The node passed in must be spun elsewhere (calls wait on the service future); a missing server
yields `unavailable` after `wait_for_service_sec`, never a hang.

Verified: 6 tests against a stand-in server, plus a gated end-to-end test (`CUROBO_START=...`,
node running) in the `full` image: C++ client -> ROS -> Python node -> cuRobo on the RTX 3050 ->
trajectory that reaches the goal, and `no_path` once a slab swallows the robot. Start the node with
the venv Python (`/opt/venv/bin/python -m curobo_bridge.ros_node`): `ros2 run` uses the system
Python, which has no torch, and the node then answers `unavailable`.

## ESDF world (verified on the GPU)

Reserve a voxel grid when the planner is built, then points (or a ready-made field) become one
distance field instead of thousands of cubes:

```python
planner = CuroboPlanner('franka.yml', esdf=EsdfSpec(dims=(2.0, 2.0, 2.0), voxel_size=0.05, center=(0.3, 0, 0.5)))
planner.set_world(points=cloud)            # points -> ESDF on that grid
planner.set_world(esdf=field, esdf_center=...)   # or a field from the scene model / a mapper
```

- cuRobo allocates the voxel cache at construction (`collision_cache={"voxel": {"layers": 1, "dims":
  [...], "voxel_size": ...}}`), so grid size and voxel size are fixed; the centre can move per update.
  Voxel counts per axis must be even (the cell centre formula is `(i + 0.5 - n/2) * voxel`).
- **Convention:** positive outside obstacles, negative inside (the empty cache starts at +10000).
  Our `EsdfGrid` uses the same, so a field goes in unchanged. (`VoxelGrid.get_occupied_voxels` reads
  as if the sign were the other way round; do not trust it.)
- **Layout:** x-major (`ix * ny * nz + iy * nz + iz`), i.e. a C-order `(nx, ny, nz)` array flattened.
  Checked end to end: an obstacle blob at the robot's tool position blocks the start pose, the same
  blob with x and z swapped does not.
- `update_world` keeps the previous voxel layer unless a new one is given, so `clear_world` submits
  an all-free grid; otherwise a stale ESDF would stay in the world silently.
- `points_to_esdf` (numpy + scipy EDT) is the bridge for sparse filtered clouds; dense depth should
  use cuRobo's own `Mapper`. The node takes `esdf_dims`, `esdf_voxel_size`, `esdf_center` parameters.

## IK and validity checks

`CuroboPlanner.solve_ik` (cuRobo `ik_solver.solve_pose`, collision-free, seedable),
`check_states` (joint limits + self-collision + world, via the graph planner's feasibility check)
and `check_motion` (the same on interpolated states, largest joint step `max_step`; a sampled check,
so gaps thinner than `max_step` can be missed). ROS: `~/solve_ik`, `~/check_motion`; the C++ client's
`ik()` / `collision()` call them. Checked on the GPU: the default pose is valid; out-of-limits, a folded
(self-colliding) pose and a pose inside an obstacle are not; IK of a pose under an obstacle fails
with `no_solution`.

### One planner per process

Building a planner with a voxel cache (an ESDF grid) while a plain planner already runs in the same
process breaks the plain one: Warp specialises its collision kernels on the cache layout, and the
older planner then fails with `illegal memory access` (two plain planners are fine, and a lone ESDF
planner is fine). The node builds exactly one planner, so deployment is unaffected; in tests call
`CuroboPlanner.close()` before building one with a different layout (the ESDF tests live in their own
module for this reason).

## Our C++ ESDF into cuRobo (verified end to end)

`CuroboClientPlanner::setEsdf(const scene_model::EsdfGrid&)` -> `~/set_esdf` ->
`CuroboPlanner.set_esdf`, which replaces only the distance field and keeps the explicit boxes.

- Layout: scene_model is x-fastest (`i + nx * (j + ny * k)`), cuRobo x-major
  (`ix * ny * nz + iy * nz + iz`); the client reorders.
- Origin: scene_model's `origin` is the centre of voxel (0,0,0); cuRobo wants the centre of the whole
  grid box, `origin + voxel * (n - 1) / 2` per axis.
- The node must run with an ESDF grid of the same shape and voxel size (`esdf_dims`,
  `esdf_voxel_size`, `esdf_center`); a mismatch is answered with a message, not applied.
- Checked on the GPU: a voxel blob at the robot's tool turned into an EsdfGrid (`computeEsdf`) blocks
  the plan; an all-free field of the same grid unblocks it.

## cuRobo `Mapper` standalone: evaluation on the 4 GB GPU (measured 2026-11-18)

Question from the roadmap: does `curobo.perception.Mapper` run standalone, and what does it cost in VRAM on the RTX 3050 Laptop (4 GB)?
Answer: it runs standalone (no ROS, no dataset) in the `motion-planner:gpu` image, needs about 0.2-0.3 GB at the sizes we need and about 1.4 GB
at 6x6x2 m / 1 cm, and its ESDF feeds `CuroboPlanner.set_esdf` with a conversion of a few lines. Accuracy has two caveats (below).

Script: `docker/tools/eval_curobo_mapper.py` (synthetic scene: wall plane at x = 1 m plus a 0.3 m box, numpy ray cast into a pinhole depth image,
50 frames on a small arc, clean depth unless `--noise`). Reproduce (about 10 minutes for the whole sweep, one subprocess per configuration):

```bash
docker run --rm --gpus all -v "$PWD":/ws:ro motion-planner:gpu \
    /opt/venv/bin/python /ws/docker/tools/eval_curobo_mapper.py --stress            # table below
docker run --rm --gpus all -v "$PWD":/ws:ro motion-planner:gpu \
    /opt/venv/bin/python /ws/docker/tools/eval_curobo_mapper.py --extent 4 --voxel 0.04 --res 640x480   # one configuration
docker run --rm --gpus all -v "$PWD":/ws:ro motion-planner:gpu \
    /opt/venv/bin/python /ws/docker/tools/eval_curobo_mapper.py --bridge --n-states 6000                # ESDF -> set_esdf
```

### API as used (cuRobo v2 at 78fd485)

```python
cfg = MapperCfg(extent_meters_xyz=(2, 2, 2), voxel_size=0.04, esdf_voxel_size=0.04, extent_esdf_meters_xyz=(2, 2, 2),
                grid_center=torch.tensor([0.5, 0, 0]), truncation_distance=4 * 0.04, depth_maximum_distance=4.0,
                minimum_tsdf_weight=1.0, image_height=240, image_width=320)        # image size is required
mapper = Mapper(cfg)
mapper.integrate(CameraObservation(depth_image=(1,H,W) float32 m, rgb_image=(1,H,W,3) uint8 zeros, pose=Pose.from_matrix(T_world_cam),
                                   intrinsics=(1,3,3)))
grid = mapper.compute_esdf()          # VoxelGrid: feature_tensor (nx,ny,nz) float16 on the GPU, voxel_size, pose = grid centre
```

- Gotchas found: `image_height` / `image_width` are mandatory (scratch buffers are pre-allocated, error text "MapperCfg requires image_height and
  image_width ..."); the ESDF grid is **128^3 at `esdf_voxel_size` (default 0.05 m) unless `extent_esdf_meters_xyz` is set** (it does not follow
  `extent_meters_xyz`); a geometry-only mapper still needs an (all zero, uint8) `rgb_image`; the camera frame is OpenCV (x right, y down, z forward;
  the synthetic frames reconstruct the scene correctly that way); the first `integrate` and the first `compute_esdf` compile Warp kernels
  (about 2.4-4.9 s and 3.9 s, cached under `~/.cache/warp`; a new grid shape recompiles, so a fixed size per process is the practical rule).
- Memory is block sparse: `Mapper.memory_usage_mb()` is the pre-allocated pool, sized from the extent by a surface heuristic
  (`max_blocks` 10000 for 2x2x2 and 4x4x2, 60000 for 4x4x2 at 1 cm). Our scene never came near it (at most 1230 blocks, 0 allocation failures).
- Integration time does not depend on the extent or the voxel size here (1.0-1.7 ms per 320x240 or 640x480 frame, CUDA events, the scene is small
  and static); ESDF time grows with the ESDF cell count (0.2 ms at 25^3, 6 ms at 200x200x100, 36 ms at 400x400x200, 89 ms at 600x600x200).

### Results (RTX 3050 Laptop 4 GB, 50 frames, truncation 4 voxels)

VRAM is the whole device's used memory (`torch.cuda.mem_get_info`, same counter as nvidia-smi, polled every 20 ms) including the CUDA context and
Warp; torch's own `max_memory_allocated` is only about 50 MiB because Warp allocates outside torch. The GPU was otherwise idle (15 MiB used before
the run). "near" = cells 0-0.25 m in front of a surface, "far" = 0.25-1 m, inside the central 1 x 0.8 m window of the wall, excluding the box's shadow;
error = |Mapper ESDF - analytic signed distance|.

| extent m | voxel m | image | seeding | VRAM peak MiB (device total) | integrate ms/frame | ESDF ms | ESDF grid | near error mean/max m | far error mean/max m |
|---|---|---|---|---|---|---|---|---|---|
| 2x2x2 | 0.08 | 320x240 | gather | 192 | 1.04 | 0.18 | 25x25x25 | 0.054 / 0.150 | 0.281 / 0.972 |
| 2x2x2 | 0.08 | 640x480 | gather | 274 | 1.1 | 0.17 | 25x25x25 | 0.079 / 0.243 | 0.508 / 0.978 |
| 2x2x2 | 0.04 | 320x240 | gather | 196 | 1.07 | 0.36 | 50x50x50 | 0.031 / 0.117 | 0.037 / 0.050 |
| 2x2x2 | 0.04 | 640x480 | gather | 278 | 1.19 | 0.36 | 50x50x50 | 0.030 / 0.117 | 0.029 / 0.050 |
| 2x2x2 | 0.02 | 320x240 | gather | 194 | 1.03 | 1.79 | 100x100x100 | 0.028 / 0.211 | 0.014 / 0.049 |
| 2x2x2 | 0.02 | 640x480 | gather | 296 | 1.39 | 1.78 | 100x100x100 | 0.026 / 0.211 | 0.013 / 0.049 |
| 4x4x2 | 0.08 | 320x240 | gather | 192 | 0.99 | 0.26 | 50x50x25 | 0.069 / 0.244 | 0.371 / 0.862 |
| 4x4x2 | 0.08 | 640x480 | gather | 274 | 1.31 | 0.26 | 50x50x25 | 0.131 / 0.246 | 0.471 / 0.851 |
| 4x4x2 | 0.04 | 320x240 | gather | 194 | 1.03 | 0.99 | 100x100x50 | 0.039 / 0.155 | 0.043 / 0.070 |
| 4x4x2 | 0.04 | 640x480 | gather | 276 | 1.43 | 0.99 | 100x100x50 | 0.047 / 0.250 | 0.155 / 0.266 |
| 4x4x2 | 0.02 | 320x240 | gather | 256 | 1.12 | 5.98 | 200x200x100 | 0.022 / 0.191 | 0.014 / 0.039 |
| 4x4x2 | 0.02 | 640x480 | gather | 334 | 1.48 | 5.98 | 200x200x100 | 0.021 / 0.191 | 0.014 / 0.039 |
| 2x2x2 | 0.08 | 320x240 | scatter | 172 | 0.98 | 0.71 | 25x25x25 | 0.034 / 0.102 | 0.252 / 0.947 |
| 2x2x2 | 0.08 | 640x480 | scatter | 254 | 1.16 | 0.79 | 25x25x25 | 0.064 / 0.243 | 0.484 / 0.978 |
| 2x2x2 | 0.04 | 320x240 | scatter | 174 | 1.0 | 0.74 | 50x50x50 | 0.011 / 0.117 | 0.007 / 0.046 |
| 2x2x2 | 0.04 | 640x480 | scatter | 256 | 1.15 | 0.85 | 50x50x50 | 0.012 / 0.117 | 0.011 / 0.046 |
| 2x2x2 | 0.02 | 320x240 | scatter | 172 | 1.05 | 1.46 | 100x100x100 | 0.022 / 0.214 | 0.007 / 0.068 |
| 2x2x2 | 0.02 | 640x480 | scatter | 274 | 1.43 | 1.5 | 100x100x100 | 0.020 / 0.230 | 0.007 / 0.069 |
| 4x4x2 | 0.08 | 320x240 | scatter | 172 | 1.01 | 0.73 | 50x50x25 | 0.049 / 0.244 | 0.325 / 0.783 |
| 4x4x2 | 0.08 | 640x480 | scatter | 254 | 1.36 | 0.81 | 50x50x25 | 0.120 / 0.246 | 0.439 / 0.771 |
| 4x4x2 | 0.04 | 320x240 | scatter | 172 | 1.02 | 1.03 | 100x100x50 | 0.021 / 0.155 | 0.015 / 0.043 |
| 4x4x2 | 0.04 | 640x480 | scatter | 254 | 1.44 | 1.07 | 100x100x50 | 0.029 / 0.237 | 0.123 / 0.237 |
| 4x4x2 | 0.02 | 320x240 | scatter | 234 | 1.16 | 3.87 | 200x200x100 | 0.014 / 0.191 | 0.006 / 0.044 |
| 4x4x2 | 0.02 | 640x480 | scatter | 312 | 1.5 | 3.61 | 200x200x100 | 0.013 / 0.191 | 0.006 / 0.044 |
| 4x4x2 | 0.01 | 640x480 | gather | 812 | 1.39 | 36.49 | 400x400x200 | 0.019 / 0.245 | 0.011 / 0.063 |
| 6x6x2 | 0.02 | 640x480 | gather | 440 | 1.59 | 12.62 | 300x300x100 | 0.021 / 0.191 | 0.014 / 0.039 |
| 8x8x2 | 0.02 | 640x480 | gather | 564 | 1.41 | 21.98 | 400x400x100 | 0.021 / 0.191 | 0.014 / 0.039 |
| 8x8x2 | 0.01 | 640x480 | gather | killed (exit -9, no traceback) | - | - | - | - | - |

Beyond the requested set (stress, 640x480, gather): 4x4x2 at 0.01 m 812 MiB total (ESDF 36 ms); 6x6x2 at 0.01 m 1448 MiB (89 ms); 8x8x2 at 0.015 m
930 MiB (46 ms); 8x8x2 at 0.02 m 564 MiB. 8x8x2 at 0.01 m (800x800x200 ESDF, 128 M cells, about 19 bytes per cell by extrapolation, so roughly
2.5 GB) was **killed**: the container exited with 137 / SIGKILL and no Python or CUDA error text, so this is not a clean `OutOfMemoryError`. The
cause (VRAM, host RAM or the driver) was not established; treat that size as the ceiling.

### What fits in 4 GB with room for a planner

The planner needs 1-2 GB (our `CuroboPlanner` ESDF grid is small), leaving about 2 GB for the map. Every requested configuration (2x2x2 and 4x4x2
at 0.02 / 0.04 / 0.08 m, both image sizes) uses 0.17-0.34 GB, so all of them fit with the planner. The practical limit is the ESDF cell count, about
19 bytes per cell: up to about 70 M cells (for example 6x6x2 at 1 cm, 1.45 GB) still leaves about 2.5 GB. Not measured: the Mapper and a planner in
the same process (the planner's 1-2 GB is the working assumption, not measured here), several cameras,
colour, lidar, `enable_static`, or a moving window (`esdf_origin`).

### Correctness against the analytic distance

- **Sign convention: positive outside obstacles, negative inside**, the same as our `EsdfGrid` and cuRobo's `VoxelGrid` for the planner (min over the
  grid was -0.10 m, max +1.41 m on the 2 m grid). Inside is only meaningful in a thin band (about the truncation distance) behind a seen surface: depth
  cannot see the interior of the wall or box, so deeper cells are not negative (sign agreement inside the 0.25 m band was 0.07-0.67, i.e. do not rely on
  depth-only maps for penetration depth). An empty mapper returns +10000 everywhere (so does a grid with no surface), clamp before use.
- **Voxel size**: from 0.04 m down, the far-field error is 0.4-1.4 cm with `seeding_method="scatter"` and about one voxel (4 cm at 0.04, 1.4 cm at 0.02)
  too small with the default `"gather"`. Gather is the dilated seed band the docstring describes (it is on the conservative side: the surface appears
  about one voxel in front of the truth, wall zero crossing at x = 0.96 for 0.04 m instead of 1.00; scatter gave 1.00). Near-surface mean errors are
  1-3 cm at 0.02-0.04 m, with maxima of 0.12-0.25 m at box edges (silhouette cells).
- **0.08 m is too coarse** with 4 voxel truncation and 8^3 blocks: far-field error 0.25-0.5 m (mean). Spurious near-zero cells at the truncation edge
  pull the ESDF down; use 0.04 m or finer.
- **Under-estimates in the far field at 640x480 for 4x4x2 at 0.04 m** (mean 0.12-0.16 m against 0.015-0.043 m at 320x240 or at 2x2x2): about 60 isolated
  cells about one truncation distance in front of the box edge read as surface (a depth-discontinuity artefact). It errs towards "closer than real", so
  it is conservative for planning, not a miss. `FilterDepth` (default bilateral 3 + flying pixel 0.5) made this configuration worse, not better
  (far error 0.44 m clean, 0.41 m with 5 mm noise and `scatter`; 0.30 m with noise and no filter); the cause was not pinned down.
- The wall appears as a slab about one truncation distance thick (the back side of the band is also zero), which is fine for collision.
- Not tested: depth noise beyond 5 mm, moving objects (`decay_factor`), real sensor frames, rolling shutter or pose error.

### Feeding the Mapper's ESDF to `CuroboPlanner.set_esdf`

Compatible, with conversion (checked: `set_esdf` accepted it, the arm then sees the wall and box):

| | Mapper `compute_esdf()` | `CuroboPlanner.set_esdf` |
|---|---|---|
| layout | `(nx, ny, nz)`, X slowest, Z fastest, cell centre `centre + (i + 0.5 - n/2) * voxel` | the same (`EsdfSpec`) |
| sign | positive outside, negative inside | the same |
| origin | `VoxelGrid.pose[:3]` is the grid centre (= `MapperCfg.grid_center` or `esdf_origin`) | `center=` argument |
| dtype / device | float16, CUDA | float32 numpy |
| unobserved / empty | +10000 | clamp to the planner's range |
| voxel counts | `ceil(extent / voxel)`, can be odd (2 m at 0.08 m gives 25) | must be even, else `ValueError` |

Conversion: `field = np.minimum(grid.feature_tensor.float().cpu().numpy(), 1.0)`, build `EsdfSpec(dims, voxel_size, center=tuple(grid.pose[:3]))`
with an even count per axis (choose `extent_esdf_meters_xyz` so `extent / voxel` is even), then `planner.set_esdf(field, center=spec.center)`.
The GPU to CPU copy is a round trip (the planner re-uploads); `_voxel_scene` takes a CUDA tensor internally, so a direct GPU path would be a
small follow-up, not done here. No change to `curobo_bridge` was needed.

Functional check (Franka at the origin, 2 m grid at 0.05 m centred at (0.5, 0, 0), 20 frames): of 6000 random joint states 5572 are valid in an
empty world, 5542 with the analytic ESDF of the scene and 5521 with the Mapper's ESDF. The two ESDFs disagree on 27 states: 24 invalid only with the
Mapper (conservative: the dilated band), 3 valid with the Mapper but invalid with the analytic one (parts of the box the camera never saw). Only
30 states touch the obstacles, so this is a smoke test of the layout and sign, not a statistic of planning success; no motion was planned.

### Not verified

Mapper plus planner running together in one process and their combined VRAM; real depth (noise, holes, specular surfaces) and sensor/robot pose
error; the robot self-filter (`RobotSegmenter`); more than one camera; colour, lidar, static obstacles, `clear_region`, a sliding ESDF window; the
SIGKILL at 8x8x2 / 1 cm; the cause of the `FilterDepth` result; ROS integration (a node that subscribes to depth and publishes `SetEsdf`).

## Upgrade to v0.9.0 (cuRoboV2-Fast)

Pin moved from `78fd485` (0.8.0 line, 2026-09-10) to tag `v0.9.0`, commit `42498cf4e7d0d6fbd4ef9fbcec1243c00c888a83`
(`docker/requirements-gpu.txt`). Only the `gpu` image was rebuilt (`docker build -f docker/Dockerfile --target gpu -t motion-planner:gpu .`);
`motion-planner:full` was removed first to make disk room (22 GB free before) and is not rebuilt yet. Installed metadata:
`nvidia-curobo 0.9.0` (the distribution name is unchanged), `direct_url.json` shows the pinned commit. Torch stays 2.8.0+cu128.
The notes above describe the 78fd485 API; every call used by `curobo_bridge` and the Mapper script still works as written.

### What changed for us

Nothing broke and no code in `curobo_bridge` was changed. New upstream things that exist in this version and that we do not use yet:
`MotionPlanner.plan_pose_waypoints` (below), TimeOpt (`timeopt_duration` / `timeopt_time` on the result), B-spline trajectory
optimisation (the interpolated trajectory has 1000 steps for a 12-waypoint plan), payload-aware torque limits, GPU bidirectional RRT,
a C++ runtime. We did not evaluate those.
The Mapper API (`MapperCfg`, `Mapper.integrate`, `compute_esdf`) is unchanged.

### Test results (RTX 3050 Laptop 4 GB, `--gpus all`, one pytest process per module because of the one-planner-per-process rule)

Command: `docker run --rm --gpus all -e HOME=/tmp -v "$PWD":/ws:ro -e PYTHONPATH=/ws/src/manipulation/curobo_bridge motion-planner:gpu /opt/venv/bin/python -m pytest -q /ws/src/manipulation/curobo_bridge/test/<module>.py`

| Module | Result on v0.9.0 |
|---|---|
| `test_planner.py` (joint / pose planning, world boxes, IK, validity) | 16 passed, 1 skipped (the generated-robot test, needs `CUROBO_TEST_ROBOT`), 190 s |
| `test_planner_esdf.py` (ESDF world, set_esdf, IK and validity with an ESDF) | 8 passed, 594 s (cold Warp cache) |
| `test_closest_solution.py`, `test_world_esdf_kept.py` | 5 and 2 passed |
| `test_ros_node.py` | skipped (no rclpy / `curobo_bridge_msgs` in the gpu image) |
| C++ `curobo_planner` end-to-end client test | not run: needs the `full` image (ROS), which was removed for disk |

"Old vs new": the 78fd485 image was overwritten by the rebuild and no per-test counts of the earlier runs were kept in these notes, so there is
no side-by-side; the earlier notes say the same tests passed on 78fd485. The old run was not repeated.

Mapper (`docker/tools/eval_curobo_mapper.py --extent 2 --voxel 0.04 --res 640x480`, gather seeding): works, VRAM peak 278 MiB (same as the 78fd485
table: 278), ESDF 50^3 in 0.4 ms (0.36 before), accuracy identical (near 0.030 / 0.117 m, far 0.029 / 0.050 m mean / max abs, wall at x = 0.96).
Integrate time was 4.6 ms mean against 1.19 ms before, measured while the simulator ran on the same GPU and the host load average was 11 to 14, so
treat the integrate figure as not comparable.

### 7-joint robot: lift + UR10e arm (`lift_arm` group)

Joint names and order equal the manager's `lift_arm` group: `lift_joint, arm_shoulder_pan_joint, ..., arm_wrist_3_joint`. Generated from the
profile `ur10e_telescopic.yaml` (xacro `arm:=ur10e`, the profile's own lift settings, no `lift_fixed_height`), with the full model, mimic stages included:

```bash
docker/tools/generate_robot_assets.sh src/description/mobile_manipulator_description/config/profiles/ur10e_telescopic.yaml \
    --out /tmp/ra/ur10e_telescopic --only urdf                     # robot.urdf (full, lift moving)
# then in the build image (ROS sourced): resolve package:// meshes and turn primitives into STL
python3 docker/tools/resolve_package_uris.py /tmp/ra/ur10e_telescopic/robot.urdf /tmp/ra/ur10e_telescopic/full_resolved
python3 docker/tools/primitives_to_meshes.py /tmp/ra/ur10e_telescopic/full_resolved/robot.urdf /tmp/ra/ur10e_telescopic/meshed
cp -r /tmp/ra/ur10e_telescopic/full_resolved/ur_description /tmp/ra/ur10e_telescopic/meshed/   # primitives_to_meshes copies only its STLs
# in the gpu image (GPU), same paths mounted:
docker run --rm --gpus all -v /tmp/ra:/tmp/ra -v "$PWD/docker/tools:/tools:ro" motion-planner:gpu \
    /opt/venv/bin/python /tools/generate_curobo_robot.py --urdf /tmp/ra/ur10e_telescopic/meshed/robot.urdf \
    --tool-frame tcp_link --out /tmp/ra/ur10e_telescopic/lift_arm.yml
```

- cuRobo's URDF parser handles `<mimic>`: the two stage joints (`lift_stage_1_joint`, `lift_stage_2_joint`) are not part of the 7 actuated
  joints; their links stay as collision links (13 collision links, `base_link` to `tool_mount_link`), and no fixed-stage URDF was needed. The
  generated config lists 7 joints, `tool_frames = [tcp_link]`, lift limits 0 to 1.5 m, default state `[0.75, 0, ...]`. Forward kinematics: the tool
  moves exactly +1.000 m in z when `lift_joint` goes 0 to 1. Not checked separately: the poses of the stage links themselves (a visual check that the stage
  collision cylinders follow the lift).
- Generation took well under a minute on the 4 GB card (not timed exactly) (fitted spheres, default density). Arm limits come from the URDF (the cuRobo velocity / acceleration
  / jerk scales stay at their defaults: the lift's 0.2 m/s from the profile is not in the config; check the time scaling before using it on the real lift).
- `CuroboPlanner(path, prismatic_joints=['lift_joint'])` (so IK weights treat the lift's metres separately). Verified in the gpu image:
  `plan_joint` from lift 0.1 to 0.9 with the arm moving too: success, 177 rows, first row lift 0.10, last 0.90, goal error 7e-8, duration 4.4 s;
  lift-only move succeeds; a 8 x 8 x 0.1 m slab at z = 1.55 through the arm: `no_path` (stage `planning`), and success again after `clear_world`;
  `plan_pose` to a tcp pose succeeds; `tool_frames == ['tcp_link']`. torch peak allocation 0.69 GB (Warp memory not counted). Verification script:
  not committed (about 40 lines over `CuroboPlanner`); the test `test_generated_robot_config_plans_and_respects_obstacles` asserts dimension 6, so it
  skips for the 7-joint file.
- The YAML is 100 KB but is **not committed**: it stores absolute paths to the URDF and meshes (`asset_root_path`), so it only works where it was
  generated, and the repo keeps generated robot assets under the ignored `build/robot_assets/` (see `robot_assets.md`). Regenerate with the commands above.
  The `curobo` stage of `generate_robot_assets.sh` still builds the 6-joint model (lift pinned); it was not changed to also write the 7-joint one.

### `plan_pose_waypoints` (tried once, Franka)

Signature: `plan_pose_waypoints(GoalToolPose[batch, N (1..256), links, 1, 3 or 4], current_state [batch, dof], *, path_tolerance=(m, rad),
final_tolerance=(m, rad), ...)`; the waypoints exclude the current pose. Result: `TrajOptSolverResult` (`success`, `path_position_error`,
`interpolated_trajectory`, `motion_time`).
Try: 12 waypoints on a straight 0.24 m x 0.18 m segment, orientation held, Franka default start. With path tolerance 1 cm: failed in 1.4 s
(path error 1.4 cm), with 3 cm: success in 45 ms (path error 1.7 cm, endpoint error 1.5e-7 m, trajectory 1000 x 9 including the two finger joints).
With `track_orientation=False` and 1 cm it also failed. A box across the path made it return `None` (no result object), so callers must handle that.
Worth exposing: yes for coverage strokes (it is the cuRobo equivalent of a MoveIt Cartesian path, with collision checking on the GPU and time
parameterisation included), but it needs a new bridge method, a service and a `Planner` port method (the port has no waypoint call), so it was not
wired. The tolerance behaviour above (1 cm failing on a straight line) should be understood first; only one start pose and one line were tried.

### Not verified

The C++ end-to-end test and the ROS node tests (need the `full` image); any comparison of planning time or success rate against 78fd485; TimeOpt,
payload / torque limits, the C++ runtime; the 7-joint config in the ROS node or on the real lift limits; VRAM of a whole planner on the 4 GB card in Warp terms
(only torch's allocation was read); `plan_pose_waypoints` with obstacles or on the 7-joint robot.

## Planning times and the RRT route (measured 2026-11-18, cuRobo 0.9.0)

Joint-space planning times on the 158-problem benchmark, the three routes through `plan_cspace` (defaults, GPU bidirectional RRT + trajectory
optimisation, RRT seed only), warmup and first call, VRAM (RRT modes 3.3 GB of the 4 GB card against 1 GB for the defaults) and the ROS node overhead
(3 to 6 ms per call) are in [planner_benchmark.md](planner_benchmark.md). `CuroboPlanner(graph_planner_algorithm='rrt', enable_trajopt=..., enable_graph_attempt=...)`
selects the route; the node uses cuRobo's defaults plus the fallback below.

### Dock poses, trajectory optimisation and the graph-seed fallback (2026-11-19)

With the matrix `generate_curobo_robot.py` produced by default (cuRobo's sampled collision matrix), the default `plan_cspace` failed every
problem with a dock pose of the UR10e benchmark as an endpoint (52 of 158 in the first run, 62 in a regenerated config): the optimiser's
trajectory penetrated 0.5 to 1.7 mm between the `arm_forearm_link` and `arm_wrist_2_link` spheres (two joints apart, not in the ignore list), the
dense validation rejected it (`debug_info['validation']['collision_valid']` false), while the graph seed itself was valid. Evidence, the one-change-at-a-time
table and the numbers are in [planner_benchmark.md](planner_benchmark.md) ("Dock failure: cause and fix"). To look at a failing solve:
wrap `planner._planner.trajopt_solver.solve_cspace` and read `result.debug_info['validation']` (`path_valid`, `collision_valid`,
`limits_and_dynamics_valid`) and `position_error` / `rotation_error`; `planner._planner._last_graph_call_telemetry` and `_last_planning_attempts` say how many
graph calls succeeded.

- Fix 1, the model: `robot_assets.py` passes `--ignore-hops` (default 2, same as the VAMP SRDF) to the generator. Regenerate cuRobo configs
  generated earlier (`generate_robot_assets.sh <profile> --only curobo`); 158/158 afterwards.
- Fix 2, a guard: `CuroboPlanner(seed_fallback=True, fallback_attempts=6, max_attempts=20)` and the node parameters of the same names. When
  `plan_cspace` returns no trajectory, `plan_joint` retries with `config.always_run_trajopt = False` and `config.run_trajopt_on_graph_seed = False`
  (the planner config is mutable; both are put back) and `enable_graph_attempt=0`, which returns the first graph seed that survives cuRobo's dense validation.
  `PlanResult.route` is `'graph_seed'` then (`'trajopt'` otherwise) and the message says the trajectory is not optimised. Cost: the failed first pass runs
  its 20 attempts (6 to 33 s measured), then the fallback 0.3 to 0.4 s. `seed_fallback=False` restores cuRobo's own behaviour.
- Node hardening (review fixes): every service handler turns an exception into a failed response (`failure_reason_code` `internal_error`) instead
  of killing the node; `ik_weights` is validated against the joint count at construction and a zero keeps its place (zero = ignore the joint;
  the node used to drop zeros and leave a short vector that failed every IK call); `prismatic_joints` naming an unknown joint is an error;
  `voxelize_points` refuses a cap of 0 or an unsatisfiable cap (it returned one cube of up to 204.8 m); `check_motion` caps the interpolation at
  100000 states; `PlanJoint` / `PlanPose` responses carry `route` ('trajopt' | 'graph_seed'), which the C++ client maps to
  `PlanningResult::status` `success_graph_seed` (message: not optimised). `CuroboClientPlanner::collision` checks `joint_values` and a trajectory as two requests
  (one request also interpolated the line `joint_values -> trajectory[0]`). GPU tests now check that `clear_world()` and a smaller `set_world()` really
  remove cuboids from cuRobo (they do: the stale-layer behaviour noted for voxels does not apply to cuboids).

### Low-lift docked-to-stow `no_path`: what the code shows and what a GPU run measured (2026-11-22)

Source: the two-stand runs in [sim_mujoco.md](sim_mujoco.md) ("Running a pass with cuRobo as the planner"). The code findings were read from code and config; the measured outcomes follow them.

Found in the code (not hypotheses):

- `generate_curobo_7dof.sh` did not pass `--ignore-hops` (its `CUROBO_SPHERE_ARGS` defaulted to empty), while the 6-joint `robot_assets.py` curobo stage does (default 2). A 7-joint config therefore had cuRobo's sampled collision matrix alone, the configuration that rejected 52 of 158 dock-pose problems above. It now defaults to `--ignore-hops 2` (set `CUROBO_SPHERE_ARGS=` to get the old behaviour). Regenerate `lift_arm.yml` to pick it up.
- A plan with an invalid start ran all `max_attempts` (20) and then the graph-seed fallback, which is where 37 to 40 s of `no_path` comes from, and the code could not say which end was the problem. `plan_joint` now asks the state check (`check_states`) about start and goal first and answers `start_in_collision` / `goal_in_collision` (stage `precheck`) in milliseconds (`precheck_endpoints`, node parameter, default true; fail-open if the check cannot run). The manager's `free_space_fallback` then takes over at once instead of after 40 s.
- Extra ignore pairs for a pair the sampled matrix keeps although the parts never touch: `generate_curobo_robot.py --ignore-pair A:B` and `--ignore-group A,B,C` (merged on top of the hop rule; names are checked against the links that have spheres). Not used by default, because a wrongly ignored pair hides a real self-collision.
- Latency knobs that exist: node parameters `max_attempts` (20), `fallback_attempts` (6), `seed_fallback` (true), `run_curobo_node.sh start --max-attempts N`; plugin `timeout_sec` / `wait_for_service_sec` in `planning_curobo.yaml`. The node is single threaded and a plan call holds it, so a `set_world` or a second plan issued meanwhile waits behind it (that is the `unavailable` / timed-out world load seen while busy). `timeout_sec` must exceed the longest plan the node may run (`max_attempts` times one attempt plus the fallback); with 11 to 16 s per successful plan on the shared 4 GB card, 60 s is tight for a failing one. Lowering `max_attempts` to about 6 trades some success on hard problems for a shorter failure; not measured.

Measured on a GPU (2026-11-22, RTX 3050 4 GB, motion-planner:gpu, EMPTY world, GPU not shared with a sim, warm kernel cache). Command (queued, one GPU job):

```bash
SIMQ_BY=<name> ~/.claude/coordination/motion-planner/simq_add.sh 30 curobo-lowlift "$PWD/docker/tools/curobo_lowlift_run.sh"
# VARIANTS="hops2 sampled hops2_pair" selects configs: --ignore-hops 2 (the default), CUROBO_SPHERE_ARGS= (sampled matrix only),
# --ignore-hops 2 --ignore-pair base_link:lift_stage_2_link. Output: build/lowlift/<variant>/probe.{json,log}
```

`docker/tools/curobo_lowlift_probe.py` takes 19 docked arm poses logged in two-stand MuJoCo passes (`docker/tools/curobo_lowlift_poses.json`; q = lift, then the six arm joints), sets the lift to as-logged / 0 / 0.01 / 0.05 / 0.2 / 0.4, and asks `check_states` (joint limits + self-collision). An independent numpy sphere-pair check (URDF FK, the yml's spheres and ignore table) names the pairs; it agreed with cuRobo on every state of every variant (0 disagreements), so the pairs below are the cause, not a guess. Valid states of 19 by lift:

| config | as logged | 0.0 | 0.01 | 0.05 to 0.4 |
|---|---|---|---|---|
| sampled matrix only (old 7-joint default) | 10 | 0 | 0 | 18 |
| `--ignore-hops 2` (default now) | 12 | 0 | 18 | 18 |
| `--ignore-hops 2 --ignore-pair base_link:lift_stage_2_link` | 18 | 18 | 18 | 18 |

The stow pose is valid in all three.

1. **Confirmed: the old `no_path` was an invalid start (`start_in_collision`), a sphere-model artefact.** With the sampled matrix alone every docked pose at lift 0 to 0.01 is rejected. Pairs: `base_link`/`lift_stage_1_link`, `base_link`/`lift_stage_2_link`, `lift_carriage_link`/`lift_stage_1_link` (overlaps 2.5 to 6.2 mm at lift 0.01, 16 mm at 0), plus `arm_base_link_inertia`/`lift_base_link` (3 mm) at lift 0, and `arm_forearm_link`/`arm_wrist_2_link` (0.7 mm, pose `docked_wall_column_3` only, at every lift). `plan_joint` now answers `start_in_collision` at `precheck` in 0.01 s; with the precheck off the same start is a `no_path` after 3 to 25 s here (`max_attempts` 6; the sim run's 37 to 40 s was 20 attempts plus the graph fallback). MuJoCo does not collide in these poses, so the model, not the robot, is at fault.
2. **True at lift 0.01, false at lift exactly 0.0.** `--ignore-hops 2` makes 18 of 19 poses valid at 0.01 (the lift-stage links fall inside the hop rule); docked to stow then plans in 0.09 to 0.15 s, `route=trajopt` (poses 4 to 8, empty world). It does not help at lift 0.0: `base_link`/`lift_stage_2_link` overlaps 5 mm, all 19 poses invalid. The manager's logged docks are often at lift 0.0000, so with `--ignore-hops 2` alone those starts stay `start_in_collision`. Adding `--ignore-pair base_link:lift_stage_2_link` makes lift 0.0 valid for 18 of 19 (same planning times at 0.01 for poses 4 to 8). What that pair ignore hides: any real contact between the base body spheres and the second lift stage. Those parts are a rigidly mounted column over the base, they do not move relative to each other except along the lift axis, and the overlap is 5 mm at the retracted end only, so it is an artefact of the sphere fit, not a measured clearance; this was not checked against the mesh. Not made the default.
   - Caveat, any config: `docked_wall_column_3` is invalid at every lift (`arm_forearm_link`/`arm_wrist_3_link` 0.3 mm with hops, `arm_forearm_link`/`arm_wrist_2_link` 0.7 mm without). It is a wrist pose, not a lift problem; an ignore for it could hide a real wrist self-collision and was not tested.
3. **A retract-to-0.4 step does not rescue an invalid start and is not needed when the start is valid.** Sampled matrix: lift-only motion docked to 0.4 fails `check_motion` (the start is invalid), folding the arm at lift 0.01 is invalid, and only mid (lift 0.4, arm swung) to stow plans (0.33 s). With `--ignore-hops 2` (0.01) or the pair ignore (0.0) the direct docked-to-stow plan already succeeds, and the retract (`check_motion` true, mid to stow 0.1 to 0.25 s) and fold-first variants succeed too. Not implemented in the manager.
4. The 11 to 16 s plan time under sim load: reproduced on a stand-in for the load, see "Plan time, idle and shared" below.

### Plan time, idle and shared (measured 2026-11-26)

`docker/tools/curobo_plan_timing.py`: the 7-joint `ur10e_telescopic` config (694 spheres, `--ignore-hops 2`), empty world, warm Warp
cache, the 19 logged docked poses at lift 0.05, docked to stow and stow to docked, 2 repeats, so 76 plans per row. Run on an
RTX 5080 (16 GB), not the 4 GB laptop. "Shared" is a stand-in for the simulator: another process looping 4096 x 4096 matmuls on the
GPU and 8 busy CPU processes on the 8 cores the planner may use. "About 3 GB free" is another process holding 12 GB of the card.

| condition | warmup | solved | median | p95 | worst | failed plans (median / worst) | graph-seed plans (median / worst) |
|---|---|---|---|---|---|---|---|
| idle | 98 s | 72/76 | 25 ms | 1.32 s | 5.9 s | 5.5 / 7.7 s | 1.4 / 5.9 s |
| about 3 GB free | 69 s | 72/76 | 26 ms | 1.33 s | 7.3 s | 5.3 / 5.6 s | 1.4 / 7.3 s |
| shared | 179 s | 72/76 | 87 ms | 6.32 s | 26.7 s | 16.8 / 17.7 s | 6.4 / 26.7 s |
| shared and about 3 GB free | 174 s | 72/76 | 87 ms | 4.31 s | 26.7 s | 16.6 / 17.6 s | 5.4 / 26.7 s |
| idle, `max_attempts` 6 | 68 s | 72/76 | 25 ms | 0.45 s | 5.3 s | 2.4 / 2.5 s | 0.5 / 5.3 s |
| shared, `max_attempts` 6 | 191 s | 72/76 | 66 ms | 1.21 s | 13.9 s | 6.0 / 6.7 s | 1.2 / 13.9 s |

- A plan on cuRobo's normal route takes 25 ms idle (all 64 of them 30 ms or less) and about 90 ms shared. The slow plans are the ones
  that fail (`no_path`, 20 attempts) or end on the graph-seed fallback, and they sit on a few poses (docked columns 9, 10 and 18).
  Shared, a failed plan takes about 17 s: the 11 to 16 s seen in the sim passes fits this path, not the normal one.
- VRAM is not the limit: the planner took 1.2 GB, and 3 GB free changed nothing.
- `max_attempts` 6 solved the same 72 of 76 in every condition it was run in. Idle it cut failed plans from 5.5 s to 2.4 s and p95 from
  1.3 s to 0.45 s. Shared it cut failed plans from 16.8 s to 6.0 s, p95 from 6.3 s to 1.2 s and the worst plan from 26.7 s to 13.9 s.
  The graph-seed fallback is not bounded by it (worst 13.9 s shared): it comes after the attempts, from the separate `fallback_attempts`.
- Not measured: a wall in the world, the laptop's own
  GPU and CPU, the simulator itself instead of the stand-in, one run per row. The unsolved four are the same two problems in every
  row (`docked_wall_column_10`, both directions, 2 repeats); why cuRobo cannot plan them is not investigated here.

Not verified: planning with the wall in the world (empty world only); a full docked to stow from lift exactly 0.0 under the manager; the physical clearance of the retracted lift stages against the base (mesh check); the `--ignore-pair` variant on poses other than the 19 logged (columns 0 to 18 of one wall); behaviour on a second robot. Generator side effect found by the run: `--ignore-pair` together with `--ignore-hops` was rejected ("links without collision spheres": the check saw the hop table's frame names); fixed (`generate_curobo_robot.py`, test added). Recommendation only: keep `--ignore-hops 2` as the default; if docks at lift 0.0 are needed, add the base_link:lift_stage_2_link ignore to `CUROBO_SPHERE_ARGS` after a mesh clearance check, or keep the dock lift at 0.01 or above.

Tests (no GPU): `docker/tools/tests/test_curobo_robot_config.py` (ignore-table helpers, the script default), `src/manipulation/curobo_bridge/test/test_endpoint_precheck.py` (precheck logic with a stub state check).

## Jerk limit in the joint limit guard

`curobo_planner/joint_limit_guard.hpp` can also bound jerk (default off: a joint with `jerk` 0 is not checked, so the
velocity/acceleration behaviour is unchanged). Sources, as for the other limits: `max_jerk` in the MoveIt
`joint_limits.yaml` when `has_jerk_limits: true` (the shipped file has it false), the optional fourth field of the inline
`joint_limits` param (`name:v:a:j`), or the plugin param `limit_jerk` (rad/s^3 or m/s^3, applied to every joint that has no
jerk of its own; `limit_safety_factor` applies). Jerk is the finite difference of the finite-difference acceleration and needs
at least four waypoints. In `limit_mode: reject` a trajectory over the jerk limit (and inside velocity and acceleration) fails
with `failure_reason_code: jerk_violation`, naming joint and waypoint; velocity or acceleration over the limit stays
`limit_violation`. In `retime` mode the time axis is stretched by `max(velocity/acceleration scale, cbrt(peak jerk ratio /
factor))` (jerk scales by 1/s^3), re-checked in a bounded loop, never shortened; the path is unchanged. Status: unit tests
built and run on the host with g++/gtest; not built under colcon, not run with the plugin or in the sim. The stock config keeps
jerk off (`# limit_jerk` in `planning_curobo.yaml`), and the value for the real arm is not chosen here.
