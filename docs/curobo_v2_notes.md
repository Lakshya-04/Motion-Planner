# cuRobo v2 integration notes

Research notes for using [cuRobo v2](https://github.com/NVlabs/curobo) as the
planning backend and as the perception/world-model source. Written from the
upstream source at commit `78fd485` (2026-09-10) and the
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

Not done: a cuRobo robot config for our mobile manipulator (URDF + sphere fit via cuRobo's
`robot_builder`), the point-cloud ESDF path, and the ROS / C++ client.

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
