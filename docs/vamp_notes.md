# VAMP integration notes

[VAMP](https://github.com/KavrakiLab/vamp) (Vector-Accelerated Motion Planning, Kavraki Lab,
Apache-2.0) is a CPU sampling-based planner that uses SIMD (AVX2 / NEON) to check many robot
configurations at once. Collision checks take microseconds and free-space plans take
milliseconds, with no GPU and no planning-scene infrastructure. The cuRoboV2 paper
([arXiv 2603.05493](https://arxiv.org/abs/2603.05493)) benchmarks against it as the
state-of-the-art sampling-based baseline ("Motions in microseconds via vectorized
sampling-based planning", with RRT-Connect and AORRTC variants).

It is a third backend next to MoveIt and cuRobo v2, behind the same ports.

## What VAMP is (and is not)

- **Is:** FK + self/world collision checking + RRT-Connect / AORRTC / PRM* + path
  simplification, all vectorized. World = spheres, capsules, cuboids, cylinders and a point
  cloud held in a CAPT (collision-affording point tree).
- **Is not:** an IK solver, a Cartesian planner, or a time parameterizer. Pair it with an IK
  backend (TRAC-IK / KDL, or cuRobo's IK) and TOPP-RA for timing.
- Upstream: `KavrakiLab/vamp` (Apache-2.0). In the cuRoboV2 benchmark (Franka, 2,600
  problems) the sampling-based group reached 98.9% kinematic success but fell to 72-75% with
  a 3 kg payload, since it plans geometry only and times it afterwards; cuRoboV2 held 99.7%.
  Use VAMP for fast free-space transitions and validity checks, not for dynamics-aware moves.

## Robot model: compile-time generated header

VAMP robots are C++ headers generated per robot (+ tool) configuration:

```
URDF + meshes --FOAM--> per-link spheres + spherized URDF --Cricket--> <robot>.hh --> VAMP
```

- [FOAM](https://github.com/CoMMALab/foam) (MIT): spherizes URDF meshes.
- [Cricket](https://github.com/CoMMALab/cricket) (MIT): traces the spherized URDF with
  Pinocchio + CppAD and emits straight-line FK + sphere code. Handles mimic joints when
  Pinocchio is built with mimic support.
- Verify generated FK against Pinocchio (< 1 mm) before trusting a header.
- Headers are large (tens of MB); generate at build time or keep out of git history.
- One header per tool variant if the tool changes collision geometry; an arm-only variant
  (lift pinned as a parameter) gives a much cheaper 6-DOF C-space when the lift is not moving.

## Upstream state (checked at `27cb9b6`, 2026-09-28)

- Header-only C++17 `INTERFACE` target `vamp::vamp`, installable with CMake exports
  (`VAMP_INSTALL_CPP_LIBRARY=ON`); Python bindings optional (`VAMP_BUILD_PYTHON_BINDINGS=OFF`).
- Dependencies pulled with CPM at configure time: Eigen 3.4, `nigh`, `pdqsort`, `cddlib`,
  `SIMDxorshift` (x86). Builds need network once, or a pre-filled `CPM_SOURCE_CACHE`.
- Planners: RRT-Connect, PRM, FCIT, AORRTC, all multi-goal. Simplifiers: shortcut, reduce,
  B-spline, perturb.
- Environment: spheres, capsules, cuboids, **heightfields**, **convex polytopes (GJK)** with
  a mesh -> convex-decomposition helper, point clouds via CAPT. So meshes can be native
  obstacles (as polytopes), not only sampled points.
- Attachments (sphere sets on the end effector) for UR5 / Panda / Fetch, so a held tool can
  be added without regenerating the robot.
- Bundled robots: UR5, Panda, Fetch, Baxter, sphere. Custom robots via FOAM + Cricket.

## How we integrate it

Own layer on top of **unmodified upstream**, so updates are a one-line commit bump:

| Package | Contents |
|---|---|
| `vamp_vendor` | `FetchContent` of `KavrakiLab/vamp` pinned to a commit (bindings off), installs `vamp::vamp` |
| `vamp_robots` | our generated robot headers + the FOAM/Cricket generation script and FK check |
| `vamp_planner` | pimpl facade + runtime robot factory, implementing our `ScenePort` / planner / collision ports; TOPP-RA timing; ROS adapter at the edge |

Never patch upstream files; anything robot- or task-specific lives in our packages.

## Lessons from an earlier production integration

- Wrap VAMP behind one non-template facade (pimpl): robot templates and VAMP/Eigen types stay
  in `.cpp` files. Robot selection = runtime factory over compiled-in robot types.
- **ABI:** VAMP is built with `-march=native` / `-mavx2`. Force `EIGEN_MAX_ALIGN_BYTES=0`
  (or keep Eigen types off the boundary) when linking into nodes built without those flags;
  link PRIVATE, build PIC.
- Point-cloud path: TF to the planning frame -> clip to a capsule around the arm
  (~1.6 m radius) -> voxel downsample (~6 cm) -> `setPointCloud(points, radius)`. Disable
  VAMP's own CAPT filter when this already ran; double filtering degrades the tree.
- Contact work: check the **tool-less** robot against the wall cloud (tool may touch) and the
  full robot against the obstacle cloud; OR the results.
- Pool of planner instances (one per thread) for parallel transition planning; straight-line
  `isMotionInCollision` probe first, plan only if it fails.
- Cap waypoints handed to TOPP-RA (~100) to avoid huge allocations; plan budget = deadline +
  cancel flag (maps onto ROS 2 action cancellation).
- "No world loaded -> refuse to plan" when the task needs one.

## Mapping onto this repo

| Port | VAMP adapter |
|---|---|
| `ScenePort` boxes / primitives | `addCuboid*`, `addSphere`, `addCapsule*`, `addCylinder*` |
| `ScenePort` points | `setPointCloud(points, radius)` |
| meshes | convex decomposition -> `ConvexPolytope`s (or sampled surface points) |
| voxels / ESDF | occupied voxel centers -> point cloud |
| heightfields | native `Heightfield` |
| free-space `Planner::plan` | RRT-Connect / AORRTC + simplify, then TOPP-RA |
| `Planner::collision` | `isStateInCollision`, `isMotionInCollision`, batched validity |
| `Planner::ik` | not provided; delegate to the IK backend |

## Status: `vamp_planner` (first adapter)

`src/manipulation/vamp_planner` implements the `motion_planner::Planner` port over VAMP, ROS-free:
free-space RRT-Connect (+ shortcut/B-spline simplification), `collision()` for states and
trajectories, `setWorld()` from `WorldGeometryUpdate` (boxes -> cuboids, points -> CAPT point
cloud), structured failures (`start_in_collision`, `goal_in_collision`, `no_path`, `no_world`,
`invalid_request`, `unsupported` for IK). Robots are compiled in; `src/robots.cpp` has a one-line
table (bundled `ur5` today, our Cricket-generated headers next). Tested in the `full` image.

Things found while integrating upstream `27cb9b6` (all worked around without patching upstream):

- The installed `vampConfig.cmake` is unusable: it calls `find_dependency` without including the
  macro, exports `vamp::vamp_cpp` but checks for `vamp::vamp`, and its include path is one level
  too deep for `#include <vamp/...>`. We describe the install ourselves in `CMakeLists.txt`.
- `cmake --install` ships only VAMP's own headers, but they include `pdqsort.h`, `nigh/`,
  `simdxorshift128plus.h` and cddlib's headers. The `vamp-build` Docker stage now copies those
  from the CPM tree into `/opt/vamp/include`.
- A single-state check is `validate_motion<Robot, rake, 1>(q, q, env)`; motions use
  `Robot::resolution`.
- VAMP's UR5 model has no collision spheres on the base link, so an obstacle around the base
  alone does not collide; test obstacles must be placed where the arm actually is.

## Generating our own robot (FOAM + Cricket), verified end to end

```
docker/tools/build_robot_tools.sh                       # builds motion-planner-foam / -cricket images
xacro .../mobile_manipulator.urdf.xacro arm:=generic_6dof lift_fixed_height:=0.5 -o robot.urdf
docker/tools/generate_vamp_robot.sh GenericArm robot.urdf - tcp_link build/vamp_robots
cmake ... -DVAMP_PLANNER_ROBOT_DIR=build/vamp_robots   # every <Name>.hh there is compiled in and registered
```

- `lift_fixed_height` pins the lift, giving a 6-DOF arm-only model (the lift as a joint would add
  mimic stages as extra DOF). It is a 1.5 MB header, 54 spheres, generated in ~80 s.
- FOAM spherizes **mesh** collisions only and silently skips URDF primitives, so
  `docker/tools/primitives_to_meshes.py` converts boxes / cylinders / spheres to STL first.
  `package://` mesh URIs must be resolved to relative paths before generation (the script refuses
  otherwise).
- Upstream FOAM's Dockerfile is a developer shell that expects a file missing from the repo, so we
  ship `docker/tools/foam.Dockerfile`; Cricket's own image only contains `fkcc_gen`, so the code
  templates are taken from the pinned checkout.
- **Pass `-` for the SRDF.** An SRDF computed for the primitive model made every sampled
  configuration invalid (0 of 2000): spherization bulges, so two link pairs overlap in the sphere
  model that were free in the source model. Letting Cricket sample the allowed pairs from the sphere
  model itself gave 633 valid of 2000.
- Joint order and names in the header follow the URDF (`arm_joint1..6` for the generic arm), and
  the header's world frame is the URDF root, so planning scenes must be in the robot base frame.

## UR10e: cuRobo works, VAMP/Cricket model fixed (root cause below)

Pipeline used for the real arm: `xacro ... arm:=ur10e lift_fixed_height:=0.5` ->
`docker/tools/resolve_package_uris.py` (copies the 7 `package://ur_description` collision meshes and
rewrites the URIs) -> `primitives_to_meshes.py` (also mirrors collision geometry as visual, because
cuRobo's `RobotBuilder` picks links from visuals even with `use_collision_mesh=True`).

- cuRobo: 716 spheres on 13 links, 44% of random poses collision-free, plans in 1.5 s and fails with
  `no_path` through an obstacle. Needed `--batch-size 1000` (its default 10000 wants >7 GB of GPU
  memory; the 4 GB card ran out).
- VAMP/Cricket (before the fix): header generated (55 spheres, real joint names) but every sampled
  configuration was reported in collision (0 of 2000), including home poses, while cuRobo and the
  generic 6-DOF arm were fine.

### Root cause and fix

Cause: in the sphere model two non-adjacent link pairs overlap at every configuration, so every
configuration is invalid whatever else is allowed. Evaluating the spherized URDF with
`docker/tools/sphere_self_collision.py` and the previous SRDF (rigid + parent/child pairs only)
leaves exactly:

- `arm_base_link_inertia` / `arm_upper_arm_link` (100% of poses): the upper arm's first sphere sits on
  the shoulder-lift axis, about 0.13 m above the base sphere, with radii 0.13 + 0.11. The shoulder
  link is between them, so they are two joints apart, not parent/child. The source SRDF lists such
  pairs as "never collides".
- `lift_carriage_link` / `arm_shoulder_link` (100%): the carriage is rigidly attached to the arm base
  (through sphere-less mount links) and the shoulder is the base's child, but the SRDF had only
  per-link parent/child pairs, not parent/child between rigid groups.

The earlier attempts (Cricket's sampled pairs, an SRDF from cuRobo's matrix, more spheres) all kept
these two pairs, so none changed the result. It was not the base or tool frame, and not the radii.

Fix: `rigid_pairs_srdf.py` now treats each rigid group as one body (the neighbours of a group are
neighbours of every member) and takes `--hops N`, allowing links up to N joints apart. Use `--hops 2`
for a serial 6-DOF arm, then hand the SRDF to Cricket instead of `-`:

```
python3 docker/tools/rigid_pairs_srdf.py --hops 2 spherized.urdf > robot.srdf
```

Verified on the same spherized UR10e arm-only URDF (lift fixed), by compiling the generated header
with VAMP and calling `validate_motion<UR10eArm, FloatVectorWidth, 1>(q, q, empty env)`, the same
call `vamp_planner` uses:

| | old header | new header (`--hops 2` SRDF) |
|---|---|---|
| home `[0,-1.57,1.57,-1.57,-1.57,0]` | collision | free |
| ready `[0,-1.2,1.4,-1.8,-1.57,0]` | collision | free |
| elbow folded back `[0,0,3.1,0,0,0]` | collision | collision |
| arm driven into the lift column `[0,1.5,0,0,0,0]` | collision | collision |
| 2000 random poses free | 0 | 1023 (51%; cuRobo model: 44%) |

`sphere_self_collision.py` independently gives 254 of 500 free with the same pose results. With
`--hops 2` the upper arm is never reported touching the base or wrist 1, which matches the stock UR
SRDF.
