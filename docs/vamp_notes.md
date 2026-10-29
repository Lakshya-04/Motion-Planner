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
