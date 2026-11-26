---
type: Reference
title: diff_ik_core
description: Package README for the ROS-free differential-IK core -- damped least squares with singular-value-adaptive damping, null-space costs, uniform task scaling at limits, Jacobian quality measures and a pose-error feedback helper.
tags: [diff-ik, jacobian, singularity, null-space, ros-free-core, package-readme]
status: experimental
---

# diff_ik_core

## Purpose

The math for tracking a Cartesian path with joint velocities: a damped-least-squares step that
stays bounded near a singularity, keeps the path direction at the velocity limits and uses the
redundancy of a 7- or 8-joint chain (arm plus lift) for secondary goals. It is the ROS-free core
for a later chainable `cartesian_tracking_controller` (see `docs/development_plan.md`, section 10;
not written yet).

## Tier

ROS-free core (`docs/package_layout.md`). Eigen only; no `rclcpp`, no message packages, no clock,
no I/O. Builds and tests with ROS not sourced. Does not depend on `reachability`: the quality
measures restate its `jointLimitMargin` semantics instead.

## Inputs and outputs

| Function | Input | Output |
|---|---|---|
| `jacobianQuality`, `minSingularValue` | Jacobian (any size) | sigma_min, sigma_max, condition number, Yoshikawa manipulability |
| `directionalGain` | Jacobian, task direction | largest speed along that direction per unit joint speed (velocity ellipsoid), 0 if the direction is out of range |
| `jointLimitMargin` | q, limits | smallest share of range to a limit, 0..0.5, continuous joints ignored |
| `computeStep` | 6xN Jacobian (base frame), twist, axis mask, q, limits, params | `DiffIkResult`: `qd`, sigma_min, damping, scale, null-space share kept, limit hits, per-joint saturation, achieved twist |
| `integrate` | q, previous and commanded `qd`, dt, limits | next q and `qd` (acceleration clamp, semi-implicit) |
| `poseError`, `rotationLog` | current and target pose | 6D error: position difference, rotation vector of `R_t R_c^T` (angle in [0, pi]) |
| `trackingTwist` | feed-forward twist, error, gains | `v = v_ff + Kp e`, each feedback term limited in norm before `v_ff` is added |

Behaviour of `computeStep`:

- Damping: `lambda^2 = 0` above `sigma_threshold`, `(1 - (sigma_min / eps)^2) lambda_max^2` below it
  (Nakamura-Hanafusa / Chiaverini). Near a singularity the joint speed stays bounded and the
  commanded direction is followed less and less; at the singularity the velocity is 0.
- Weights: `joint_weights` (larger = moves less) give a weighted pseudo-inverse, so a heavily
  weighted lift moves little for the task as well as in the null space.
- Null space: `N * sum(gain_i * ascent_i)` with `N` the true projector of the undamped (masked,
  weighted) Jacobian: components along singular directions above `rank_tolerance` (default 1e-6) are
  removed, so with unit weights and row scales the null-space part moves the tool by at most
  `rank_tolerance` times its size, also while the task damping is active (task tracking is damped,
  null-space motion is not). Directions below the tolerance are rank-deficient and left free. Costs: `JointCenteringCost`,
  `LimitAvoidanceCost` (weighted, zone at the ends), `JointEndBiasCost` (keeps one joint, e.g. a lift,
  away from its ends); add your own by deriving `NullSpaceCost`.
- Velocity limits: the whole task part is scaled by one factor so the worst joint reaches its limit
  (direction kept); the null-space part is then cut to what still fits.
- A velocity limit of 0 locks the joint (its velocity is 0 and the other joints do the task); an
  explicit `+infinity` means unlimited. Strict input check: every limit vector must have size N,
  all inputs finite, `q_min <= q_max`, 1 <= N <= 8; otherwise `valid = false`, `status` says why and
  `qd` is zero. `integrate` holds `q` on bad input.
- One `sigma_threshold` applies to a masked Jacobian whose rows mix metres and radians, so its meaning
  depends on robot scale; `row_scale` (per task row, default 1) puts the rows on a common footing.
- Position limits: inside `position_margin` of a limit the velocity into it is zeroed (this can
  deviate from the task, and is counted in `position_limit_hits`). `DiffIkLimits::continuous` flags
  joints explicitly; when it is empty a joint is continuous if its range is not positive or is 2*pi or
  more (a revolute joint limited to +-pi must be flagged `false` to get its stop). If `q` starts
  outside a limit, `integrate` cancels only the step further out and does not snap `q` back.
- Real time: for N <= 8 and at most 6 task rows, `computeStep` and `integrate` use inline
  fixed-capacity Eigen storage and do not allocate (counted in `diff_ik_alloc_test`); user-written
  `NullSpaceCost` classes are outside that guarantee.
- A masked task (`AxisMask`) tracks a subset of x, y, z, rx, ry, rz and leaves the others free.

## Tool correction

`tool_correction.hpp` moves the Jacobian from the flange to the contact point and gives a small
corrective twist to hold a wall standoff and keep the tool square to it.

| Function | Input | Output |
|---|---|---|
| `ToolOffset` | `xyz`, `rpy` (flange -> contact frame, URDF `Rz Ry Rx`, same as `tcp_xyz` / `tcp_rpy` in the calibration record) | `rotation()`, `transform()` |
| `shiftJacobianToTool` | flange Jacobian (6xN, world frame, `[lin; ang]`), `R_world_flange`, offset, frame | contact-point Jacobian: `lin - skew(R p) * ang`, angular rows unchanged; `TwistFrame::Contact` rotates both blocks into the contact axes |
| `toolPoseFromFlange` | flange pose, offset | contact frame pose in the world |
| `planeServoTwist` | wall normal (away from the wall), standoff error (target - measured, m), tool axis, gains and limits | world twist at the contact point: `Kp_d * error * n` along the normal, `Kp_t * (axis x -n)` for tilt, each limited; zero on target |

Feed the shifted Jacobian to `computeStep`; add the servo twist to the path twist.

Not done: no controller plugin, no calibration capture (the offset comes from a record someone
else made), no wear tracking, not run in any simulator. Host gtests only, on kinematic chains.

## Dependencies

`Eigen3` (build and run), `ament_cmake`. Tests: `ament_cmake_gtest`.

## Use on its own

```cpp
#include "diff_ik_core/diff_ik.hpp"
#include "diff_ik_core/pose_error.hpp"

diff_ik_core::DiffIkLimits lim;           // q_min, q_max, qd_max, qdd_max, position_margin
diff_ik_core::DiffIkParams params;        // thresholds, weights, null-space terms
params.null_space.push_back({std::make_shared<diff_ik_core::JointCenteringCost>(), 1.0});

const auto e = diff_ik_core::poseError(current, target);
const auto v = diff_ik_core::trackingTwist(v_ff, e, feedback);
const auto r = diff_ik_core::computeStep(J, v, diff_ik_core::kAllAxes, q, lim, params);
const auto next = diff_ik_core::integrate(q, qd, r.qd, dt, lim);
```

## Tests

Five gtest executables (`jacobian_quality_test`, `diff_ik_test`, `pose_error_test`,
`tool_correction_test`, `diff_ik_alloc_test`) with a generic DH chain helper in `test/chain_helper.hpp` (analytic geometric Jacobian, checked
against finite differences): planar 2R and 3R against the analytic inverse, spatial 6R and 7R,
self-motion that leaves the twist unchanged, a path through a singularity (bounded joint velocity,
stops), uniform scaling that keeps the direction, limits never exceeded over random
configurations, a weighted lift, masked axes, rotation-log sign and wrap near pi, closed-loop
convergence to a target pose, null-space invariance under active damping, input rejection, locked
joints, the continuous flag, integrate from outside a limit, zero heap allocations.

```bash
colcon test --packages-select diff_ik_core
```

## Status and limits

- Simulation-only math: tested on synthetic kinematic chains, not run on hardware or in a simulator.
- There is no controller plugin yet; nothing in the repo calls this package.
- No stability analysis (contact stiffness, loop delay, discretisation) has been done; the closed-loop
  test is one noise-free kinematic case.
- Position zones are a hard velocity stop, not a smooth barrier; acceleration limits are applied only
  in `integrate`, not inside `computeStep`.
- Damping uses the smallest singular value only. The null-space projector is undamped, so near a
  singularity the null-space part can be large in joint space (it is still capped by the velocity
  limits) while it leaves the tool where it is.
- At most 8 joints (`kMaxJoints`); larger chains are rejected.
- Limit-based null-space costs skip joints whose range is 2*pi or more, also when flagged not continuous.
- Orientation error is the rotation vector in the base frame; the tool offset only shifts the Jacobian (no tool-frame pose error).
