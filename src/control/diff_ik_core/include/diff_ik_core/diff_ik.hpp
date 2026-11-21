/**
 * @file diff_ik.hpp
 * @brief Damped-least-squares differential IK step with null-space costs and limit handling.
 * @details Given the task Jacobian J (6 x N, base frame), a task twist v and the joint positions,
 *          computeStep returns joint velocities
 *
 *              qd = s * J_W^+ v  +  t * (I - J_W^+ J) * qd0
 *
 *          with J_W^+ = W^-1 J^T (J W^-1 J^T + lambda^2 I)^-1 (joint weights W: a large weight moves
 *          that joint less), lambda^2 from the smallest singular value (Nakamura-Hanafusa /
 *          Chiaverini: 0 above the threshold eps, rising smoothly to lambda_max^2 at a singularity),
 *          and qd0 the weighted sum of the null-space costs. Limits:
 *            - velocity: the WHOLE task part is scaled by one factor s <= 1 so the worst joint just
 *              reaches its limit, which keeps the direction of the path; the null-space part is then
 *              reduced (t <= 1) until it fits in what is left, so it never costs path direction;
 *            - position: inside a zone of `position_margin` at a limit, the velocity into that limit
 *              is zeroed (hard stop; this one can deviate from the task, and is reported).
 *          Continuous joints (range not below 2*pi) have no position limit. A task can be a masked
 *          subset of the six axes, which leaves the other axes free and enlarges the null space.
 *          Pure function of its inputs: deterministic, no state, no allocation beyond Eigen vectors.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <Eigen/Dense>

#include <array>
#include <vector>

#include "diff_ik_core/null_space_costs.hpp"

namespace diff_ik_core
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using AxisMask = std::array<bool, 6>;   ///< x, y, z, rx, ry, rz; true = the axis is tracked

constexpr AxisMask kAllAxes = {true, true, true, true, true, true};

struct DiffIkLimits
{
  Eigen::VectorXd q_min;     ///< position lower limits (a range of 2*pi or more = continuous)
  Eigen::VectorXd q_max;
  Eigen::VectorXd qd_max;    ///< velocity limits (> 0); non-positive or non-finite = unlimited
  Eigen::VectorXd qdd_max;   ///< acceleration limits for integrate(); same convention
  double position_margin = 0.0;   ///< width of the hard stop zone at each limited joint's ends
};

struct DiffIkParams
{
  double sigma_threshold = 0.05;   ///< eps: damping starts below this smallest singular value
  double lambda_max = 0.1;         ///< damping at sigma_min = 0
  Eigen::VectorXd joint_weights;   ///< empty = all 1; larger = the joint moves less
  std::vector<NullSpaceTerm> null_space;   ///< empty = no null-space motion
};

enum class JointSaturation { None, Velocity, PositionLimit };

struct DiffIkResult
{
  bool valid = false;                 ///< false on a size mismatch (qd is then zero)
  Eigen::VectorXd qd;                 ///< commanded joint velocities
  double sigma_min = 0.0;             ///< smallest singular value of the (masked, weighted) Jacobian
  double damping = 0.0;               ///< lambda used (not squared)
  double scale = 1.0;                 ///< uniform task scale s in [0, 1]
  double null_scale = 1.0;            ///< share t of the requested null-space motion that was kept
  int velocity_limit_hits = 0;        ///< joints at their velocity limit in the result
  int position_limit_hits = 0;        ///< joints whose velocity was zeroed by a position zone
  std::vector<JointSaturation> saturation;   ///< per joint
  Vector6d achieved_twist = Vector6d::Zero();   ///< J * qd, all six axes
};

/// Differential-IK step; see the file comment. J is 6 x N, q / limits vectors size N.
DiffIkResult computeStep(
  const Eigen::MatrixXd & J, const Vector6d & twist, const AxisMask & mask,
  const Eigen::VectorXd & q, const DiffIkLimits & limits, const DiffIkParams & params);

struct IntegrateResult
{
  Eigen::VectorXd q;
  Eigen::VectorXd qd;
};

/**
 * One semi-implicit step: the velocity moves towards `qd_cmd` by at most qdd_max * dt per joint
 * and is clamped to qd_max, then the position advances with the NEW velocity. A step that would
 * cross a position limit stops at the limit with zero velocity.
 */
IntegrateResult integrate(
  const Eigen::VectorXd & q, const Eigen::VectorXd & qd_prev, const Eigen::VectorXd & qd_cmd,
  double dt, const DiffIkLimits & limits);

}  // namespace diff_ik_core
