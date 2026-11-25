/**
 * @file diff_ik.hpp
 * @brief Damped-least-squares differential IK step with null-space costs and limit handling.
 * @details Given the task Jacobian J (6 x N, base frame), a task twist v and the joint positions,
 *          computeStep returns joint velocities
 *
 *              qd = s * J_W^+ v  +  t * N * qd0
 *
 *          with J_W^+ = W^-1 J^T (J W^-1 J^T + lambda^2 I)^-1 (joint weights W: a large weight moves
 *          that joint less), lambda^2 from the smallest singular value (Nakamura-Hanafusa /
 *          Chiaverini: 0 above the threshold eps, rising smoothly to lambda_max^2 at a singularity),
 *          and qd0 the weighted sum of the null-space costs.
 *
 *          Null space. N is the TRUE projector of the undamped (masked, weighted) Jacobian: the
 *          components of qd0 along right singular vectors with a singular value above
 *          `rank_tolerance` are removed, the rest is left alone. So the task is tracked with
 *          damping, but null-space motion never depends on the damping: with unit weights and row
 *          scales the tool twist it produces is at most rank_tolerance * |qd0|, also close to a
 *          singularity. Directions below the tolerance are rank-deficient (free by definition);
 *          motion along them is the only way the null-space part can move the tool.
 *
 *          Limits:
 *            - velocity: the WHOLE task part is scaled by one factor s <= 1 so the worst joint just
 *              reaches its limit, which keeps the direction of the path; the null-space part is then
 *              reduced (t <= 1) until it fits in what is left, so it never costs path direction.
 *              A limit of 0 locks the joint: its velocity is 0 and the other joints do the task;
 *            - position: inside a zone of `position_margin` at a limit, the velocity into that limit
 *              is zeroed (hard stop; this one can deviate from the task, and is reported).
 *          A task can be a masked subset of the six axes, which leaves the other axes free and
 *          enlarges the null space.
 *
 *          Units. One sigma threshold applies to the singular values of the masked Jacobian whose
 *          rows may mix metres (linear) and radians (angular), so its meaning depends on the scale
 *          of the robot. `DiffIkParams::row_scale` multiplies each task row (and the twist) before
 *          the SVD to put the rows on a common footing; the default is 1.
 *
 *          Input validation (strict). Everything must be finite and sized N (see DiffIkLimits);
 *          otherwise valid = false, status says why, qd is zero. 1 <= N <= kMaxJoints.
 *
 *          Real time. For N <= kMaxJoints, a masked task of at most 6 rows and the cost classes in
 *          this package, computeStep and integrate use only inline (fixed-capacity) Eigen storage
 *          and perform no heap allocation (checked by diff_ik_alloc_test). A user-written
 *          NullSpaceCost, copies of the inputs the caller makes and constructing the limits or
 *          params objects are outside that guarantee. Not lock-free-proven beyond that.
 *          Pure function of its inputs: deterministic, no state.
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
  /// Position limits, size N, q_min <= q_max. A joint that is not continuous needs finite values.
  Eigen::VectorXd q_min;
  Eigen::VectorXd q_max;
  /// Per joint: true = continuous (no position limit, no stop, ignored by the limit costs). Empty
  /// = derive from the range as before: continuous when the range is not positive or at least
  /// 2*pi. Otherwise size N and explicit, so a revolute joint limited to +-pi gets its stop when
  /// flagged false.
  std::vector<bool> continuous;
  /// Velocity limits, size N, each >= 0 or +infinity (= explicitly unlimited). 0 locks the joint.
  /// An empty vector, a NaN or a negative value is invalid.
  Eigen::VectorXd qd_max;
  /// Acceleration limits for integrate(), size N, same convention (0 = the velocity cannot change).
  /// Not read by computeStep.
  Eigen::VectorXd qdd_max;
  double position_margin = 0.0;   ///< width of the hard stop zone at each limited joint's ends
};

struct DiffIkParams
{
  double sigma_threshold = 0.05;   ///< eps: damping starts below this smallest singular value
  double lambda_max = 0.1;         ///< damping at sigma_min = 0
  /// Singular values (of the masked, weighted, row-scaled Jacobian) at or below this are treated
  /// as rank-deficient by the null-space projector. Absolute, in the units of that matrix.
  double rank_tolerance = 1e-6;
  Eigen::VectorXd joint_weights;   ///< empty = all 1; else size N, > 0; larger = the joint moves less
  /// Per-row task scaling (x, y, z, rx, ry, rz), > 0, applied to the Jacobian rows and the twist
  /// before the SVD. Changes the damping threshold's meaning per row and the compromise in the
  /// damped region; exact tracking away from a singularity is unchanged.
  Vector6d row_scale = Vector6d::Ones();
  std::vector<NullSpaceTerm> null_space;   ///< empty = no null-space motion
};

enum class JointSaturation { None, Velocity, PositionLimit };

enum class InputStatus { Ok, BadSize, NonFinite, BadValue };

struct DiffIkResult
{
  bool valid = false;                 ///< false on bad input (qd is then zero)
  InputStatus status = InputStatus::BadSize;   ///< why, when not valid
  JointVector qd;                     ///< commanded joint velocities
  double sigma_min = 0.0;             ///< smallest singular value of the (masked, weighted) Jacobian
  double damping = 0.0;               ///< lambda used (not squared)
  double scale = 1.0;                 ///< uniform task scale s in [0, 1]
  double null_scale = 1.0;            ///< share t of the requested null-space motion that was kept
  int velocity_limit_hits = 0;        ///< joints at their velocity limit in the result
  int position_limit_hits = 0;        ///< joints whose velocity was zeroed by a position zone
  std::array<JointSaturation, kMaxJoints> saturation = {};   ///< per joint (first N entries)
  Vector6d achieved_twist = Vector6d::Zero();   ///< J * qd, all six axes
};

/// Differential-IK step; see the file comment. J is 6 x N, q / limits vectors size N.
DiffIkResult computeStep(
  const Eigen::Ref<const Eigen::MatrixXd> & J, const Vector6d & twist, const AxisMask & mask,
  const JointRef & q, const DiffIkLimits & limits, const DiffIkParams & params);

struct IntegrateResult
{
  bool valid = false;   ///< false on bad input: q is held and qd is zero
  JointVector q;
  JointVector qd;
};

/**
 * One semi-implicit step: the velocity moves towards `qd_cmd` by at most qdd_max * dt per joint
 * and is clamped to qd_max, then the position advances with the NEW velocity. A step that would
 * cross a position limit stops at the limit with zero velocity. If q already starts outside a
 * limit the joint is not snapped: a step further out is cancelled (q held, velocity 0), a step
 * inwards is taken as it is. Non-finite or mis-sized input (or limits) holds q, zero velocity.
 */
IntegrateResult integrate(
  const JointRef & q, const JointRef & qd_prev, const JointRef & qd_cmd,
  double dt, const DiffIkLimits & limits);

}  // namespace diff_ik_core
