/**
 * @file pose_error.hpp
 * @brief Task-space pose error and the feedback law v = v_ff + Kp e with a gain limit.
 * @details The error is expressed in the base frame, matching a base-frame Jacobian: position
 *          error = target - current, orientation error = log(R_target R_current^T), the rotation
 *          vector (axis * angle, angle in [0, pi]) that turns the current orientation into the
 *          target by the shortest way.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <Eigen/Dense>

#include "diff_ik_core/diff_ik.hpp"

namespace diff_ik_core
{

/// Rotation vector of R by the shortest way (angle in [0, pi]); stable near 0 and near pi.
Eigen::Vector3d rotationLog(const Eigen::Matrix3d & R);

/// [position error; orientation error] from current to target, both in the base frame.
Vector6d poseError(const Eigen::Isometry3d & current, const Eigen::Isometry3d & target);

struct FeedbackParams
{
  double kp_linear = 2.0;      ///< 1/s
  double kp_angular = 2.0;     ///< 1/s
  double max_linear = 0.05;    ///< limit on the norm of the linear feedback term, m/s (<= 0: none)
  double max_angular = 0.2;    ///< limit on the norm of the angular feedback term, rad/s (<= 0: none)
};

/// v = feedforward + Kp e, with each of the two feedback terms limited in norm BEFORE the
/// feedforward is added (the limit bounds the correction, not the commanded path speed).
Vector6d trackingTwist(const Vector6d & feedforward, const Vector6d & error, const FeedbackParams & p);

}  // namespace diff_ik_core
