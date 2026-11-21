#include "diff_ik_core/pose_error.hpp"

#include <algorithm>
#include <cmath>

namespace diff_ik_core
{

Eigen::Vector3d rotationLog(const Eigen::Matrix3d & R)
{
  Eigen::Quaterniond qn(R);
  qn.normalize();
  if (qn.w() < 0.0) {qn.coeffs() = -qn.coeffs();}   // shortest way: angle <= pi
  const Eigen::Vector3d v = qn.vec();
  const double sin_half = v.norm();
  if (sin_half < 1e-12) {return 2.0 * v / std::max(qn.w(), 1e-12);}
  const double angle = 2.0 * std::atan2(sin_half, qn.w());
  return v / sin_half * angle;
}

Vector6d poseError(const Eigen::Isometry3d & current, const Eigen::Isometry3d & target)
{
  Vector6d e;
  e.head<3>() = target.translation() - current.translation();
  e.tail<3>() = rotationLog(target.linear() * current.linear().transpose());
  return e;
}

namespace
{
Eigen::Vector3d limitNorm(const Eigen::Vector3d & v, double max_norm)
{
  const double n = v.norm();
  return (max_norm > 0.0 && n > max_norm) ? Eigen::Vector3d(v * (max_norm / n)) : v;
}
}  // namespace

Vector6d trackingTwist(const Vector6d & feedforward, const Vector6d & error, const FeedbackParams & p)
{
  Vector6d v = feedforward;
  v.head<3>() += limitNorm(p.kp_linear * error.head<3>(), p.max_linear);
  v.tail<3>() += limitNorm(p.kp_angular * error.tail<3>(), p.max_angular);
  return v;
}

}  // namespace diff_ik_core
