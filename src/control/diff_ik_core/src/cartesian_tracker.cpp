#include "diff_ik_core/cartesian_tracker.hpp"

#include <algorithm>
#include <cmath>

#include "diff_ik_core/jacobian_quality.hpp"

namespace diff_ik_core
{

namespace
{
bool finite(const Eigen::Isometry3d & T) {return T.matrix().allFinite();}

// Scale that brings a norm under a limit; <= 0 or infinite means no limit.
double limitScale(double norm, double limit)
{
  if (!(limit > 0.0) || !std::isfinite(limit) || norm <= limit) {return 1.0;}
  return limit / norm;
}
}  // namespace

CartesianTracker::CartesianTracker(TrackerParams params)
: params_(std::move(params))
{
  reset();
}

void CartesianTracker::reset()
{
  const Eigen::Index n = std::min<Eigen::Index>(params_.limits.q_min.size(), kMaxJoints);
  qd_prev_ = JointVector::Zero(n);
}

TrackerOutput CartesianTracker::invalid(const JointRef & q) const
{
  TrackerOutput out;
  if (q.size() <= kMaxJoints) {
    out.q_next = q;
    out.qdot = JointVector::Zero(q.size());
  }
  return out;
}

TrackerOutput CartesianTracker::step(
  const JointRef & q, const Eigen::Ref<const Eigen::MatrixXd> & J_flange,
  const Eigen::Isometry3d & T_world_flange, const TrackerReference & reference,
  const std::optional<PlaneMeasurement> & plane)
{
  const Eigen::Index n = q.size();
  const bool shape_ok = n >= 1 && n <= kMaxJoints && J_flange.rows() == 6 && J_flange.cols() == n &&
    params_.limits.q_min.size() == n && qd_prev_.size() == n;
  bool finite_ok = shape_ok && std::isfinite(params_.dt) && params_.dt > 0.0 && q.allFinite() &&
    J_flange.allFinite() && finite(T_world_flange) && finite(reference.pose) &&
    reference.twist.allFinite();
  if (finite_ok && plane) {
    finite_ok = plane->normal.allFinite() && plane->tool_axis.allFinite() &&
      std::isfinite(plane->standoff_error);
  }
  if (!finite_ok) {
    if (shape_ok) {qd_prev_.setZero();}
    return invalid(q);
  }

  const Eigen::Isometry3d T_contact = toolPoseFromFlange(T_world_flange, params_.tool);
  const Jacobian6 Jc = shiftJacobianToTool(J_flange, T_world_flange.linear(), params_.tool);

  Vector6d err = poseError(T_contact, reference.pose);
  TrackerOutput out;
  out.position_error = err.head<3>().norm();
  out.orientation_error = err.tail<3>().norm();

  const bool servo = params_.plane_enabled && plane.has_value();
  Eigen::Vector3d nrm = Eigen::Vector3d::Zero();
  if (servo) {
    const double len = plane->normal.norm();
    if (len > 0.0) {nrm = plane->normal / len;}
  }
  // the servo owns standoff and tilt, so the pose feedback keeps only the in-plane part
  if (nrm.squaredNorm() > 0.0) {
    err.head<3>() -= err.head<3>().dot(nrm) * nrm;
    err.tail<3>() = err.tail<3>().dot(nrm) * nrm;
  }

  Vector6d twist = trackingTwist(reference.twist, err, params_.feedback);
  if (servo) {
    twist += planeServoTwist(plane->normal, plane->standoff_error, plane->tool_axis, params_.plane);
  }

  const double s_lin = limitScale(twist.head<3>().norm(), params_.max_linear_twist);
  const double s_ang = limitScale(twist.tail<3>().norm(), params_.max_angular_twist);
  out.twist_scale = std::min(s_lin, s_ang);
  twist *= out.twist_scale;
  out.twist = twist;

  const DiffIkResult ik = computeStep(Jc, twist, params_.mask, q, params_.limits, params_.ik);
  if (!ik.valid) {
    qd_prev_.setZero();
    return invalid(q);
  }

  // quality of the rows the task actually uses
  int rows = 0;
  for (bool m : params_.mask) {rows += m ? 1 : 0;}
  JacobianRows Jm(rows, n);
  int r = 0;
  for (int i = 0; i < 6; ++i) {
    if (params_.mask[static_cast<size_t>(i)]) {Jm.row(r++) = Jc.row(i);}
  }
  const JacobianQuality jq = jacobianQuality(Jm);
  out.sigma_min = jq.sigma_min;
  out.condition = jq.condition;
  out.manipulability = jq.manipulability;
  out.joint_limit_margin = jointLimitMargin(q, params_.limits.q_min, params_.limits.q_max);
  out.task_scale = out.twist_scale * ik.scale;

  const IntegrateResult next = integrate(q, qd_prev_, ik.qd, params_.dt, params_.limits);
  if (!next.valid) {
    qd_prev_.setZero();
    return invalid(q);
  }
  qd_prev_ = next.qd;
  out.qdot = next.qd;
  out.q_next = next.q;
  out.status = TrackerStatus::Ok;
  return out;
}

}  // namespace diff_ik_core
