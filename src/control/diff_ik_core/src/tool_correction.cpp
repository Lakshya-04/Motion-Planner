#include "diff_ik_core/tool_correction.hpp"

#include <algorithm>
#include <cmath>

namespace diff_ik_core
{

Eigen::Matrix3d ToolOffset::rotation() const
{
  return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

Eigen::Isometry3d ToolOffset::transform() const
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = rotation();
  T.translation() = xyz;
  return T;
}

Jacobian6 shiftJacobianToTool(
  const Eigen::Ref<const Eigen::MatrixXd> & J_flange, const Eigen::Matrix3d & R_world_flange,
  const ToolOffset & offset, TwistFrame frame)
{
  Jacobian6 Jt;
  if (J_flange.rows() != 6 || J_flange.cols() > kMaxJoints) {
    Jt.resize(6, 0);
    return Jt;
  }
  const Eigen::Vector3d r = R_world_flange * offset.xyz;
  Jt = J_flange;
  for (Eigen::Index i = 0; i < Jt.cols(); ++i) {
    // v_tool = v + w x r
    Jt.col(i).head<3>() += J_flange.col(i).tail<3>().cross(r);
  }
  if (frame == TwistFrame::Contact) {
    const Eigen::Matrix3d Rt = (R_world_flange * offset.rotation()).transpose();
    for (Eigen::Index i = 0; i < Jt.cols(); ++i) {
      const Eigen::Vector3d lin = Rt * Jt.col(i).head<3>();
      const Eigen::Vector3d ang = Rt * Jt.col(i).tail<3>();
      Jt.col(i).head<3>() = lin;
      Jt.col(i).tail<3>() = ang;
    }
  }
  return Jt;
}

Eigen::Isometry3d toolPoseFromFlange(
  const Eigen::Isometry3d & T_world_flange, const ToolOffset & offset)
{
  return T_world_flange * offset.transform();
}

namespace
{
Eigen::Vector3d limitNorm(const Eigen::Vector3d & v, double max_norm)
{
  const double n = v.norm();
  return (max_norm > 0.0 && n > max_norm) ? Eigen::Vector3d(v * (max_norm / n)) : v;
}
}  // namespace

Vector6d planeServoTwist(
  const Eigen::Vector3d & normal, double standoff_error, const Eigen::Vector3d & tool_axis,
  const PlaneServoParams & params)
{
  Vector6d v = Vector6d::Zero();
  const double nn = normal.norm();
  if (!(nn > 1e-12) || !std::isfinite(standoff_error) || !tool_axis.allFinite()) {return v;}
  const Eigen::Vector3d n = normal / nn;
  double lin = params.kp_distance * standoff_error;
  if (params.max_linear > 0.0) {
    lin = std::max(-params.max_linear, std::min(params.max_linear, lin));
  }
  v.head<3>() = lin * n;
  const double an = tool_axis.norm();
  if (an > 1e-12) {
    const Eigen::Vector3d a = tool_axis / an;
    v.tail<3>() = limitNorm(params.kp_tilt * a.cross(-n), params.max_angular);
  }
  return v;
}

}  // namespace diff_ik_core
