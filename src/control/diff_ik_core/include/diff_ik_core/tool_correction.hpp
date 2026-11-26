/**
 * @file tool_correction.hpp
 * @brief Tool (end-of-arm) offset for the Jacobian and a plane-relative corrective twist.
 * @details The offset is the calibrated flange -> contact frame transform (the `tcp_xyz` / `tcp_rpy`
 *          of tool_calibration's ToolCalibrationRecord). rpy follows the URDF convention:
 *          R = Rz(yaw) * Ry(pitch) * Rx(roll), i.e. roll about x first, all about fixed axes.
 *          Everything is a pure function of its inputs; the Jacobian shift uses inline storage and
 *          does not allocate for up to kMaxJoints columns.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <Eigen/Dense>

#include "diff_ik_core/diff_ik.hpp"

namespace diff_ik_core
{

/// Flange -> contact frame, as stored in a calibration record.
struct ToolOffset
{
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();   ///< m, in the flange frame
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();   ///< rad, R = Rz(yaw) Ry(pitch) Rx(roll)

  Eigen::Matrix3d rotation() const;      ///< R_flange_contact
  Eigen::Isometry3d transform() const;   ///< T_flange_contact
};

/// 6 x N Jacobian with inline storage (no heap), N <= kMaxJoints.
using Jacobian6 = Eigen::Matrix<double, 6, Eigen::Dynamic, 0, 6, kMaxJoints>;

enum class TwistFrame { World, Contact };

/**
 * Jacobian of the contact point from the flange Jacobian (rows [linear; angular], world frame):
 * linear_tool = linear - skew(R_world_flange * xyz) * angular, angular rows unchanged. With
 * `Contact` both blocks are then rotated into the contact frame axes (twist expressed in the
 * contact frame, same point). Wrong shape (not 6 rows, or more than kMaxJoints columns) gives
 * an empty 6 x 0 result.
 */
Jacobian6 shiftJacobianToTool(
  const Eigen::Ref<const Eigen::MatrixXd> & J_flange, const Eigen::Matrix3d & R_world_flange,
  const ToolOffset & offset, TwistFrame frame = TwistFrame::World);

/// World pose of the contact frame.
Eigen::Isometry3d toolPoseFromFlange(
  const Eigen::Isometry3d & T_world_flange, const ToolOffset & offset);

struct PlaneServoParams
{
  double kp_distance = 2.0;   ///< 1/s, linear twist per metre of standoff error
  double kp_tilt = 2.0;       ///< 1/s, angular twist per unit tilt
  double max_linear = 0.02;   ///< m/s, limit on the linear twist along the normal (<= 0: none)
  double max_angular = 0.1;   ///< rad/s, limit on the norm of the tilt twist (<= 0: none)
};

/**
 * Corrective twist in the world frame at the contact point, to hold a standoff to a wall and
 * keep the tool square to it. `normal` points away from the wall; `standoff_error` is target
 * minus measured standoff along it (positive = move away); `tool_axis` is the tool's pointing
 * axis in the world frame (towards the wall when square). Output: linear = Kp_d * error * n,
 * angular = Kp_t * (axis x -n), each limited. Zero when the standoff is on target and the axis
 * is anti-parallel to the normal. A tool axis exactly along the normal (flipped 180 deg) gives
 * no tilt twist. A zero-length normal or non-finite input gives a zero twist.
 */
Vector6d planeServoTwist(
  const Eigen::Vector3d & normal, double standoff_error, const Eigen::Vector3d & tool_axis,
  const PlaneServoParams & params);

}  // namespace diff_ik_core
