/**
 * @file cartesian_tracker.hpp
 * @brief Cartesian pose tracker: tool offset, pose feedback, plane servo, twist limit, diff IK.
 * @details Glue over the rest of this package, no ROS. Per step it moves the flange Jacobian to
 *          the contact point, builds the task twist (feedforward + feedback on the contact pose
 *          error, plus the plane servo twist when enabled and measured), scales the whole twist
 *          uniformly to the twist limits, runs computeStep and integrates. All twists are in the
 *          world frame at the contact point. With a plane measurement the servo owns the standoff
 *          and the tilt: the pose feedback keeps only the in-plane position error and the rotation
 *          about the normal. The tracker keeps the previous joint velocity (for integrate) and
 *          the tool offset; it is not thread safe. No heap allocation per step for N <= kMaxJoints.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <Eigen/Dense>

#include <optional>

#include "diff_ik_core/diff_ik.hpp"
#include "diff_ik_core/pose_error.hpp"
#include "diff_ik_core/tool_correction.hpp"

namespace diff_ik_core
{

struct TrackerParams
{
  DiffIkLimits limits;
  DiffIkParams ik;
  FeedbackParams feedback;
  AxisMask mask = kAllAxes;
  ToolOffset tool;                 ///< flange -> contact
  PlaneServoParams plane;
  bool plane_enabled = false;
  double max_linear_twist = 0.1;   ///< m/s, limit on the norm of the whole linear twist (<= 0: none)
  double max_angular_twist = 0.5;  ///< rad/s, same for the angular twist (<= 0: none)
  double dt = 0.002;               ///< s
};

struct TrackerReference
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();   ///< wanted contact pose, world
  Vector6d twist = Vector6d::Zero();                        ///< feedforward [lin; ang], world
};

struct PlaneMeasurement
{
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();      ///< away from the wall
  double standoff_error = 0.0;                            ///< target - measured, m
  Eigen::Vector3d tool_axis = -Eigen::Vector3d::UnitZ();  ///< pointing axis in the world
};

enum class TrackerStatus { Ok, Invalid };

struct TrackerOutput
{
  TrackerStatus status = TrackerStatus::Invalid;
  JointVector qdot;                   ///< applied joint velocity (after integrate)
  JointVector q_next;
  Vector6d twist = Vector6d::Zero();  ///< task twist after the twist limit
  double position_error = 0.0;        ///< m, contact point vs reference, before this step
  double orientation_error = 0.0;     ///< rad, same
  double sigma_min = 0.0;             ///< of the masked contact Jacobian (0 if it has more rows than columns)
  double condition = 0.0;
  double manipulability = 0.0;
  double twist_scale = 1.0;           ///< share of the twist kept by the twist limit
  double task_scale = 1.0;            ///< twist_scale times the diff IK velocity-limit scale
  double joint_limit_margin = 0.0;    ///< see jointLimitMargin, in [0, 0.5]
  bool ok() const {return status == TrackerStatus::Ok;}
};

class CartesianTracker
{
public:
  explicit CartesianTracker(TrackerParams params);

  /// J_flange is 6 x N, world frame, [lin; ang] at the flange origin. Non-finite or mis-sized
  /// input gives status Invalid, zero qdot and q_next = q.
  TrackerOutput step(
    const JointRef & q, const Eigen::Ref<const Eigen::MatrixXd> & J_flange,
    const Eigen::Isometry3d & T_world_flange, const TrackerReference & reference,
    const std::optional<PlaneMeasurement> & plane = std::nullopt);

  /// New calibration record; used from the next step on.
  void setToolOffset(const ToolOffset & offset) {params_.tool = offset;}
  const ToolOffset & toolOffset() const {return params_.tool;}

  /// Forget the previous joint velocity (call after a pause or a jump in q).
  void reset();

  const TrackerParams & params() const {return params_;}

private:
  TrackerOutput invalid(const JointRef & q) const;

  TrackerParams params_;
  JointVector qd_prev_;
};

}  // namespace diff_ik_core
