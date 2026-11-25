// Joint velocity / acceleration guard for trajectories that come from cuRobo (ROS-free).
//
// cuRobo's own limits come from the robot yml it was configured with, not from the MoveIt
// joint_limits.yaml the controller side of this stack uses, so a cuRobo trajectory can be much faster
// than the same move retimed by MoveIt (measured in the MuJoCo two-stand pass: about 14x). This stage
// takes a timed trajectory and per-joint limits and either rejects it or stretches its time axis until
// every joint is inside limit * safety_factor. It never changes the path, only the timing. A jerk limit is
// optional (JointLimit::jerk, 0 = none, the default) and is checked the same way.
#pragma once

#include <map>
#include <string>

#include "core_types/trajectory.hpp"
#include "motion_planner/core/time_parameterization.hpp"

namespace curobo_planner
{

enum class LimitMode
{
  kRetime,  ///< Scale the time axis so the limits hold (the trajectory is returned slower).
  kReject,  ///< Fail with "limit_violation" ("jerk_violation" for jerk only), trajectory untouched.
};

struct JointLimitPolicy
{
  /// Velocity and acceleration limit per joint name (rad/s, m/s, and their derivatives). `jerk` > 0
  /// adds a jerk limit for that joint (finite differences of the retimed trajectory); 0 = no jerk check.
  std::map<std::string, motion_planner::JointLimit> limits;
  /// Fraction of each limit the trajectory may use, in (0, 1]. 0.9 leaves headroom for the
  /// finite-difference estimate and the controller's tracking.
  double safety_factor{0.9};
  LimitMode mode{LimitMode::kRetime};
  bool enabled() const {return !limits.empty();}
};

struct LimitCheck
{
  bool ok{true};
  /// "" when ok. Otherwise one of: "limits_invalid" (bad policy value or a non-positive limit),
  /// "limit_missing" (a trajectory joint has no limit), "invalid_trajectory" (time not strictly
  /// increasing, ragged or non-finite points), "limit_violation" (mode kReject and a velocity or
  /// acceleration limit exceeded), "jerk_violation" (mode kReject, only a jerk limit exceeded; message
  /// names the joint and waypoint, see jerk_joint / jerk_waypoint).
  std::string reason_code;
  std::string message;
  double time_scale{1.0};      ///< Factor applied to the time axis (1 = untouched, > 1 = slowed).
  double max_velocity_ratio{0.0};      ///< Peak |v| / limit before the guard (1 = at the limit).
  double max_acceleration_ratio{0.0};  ///< Peak |a| / limit before the guard.
  double max_jerk_ratio{0.0};  ///< Peak |j| / limit before the guard over joints with a jerk limit (0 if none).
  std::string jerk_joint;      ///< Joint of the peak jerk ("" if no jerk limit is configured).
  int jerk_waypoint{-1};       ///< Index of the first waypoint of the interval pair holding the peak (-1: none).
  double duration_before_sec{0.0};
  double duration_after_sec{0.0};
};

/// Checks `trajectory` against `policy` and, in kRetime mode, retimes it in place. Velocity and
/// acceleration are taken from finite differences of the positions over time_from_start_sec (the
/// trajectory's own velocity / acceleration vectors, when present, are also respected and are scaled
/// along with the time axis). The trajectory is modified only when the result is ok. A policy with no
/// limits is a no-op (ok, scale 1). Time scales v by 1/s and a by 1/s^2, so
/// s = max(1, max v ratio / factor, sqrt(max a ratio / factor), cbrt(max j ratio / factor)). Jerk needs
/// at least 4 waypoints (a_i is the difference of two interval speeds, j the difference of two a_i, over
/// the later interval); jerk_waypoint i is the pair a_i, a_(i+1) at waypoints i+1 and i+2. Scaling time by s
/// divides the finite-difference jerk by s^3 exactly, so the retimed result is re-measured only to
/// guard rounding (bounded, never shortens).
LimitCheck enforceJointLimits(core_types::Trajectory & trajectory, const JointLimitPolicy & policy);

/// Reads the `joint_limits:` map of a MoveIt joint_limits.yaml (max_velocity / max_acceleration with
/// has_velocity_limits / has_acceleration_limits; max_jerk too when has_jerk_limits is true, else jerk 0). A joint without both limits enabled is not added, so
/// enforceJointLimits reports it as "limit_missing" instead of treating it as unlimited. The file's
/// default_*_scaling_factor values are returned in the scaling outputs when given (1.0 if absent).
bool parseMoveItJointLimits(
  const std::string & yaml_text, std::map<std::string, motion_planner::JointLimit> & limits,
  std::string & error, double * velocity_scaling = nullptr, double * acceleration_scaling = nullptr);
bool loadMoveItJointLimits(
  const std::string & path, std::map<std::string, motion_planner::JointLimit> & limits,
  std::string & error, double * velocity_scaling = nullptr, double * acceleration_scaling = nullptr);

/// Parses the inline form "joint:max_velocity:max_acceleration[:max_jerk],joint2:v:a" (plugin param
/// `joint_limits`; the optional jerk must be > 0). False with a message on a malformed or non-positive entry.
bool parseInlineJointLimits(
  const std::string & text, std::map<std::string, motion_planner::JointLimit> & limits,
  std::string & error);

}  // namespace curobo_planner
