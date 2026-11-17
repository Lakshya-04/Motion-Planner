/**
 * @file payload_excitation.hpp
 * @brief Conditioning of a payload-identification pose set, and a safe excitation pose sequence.
 * @details The static identification (`identifyPayload`) is a linear least squares in
 *          [m, b_F] (force) and [m c, b_T] (torque). How well it can separate mass, centre of gravity
 *          and bias depends only on the sensor orientations, so the set can be judged BEFORE any
 *          measurement: `regressorConditionNumber` is the 2-norm condition number of the column-scaled
 *          regressors (worse of the force and torque blocks). Noise on the readings is amplified by
 *          about this number.
 *
 *          `generateExcitationPoses` picks well-spread static poses for the wrist. Candidates are drawn
 *          in joint space inside the limits (minus a margin), rejected when the caller's
 *          self-collision proxy says unsafe, and chosen greedily so that the gravity directions seen in the
 *          sensor frame spread over the sphere (farthest point first). Every move between two chosen
 *          poses must itself pass the proxy along the straight joint-space path, and gets a rest-to-rest
 *          quintic time law whose peak velocity and acceleration stay inside the bounds. This follows the
 *          idea of safe, bounded excitation in arXiv 2504.21486 in its simplest form: it keeps the robot
 *          inside the bounds and checked states, it does not do the paper's online set-based safety proofs.
 *          Plain C++, no ROS, deterministic (own generator, seed in the options).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "force_core/wrench_conditioning.hpp"

namespace force_core
{

/// Condition number of the identification regressors for sensors at the given orientations
/// (gravity columns scaled by 1/|g|). Returns +infinity when the set is rank deficient.
double regressorConditionNumber(const std::vector<Mat3> & r_world_sensor, const Vec3 & gravity_world);

struct ExcitationLimits
{
  std::vector<double> q_min;       ///< Joint lower limits [rad].
  std::vector<double> q_max;       ///< Joint upper limits [rad].
  std::vector<double> v_max;       ///< Peak joint velocity allowed [rad/s].
  std::vector<double> a_max;       ///< Peak joint acceleration allowed [rad/s^2].
  double margin_rad{0.1};          ///< Kept away from the position limits.
  double velocity_scale{0.5};      ///< Fraction of v_max / a_max actually used.
};

struct ExcitationOptions
{
  std::size_t num_poses{8};        ///< Poses to select (at least 6 recommended).
  std::size_t num_candidates{400};
  std::uint32_t seed{1};
  double path_step_rad{0.05};      ///< Joint-space step of the proxy check along a move.
  double dwell_s{1.5};             ///< Hold at each pose (settle + capture window).
  double min_duration_s{1.0};      ///< Floor on one move.
  double max_condition{20.0};      ///< Report (not reject) above this.
};

struct ExcitationWaypoint
{
  std::vector<double> q;           ///< Joint positions of the static pose.
  double t_arrive_s{0.0};          ///< Time the pose is reached (from the start of the sequence).
  double t_leave_s{0.0};           ///< Time the move to the next pose starts (arrive + dwell).
  double peak_velocity_ratio{0.0}; ///< Largest |v| / v_max on the move into this pose.
  double peak_accel_ratio{0.0};    ///< Largest |a| / a_max on the move into this pose.
};

struct ExcitationPlan
{
  bool ok{false};
  std::string message;
  std::vector<ExcitationWaypoint> waypoints;   ///< Starts with the start configuration (t = 0).
  double condition_number{0.0};                ///< Of the selected poses (start included if included).
  double min_pair_angle_rad{0.0};              ///< Smallest angle between two gravity directions.
  double duration_s{0.0};
};

using ForwardKinematics = std::function<Mat3(const std::vector<double> & q)>;   ///< R_world_sensor.
using SafetyProxy = std::function<bool(const std::vector<double> & q)>;         ///< true = safe.

/// Plan the sequence from `q_start` (which must itself pass the proxy and the limits). `fk` gives the sensor
/// orientation; `is_safe` is the caller's self-collision / workspace proxy (may be empty = always safe).
ExcitationPlan generateExcitationPoses(
  const std::vector<double> & q_start, const ExcitationLimits & limits, const ForwardKinematics & fk,
  const SafetyProxy & is_safe, const Vec3 & gravity_world, const ExcitationOptions & opt = {});

/// Joint positions of the plan at time t (rest-to-rest quintic between poses, holds at the poses).
std::vector<double> sampleExcitationPlan(const ExcitationPlan & plan, double t_s);

}  // namespace force_core
