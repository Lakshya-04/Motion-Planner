/**
 * @file direct_joint_path.hpp
 * @brief Straight-line joint-space path from a start to a goal, densified and validity checked, ROS-free.
 * @details A free-space move whose goal is already a joint configuration (a named pose such as "stow", or a pose goal
 *          after IK) rarely needs a sampling planner: the straight line in joint space is usually free, and its
 *          duration is set by the slowest joint instead of by a wandering path (measured on a two-stand run: joint
 *          path up to 16x the straight-line joint distance, 37 s for a stow). This header holds the pure logic:
 *          densify with a per-joint maximum step, check every sample through a validity callback (the caller binds it
 *          to the planning scene: collision and joint bounds), and estimate the duration. The caller timestamps the
 *          result with its TimeParameterizer and falls back to the sampling planner when any sample is invalid.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>

namespace motion_planner
{

/// True when the joint configuration is valid (collision free, inside the bounds).
using JointStateValidity = std::function<bool (const std::vector<double> &)>;

struct DirectJointPathConfig
{
  double revolute_step{0.05};   ///< Largest step of a revolute joint between samples, rad.
  double prismatic_step{0.01};  ///< Largest step of a prismatic joint (a lift), m.
  /// One flag per joint; true = prismatic. Empty = all revolute.
  std::vector<bool> prismatic;
  /// Goals closer than this (max joint delta) to the start need no motion.
  double min_distance{1e-6};
};

struct DirectJointPathResult
{
  enum class Status
  {
    kOk,
    kBadInput,         ///< Size mismatch, empty, non-finite, or a non-positive step.
    kNoMotion,         ///< Start and goal coincide: nothing to move.
    kInvalidSample,    ///< A sample failed the validity callback; see invalid_index.
  };
  Status status{Status::kBadInput};
  std::vector<std::vector<double>> samples;  ///< Start first, goal last (also when invalid, up to the failure).
  std::size_t invalid_index{0};              ///< Index into samples of the first invalid one.
};

/// Step limit of joint j.
inline double jointStep(const DirectJointPathConfig & cfg, std::size_t j)
{
  const bool prismatic = j < cfg.prismatic.size() && cfg.prismatic[j];
  return prismatic ? cfg.prismatic_step : cfg.revolute_step;
}

/// Number of segments so that no joint moves more than its step per segment (>= 1).
inline std::size_t segmentCount(
  const std::vector<double> & start, const std::vector<double> & goal, const DirectJointPathConfig & cfg)
{
  std::size_t n = 1;
  for (std::size_t j = 0; j < start.size(); ++j) {
    const double step = jointStep(cfg, j);
    const double segments = std::ceil(std::fabs(goal[j] - start[j]) / step - 1e-9);
    n = std::max(n, static_cast<std::size_t>(segments));
  }
  return n;
}

/// Linear interpolation, start and goal included, at most the configured step per joint between samples.
inline std::vector<std::vector<double>> densify(
  const std::vector<double> & start, const std::vector<double> & goal, const DirectJointPathConfig & cfg)
{
  const std::size_t n = segmentCount(start, goal, cfg);
  std::vector<std::vector<double>> out;
  out.reserve(n + 1);
  for (std::size_t i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    std::vector<double> q(start.size());
    for (std::size_t j = 0; j < start.size(); ++j) {
      q[j] = (i == n) ? goal[j] : start[j] + t * (goal[j] - start[j]);
    }
    out.push_back(std::move(q));
  }
  return out;
}

/// Largest absolute joint difference.
inline double maxJointDelta(const std::vector<double> & a, const std::vector<double> & b)
{
  double d = 0.0;
  for (std::size_t j = 0; j < a.size() && j < b.size(); ++j) {
    d = std::max(d, std::fabs(a[j] - b[j]));
  }
  return d;
}

/**
 * @brief Densify start->goal and check every sample (start included: a start that is invalid refuses the shortcut).
 * @param validity must be non-null; called once per sample in order, stops at the first invalid one.
 */
inline DirectJointPathResult buildDirectJointPath(
  const std::vector<double> & start, const std::vector<double> & goal,
  const DirectJointPathConfig & cfg, const JointStateValidity & validity)
{
  DirectJointPathResult result;
  if (start.empty() || start.size() != goal.size() || !validity || !(cfg.revolute_step > 0.0) ||
    !(cfg.prismatic_step > 0.0))
  {
    return result;
  }
  for (std::size_t j = 0; j < start.size(); ++j) {
    if (!std::isfinite(start[j]) || !std::isfinite(goal[j])) {
      return result;
    }
  }
  if (maxJointDelta(start, goal) < cfg.min_distance) {
    result.status = DirectJointPathResult::Status::kNoMotion;
    return result;
  }
  result.samples = densify(start, goal, cfg);
  for (std::size_t i = 0; i < result.samples.size(); ++i) {
    if (!validity(result.samples[i])) {
      result.status = DirectJointPathResult::Status::kInvalidSample;
      result.invalid_index = i;
      result.samples.resize(i + 1);
      return result;
    }
  }
  result.status = DirectJointPathResult::Status::kOk;
  return result;
}

/// Time of a rest-to-rest move over distance d with limits v and a: trapezoid, or triangle when v is never reached.
inline double restToRestDuration(double distance, double v_max, double a_max)
{
  if (!(distance > 0.0) || !(v_max > 0.0) || !(a_max > 0.0)) {
    return 0.0;
  }
  const double d_ramp = v_max * v_max / a_max;  // accelerate to v_max and brake
  if (distance >= d_ramp) {
    return distance / v_max + v_max / a_max;
  }
  return 2.0 * std::sqrt(distance / a_max);
}

/// Duration of the straight joint-space move: the slowest joint decides (a lower bound for the retimed path; the
/// retimer scales all joints together, so the true time is at least this).
inline double slowestJointDuration(
  const std::vector<double> & start, const std::vector<double> & goal,
  const std::vector<double> & velocity_limits, const std::vector<double> & acceleration_limits)
{
  double t = 0.0;
  for (std::size_t j = 0; j < start.size() && j < goal.size(); ++j) {
    const double v = j < velocity_limits.size() ? velocity_limits[j] : 0.0;
    const double a = j < acceleration_limits.size() ? acceleration_limits[j] : 0.0;
    t = std::max(t, restToRestDuration(std::fabs(goal[j] - start[j]), v, a));
  }
  return t;
}

}  // namespace motion_planner
