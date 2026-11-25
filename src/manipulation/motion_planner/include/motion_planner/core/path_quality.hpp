/**
 * @file path_quality.hpp
 * @brief Quality measures and best-candidate selection for free-space joint paths, ROS-free.
 * @details A free-space plan from a sampling planner wanders (measured: joint path 1.7x to 16x the straight-line joint
 *          distance for simple transitions), and executing a long path takes the same wall time as planning it
 *          would, so the planner is given time to find the best of several candidates. This header holds the pure
 *          parts: the joint-limit weighted path length (the lift's metres and the arm's radians become comparable
 *          by dividing each joint by its range), a TCP-agnostic duration estimate from per-joint velocity and
 *          acceleration limits, the straight-line baseline, the efficiency ratio (baseline / value, 1 = straight)
 *          and selectBestCandidate. The caller (OmplPlanner) fills candidates from the direct joint path and from
 *          several asymptotically optimal planner runs, and may pass a measured duration (time-optimal
 *          parameterisation) instead of the estimate.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "motion_planner/core/direct_joint_path.hpp"

namespace motion_planner
{

enum class FreeSpaceObjective
{
  kLength,    ///< Smallest weighted joint-space path length.
  kDuration,  ///< Smallest (time-parameterised) duration.
};

inline const char * objectiveName(FreeSpaceObjective o)
{
  return o == FreeSpaceObjective::kLength ? "length" : "duration";
}

/// "duration" selects kDuration, anything else kLength (the default of the user requirement: shortest path).
inline FreeSpaceObjective parseObjective(const std::string & s)
{
  return s == "duration" ? FreeSpaceObjective::kDuration : FreeSpaceObjective::kLength;
}

struct PathQualityConfig
{
  /// Per-joint weight (typically 1 / joint range, times the lift cost weight on the lift joint, see
  /// applyLiftCostWeight). Empty or short = 1 for the missing joints.
  std::vector<double> weights;
  std::vector<double> velocity_limits;      ///< Per joint; <= 0 = unknown (ignored).
  std::vector<double> acceleration_limits;  ///< Per joint; <= 0 = unknown (ignored).
};

/// Weights 1/range for each joint (range <= 0 or non-finite gives 1).
inline std::vector<double> weightsFromRanges(const std::vector<double> & ranges)
{
  std::vector<double> w;
  w.reserve(ranges.size());
  for (const double r : ranges) {
    w.push_back((std::isfinite(r) && r > 1e-9) ? 1.0 / r : 1.0);
  }
  return w;
}

/// Default cost multiplier of the lift (prismatic) joint relative to an arm joint: moving the lift by a given
/// normalised distance is this many times as costly as moving an arm joint, so plans prefer arm motion.
/// 1.0 switches the preference off (the plain 1/range weighting).
constexpr double kDefaultLiftCostWeight = 2.0;

/// `weights` with every joint flagged in `is_lift` multiplied by `lift_cost_weight` (the per-joint cost weighting:
/// lift 2.0, arm joints 1.0 by default). Missing flags count as arm joints; `weights` shorter than `is_lift` are
/// padded with 1. A weight of 1.0 (or a non-finite / non-positive one) returns `weights` unchanged.
inline std::vector<double> applyLiftCostWeight(
  std::vector<double> weights, const std::vector<bool> & is_lift, double lift_cost_weight)
{
  if (!(std::isfinite(lift_cost_weight) && lift_cost_weight > 0.0) || lift_cost_weight == 1.0) {
    return weights;
  }
  if (weights.size() < is_lift.size()) {
    weights.resize(is_lift.size(), 1.0);
  }
  for (std::size_t j = 0; j < is_lift.size(); ++j) {
    if (is_lift[j]) {
      weights[j] *= lift_cost_weight;
    }
  }
  return weights;
}

inline double jointWeight(const PathQualityConfig & cfg, std::size_t j)
{
  return j < cfg.weights.size() ? cfg.weights[j] : 1.0;
}

/// Euclidean length of a - b with the joint weights applied.
inline double weightedDistance(
  const std::vector<double> & a, const std::vector<double> & b, const PathQualityConfig & cfg)
{
  double sum = 0.0;
  for (std::size_t j = 0; j < a.size() && j < b.size(); ++j) {
    const double d = (a[j] - b[j]) * jointWeight(cfg, j);
    sum += d * d;
  }
  return std::sqrt(sum);
}

/// Weighted joint-space length of the polyline.
inline double weightedPathLength(
  const std::vector<std::vector<double>> & points, const PathQualityConfig & cfg)
{
  double length = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    length += weightedDistance(points[i - 1], points[i], cfg);
  }
  return length;
}

/// The straight-line baseline: weighted distance between the ends of the path.
inline double straightLineLength(
  const std::vector<std::vector<double>> & points, const PathQualityConfig & cfg)
{
  return points.size() < 2 ? 0.0 : weightedDistance(points.front(), points.back(), cfg);
}

/// Time a rest-to-rest straight joint move needs (slowest joint decides): lower bound of any path's duration.
inline double straightLineDuration(
  const std::vector<std::vector<double>> & points, const PathQualityConfig & cfg)
{
  if (points.size() < 2) {
    return 0.0;
  }
  return slowestJointDuration(points.front(), points.back(), cfg.velocity_limits, cfg.acceleration_limits);
}

/**
 * @brief TCP-agnostic duration estimate of a path from the per-joint limits.
 * @details Each segment takes as long as its slowest joint at the velocity limit; one ramp (v/a of the slowest
 *          joint) is added for the start and stop. It ignores the speed lost in corners, so a wiggly path is
 *          rated optimistically; the measured time-optimal duration replaces it when available.
 */
inline double estimatePathDuration(
  const std::vector<std::vector<double>> & points, const PathQualityConfig & cfg)
{
  double t = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    double seg = 0.0;
    for (std::size_t j = 0; j < points[i].size() && j < points[i - 1].size(); ++j) {
      const double v = j < cfg.velocity_limits.size() ? cfg.velocity_limits[j] : 0.0;
      if (v > 0.0) {
        seg = std::max(seg, std::fabs(points[i][j] - points[i - 1][j]) / v);
      }
    }
    t += seg;
  }
  double ramp = 0.0;
  for (std::size_t j = 0; j < cfg.velocity_limits.size() && j < cfg.acceleration_limits.size(); ++j) {
    if (cfg.velocity_limits[j] > 0.0 && cfg.acceleration_limits[j] > 0.0) {
      ramp = std::max(ramp, cfg.velocity_limits[j] / cfg.acceleration_limits[j]);
    }
  }
  return points.size() < 2 ? 0.0 : t + ramp;
}

/// baseline / value in (0, 1]; 1 = as good as the straight line. 1 when both are 0, 0 when only the baseline is.
inline double efficiencyRatio(double baseline, double value)
{
  if (!(value > 1e-12)) {
    return baseline > 1e-12 ? 0.0 : 1.0;
  }
  return std::min(1.0, baseline / value);
}

/// True when `end` is within `tolerance` of `goal` in every joint (raw units: rad and m). Used to accept a Cartesian
/// line for a JOINT goal only when its inverse kinematics landed on the goal's own branch: a path to the same tool
/// pose on another branch is not the move that was asked for (wrist winding, elbow flip).
inline bool reachesJointGoal(const std::vector<double> & end, const std::vector<double> & goal, double tolerance)
{
  if (end.size() != goal.size() || end.empty()) {
    return false;
  }
  for (std::size_t j = 0; j < end.size(); ++j) {
    if (!(std::fabs(end[j] - goal[j]) <= tolerance)) {
      return false;
    }
  }
  return true;
}

/// Largest absolute joint step along the polyline (a Cartesian line that hides a branch jump shows up here).
inline double maxJointStep(const std::vector<std::vector<double>> & points)
{
  double m = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    for (std::size_t j = 0; j < points[i].size() && j < points[i - 1].size(); ++j) {
      m = std::max(m, std::fabs(points[i][j] - points[i - 1][j]));
    }
  }
  return m;
}

struct PathCandidate
{
  std::string source;                         ///< "direct", "cartesian", or the planner config name.
  std::vector<std::vector<double>> points;    ///< Joint-space polyline, start first.
  /// Measured duration (time-optimal parameterisation), seconds; negative = not measured, use the estimate.
  double duration_s{-1.0};
};

struct CandidateSelection
{
  int best_index{-1};         ///< -1 when there is no usable candidate.
  double length{0.0};         ///< Weighted length of the best one.
  double duration{0.0};       ///< Duration of the best one (measured or estimated).
  double baseline_length{0.0};
  double baseline_duration{0.0};
  double value{0.0};          ///< Objective value of the best one.
  double baseline{0.0};       ///< Straight-line baseline in the objective's unit.
  double efficiency{0.0};     ///< baseline / value.
};

inline double candidateDuration(const PathCandidate & c, const PathQualityConfig & cfg)
{
  return c.duration_s >= 0.0 ? c.duration_s : estimatePathDuration(c.points, cfg);
}

/**
 * @brief Pick the candidate with the smallest objective value; ties (within 1e-9 relative) keep the earlier one,
 *        so the direct path, listed first, wins an equal race, and the other measure breaks a tie.
 * @details Candidates with fewer than 2 points are skipped. The baseline is the straight line between the ends of
 *          the chosen candidate.
 */
inline CandidateSelection selectBestCandidate(
  const std::vector<PathCandidate> & candidates, FreeSpaceObjective objective, const PathQualityConfig & cfg)
{
  CandidateSelection sel;
  double best_value = std::numeric_limits<double>::infinity();
  double best_other = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const auto & c = candidates[i];
    if (c.points.size() < 2) {
      continue;
    }
    const double length = weightedPathLength(c.points, cfg);
    const double duration = candidateDuration(c, cfg);
    const double value = objective == FreeSpaceObjective::kLength ? length : duration;
    const double other = objective == FreeSpaceObjective::kLength ? duration : length;
    const double tol = 1e-9 * std::max(1.0, std::fabs(best_value));
    const bool better = value < best_value - tol ||
      (std::fabs(value - best_value) <= tol && other < best_other - 1e-9);
    if (sel.best_index < 0 || better) {
      best_value = value;
      best_other = other;
      sel.best_index = static_cast<int>(i);
      sel.length = length;
      sel.duration = duration;
      sel.value = value;
    }
  }
  if (sel.best_index >= 0) {
    const auto & best = candidates[static_cast<std::size_t>(sel.best_index)];
    sel.baseline_length = straightLineLength(best.points, cfg);
    sel.baseline_duration = straightLineDuration(best.points, cfg);
    sel.baseline = objective == FreeSpaceObjective::kLength ? sel.baseline_length : sel.baseline_duration;
    sel.efficiency = efficiencyRatio(sel.baseline, sel.value);
  }
  return sel;
}

/// "free-space plan: candidates N, best <objective>=X (baseline straight Y, efficiency Z), <planner> used".
inline std::string formatFreeSpaceLog(
  std::size_t candidates, FreeSpaceObjective objective, const CandidateSelection & sel, const std::string & planner)
{
  char buffer[256];
  std::snprintf(
    buffer, sizeof(buffer),
    "free-space plan: candidates %zu, best %s=%.3f%s (baseline straight %.3f, efficiency %.2f), %s used",
    candidates, objectiveName(objective), sel.value, objective == FreeSpaceObjective::kDuration ? " s" : "",
    sel.baseline, sel.efficiency, planner.c_str());
  return buffer;
}

}  // namespace motion_planner
