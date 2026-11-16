#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace manipulation_manager
{

/// Settings of the recovery jog (see planRecoveryJog).
struct RecoveryJogParams
{
  double max_step_rad{0.05};        ///< largest change of any joint between two waypoints
  double max_speed{0.15};           ///< fastest joint (rad/s, m/s for the lift) along the jog
  double min_duration_s{1.0};       ///< a jog never takes less than this
  double max_total_rad{2.5};        ///< refuse a jog where some joint has to move further than this
  double done_tolerance_rad{0.01};  ///< already this close to the target: nothing to do
  double limit_margin{0.01};        ///< the target is kept this far inside the joint limits
  std::vector<double> lower;        ///< joint limits, same order as the state (empty = not known)
  std::vector<double> upper;
};

struct RecoveryJogPoint
{
  std::vector<double> positions;
  double time_from_start_s{0.0};
};

struct RecoveryJog
{
  bool ok{false};                         ///< false: refused, `reason` says why
  bool needed{false};                     ///< false: already at the target, `points` is empty
  std::string reason;
  std::vector<double> target;             ///< where the jog ends (the good state, kept inside the limits)
  std::vector<RecoveryJogPoint> points;   ///< after the current state, ending at `target`
};

/// True when a joint is within `margin` of one of its limits (or outside). Joints without limits are ignored.
inline bool nearJointLimits(
  const std::vector<double> & q, const std::vector<double> & lower, const std::vector<double> & upper,
  double margin)
{
  for (size_t i = 0; i < q.size(); ++i) {
    if (i < lower.size() && q[i] < lower[i] + margin) {
      return true;
    }
    if (i < upper.size() && q[i] > upper[i] - margin) {
      return true;
    }
  }
  return false;
}

/// A small joint-space move from a start state that no planner will accept (self-collision at the end of the
/// arm's reach, a joint on its limit) back toward the last configuration known to be good. The path is the
/// straight line in joint space, cut into steps of at most `max_step_rad`, timed with a smooth ramp so that the
/// fastest joint stays under `max_speed`. The target is clamped inside the joint limits (when given), so the
/// jog also moves a state off the limits. No planning and no collision check: the straight line back toward
/// a state the arm came from is what makes it safe, so a jog is only for a start state the planners refuse, and
/// it is bounded by `max_total_rad`.
inline RecoveryJog planRecoveryJog(
  const std::vector<double> & current, const std::vector<double> & good, const RecoveryJogParams & p)
{
  RecoveryJog jog;
  if (current.empty() || current.size() != good.size()) {
    jog.reason = "current and good state differ in size or are empty";
    return jog;
  }
  if (p.max_step_rad <= 0.0 || p.max_speed <= 0.0) {
    jog.reason = "max_step_rad and max_speed must be positive";
    return jog;
  }
  jog.target = good;
  for (size_t i = 0; i < jog.target.size(); ++i) {
    const bool lo = i < p.lower.size();
    const bool hi = i < p.upper.size();
    // a range too narrow for the margin keeps its middle
    if (lo && hi && p.upper[i] - p.lower[i] <= 2.0 * p.limit_margin) {
      jog.target[i] = 0.5 * (p.lower[i] + p.upper[i]);
      continue;
    }
    if (hi) {
      jog.target[i] = std::min(jog.target[i], p.upper[i] - p.limit_margin);
    }
    if (lo) {
      jog.target[i] = std::max(jog.target[i], p.lower[i] + p.limit_margin);
    }
  }
  double dmax = 0.0;
  for (size_t i = 0; i < current.size(); ++i) {
    dmax = std::max(dmax, std::fabs(jog.target[i] - current[i]));
  }
  if (dmax <= p.done_tolerance_rad) {
    jog.ok = true;
    jog.needed = false;
    return jog;
  }
  if (dmax > p.max_total_rad) {
    jog.reason = "the good state is too far away for a jog";
    return jog;
  }
  const size_t steps = std::max<size_t>(1, static_cast<size_t>(std::ceil(dmax / p.max_step_rad)));
  // A smoothstep ramp s(u) = 3u^2 - 2u^3 has a peak slope of 1.5, so the duration is 1.5 * dmax / max_speed.
  const double duration = std::max(p.min_duration_s, 1.5 * dmax / p.max_speed);
  for (size_t k = 1; k <= steps; ++k) {
    const double u = static_cast<double>(k) / static_cast<double>(steps);
    const double s = u * u * (3.0 - 2.0 * u);
    RecoveryJogPoint pt;
    pt.positions.resize(current.size());
    for (size_t i = 0; i < current.size(); ++i) {
      pt.positions[i] = current[i] + s * (jog.target[i] - current[i]);
    }
    pt.time_from_start_s = u * duration;
    jog.points.push_back(std::move(pt));
  }
  jog.ok = true;
  jog.needed = true;
  return jog;
}

}  // namespace manipulation_manager
