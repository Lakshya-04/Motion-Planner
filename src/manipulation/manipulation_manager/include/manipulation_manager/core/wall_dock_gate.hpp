/**
 * @file wall_dock_gate.hpp
 * @brief The optional wall-dock step after a stand arrival, without ROS: dock, judge the measured wall gap, go or abort.
 * @details After `navigate_to_stand` the manager may ask the `WallDock` action to servo the base to a standoff from, and
 *          square to, the fitted wall plane. The action reports the standoff error (measured minus goal) and the yaw
 *          error against the last plane. Those are the measured inputs of a stand check: the same `StandCheck` decides
 *          go / re-dock / abort (position tolerance = standoff tolerance, a fixed yaw tolerance). A re-dock sends the
 *          goal again; `max_attempts` counts the goals sent. A dock that fails (no wall, timeout) is retried inside the
 *          budget and then returns not-ok without abort (the stand's normal navigation retry applies); a measured miss
 *          out of budget aborts the stand. The measured error of the last dock is kept for the verify result.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include "manipulation_manager/core/stand_check.hpp"

namespace manipulation_manager
{

/// What one WallDock goal returned.
struct WallDockMeasured
{
  bool docked{false};            ///< The action succeeded (a measurement exists).
  double standoff_error_m{0.0};  ///< Measured standoff minus the goal.
  double yaw_error_rad{0.0};     ///< Rotation still needed to square to the wall.
  std::string message;           ///< The action's error text when not docked.
};

/// What the step needs from the action side.
class WallDockPort
{
public:
  virtual ~WallDockPort() = default;
  /// Send one WallDock goal and wait for its result.
  virtual WallDockMeasured dock() = 0;
  /// True when the run was canceled.
  virtual bool stopRequested() const = 0;
};

struct WallDockConfig
{
  double standoff_tol_m{0.02};   ///< |standoff error| accepted.
  double yaw_tol_rad{0.05};      ///< |yaw error| accepted.
  int max_attempts{2};           ///< WallDock goals sent at most (>= 1).
};

struct WallDockOutcome
{
  bool ok{false};                ///< Within tolerance: priming/docking may go on.
  bool abort{false};             ///< The stand is given up (no further navigation tries).
  int attempts{0};               ///< Goals sent.
  bool measured{false};          ///< A measurement of the last goal exists.
  double standoff_error_m{0.0};  ///< Of the last measurement.
  double yaw_error_rad{0.0};
  std::string message;
};

inline WallDockOutcome runWallDock(WallDockPort & port, const WallDockConfig & cfg)
{
  StandCheckConfig sc;
  sc.min_tol_m = 0.0;
  sc.max_tol_m = cfg.standoff_tol_m;
  sc.fallback_tol_m = cfg.standoff_tol_m;
  sc.yaw_tol_rad = cfg.yaw_tol_rad;
  sc.max_redocks = std::max(1, cfg.max_attempts) - 1;
  const StandCheck check(sc);

  WallDockOutcome out;
  for (;;) {
    if (port.stopRequested()) {
      out.message = "canceled";
      return out;
    }
    const WallDockMeasured m = port.dock();
    ++out.attempts;
    if (!m.docked) {
      out.message = m.message.empty() ? std::string("wall dock failed") : m.message;
      if (out.attempts >= std::max(1, cfg.max_attempts)) {
        return out;   // not an abort: the stand's navigation retry applies
      }
      continue;
    }
    out.measured = true;
    out.standoff_error_m = m.standoff_error_m;
    out.yaw_error_rad = m.yaw_error_rad;
    const auto r = check.evaluate(
      StandAchieved{std::fabs(m.standoff_error_m), m.yaw_error_rad}, -1.0, out.attempts - 1);
    if (r.decision == StandDecision::kGo) {
      out.ok = true;
      out.message.clear();
      return out;
    }
    if (r.decision == StandDecision::kAbort) {
      out.message = std::string("wall dock check: ") + toString(r.reason);
      out.abort = true;
      return out;
    }
    // kRedock: loop and send the goal again
  }
}

}  // namespace manipulation_manager
