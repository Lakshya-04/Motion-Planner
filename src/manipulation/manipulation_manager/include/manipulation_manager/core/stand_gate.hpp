/**
 * @file stand_gate.hpp
 * @brief The stand-check loop and the navigation verdict handling of the multi-stand run, without ROS.
 * @details After the base arrived at a stand the manager asks `StandCheck` whether the arm may move. A miss inside the
 *          re-dock budget drives to the stand again (through a port) and checks the new arrival; a miss out of budget
 *          gives the stand up. This used to live inside the node (`gateStand`); here it is testable with a fake navigator.
 *          `resolveNavigation` then turns the outcome into the state machine's verdict: a stand the gate gave up is not
 *          retried with further navigation tries.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <string>

#include "manipulation_manager/core/coverage_task.hpp"
#include "manipulation_manager/core/stand_check.hpp"

namespace manipulation_manager
{

/// What the gate needs from the navigation side.
class StandNavigatorPort
{
public:
  virtual ~StandNavigatorPort() = default;
  /// Drive to the stand (again). False with `message` set when the drive failed.
  virtual bool navigate(std::string & message) = 0;
  /// How the last arrival went.
  virtual StandAchieved achieved() const = 0;
  /// True when the run was canceled.
  virtual bool stopRequested() const = 0;
};

struct StandGateOutcome
{
  bool ok{false};            ///< The arm may move.
  bool abort{false};         ///< The stand is given up (no further navigation tries).
  std::string message;       ///< Why not ok (empty when ok and no remark).
  int redocks{0};            ///< Re-dock drives made.
  StandReason reason{StandReason::kWithinTolerance};
};

/// Go, re-dock (drive again) or abort. `reach_margin_m` negative = unknown. A failed re-dock drive returns ok=false,
/// abort=false with the navigator's message (the normal retry applies); a canceled run returns "canceled".
inline StandGateOutcome runStandGate(StandNavigatorPort & nav, const StandCheck & check, double reach_margin_m)
{
  StandGateOutcome out;
  for (;;) {
    const auto r = check.evaluate(nav.achieved(), reach_margin_m, out.redocks);
    out.reason = r.reason;
    if (r.decision == StandDecision::kGo) {
      out.ok = true;
      out.message = r.message;
      return out;
    }
    if (r.decision == StandDecision::kAbort) {
      out.message = std::string("stand check: ") + toString(r.reason);
      out.abort = r.stand_failed;
      return out;
    }
    ++out.redocks;
    std::string msg;
    if (nav.stopRequested() || !nav.navigate(msg)) {
      out.message = msg.empty() ? std::string("canceled") : msg;
      return out;
    }
  }
}

/// The state machine's verdict for one navigation attempt: when the gate gave the stand up (`gate_abort`) the attempts
/// left are used up at once, so the result is kSkip or kFinish, never kRetry.
inline NavVerdict resolveNavigation(TaskStateMachine & machine, bool ok, bool gate_abort)
{
  auto verdict = machine.navigationResult(ok);
  while (gate_abort && verdict == NavVerdict::kRetry) {
    verdict = machine.navigationResult(false);
  }
  return verdict;
}

}  // namespace manipulation_manager
