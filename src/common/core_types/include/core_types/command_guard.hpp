/**
 * @file command_guard.hpp
 * @brief Joint command guard with a fault latch: NaN and limit check, hold, report.
 * @details ROS-free. Call `apply(command, measured)` once per control cycle, right before the command is
 *          written to the hardware interface, and write `Result::positions` instead of the command.
 *
 *          A cycle is bad when the command or the measured state has a non-finite entry (a non-finite
 *          measurement is a dead axis: it is never reported healthy), or a command entry is outside
 *          `[lower - tolerance, upper + tolerance]`, or the sizes disagree. A bad cycle never reaches the
 *          hardware: the output holds the last good command (or the measured position before any good
 *          command). After `debounce_cycles` bad cycles in a row the guard latches a fault: the output stays
 *          held, `faulted` stays true and `code` names the first cause, until `reset()`. A good cycle before
 *          the latch clears the count. The caller turns `faulted` into its own error (the ros2_control
 *          `update()` returns ERROR, a node raises its fault topic and forces the tool off).
 *          No "degraded mode": a latched fault does not recover on its own.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace core_types
{

enum class GuardFault
{
  NONE,
  NON_FINITE_COMMAND,
  NON_FINITE_STATE,    ///< Dead axis: the measured position is NaN or Inf.
  LIMIT_VIOLATION,
  SIZE_MISMATCH,
};

inline const char * toString(GuardFault f)
{
  switch (f) {
    case GuardFault::NONE: return "NONE";
    case GuardFault::NON_FINITE_COMMAND: return "NON_FINITE_COMMAND";
    case GuardFault::NON_FINITE_STATE: return "NON_FINITE_STATE";
    case GuardFault::LIMIT_VIOLATION: return "LIMIT_VIOLATION";
    case GuardFault::SIZE_MISMATCH: return "SIZE_MISMATCH";
  }
  return "?";
}

struct CommandGuardConfig
{
  std::vector<double> lower;   ///< Per-joint lower position limit (empty = unchecked).
  std::vector<double> upper;   ///< Per-joint upper position limit (empty = unchecked).
  double tolerance{1e-6};      ///< Slack on the limits.
  int debounce_cycles{3};      ///< Bad cycles in a row before the latch (>= 1).
};

class CommandGuard
{
public:
  struct Result
  {
    std::vector<double> positions;  ///< What to write. Empty when there is nothing safe to write.
    bool bad{false};                ///< This cycle's command was rejected.
    bool faulted{false};            ///< Latched.
    GuardFault code{GuardFault::NONE};  ///< Cause of the latched fault, or of this bad cycle.
  };

  explicit CommandGuard(const CommandGuardConfig & cfg)
  : cfg_(cfg)
  {
    if (cfg_.debounce_cycles < 1) {
      cfg_.debounce_cycles = 1;
    }
  }

  bool faulted() const {return faulted_;}
  GuardFault faultCode() const {return fault_code_;}
  int badCycles() const {return bad_cycles_;}

  /// Clear the latch (an operator or a re-activation, never automatic).
  void reset()
  {
    faulted_ = false;
    fault_code_ = GuardFault::NONE;
    bad_cycles_ = 0;
    have_hold_ = false;
  }

  Result apply(const std::vector<double> & command, const std::vector<double> & measured)
  {
    Result r;
    GuardFault cause = GuardFault::NONE;
    if (!faulted_) {
      cause = classify(command, measured);
    } else {
      cause = fault_code_;
    }

    if (!faulted_ && cause == GuardFault::NONE) {
      bad_cycles_ = 0;
      hold_ = command;
      have_hold_ = true;
      r.positions = command;
      return r;
    }

    if (!faulted_) {
      ++bad_cycles_;
      if (bad_cycles_ >= cfg_.debounce_cycles) {
        faulted_ = true;
        fault_code_ = cause;
      }
    }

    if (!have_hold_ && allFinite(measured) && !measured.empty()) {
      hold_ = measured;
      have_hold_ = true;
    }
    if (have_hold_) {
      r.positions = hold_;
    }
    r.bad = true;
    r.faulted = faulted_;
    r.code = cause;
    return r;
  }

private:
  static bool allFinite(const std::vector<double> & v)
  {
    for (double x : v) {
      if (!std::isfinite(x)) {
        return false;
      }
    }
    return true;
  }

  GuardFault classify(const std::vector<double> & cmd, const std::vector<double> & meas) const
  {
    if (cmd.size() != meas.size() ||
      (!cfg_.lower.empty() && cfg_.lower.size() != cmd.size()) ||
      (!cfg_.upper.empty() && cfg_.upper.size() != cmd.size()))
    {
      return GuardFault::SIZE_MISMATCH;
    }
    if (!allFinite(meas)) {
      return GuardFault::NON_FINITE_STATE;
    }
    if (!allFinite(cmd)) {
      return GuardFault::NON_FINITE_COMMAND;
    }
    for (std::size_t i = 0; i < cmd.size(); ++i) {
      if ((!cfg_.lower.empty() && cmd[i] < cfg_.lower[i] - cfg_.tolerance) ||
        (!cfg_.upper.empty() && cmd[i] > cfg_.upper[i] + cfg_.tolerance))
      {
        return GuardFault::LIMIT_VIOLATION;
      }
    }
    return GuardFault::NONE;
  }

  CommandGuardConfig cfg_;
  std::vector<double> hold_;
  bool have_hold_{false};
  bool faulted_{false};
  GuardFault fault_code_{GuardFault::NONE};
  int bad_cycles_{0};
};

}  // namespace core_types
