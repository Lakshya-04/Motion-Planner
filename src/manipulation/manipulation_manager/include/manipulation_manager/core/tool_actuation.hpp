/**
 * @file tool_actuation.hpp
 * @brief Tool actuation executor core: runs a plan's tool command (`tool_command`, `tool_off_lead_s`,
 *        `tool_on_at_speed_ratio`) along trajectory time.
 * @details Pure logic, ROS-free. The caller feeds it the trajectory time (which stands still while the
 *          motion is paused), the wall clock, the echoed tool state and, for a pump, the speed ratio, and
 *          publishes `ToolOutput::command` whenever `ToolOutput::send` is true.
 *
 *          - Schedule: on at `on_s`, off at `end_s - tool_off_lead_s` (never before `on_s`), both in
 *            trajectory time. A pump also waits for `speed_ratio >= tool_on_at_speed_ratio`.
 *          - A pause (`paused` true) drives the tool off and the schedule resumes where trajectory time
 *            stopped, so a spray valve never sits open on a stationary arm. `pause_keeps_state` leaves
 *            the tool as it is instead.
 *          - Verification: when the echo is wired (`echo_valid`), an unconfirmed command is resent every
 *            `retry_s` until the echo equals it; after `response_timeout_s` the result is
 *            TOOL_NOT_RESPONSIVE. Without an echo nothing is verified (the sim case).
 *          - Exit paths: `cancel()`, `fault()` and the end of the schedule all drive the tool off, resent
 *            and polled until the echo confirms it; the result is final only once the tool is confirmed
 *            off (or timed out). A tool that reports a state different from its confirmed command later
 *            is TOOL_FAULT.
 *          - Latency: time from command to matching echo, kept for on and off as profile data.
 *          Tool I/O is not safety-critical: this never stops the arm, it reports.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <algorithm>
#include <string>

namespace manipulation_manager
{

enum class ToolResult
{
  RUNNING,
  OK,
  CANCELLED,
  TOOL_FAULT,
  TOOL_NOT_RESPONSIVE,
};

inline const char * toString(ToolResult r)
{
  switch (r) {
    case ToolResult::RUNNING: return "RUNNING";
    case ToolResult::OK: return "OK";
    case ToolResult::CANCELLED: return "CANCELLED";
    case ToolResult::TOOL_FAULT: return "TOOL_FAULT";
    case ToolResult::TOOL_NOT_RESPONSIVE: return "TOOL_NOT_RESPONSIVE";
  }
  return "?";
}

struct ToolActuationConfig
{
  std::string tool_command{"valve"};  ///< "none" | "pump" | "valve".
  double off_lead_s{0.0};             ///< Close this long before the window end.
  double on_at_speed_ratio{0.0};      ///< Pump: on only at or above this speed ratio.
  double response_timeout_s{1.0};     ///< Echo must match within this (wall time).
  double retry_s{0.25};               ///< Resend an unconfirmed command this often.
  bool pause_keeps_state{false};      ///< false: a pause drives the tool off.
};

/// What to do this tick.
struct ToolOutput
{
  bool send{false};     ///< Publish `command` now.
  bool command{false};  ///< Desired tool state.
  ToolResult result{ToolResult::RUNNING};
};

class ToolActuation
{
public:
  explicit ToolActuation(const ToolActuationConfig & cfg)
  : cfg_(cfg) {}

  /// Start a window in trajectory time. A tool_command of "none" never switches anything.
  void start(double on_s, double end_s)
  {
    on_s_ = on_s;
    end_s_ = std::max(end_s, on_s);
    started_ = true;
    exiting_ = false;
    off_sent_ = false;
    commanded_ = false;
    confirmed_ = false;
    pending_ = false;
    result_ = ToolResult::RUNNING;
  }

  /// Exit paths: drive the tool off, then report. The first failure is kept.
  void cancel() {beginExit(ToolResult::CANCELLED);}
  void fault() {beginExit(ToolResult::TOOL_FAULT);}

  ToolResult result() const {return result_;}
  bool commanded() const {return commanded_;}
  /// Seconds from command to echo, 0 until measured.
  double onLatencyS() const {return on_latency_s_;}
  double offLatencyS() const {return off_latency_s_;}

  ToolOutput tick(
    double traj_time_s, double wall_now_s, bool paused, double speed_ratio,
    bool echo_valid, bool echo_on)
  {
    ToolOutput out;
    out.command = commanded_;
    out.result = result_;
    if (!started_ || cfg_.tool_command == "none" || result_ != ToolResult::RUNNING) {
      return out;
    }
    if (!echo_valid) {
      pending_ = false;
    }

    if (echo_valid) {
      if (pending_ && echo_on == commanded_) {
        (commanded_ ? on_latency_s_ : off_latency_s_) = wall_now_s - command_wall_s_;
        pending_ = false;
        confirmed_ = true;
      } else if (!pending_ && confirmed_ && echo_on != commanded_ && !exiting_) {
        beginExit(ToolResult::TOOL_FAULT);
      }
    }

    bool want = false;
    if (!exiting_) {
      const double off_at = std::max(on_s_, end_s_ - cfg_.off_lead_s);
      if (traj_time_s >= end_s_) {
        beginExit(ToolResult::OK);
      } else if (paused) {
        want = cfg_.pause_keeps_state ? commanded_ : false;
      } else if (traj_time_s >= on_s_ && traj_time_s < off_at) {
        want = cfg_.tool_command != "pump" || speed_ratio >= cfg_.on_at_speed_ratio;
      }
    }

    if (want != commanded_ || (exiting_ && !off_sent_)) {
      commanded_ = want;
      off_sent_ = off_sent_ || (exiting_ && !want);
      command_wall_s_ = last_send_wall_s_ = wall_now_s;
      pending_ = echo_valid;
      confirmed_ = false;
      out.send = true;
      out.command = want;
    } else if (pending_) {
      if (wall_now_s - command_wall_s_ > cfg_.response_timeout_s) {
        result_ = ToolResult::TOOL_NOT_RESPONSIVE;  // never confirmed; reported, not hidden
        pending_ = false;
      } else if (wall_now_s - last_send_wall_s_ >= cfg_.retry_s) {
        last_send_wall_s_ = wall_now_s;
        out.send = true;
      }
    }

    if (exiting_ && off_sent_ && !commanded_ && !pending_ && result_ == ToolResult::RUNNING) {
      result_ = exit_result_;
    }
    out.result = result_;
    return out;
  }

private:
  void beginExit(ToolResult r)
  {
    if (!exiting_ || exit_result_ == ToolResult::OK) {
      exit_result_ = r;
    }
    exiting_ = true;
  }

  ToolActuationConfig cfg_;
  double on_s_{0.0}, end_s_{0.0};
  double command_wall_s_{0.0}, last_send_wall_s_{0.0};
  double on_latency_s_{0.0}, off_latency_s_{0.0};
  bool started_{false}, exiting_{false}, off_sent_{false};
  bool commanded_{false}, confirmed_{false}, pending_{false};
  ToolResult result_{ToolResult::RUNNING};
  ToolResult exit_result_{ToolResult::OK};
};

}  // namespace manipulation_manager
