/**
 * @file teleop_mode_manager.hpp
 * @brief AUTO / TELEOP / HOLD mode arbitration with an explicit transition table.
 * @details Plain C++, no clock, no ROS: the caller passes time. Every request returns a
 *          TransitionResult with a reason code. Refused requests are counted.
 *
 *          Transition table (from -> to : condition):
 *            AUTO   -> HOLD   : stop or safety event (always)
 *            AUTO   -> TELEOP : operator request AND the running segment is halted (`segment_halted`);
                               refused with kInvalidTime for a non-finite time
 *            TELEOP -> HOLD   : stop or safety event (always), or no operator input for `input_timeout_s`;
                               a clock that went backwards, an input stamped after the tick time, or a
                               non-finite tick time also count as a timeout (fail safe)
 *            TELEOP -> AUTO   : operator confirm AND replan-from-current-pose acknowledged
 *            HOLD   -> TELEOP : operator request
 *            HOLD   -> AUTO   : operator request; if teleoperation moved the arm since the last AUTO
 *                               it also needs the replan acknowledgement (the old plan starts elsewhere)
 *          A request for the mode already active is refused with kAlreadyInMode and not counted as illegal.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cstddef>

namespace teleop_core
{

enum class Mode {kAuto = 0, kTeleop, kHold};

enum class Reason
{
  kOk = 0,
  kStopRequested,        ///< to HOLD: operator or system stop
  kSafetyStop,           ///< to HOLD: safety layer
  kInputTimeout,         ///< TELEOP -> HOLD: no operator input
  kOperatorRequest,      ///< to TELEOP / HOLD -> AUTO by request
  kOperatorConfirmed,    ///< TELEOP -> AUTO with confirm and replan acknowledgement
  kSegmentStillRunning,  ///< refused: AUTO -> TELEOP while the segment is not halted
  kNeedsConfirm,         ///< refused: TELEOP -> AUTO without operator confirm
  kNeedsReplanAck,       ///< refused: return to AUTO without replan acknowledgement
  kAlreadyInMode,        ///< no-op
  kInvalidTime,          ///< refused: non-finite time for a TELEOP request
};

struct TransitionResult
{
  bool accepted{false};
  Mode mode{Mode::kAuto};
  Reason reason{Reason::kOk};
};

struct TeleopModeConfig
{
  /// TELEOP without input for this long falls back to HOLD. Must be finite and > 0, otherwise the
  /// constructor throws std::invalid_argument.
  double input_timeout_s{2.0};
};

class TeleopModeManager
{
public:
  explicit TeleopModeManager(const TeleopModeConfig & config = TeleopModeConfig{});

  Mode mode() const {return mode_;}
  std::size_t refusedCount() const {return refused_;}
  std::size_t acceptedCount() const {return accepted_;}
  bool needsReplan() const {return needs_replan_;}

  /// Stop or safety event: any mode -> HOLD.
  TransitionResult onStop(bool safety, double now_s);
  /// Operator wants jog control.
  TransitionResult requestTeleop(bool segment_halted, double now_s);
  /// Operator wants to resume automatic execution.
  TransitionResult requestAuto(bool operator_confirm, bool replan_acknowledged, double now_s);
  /// Operator input arrived (refreshes the TELEOP watchdog).
  void noteInput(double now_s);
  /// Periodic call; applies the TELEOP input timeout. Returns the transition if one happened.
  TransitionResult tick(double now_s);

private:
  TransitionResult accept(Mode to, Reason r, double now_s);
  TransitionResult refuse(Reason r);

  TeleopModeConfig cfg_;
  bool hold_from_safety_{false};
  Mode mode_{Mode::kAuto};
  double last_input_s_{0.0};
  bool needs_replan_{false};
  std::size_t refused_{0};
  std::size_t accepted_{0};
};

}  // namespace teleop_core
