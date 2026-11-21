#include "teleop_core/teleop_mode_manager.hpp"

#include <cmath>
#include <stdexcept>

namespace teleop_core
{

TeleopModeManager::TeleopModeManager(const TeleopModeConfig & config)
: cfg_(config)
{
  if (!std::isfinite(cfg_.input_timeout_s) || cfg_.input_timeout_s <= 0.0) {
    throw std::invalid_argument("TeleopModeConfig.input_timeout_s must be finite and > 0");
  }
}

TransitionResult TeleopModeManager::accept(Mode to, Reason r, double now_s)
{
  mode_ = to;
  ++accepted_;
  hold_from_safety_ = (to == Mode::kHold && r == Reason::kSafetyStop);
  if (to == Mode::kTeleop) {
    last_input_s_ = now_s;
    needs_replan_ = true;
  } else if (to == Mode::kAuto) {
    needs_replan_ = false;
  }
  return {true, mode_, r};
}

TransitionResult TeleopModeManager::refuse(Reason r)
{
  if (r != Reason::kAlreadyInMode) {
    ++refused_;
  }
  return {false, mode_, r};
}

TransitionResult TeleopModeManager::onStop(bool safety, double now_s)
{
  if (mode_ == Mode::kHold) {
    return refuse(Reason::kAlreadyInMode);
  }
  return accept(Mode::kHold, safety ? Reason::kSafetyStop : Reason::kStopRequested, now_s);
}

TransitionResult TeleopModeManager::requestTeleop(bool segment_halted, double now_s)
{
  switch (mode_) {
    case Mode::kTeleop:
      return refuse(Reason::kAlreadyInMode);
    case Mode::kAuto:
      if (!segment_halted) {
        return refuse(Reason::kSegmentStillRunning);
      }
      if (!std::isfinite(now_s)) {
        return refuse(Reason::kInvalidTime);
      }
      return accept(Mode::kTeleop, Reason::kOperatorRequest, now_s);
    case Mode::kHold:
      if (!std::isfinite(now_s)) {
        return refuse(Reason::kInvalidTime);
      }
      return accept(Mode::kTeleop, Reason::kOperatorRequest, now_s);
  }
  return refuse(Reason::kAlreadyInMode);
}

TransitionResult TeleopModeManager::requestAuto(
  bool operator_confirm, bool replan_acknowledged, double now_s)
{
  switch (mode_) {
    case Mode::kAuto:
      return refuse(Reason::kAlreadyInMode);
    case Mode::kTeleop:
      if (!operator_confirm) {
        return refuse(Reason::kNeedsConfirm);
      }
      if (!replan_acknowledged) {
        return refuse(Reason::kNeedsReplanAck);
      }
      return accept(Mode::kAuto, Reason::kOperatorConfirmed, now_s);
    case Mode::kHold:
      if (hold_from_safety_ && !operator_confirm) {
        return refuse(Reason::kNeedsConfirm);
      }
      if (needs_replan_ && !replan_acknowledged) {
        return refuse(Reason::kNeedsReplanAck);
      }
      return accept(Mode::kAuto, Reason::kOperatorRequest, now_s);
  }
  return refuse(Reason::kAlreadyInMode);
}

void TeleopModeManager::noteInput(double now_s)
{
  // A non-finite stamp is ignored. A stamp later than the next tick time is caught in tick().
  if (mode_ == Mode::kTeleop && std::isfinite(now_s)) {
    last_input_s_ = now_s;
  }
}

TransitionResult TeleopModeManager::tick(double now_s)
{
  if (mode_ == Mode::kTeleop) {
    // Fail safe on every bad clock: non-finite time, time before the last input (clock jumped back or
    // a future input stamp), or a plain timeout.
    const bool bad_time = !std::isfinite(now_s) || !std::isfinite(last_input_s_) || now_s < last_input_s_;
    if (bad_time || now_s - last_input_s_ > cfg_.input_timeout_s) {
      return accept(Mode::kHold, Reason::kInputTimeout, now_s);
    }
  }
  return {false, mode_, Reason::kOk};
}

}  // namespace teleop_core
