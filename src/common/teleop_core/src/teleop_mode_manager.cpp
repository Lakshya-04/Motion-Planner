#include "teleop_core/teleop_mode_manager.hpp"

#include <cmath>

namespace teleop_core
{

TeleopModeManager::TeleopModeManager(const TeleopModeConfig & config)
: cfg_(config) {}

TransitionResult TeleopModeManager::accept(Mode to, Reason r, double now_s)
{
  mode_ = to;
  ++accepted_;
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
      return accept(Mode::kTeleop, Reason::kOperatorRequest, now_s);
    case Mode::kHold:
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
      if (needs_replan_ && !replan_acknowledged) {
        return refuse(Reason::kNeedsReplanAck);
      }
      return accept(Mode::kAuto, Reason::kOperatorRequest, now_s);
  }
  return refuse(Reason::kAlreadyInMode);
}

void TeleopModeManager::noteInput(double now_s)
{
  if (mode_ == Mode::kTeleop && std::isfinite(now_s)) {
    last_input_s_ = now_s;
  }
}

TransitionResult TeleopModeManager::tick(double now_s)
{
  if (mode_ == Mode::kTeleop && (!std::isfinite(now_s) || now_s - last_input_s_ > cfg_.input_timeout_s)) {
    return accept(Mode::kHold, Reason::kInputTimeout, now_s);
  }
  return {false, mode_, Reason::kOk};
}

}  // namespace teleop_core
