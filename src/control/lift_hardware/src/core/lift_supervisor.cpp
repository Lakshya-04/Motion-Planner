#include "lift_hardware/core/lift_supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace lift_hardware
{

namespace
{
constexpr double kLandedEps = 1e-9;
constexpr double kMaxShapeStepS = 0.25;
constexpr double kReadingMarginM = 0.01;
constexpr double kMaxStrokeM = 100.0;  // a device claiming more travel than this is lying (1e300 m overflowed the velocity estimate)
}  // namespace

const char * to_string(Phase phase)
{
  switch (phase) {
    case Phase::kDisconnected:
      return "disconnected";
    case Phase::kHoming:
      return "homing";
    case Phase::kReady:
      return "ready";
    case Phase::kHeld:
      return "held";
  }
  return "unknown";
}

LiftSupervisor::LiftSupervisor(std::unique_ptr<LiftTransport> transport, const LiftConfig & config)
: transport_(std::move(transport)),
  config_(config),
  next_attempt_s_(-std::numeric_limits<double>::infinity()),
  backoff_s_(config.reconnect_initial_s)
{
}

void LiftSupervisor::poll(double now_s)
{
  if (shut_down_) {
    return;
  }
  switch (snapshot_.phase) {
    case Phase::kDisconnected:
      if (now_s >= next_attempt_s_) {
        try_connect(now_s);
      }
      break;
    case Phase::kHoming:
      poll_homing(now_s);
      break;
    case Phase::kReady:
      poll_ready(now_s);
      break;
    case Phase::kHeld:
      poll_held(now_s);
      break;
  }
}

void LiftSupervisor::try_connect(double now_s)
{
  double stroke = 0.0;
  Fault fault = transport_->connect();
  if (fault == Fault::kNone) {
    fault = transport_->read_stroke(stroke);
  }
  if (fault == Fault::kNone && !(std::isfinite(stroke) && stroke > 0.0 && stroke <= kMaxStrokeM)) {
    fault = Fault::kOutOfRange;
  }
  if (fault != Fault::kNone) {
    transport_->disconnect();
    snapshot_.fault = fault == Fault::kUnsupported ? Fault::kUnsupported : Fault::kLinkLost;
    next_attempt_s_ = now_s + backoff_s_;
    backoff_s_ = std::min(backoff_s_ * config_.reconnect_factor, config_.reconnect_max_s);
    return;
  }
  backoff_s_ = config_.reconnect_initial_s;
  snapshot_.fault = Fault::kNone;
  snapshot_.stroke_m = config_.stroke_m > 0.0 ? std::min(config_.stroke_m, stroke) : stroke;
  begin_homing(now_s);
}

void LiftSupervisor::begin_homing(double now_s)
{
  snapshot_.phase = Phase::kHoming;
  snapshot_.homed = false;
  snapshot_.velocity_mps = 0.0;
  snapshot_.stop_confirmed = true;
  have_previous_height_ = false;
  io_failures_ = 0;
  phase_start_s_ = now_s;
  const double home = std::clamp(config_.home_height_m, 0.0, snapshot_.stroke_m);
  const Fault fault = transport_->move_to(home);
  home_sent_ = fault == Fault::kNone;
  note_io_result(fault, now_s);
}

bool LiftSupervisor::read_device(double now_s, Reading & reading)
{
  const Fault fault = transport_->read(reading);
  note_io_result(fault, now_s);
  if (fault != Fault::kNone) {
    return false;
  }
  if (reading.state == DeviceState::kFaulted) {
    enter_fault(reading.fault != Fault::kNone ? reading.fault : Fault::kDevice, now_s);
    return false;
  }
  const bool sane = std::isfinite(reading.height_m) && reading.height_m >= -kReadingMarginM &&
    reading.height_m <= snapshot_.stroke_m + std::max(config_.tolerance_m, kReadingMarginM);
  if (!sane) {
    enter_fault(Fault::kOutOfRange, now_s);
    return false;
  }
  snapshot_.height_m = reading.height_m;
  update_velocity(now_s, reading.height_m);
  return true;
}

void LiftSupervisor::update_velocity(double now_s, double height_m)
{
  if (have_previous_height_ && now_s > previous_height_s_) {
    const double dt = now_s - previous_height_s_;
    const double raw = (height_m - previous_height_m_) / dt;
    const double alpha = dt / (config_.velocity_filter_s + dt);
    snapshot_.velocity_mps += alpha * (raw - snapshot_.velocity_mps);
  }
  have_previous_height_ = true;
  previous_height_m_ = height_m;
  previous_height_s_ = now_s;
}

void LiftSupervisor::note_io_result(Fault fault, double now_s)
{
  switch (fault) {
    case Fault::kNone:
      io_failures_ = 0;
      return;
    case Fault::kLinkLost:
      lose_link(now_s);
      return;
    case Fault::kOutOfRange:
    case Fault::kDevice:
    case Fault::kUnsupported:
      enter_fault(fault, now_s);
      return;
    case Fault::kTimeout:
      if (++io_failures_ >= config_.max_io_failures) {
        lose_link(now_s);
      }
      return;
  }
}

void LiftSupervisor::lose_link(double now_s)
{
  transport_->stop();  // best effort; the device may still hear it
  transport_->disconnect();
  snapshot_.phase = Phase::kDisconnected;
  snapshot_.fault = Fault::kLinkLost;
  snapshot_.homed = false;
  snapshot_.velocity_mps = 0.0;
  snapshot_.stroke_m = 0.0;
  snapshot_.stop_confirmed = true;
  shaping_started_ = false;
  have_previous_height_ = false;
  next_attempt_s_ = now_s + backoff_s_;
  backoff_s_ = std::min(backoff_s_ * config_.reconnect_factor, config_.reconnect_max_s);
}

void LiftSupervisor::enter_fault(Fault fault, double now_s)
{
  snapshot_.phase = Phase::kHeld;
  snapshot_.fault = fault;
  snapshot_.velocity_mps = 0.0;
  snapshot_.stop_confirmed = false;
  shaping_started_ = false;
  have_previous_height_ = false;
  io_failures_ = 0;
  const Fault stopped = transport_->stop();
  if (stopped == Fault::kNone) {
    snapshot_.stop_confirmed = true;
  } else if (stopped == Fault::kLinkLost) {
    lose_link(now_s);
  }
}

void LiftSupervisor::poll_homing(double now_s)
{
  if (now_s - phase_start_s_ > config_.home_timeout_s) {
    enter_fault(Fault::kTimeout, now_s);
    return;
  }
  Reading reading;
  if (!read_device(now_s, reading) || snapshot_.phase != Phase::kHoming) {
    return;
  }
  const double home = std::clamp(config_.home_height_m, 0.0, snapshot_.stroke_m);
  if (!home_sent_) {
    const Fault fault = transport_->move_to(home);
    home_sent_ = fault == Fault::kNone;
    note_io_result(fault, now_s);
    return;
  }
  if (std::abs(reading.height_m - home) <= config_.tolerance_m && reading.state == DeviceState::kIdle) {
    snapshot_.phase = Phase::kReady;
    snapshot_.homed = true;
    snapshot_.velocity_mps = 0.0;
    const double inside = std::clamp(reading.height_m, 0.0, snapshot_.stroke_m);   // a reading may be a hair outside
    reference_ = {inside, 0.0};
    target_m_ = inside;
    last_sent_m_ = home;
    sent_any_ = true;
    last_send_s_ = now_s;
    shaping_started_ = false;
    at_target_ = true;
    out_of_tolerance_ = false;
    snapshot_.reference_m = inside;
  }
}

void LiftSupervisor::poll_ready(double now_s)
{
  Reading reading;
  if (!read_device(now_s, reading) || snapshot_.phase != Phase::kReady) {
    return;
  }
  const bool missed = at_target_ && std::abs(reading.height_m - target_m_) > config_.tolerance_m;
  if (!missed) {
    out_of_tolerance_ = false;
    return;
  }
  if (!out_of_tolerance_) {
    out_of_tolerance_ = true;
    out_of_tolerance_since_s_ = now_s;
  } else if (now_s - out_of_tolerance_since_s_ > config_.move_timeout_s) {
    enter_fault(Fault::kTimeout, now_s);
  }
}

void LiftSupervisor::poll_held(double now_s)
{
  if (!snapshot_.stop_confirmed) {
    const Fault stopped = transport_->stop();
    if (stopped == Fault::kNone) {
      snapshot_.stop_confirmed = true;
      io_failures_ = 0;
    } else if (stopped == Fault::kLinkLost || ++io_failures_ >= config_.max_io_failures) {
      lose_link(now_s);
      return;
    }
  }
  Reading reading;
  const Fault fault = transport_->read(reading);
  if (fault == Fault::kLinkLost) {
    lose_link(now_s);
  } else if (fault == Fault::kNone && std::isfinite(reading.height_m)) {
    snapshot_.height_m = reading.height_m;  // report where the lift really stands
  }
}

void LiftSupervisor::command(double now_s, double target_m)
{
  if (shut_down_ || snapshot_.phase != Phase::kReady || !std::isfinite(target_m)) {
    return;
  }
  target_m_ = std::clamp(target_m, 0.0, snapshot_.stroke_m);

  const double dt =
    shaping_started_ ? std::clamp(now_s - last_shape_s_, 0.0, kMaxShapeStepS) : 0.0;
  if (!shaping_started_) {
    shaping_started_ = true;
    // a reading may sit a little outside [0, stroke] (the sanity margin): the reference starts inside the travel
    reference_ = {std::clamp(snapshot_.height_m, 0.0, snapshot_.stroke_m), 0.0};
  }
  last_shape_s_ = now_s;
  reference_ = advance_toward(
    reference_, target_m_, config_.max_velocity_mps, config_.max_acceleration_mps2, dt);
  // A target that reverses while the reference runs toward a travel end brakes past it (v^2 / 2a): the shaped
  // reference, and so every height sent to the device, stays inside [0, stroke]; at the end it stops.
  if (reference_.position < 0.0 || reference_.position > snapshot_.stroke_m) {
    reference_.position = std::clamp(reference_.position, 0.0, snapshot_.stroke_m);
    reference_.velocity = 0.0;
  }
  snapshot_.reference_m = reference_.position;
  at_target_ = std::abs(reference_.position - target_m_) < kLandedEps;

  const bool due = !sent_any_ || now_s - last_send_s_ >= config_.send_period_s;
  const bool moved = std::abs(reference_.position - last_sent_m_) > config_.resend_deadband_m;
  const bool final_pending = at_target_ && std::abs(last_sent_m_ - target_m_) > kLandedEps;
  if (due && (moved || final_pending)) {
    const Fault fault = transport_->move_to(reference_.position);
    if (fault == Fault::kNone) {
      last_sent_m_ = reference_.position;
      last_send_s_ = now_s;
      sent_any_ = true;
      io_failures_ = 0;
    } else {
      note_io_result(fault, now_s);
    }
  }
}

bool LiftSupervisor::reset_fault(double now_s)
{
  if (shut_down_ || snapshot_.phase != Phase::kHeld) {
    return false;
  }
  snapshot_.fault = Fault::kNone;
  begin_homing(now_s);
  return true;
}

void LiftSupervisor::shutdown()
{
  if (shut_down_) {
    return;
  }
  transport_->stop();
  transport_->disconnect();
  shut_down_ = true;
  snapshot_.phase = Phase::kDisconnected;
  snapshot_.homed = false;
  snapshot_.velocity_mps = 0.0;
}

}  // namespace lift_hardware
