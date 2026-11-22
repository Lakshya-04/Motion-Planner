#include "lift_hardware/core/mock_transport.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace lift_hardware
{

namespace
{
constexpr double kMaxSubstepS = 0.01;

double steady_seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

MockTransport::MockTransport(const MockConfig & config, Clock clock)
: config_(config),
  clock_(clock ? std::move(clock) : Clock(steady_seconds)),
  motion_{config.initial_height_m, 0.0},
  target_m_(config.initial_height_m)
{
}

Fault MockTransport::connect()
{
  connected_ = true;
  have_time_ = false;
  return Fault::kNone;
}

void MockTransport::disconnect()
{
  connected_ = false;
  motion_.velocity = 0.0;
  moving_ = false;
}

void MockTransport::advance()
{
  const double now = clock_();
  if (!have_time_) {
    have_time_ = true;
    last_time_s_ = now;
    return;
  }
  double remaining = std::max(0.0, now - last_time_s_);
  last_time_s_ = now;
  while (remaining > 0.0) {
    const double dt = std::min(remaining, kMaxSubstepS);
    motion_ = advance_toward(
      motion_, target_m_, config_.max_velocity_mps, config_.max_acceleration_mps2, dt);
    remaining -= dt;
  }
  motion_.position = std::clamp(motion_.position, 0.0, config_.stroke_m);
  moving_ = std::abs(motion_.velocity) > 1e-9 || std::abs(target_m_ - motion_.position) > 1e-9;
}

Fault MockTransport::read_stroke(double & stroke_m)
{
  if (!connected_) {
    return Fault::kLinkLost;
  }
  stroke_m = config_.stroke_m;
  return Fault::kNone;
}

Fault MockTransport::move_to(double height_m)
{
  if (!connected_) {
    return Fault::kLinkLost;
  }
  advance();
  if (!std::isfinite(height_m) || height_m < 0.0 || height_m > config_.stroke_m) {
    return Fault::kOutOfRange;
  }
  target_m_ = height_m;
  moving_ = std::abs(target_m_ - motion_.position) > 1e-9;
  return Fault::kNone;
}

Fault MockTransport::stop()
{
  if (!connected_) {
    return Fault::kLinkLost;
  }
  advance();
  // Brake at the configured deceleration to a standstill.
  const double v = motion_.velocity;
  const double braking = v * std::abs(v) / (2.0 * config_.max_acceleration_mps2);
  target_m_ = std::clamp(motion_.position + braking, 0.0, config_.stroke_m);
  return Fault::kNone;
}

Fault MockTransport::read(Reading & reading)
{
  if (!connected_) {
    return Fault::kLinkLost;
  }
  advance();
  reading.height_m = motion_.position;
  reading.state = moving_ ? DeviceState::kMoving : DeviceState::kIdle;
  reading.fault = Fault::kNone;
  return Fault::kNone;
}

}  // namespace lift_hardware
