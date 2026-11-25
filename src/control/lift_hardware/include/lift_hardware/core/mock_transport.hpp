/**
 * @file mock_transport.hpp
 * @brief Simulated lift for simulation and CI: trapezoidal motion with travel limits.
 */

#pragma once

#include <functional>

#include "lift_hardware/core/lift_transport.hpp"
#include "lift_hardware/core/profile.hpp"

namespace lift_hardware
{

struct MockConfig
{
  double stroke_m{0.8};
  double max_velocity_mps{0.1};
  double max_acceleration_mps2{0.2};
  /// Where the simulated carriage starts, so that homing has something to do.
  double initial_height_m{0.25};
};

/**
 * Moves toward the last accepted target at the configured limits, advancing with the time the
 * clock reports at each call, so it needs no thread. A target outside [0, stroke] is rejected
 * with `Fault::kOutOfRange` like a real device would. Calls other than `connect` fail with
 * `Fault::kLinkLost` while disconnected. `connect` never fails.
 */
class MockTransport : public LiftTransport
{
public:
  using Clock = std::function<double()>;  ///< seconds on a monotonic clock

  /// Uses the steady clock when `clock` is empty.
  explicit MockTransport(const MockConfig & config, Clock clock = Clock());

  Fault connect() override;
  void disconnect() override;
  Fault read_stroke(double & stroke_m) override;
  Fault move_to(double height_m) override;
  Fault stop() override;
  Fault read(Reading & reading) override;

private:
  void advance();

  MockConfig config_;
  Clock clock_;
  bool connected_{false};
  bool have_time_{false};
  double last_time_s_{0.0};
  Kinematics motion_;
  double target_m_;
  bool moving_{false};
};

}  // namespace lift_hardware
