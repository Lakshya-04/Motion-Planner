/**
 * @file lift_config.hpp
 * @brief Tunables of the lift supervisor and their parsing from a string map.
 */

#pragma once

#include <string>
#include <unordered_map>

namespace lift_hardware
{

struct LiftConfig
{
  /// Travel limit to enforce in metres; 0 uses what the device reports (a smaller value wins).
  double stroke_m{0.0};
  double max_velocity_mps{0.1};
  double max_acceleration_mps2{0.2};

  /// Height driven to after every connect; the lift counts as homed once it is there.
  double home_height_m{0.0};
  double home_timeout_s{60.0};

  /// A target counts as reached within this distance.
  double tolerance_m{0.005};
  /// The device may take this long to arrive after the shaped reference reached the target.
  double move_timeout_s{20.0};

  /// Targets are re-sent no more often than this, and only when they moved by `resend_deadband_m`.
  double send_period_s{0.05};
  double resend_deadband_m{0.0005};

  double reconnect_initial_s{0.5};
  double reconnect_max_s{8.0};
  double reconnect_factor{2.0};

  /// Consecutive failed calls (timeouts and the like) tolerated before the link is declared lost.
  int max_io_failures{5};

  /// Time constant of the low-pass filter on the velocity estimate.
  double velocity_filter_s{0.1};
};

/// Parse the number under `key` into `value`. A missing key leaves `value` alone and returns true;
/// text that is not entirely a finite number sets `error` and returns false.
bool read_number(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  double & value, std::string * error);

/// Empty when `config` is usable, otherwise what is wrong with it.
std::string validate(const LiftConfig & config);

/// Read the keys named like the fields above (`max_velocity_mps`, ...) from `params`; absent keys
/// keep their defaults. A malformed number or a failed validation sets `error` and returns the
/// defaults.
LiftConfig parse_lift_config(
  const std::unordered_map<std::string, std::string> & params, std::string * error);

}  // namespace lift_hardware
