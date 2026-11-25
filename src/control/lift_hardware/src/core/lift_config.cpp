#include "lift_hardware/core/lift_config.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>

namespace lift_hardware
{

bool read_number(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  double & value, std::string * error)
{
  const auto it = params.find(key);
  if (it == params.end()) {
    return true;
  }
  const char * text = it->second.c_str();
  char * end = nullptr;
  errno = 0;
  const double parsed = std::strtod(text, &end);
  if (end == text || *end != '\0' || errno != 0 || !std::isfinite(parsed)) {
    if (error != nullptr) {
      *error = "parameter '" + key + "' is not a number: '" + it->second + "'";
    }
    return false;
  }
  value = parsed;
  return true;
}

std::string validate(const LiftConfig & c)
{
  const auto finite = [](double v) {return std::isfinite(v);};
  if (!finite(c.stroke_m) || c.stroke_m < 0.0) {
    return "stroke_m must be >= 0";
  }
  if (!finite(c.max_velocity_mps) || c.max_velocity_mps <= 0.0) {
    return "max_velocity_mps must be > 0";
  }
  if (!finite(c.max_acceleration_mps2) || c.max_acceleration_mps2 <= 0.0) {
    return "max_acceleration_mps2 must be > 0";
  }
  if (!finite(c.home_height_m) || c.home_height_m < 0.0) {
    return "home_height_m must be >= 0";
  }
  if (c.stroke_m > 0.0 && c.home_height_m > c.stroke_m) {
    return "home_height_m must not exceed stroke_m";
  }
  if (!finite(c.home_timeout_s) || c.home_timeout_s <= 0.0) {
    return "home_timeout_s must be > 0";
  }
  if (!finite(c.tolerance_m) || c.tolerance_m <= 0.0) {
    return "tolerance_m must be > 0";
  }
  if (!finite(c.move_timeout_s) || c.move_timeout_s <= 0.0) {
    return "move_timeout_s must be > 0";
  }
  if (!finite(c.send_period_s) || c.send_period_s < 0.0) {
    return "send_period_s must be >= 0";
  }
  if (!finite(c.resend_deadband_m) || c.resend_deadband_m < 0.0) {
    return "resend_deadband_m must be >= 0";
  }
  if (!finite(c.reconnect_initial_s) || c.reconnect_initial_s <= 0.0) {
    return "reconnect_initial_s must be > 0";
  }
  if (!finite(c.reconnect_max_s) || c.reconnect_max_s < c.reconnect_initial_s) {
    return "reconnect_max_s must be >= reconnect_initial_s";
  }
  if (!finite(c.reconnect_factor) || c.reconnect_factor < 1.0) {
    return "reconnect_factor must be >= 1";
  }
  if (c.max_io_failures < 1) {
    return "max_io_failures must be >= 1";
  }
  if (!finite(c.velocity_filter_s) || c.velocity_filter_s < 0.0) {
    return "velocity_filter_s must be >= 0";
  }
  return {};
}

LiftConfig parse_lift_config(
  const std::unordered_map<std::string, std::string> & params, std::string * error)
{
  LiftConfig c;
  double io_failures = c.max_io_failures;
  const bool parsed =
    read_number(params, "stroke_m", c.stroke_m, error) &&
    read_number(params, "max_velocity_mps", c.max_velocity_mps, error) &&
    read_number(params, "max_acceleration_mps2", c.max_acceleration_mps2, error) &&
    read_number(params, "home_height_m", c.home_height_m, error) &&
    read_number(params, "home_timeout_s", c.home_timeout_s, error) &&
    read_number(params, "tolerance_m", c.tolerance_m, error) &&
    read_number(params, "move_timeout_s", c.move_timeout_s, error) &&
    read_number(params, "send_period_s", c.send_period_s, error) &&
    read_number(params, "resend_deadband_m", c.resend_deadband_m, error) &&
    read_number(params, "reconnect_initial_s", c.reconnect_initial_s, error) &&
    read_number(params, "reconnect_max_s", c.reconnect_max_s, error) &&
    read_number(params, "reconnect_factor", c.reconnect_factor, error) &&
    read_number(params, "max_io_failures", io_failures, error) &&
    read_number(params, "velocity_filter_s", c.velocity_filter_s, error);
  if (!parsed) {
    return LiftConfig{};
  }
  if (io_failures != std::floor(io_failures) || io_failures > 1.0e6) {
    if (error != nullptr) {
      *error = "parameter 'max_io_failures' must be a whole number";
    }
    return LiftConfig{};
  }
  c.max_io_failures = static_cast<int>(io_failures);
  const std::string problem = validate(c);
  if (!problem.empty()) {
    if (error != nullptr) {
      *error = problem;
    }
    return LiftConfig{};
  }
  return c;
}

}  // namespace lift_hardware
