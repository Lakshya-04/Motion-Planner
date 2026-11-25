/**
 * @file heartbeat_watchdog.hpp
 * @brief Reports expiry when a heartbeat is missing for longer than a timeout.
 * @details Fail-safe: never beaten, a non-finite or negative time, or a clock that went backwards
 *          past the last beat all count as expired. The clock is the caller's (seconds).
 *          NOT A CERTIFIED SAFETY FUNCTION, see the package README.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cmath>
#include <stdexcept>

namespace safety_monitor
{

class HeartbeatWatchdog
{
public:
  /// `timeout_s` must be finite and >= 0, otherwise std::invalid_argument is thrown.
  explicit HeartbeatWatchdog(double timeout_s) : timeout_s_(timeout_s)
  {
    if (!std::isfinite(timeout_s) || timeout_s < 0.0) {
      throw std::invalid_argument("HeartbeatWatchdog: timeout must be finite and >= 0");
    }
  }

  /// Records a heartbeat at `now_s`; an invalid time is ignored (the watchdog keeps ageing).
  void beat(double now_s)
  {
    if (std::isfinite(now_s) && now_s >= 0.0) {
      last_ = now_s;
      beaten_ = true;
    }
  }

  bool expired(double now_s) const
  {
    if (!beaten_ || !std::isfinite(now_s) || now_s < 0.0) {return true;}
    const double age = now_s - last_;
    return age < 0.0 || age > timeout_s_;
  }

  double timeout() const {return timeout_s_;}

private:
  double timeout_s_;
  double last_{0.0};
  bool beaten_{false};
};

}  // namespace safety_monitor
