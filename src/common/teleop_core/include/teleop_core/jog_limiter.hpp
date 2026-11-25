/**
 * @file jog_limiter.hpp
 * @brief Last stage before the servo engine: applies the safety speed scale and the stop request.
 * @details The twist is scaled uniformly (direction kept) by `speed_scale` in [0, 1]. A stop request forces
 *          zero. The scale must be refreshed: a scale never received, older than `stale_timeout_s`, or not
 *          finite gives zero (fail safe). The caller passes time; no clock, no ROS.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include "teleop_core/jog_shaper.hpp"

namespace teleop_core
{

enum class LimitReason {kNone = 0, kScaled, kStop, kStaleScale, kInvalidTwist};

struct JogLimiterConfig
{
  /// Must be finite and > 0, otherwise the constructor throws std::invalid_argument.
  double stale_timeout_s{0.5};
};

struct LimitedTwist
{
  Vector6d twist{Vector6d::Zero()};
  double scale_applied{0.0};
  LimitReason reason{LimitReason::kStaleScale};
};

class JogLimiter
{
public:
  explicit JogLimiter(const JogLimiterConfig & config = JogLimiterConfig{});

  /// Latest safety speed scale (from the safety layer), stamped at `now_s`.
  void setSpeedScale(double scale, double now_s);
  void setStop(bool stop) {stop_ = stop;}
  LimitedTwist limit(const Vector6d & twist, double now_s) const;

private:
  JogLimiterConfig cfg_;
  bool have_scale_{false};
  double scale_{0.0};
  double scale_stamp_s_{0.0};
  bool stop_{false};
};

}  // namespace teleop_core
