#include "teleop_core/jog_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace teleop_core
{

JogLimiter::JogLimiter(const JogLimiterConfig & config)
: cfg_(config)
{
  if (!std::isfinite(cfg_.stale_timeout_s) || cfg_.stale_timeout_s <= 0.0) {
    throw std::invalid_argument("JogLimiterConfig.stale_timeout_s must be finite and > 0");
  }
}

void JogLimiter::setSpeedScale(double scale, double now_s)
{
  if (!std::isfinite(scale) || !std::isfinite(now_s)) {
    have_scale_ = false;  // a bad value invalidates the previous good one: fail safe
    return;
  }
  scale_ = std::clamp(scale, 0.0, 1.0);
  scale_stamp_s_ = now_s;
  have_scale_ = true;
}

LimitedTwist JogLimiter::limit(const Vector6d & twist, double now_s) const
{
  LimitedTwist out;
  if (stop_) {
    out.reason = LimitReason::kStop;
    return out;
  }
  if (!have_scale_ || !std::isfinite(now_s) || now_s - scale_stamp_s_ > cfg_.stale_timeout_s ||
    now_s < scale_stamp_s_)
  {
    out.reason = LimitReason::kStaleScale;
    return out;
  }
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(twist[i])) {
      out.reason = LimitReason::kInvalidTwist;
      return out;
    }
  }
  out.scale_applied = scale_;
  out.twist = twist * scale_;
  out.reason = scale_ < 1.0 ? LimitReason::kScaled : LimitReason::kNone;
  return out;
}

}  // namespace teleop_core
