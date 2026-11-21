#include "teleop_core/jog_shaper.hpp"

#include <algorithm>
#include <cmath>

namespace teleop_core
{

JogShaper::JogShaper(const JogShaperConfig & config)
{
  setConfig(config);
}

void JogShaper::setConfig(const JogShaperConfig & config)
{
  cfg_ = config;
  for (auto & d : cfg_.deadband) {
    d = std::clamp(std::isfinite(d) ? d : 0.0, 0.0, 0.99);
  }
  cfg_.expo = std::clamp(std::isfinite(cfg_.expo) ? cfg_.expo : 0.0, 0.0, 1.0);
  cfg_.max_linear_mps = std::max(0.0, std::isfinite(cfg_.max_linear_mps) ? cfg_.max_linear_mps : 0.0);
  cfg_.max_angular_rps = std::max(0.0, std::isfinite(cfg_.max_angular_rps) ? cfg_.max_angular_rps : 0.0);
}

void JogShaper::reset()
{
  input_.setZero();
  prev_.setZero();
  have_input_ = false;
  last_invalid_ = false;
}

void JogShaper::setInput(const Vector6d & input, double now_s)
{
  bool valid = std::isfinite(now_s);
  for (int i = 0; i < 6; ++i) {
    if (std::isfinite(input[i])) {
      input_[i] = input[i];
    } else {
      input_[i] = 0.0;
      valid = false;
    }
  }
  last_invalid_ = !valid;
  if (valid) {
    have_input_ = true;
    last_input_s_ = now_s;
  }
}

double JogShaper::shapeAxis(double x, double deadband, double expo)
{
  const double a = std::fabs(x);
  if (a <= deadband) {
    return 0.0;
  }
  const double r = (a - deadband) / (1.0 - deadband);
  const double e = (1.0 - expo) * r + expo * r * r * r;
  return x < 0.0 ? -e : e;
}

JogOutput JogShaper::update(double now_s, double dt, const FrameRotations & rot)
{
  JogOutput out;
  out.invalid_input = last_invalid_;
  if (!have_input_ || !std::isfinite(now_s) || now_s - last_input_s_ > cfg_.watchdog_s) {
    out.watchdog_tripped = true;
    prev_.setZero();  // immediate zero, no ramp: fail safe
    return out;
  }

  Vector6d target = Vector6d::Zero();
  for (int i = 0; i < 6; ++i) {
    double x = input_[i];
    if (std::fabs(x) > 1.0) {
      out.saturated = true;
      x = std::clamp(x, -1.0, 1.0);
    }
    const double s = cfg_.enabled[i] ? shapeAxis(x, cfg_.deadband[i], cfg_.expo) : 0.0;
    target[i] = s * (i < 3 ? cfg_.max_linear_mps : cfg_.max_angular_rps);
  }

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  if (cfg_.frame == JogFrame::kTool) {
    R = rot.tool_in_base;
  } else if (cfg_.frame == JogFrame::kWorld) {
    R = rot.world_in_base;
  }
  Eigen::Vector3d v = R * target.head<3>();
  Eigen::Vector3d w = R * target.tail<3>();

  const double step = std::isfinite(dt) ? std::max(0.0, dt) : 0.0;
  auto slew = [step](const Eigen::Vector3d & prev, const Eigen::Vector3d & want, double accel) {
      if (accel <= 0.0) {
        return want;
      }
      const Eigen::Vector3d d = want - prev;
      const double limit = accel * step;
      const double n = d.norm();
      return n <= limit ? want : Eigen::Vector3d(prev + d * (limit / n));
    };
  v = slew(prev_.head<3>(), v, cfg_.max_linear_accel);
  w = slew(prev_.tail<3>(), w, cfg_.max_angular_accel);
  prev_.head<3>() = v;
  prev_.tail<3>() = w;
  out.twist = prev_;
  return out;
}

}  // namespace teleop_core
