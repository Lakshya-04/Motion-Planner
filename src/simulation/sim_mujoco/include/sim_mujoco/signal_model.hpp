#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <random>

namespace sim_mujoco
{

/**
 * Sensor / actuator non-idealities for one scalar signal, applied once per fixed control period dt.
 * Every effect is off at its default, and a default-constructed config passes the signal through
 * unchanged (enabled() == false).
 *
 * Order per sample: x + bias + white noise -> first-order low-pass -> fixed delay -> quantisation.
 * (The low-pass and the delay model the sensor and the driver, the noise and bias the sensing element.)
 * NaN input is passed through untouched (commands that are not set yet), also when the model is on.
 */
struct SignalConfig
{
  double noise_density{0.0};   // white noise, unit/sqrt(Hz); per-sample std = density / sqrt(2 dt)
  double bias_walk{0.0};       // bias random walk, unit/sqrt(s); starts at 0
  double lowpass_hz{0.0};      // first-order low-pass corner, Hz (0 = off)
  double delay_s{0.0};         // fixed delay, rounded to whole periods
  double quantum{0.0};         // quantisation step, unit (0 = off)
  uint64_t seed{1};

  bool enabled() const
  {
    return noise_density != 0.0 || bias_walk != 0.0 || lowpass_hz > 0.0 || delay_s > 0.0 || quantum > 0.0;
  }
};

class SignalModel
{
public:
  SignalModel(const SignalConfig & cfg, double dt)
  : cfg_(cfg), rng_(cfg.seed)
  {
    noise_std_ = cfg.noise_density / std::sqrt(2.0 * dt);
    walk_std_ = cfg.bias_walk * std::sqrt(dt);
    alpha_ = cfg.lowpass_hz > 0.0 ? 1.0 - std::exp(-2.0 * M_PI * cfg.lowpass_hz * dt) : 1.0;
    delay_n_ = cfg.delay_s > 0.0 ? static_cast<size_t>(std::llround(cfg.delay_s / dt)) : 0;
  }

  double step(double x)
  {
    if (!cfg_.enabled() || std::isnan(x)) {
      return x;
    }
    bias_ += walk_std_ * gauss_(rng_);
    double y = x + bias_ + noise_std_ * gauss_(rng_);
    if (!primed_) {
      lp_ = y;
      primed_ = true;
    } else {
      lp_ += alpha_ * (y - lp_);
    }
    y = lp_;
    if (delay_n_ > 0) {
      queue_.push_back(y);
      y = queue_.front();  // until the line is full the first sample is held
      if (queue_.size() > delay_n_) {
        queue_.pop_front();
      }
    }
    if (cfg_.quantum > 0.0) {
      y = cfg_.quantum * std::round(y / cfg_.quantum);
    }
    return y;
  }

  double bias() const {return bias_;}
  size_t delaySamples() const {return delay_n_;}

private:
  SignalConfig cfg_;
  double noise_std_{0.0}, walk_std_{0.0}, alpha_{1.0}, bias_{0.0}, lp_{0.0};
  size_t delay_n_{0};
  bool primed_{false};
  std::deque<double> queue_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> gauss_{0.0, 1.0};
};

}  // namespace sim_mujoco
