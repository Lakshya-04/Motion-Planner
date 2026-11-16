#include "force_core/bias_tracker.hpp"

#include <cmath>

namespace force_core
{
namespace
{
double & at(core_types::Wrench & w, std::size_t i)
{
  double * p[6] = {&w.force_x, &w.force_y, &w.force_z, &w.torque_x, &w.torque_y, &w.torque_z};
  return *p[i];
}
double val(const core_types::Wrench & w, std::size_t i)
{
  return std::array<double, 6>{w.force_x, w.force_y, w.force_z, w.torque_x, w.torque_y, w.torque_z}[i];
}
}  // namespace

BiasTracker::BiasTracker(const core_types::Wrench & initial_bias, const BiasTrackerConfig & config)
: config_(config), bias_(initial_bias), initial_(initial_bias)
{
  for (std::size_t i = 0; i < 6; ++i) {
    var_[i] = i < 3 ? config_.initial_variance_force : config_.initial_variance_torque;
  }
}

BiasUpdateStatus BiasTracker::update(const core_types::Wrench & residual, bool free_of_contact, bool still)
{
  if (!free_of_contact || !still) {return BiasUpdateStatus::kSkippedNotFree;}
  // Gate on every axis first so a sample is used whole or not at all.
  std::array<double, 6> innov;
  for (std::size_t i = 0; i < 6; ++i) {
    innov[i] = val(residual, i) - val(bias_, i);
    const double thr = i < 3 ? config_.reject_threshold_force : config_.reject_threshold_torque;
    if (std::abs(innov[i]) > thr) {
      ++rejected_;
      ++consecutive_rejects_;
      return BiasUpdateStatus::kRejectedInnovation;
    }
  }
  core_types::Wrench next = bias_;
  std::array<double, 6> next_var = var_;
  for (std::size_t i = 0; i < 6; ++i) {
    const double q = i < 3 ? config_.process_noise_force : config_.process_noise_torque;
    const double r = i < 3 ? config_.measurement_noise_force : config_.measurement_noise_torque;
    const double p = var_[i] + q;
    const double k = p / (p + r);
    at(next, i) = val(bias_, i) + k * innov[i];
    next_var[i] = (1.0 - k) * p;
    const double cap = i < 3 ? config_.max_total_correction_force : config_.max_total_correction_torque;
    if (std::abs(val(next, i) - val(initial_, i)) > cap) {
      ++rejected_;
      ++consecutive_rejects_;
      return BiasUpdateStatus::kRejectedCap;
    }
  }
  bias_ = next;
  var_ = next_var;
  ++accepted_;
  consecutive_rejects_ = 0;
  return BiasUpdateStatus::kUpdated;
}

}  // namespace force_core
