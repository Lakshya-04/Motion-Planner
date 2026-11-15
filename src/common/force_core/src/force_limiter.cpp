#include "force_core/force_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace force_core
{

ForceLimiter::ForceLimiter(const ForceLimiterParams & params)
: p_(params), k_(params.k_max_n_per_m), cov_(1e-6)
{
  if (!(p_.f_max_n > 0.0) || !(p_.alpha_per_s > 0.0) || !(p_.k_min_n_per_m > 0.0) ||
    !(p_.k_max_n_per_m >= p_.k_min_n_per_m) || !(p_.forgetting > 0.0 && p_.forgetting <= 1.0) ||
    !(p_.contact_force_n >= 0.0) || !(p_.min_step_m > 0.0) || !(p_.free_space_speed_mps > 0.0))
  {
    throw std::invalid_argument("force limiter: parameter out of range");
  }
}

void ForceLimiter::reset()
{
  k_ = p_.k_max_n_per_m;
  cov_ = 1e-6;
  has_estimate_ = false;
  have_prev_ = false;
}

void ForceLimiter::update(double normal_pos_m, double force_n)
{
  const bool in_contact = force_n >= p_.contact_force_n;
  if (have_prev_ && in_contact && prev_force_ >= p_.contact_force_n) {
    const double dx = normal_pos_m - prev_pos_;
    const double df = force_n - prev_force_;
    if (std::fabs(dx) >= p_.min_step_m) {
      // Scalar RLS for df = k dx.
      const double gain = cov_ * dx / (p_.forgetting + dx * cov_ * dx);
      const double k_new = k_ + gain * (df - k_ * dx);
      cov_ = (cov_ - gain * dx * cov_) / p_.forgetting;
      if (!has_estimate_) {
        // First informative step: start from this ratio rather than blending with the prior.
        k_ = df / dx;
        cov_ = 1.0 / (dx * dx) * 1e-3;
        has_estimate_ = std::isfinite(k_);
        if (!has_estimate_) {k_ = p_.k_max_n_per_m;}
      } else if (std::isfinite(k_new)) {
        k_ = k_new;
      }
      k_ = std::clamp(k_, p_.k_min_n_per_m, p_.k_max_n_per_m);
      cov_ = std::clamp(cov_, 1e-12, 1.0);
    }
  }
  prev_pos_ = normal_pos_m;
  prev_force_ = force_n;
  have_prev_ = true;
}

double ForceLimiter::maxApproachVelocity(double force_n) const
{
  if (force_n < p_.contact_force_n) {return p_.free_space_speed_mps;}
  return p_.alpha_per_s * (p_.f_max_n - force_n) / k_;
}

double ForceLimiter::limit(double v_cmd, double force_n) const
{
  return std::min(v_cmd, maxApproachVelocity(force_n));
}

}  // namespace force_core
