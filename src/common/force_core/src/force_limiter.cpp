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
        // First informative step: start from this ratio rather than blending with the prior. A
        // non-positive ratio is sensor noise (a spring wall pushes back), not a stiffness: keep
        // the conservative prior instead of an optimistic soft estimate.
        if (df / dx > 0.0) {
          k_ = df / dx;
          cov_ = 1.0 / (dx * dx) * 1e-3;
          has_estimate_ = std::isfinite(k_);
          if (!has_estimate_) {k_ = p_.k_max_n_per_m;}
        } else {
          cov_ = 1e-6;
        }
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

NormalAxisLimiter::NormalAxisLimiter(const NormalAxisLimiterParams & params)
: p_(params), limiter_(params.limiter)
{
  if (!(p_.max_offset_m > 0.0) || !(p_.windup_gain_n_per_m >= 0.0) ||
    !(p_.retreat_speed_mps > 0.0) || !(p_.reference_speed_cap_mps > 0.0))
  {
    throw std::invalid_argument("normal axis limiter: parameter out of range");
  }
}

void NormalAxisLimiter::reset()
{
  limiter_.reset();
  have_prev_ = false;
  windup_force_n_ = 0.0;
  reset_requested_ = false;
  have_prev_ref_ = false;
}

double NormalAxisLimiter::step(
  double dt, double p_desired, double p_reference, double p_measured, double force_n)
{
  if (!std::isfinite(p_desired) || !std::isfinite(p_reference) || !std::isfinite(p_measured) ||
    !std::isfinite(force_n) || !(dt > 0.0))
  {
    return p_desired;  // nothing safe to compute; the caller's other guards decide
  }
  // Identify the stiffness from the COMMANDED position to the force: that is the gain the barrier
  // acts through, and for a soft position-controlled arm it is far below the wall's stiffness (the
  // force builds from the lead of the command over the tool, the tool itself barely moves).
  limiter_.update(have_prev_ ? prev_cmd_ : p_measured, force_n);
  const double ref_velocity = have_prev_ref_ ? (p_reference - prev_ref_) / dt : 0.0;
  prev_ref_ = p_reference;
  have_prev_ref_ = true;
  reset_requested_ = ref_velocity < -p_.retreat_speed_mps && p_desired > p_reference;
  const double cap = reset_requested_ ? 0.0 : p_.max_offset_m;
  const double excess = std::max(0.0, (p_desired - p_reference) - cap);
  windup_force_n_ = reset_requested_ ? 0.0 : p_.windup_gain_n_per_m * excess;
  const double target = std::min(p_desired, p_reference + cap);
  if (!have_prev_) {
    // Start from the measured position so the first cycle cannot jump into the wall.
    prev_cmd_ = std::min(target, p_measured);
    have_prev_ = true;
  }
  double v_max = limiter_.maxApproachVelocity(force_n);
  if (force_n < p_.limiter.contact_force_n) {
    // Free space: let the planned reference advance through (up to a cap), hold the offset's advance.
    v_max = std::max(v_max, std::min(std::max(ref_velocity, 0.0), p_.reference_speed_cap_mps));
  }
  const double cmd = std::min(target, prev_cmd_ + v_max * dt);
  prev_cmd_ = cmd;
  return cmd;
}

}  // namespace force_core
