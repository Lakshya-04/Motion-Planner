#include "force_core/contact_free_gate.hpp"

#include <algorithm>
#include <cmath>

namespace force_core
{

ContactFreeGate::ContactFreeGate(const ContactFreeGateConfig & config)
: cfg_(config)
{
  cfg_.free_below_n = std::max(0.0, cfg_.free_below_n);
  cfg_.not_free_above_n = std::max(cfg_.not_free_above_n, cfg_.free_below_n);
  cfg_.debounce_ticks = std::max(1, cfg_.debounce_ticks);
  cfg_.min_settle_s = std::max(0.0, cfg_.min_settle_s);
}

bool ContactFreeGate::isFreeState(ForceFsmState state)
{
  return state == ForceFsmState::kIdle || state == ForceFsmState::kApproach;
}

void ContactFreeGate::reset()
{
  free_ = false;
  in_free_state_ = false;
  have_time_ = false;
  low_count_ = 0;
}

bool ContactFreeGate::update(ForceFsmState state, double force_n, double now_s, bool commanded_motion)
{
  if (!std::isfinite(now_s) || (have_time_ && now_s < last_s_)) {
    reset();  // bad or going-back time: fail safe, restart the settle wait
    return false;
  }
  last_s_ = now_s;
  have_time_ = true;

  if (!isFreeState(state)) {
    in_free_state_ = false;
    free_ = false;
    low_count_ = 0;
    return false;
  }
  if (!in_free_state_) {
    in_free_state_ = true;
    settle_start_s_ = now_s;
    free_ = false;
    low_count_ = 0;
  }
  if (!std::isfinite(force_n) || (cfg_.require_no_motion && commanded_motion)) {
    free_ = false;
    low_count_ = 0;
    return false;
  }
  const double f = std::fabs(force_n);
  if (free_) {
    if (f > cfg_.not_free_above_n) {
      free_ = false;
      low_count_ = 0;
    }
    return free_;
  }
  if (f < cfg_.free_below_n) {
    ++low_count_;
  } else {
    low_count_ = 0;
  }
  if (low_count_ >= cfg_.debounce_ticks && now_s - settle_start_s_ >= cfg_.min_settle_s) {
    free_ = true;
  }
  return free_;
}

}  // namespace force_core
