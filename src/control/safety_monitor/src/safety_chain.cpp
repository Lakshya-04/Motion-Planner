/**
 * @file safety_chain.cpp
 * @brief SafetyChain implementation, see safety_chain.hpp.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#include "safety_monitor/safety_chain.hpp"

#include <algorithm>
#include <cmath>

namespace safety_monitor
{

namespace
{
constexpr double kFutureEps_s = 1e-3;
constexpr std::size_t N = static_cast<std::size_t>(ChainCause::kCount);

bool staleOrBad(bool seen, double stamp, double now, double timeout)
{
  if (!seen || !std::isfinite(stamp) || !std::isfinite(now)) {return true;}
  if (stamp > now + kFutureEps_s) {return true;}
  return (now - stamp) > timeout;
}
}  // namespace

const char * toString(ChainState s)
{
  switch (s) {
    case ChainState::kOk: return "OK";
    case ChainState::kHold: return "HOLD";
    case ChainState::kEstop: return "ESTOP";
  }
  return "?";
}

const char * toString(ChainCause c)
{
  switch (c) {
    case ChainCause::kForceOverload: return "force_overload";
    case ChainCause::kTorqueOverload: return "torque_overload";
    case ChainCause::kStaleFt: return "stale_ft";
    case ChainCause::kStaleJointState: return "stale_joint_state";
    case ChainCause::kBaseObstacleHold: return "base_obstacle_hold";
    case ChainCause::kBaseObstacleAbort: return "base_obstacle_abort";
    case ChainCause::kOperatorEstop: return "operator_estop";
    case ChainCause::kControllerFault: return "controller_fault";
    default: return "none";
  }
}

SafetyChain::SafetyChain(const ChainParams & p)
: p_(p) {}

void SafetyChain::updateFt(double force_n, double torque_nm, double stamp_s)
{
  if (!std::isfinite(stamp_s)) {ft_invalid_ = true; return;}   // unstamped sample: not fresh
  ft_seen_ = true;
  ft_invalid_ = false;
  ft_stamp_s_ = stamp_s;
  last_force_n_ = force_n;
  last_torque_nm_ = torque_nm;
  // NaN or inf magnitude counts as an overload (fail-safe).
  const bool fbad = !std::isfinite(force_n), tbad = !std::isfinite(torque_nm);
  if (p_.force_abort_n > 0.0 && (fbad || std::fabs(force_n) > p_.force_abort_n)) {force_pending_ = true;}
  if (p_.torque_abort_nm > 0.0 && (tbad || std::fabs(torque_nm) > p_.torque_abort_nm)) {torque_pending_ = true;}
}

void SafetyChain::updateJointState(double stamp_s)
{
  if (!std::isfinite(stamp_s)) {return;}
  joint_seen_ = true;
  joint_stamp_s_ = stamp_s;
}

void SafetyChain::setBaseObstacle(BaseObstacle o) {base_ = o;}

void SafetyChain::setOperatorEstop(bool a)
{
  if (a) {estop_pending_ = true;}
  estop_level_ = a;
}

void SafetyChain::setControllerFault(bool f)
{
  if (f) {fault_pending_ = true;}
  fault_level_ = f;
}

SafetyChain::Flags SafetyChain::activeLevels(double now) const
{
  Flags a{};
  auto idx = [](ChainCause c) {return static_cast<std::size_t>(c);};
  // Level view of the overloads: the latest sample is still over the limit.
  a[idx(ChainCause::kForceOverload)] = force_pending_ ||
    (p_.force_abort_n > 0.0 && ft_seen_ &&
    (!std::isfinite(last_force_n_) || std::fabs(last_force_n_) > p_.force_abort_n));
  a[idx(ChainCause::kTorqueOverload)] = torque_pending_ ||
    (p_.torque_abort_nm > 0.0 && ft_seen_ &&
    (!std::isfinite(last_torque_nm_) || std::fabs(last_torque_nm_) > p_.torque_abort_nm));
  a[idx(ChainCause::kStaleFt)] = p_.require_ft &&
    (ft_invalid_ || staleOrBad(ft_seen_, ft_stamp_s_, now, p_.ft_stale_s));
  a[idx(ChainCause::kStaleJointState)] = p_.require_joint_state &&
    staleOrBad(joint_seen_, joint_stamp_s_, now, p_.joint_stale_s);
  a[idx(ChainCause::kBaseObstacleHold)] = base_ == BaseObstacle::kHold;
  a[idx(ChainCause::kBaseObstacleAbort)] = base_ == BaseObstacle::kAbort;
  a[idx(ChainCause::kOperatorEstop)] = estop_level_ || estop_pending_;
  a[idx(ChainCause::kControllerFault)] = fault_level_ || fault_pending_;
  return a;
}

void SafetyChain::log(double t, ChainEventKind k, ChainCause c, ChainState from, ChainState to,
  const std::string & d)
{
  events_.push_back({t, k, c, from, to, d});
  while (events_.size() > std::max<std::size_t>(p_.max_events, 1)) {
    events_.pop_front();
    ++dropped_;
  }
}

ChainOutputs SafetyChain::outputs() const
{
  ChainOutputs o;
  o.state = state_;
  o.arm_stop = o.base_stop = state_ != ChainState::kOk;
  o.tool_off = o.power_removal_request = state_ == ChainState::kEstop;
  return o;
}

ChainOutputs SafetyChain::step(double now)
{
  const Flags a = activeLevels(now);
  ChainSeverity worst = ChainSeverity::kNone;
  ChainCause worst_cause = ChainCause::kCount;
  for (std::size_t i = 0; i < N; ++i) {
    const ChainCause c = static_cast<ChainCause>(i);
    const ChainSeverity sev = p_.severity[i];
    if (!a[i]) {continue;}
    if (!prev_active_[i]) {
      log(now, ChainEventKind::kCauseAsserted, c, state_, state_,
        sev == ChainSeverity::kNone ? "ignored (severity none)" : "asserted");
    }
    if (sev > worst) {worst = sev; worst_cause = c;}
  }
  // Pending one-shots are consumed; the level view above keeps them active for this step only.
  const bool any_active = std::any_of(a.begin(), a.end(), [](bool b) {return b;});
  force_pending_ = torque_pending_ = estop_pending_ = fault_pending_ = false;
  prev_active_ = activeLevels(now);

  const ChainState target = worst == ChainSeverity::kEstop ? ChainState::kEstop :
    worst == ChainSeverity::kHold ? ChainState::kHold : ChainState::kOk;
  if (target > state_) {
    log(now, ChainEventKind::kStateChange, worst_cause, state_, target,
      std::string("caused by ") + toString(worst_cause));
    if (state_ == ChainState::kOk) {latched_cause_ = worst_cause;}
    state_ = target;
    clear_since_s_ = -1.0;
  } else if (state_ == ChainState::kHold && p_.hold_auto_clear) {
    if (any_active) {
      clear_since_s_ = -1.0;
    } else {
      if (clear_since_s_ < 0.0) {clear_since_s_ = now;}
      if (now - clear_since_s_ >= p_.hold_clear_time_s) {
        log(now, ChainEventKind::kStateChange, ChainCause::kCount, state_, ChainState::kOk,
          "hold auto-cleared");
        state_ = ChainState::kOk;
        latched_cause_ = ChainCause::kCount;
        clear_since_s_ = -1.0;
      }
    }
  }
  return outputs();
}

bool SafetyChain::reset(double now)
{
  if (state_ == ChainState::kOk) {return false;}
  const Flags a = activeLevels(now);
  for (std::size_t i = 0; i < N; ++i) {
    if (a[i] && p_.severity[i] != ChainSeverity::kNone) {
      log(now, ChainEventKind::kResetRefused, static_cast<ChainCause>(i), state_, state_,
        std::string("still active: ") + toString(static_cast<ChainCause>(i)));
      return false;
    }
  }
  log(now, ChainEventKind::kResetAccepted, ChainCause::kCount, state_, ChainState::kOk, "operator reset");
  state_ = ChainState::kOk;
  latched_cause_ = ChainCause::kCount;
  prev_active_ = Flags{};
  clear_since_s_ = -1.0;
  return true;
}

}  // namespace safety_monitor
