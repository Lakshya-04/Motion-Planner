#include "force_core/force_stage_fsm.hpp"

#include <cmath>

namespace force_core
{

namespace
{
constexpr uint64_t kNanosecondsPerSecond = 1000000000ULL;

/// now_ns - since_ns, clamped to 0 rather than wrapping. now_ns is always assumed >= since_ns in
/// a purely synthetic-clock test (the only kind this FSM was validated against until a real
/// CompliancePort backed by an independent ROS wrench subscription was driven live for the first
/// time) -- but against a REAL clock, a just-cached status sample's own stamp_ns is routinely a
/// few milliseconds OLDER than "now" (ordinary publish/subscribe latency), which a caller can
/// easily capture as since_ns moments before the FIRST status() sample arrives. Plain unsigned
/// subtraction then UNDERFLOWS to a value near UINT64_MAX -- instantly exceeding any timeout on
/// the very first tick (hit live: CARTESIAN settle aborted on force_stage_timeout in under 100
/// microseconds, with the measured force already exactly at target). Clamping to 0 makes "the
/// sample is technically from before this stage began" read as "no time has elapsed yet", which
/// is the correct interpretation -- it can only ever make a timeout check LESS eager to fire.
uint64_t elapsedNs(uint64_t now_ns, uint64_t since_ns)
{
  return now_ns > since_ns ? now_ns - since_ns : 0;
}

bool isEngagedState(ForceFsmState state)
{
  switch (state) {
    case ForceFsmState::kApproach:
    case ForceFsmState::kApproachContactConfirmed:
    case ForceFsmState::kCartesianSettle:
    case ForceFsmState::kCartesianFollow:
    case ForceFsmState::kCartesianDone:
    case ForceFsmState::kRetractSettle:
    case ForceFsmState::kRetractFollow:
      return true;
    case ForceFsmState::kIdle:
    case ForceFsmState::kAborted:
      return false;
  }
  return false;
}
}  // namespace

ForceFsmStep ForceStageFsm::abort(
  const std::string & failure_reason_code,
  const std::string & message)
{
  state_ = ForceFsmState::kAborted;
  ForceFsmStep step;
  step.action = ForceFsmAction::kAbort;
  step.failure_reason_code = failure_reason_code;
  step.message = message;
  return step;
}

ForceFsmStep ForceStageFsm::checkUniversalAborts(const core_types::ComplianceStatus & status)
{
  if (!status.wrench_valid) {
    return ForceFsmStep{};  // kDwell -- an invalid sample carries no information, see class doc.
  }

  if (isEngagedState(state_)) {
    const double normal_force = core_types::normalForceN(status.wrench);
    if (normal_force > params_.overload_abort_n) {
      return abort("contact_overload", "normal force exceeded overload_abort_n");
    }
    const double tangential_torque = core_types::tangentialTorqueNm(status.wrench);
    if (tangential_torque > params_.tangential_torque_abort_nm) {
      return abort("tangential_overload", "tangential torque exceeded tangential_torque_abort_nm");
    }
  } else if (state_ == ForceFsmState::kIdle && params_.unexpected_contact_abort_n > 0.0) {
    const double normal_force = std::abs(core_types::normalForceN(status.wrench));
    if (normal_force > params_.unexpected_contact_abort_n) {
      return abort(
        "unexpected_contact",
        "non-contact stage saw force above unexpected_contact_abort_n");
    }
  }
  return ForceFsmStep{};
}

ForceFsmStep ForceStageFsm::beginSegment(
  core_types::ForceStage stage, const ForceFsmParams & params, uint64_t now_ns)
{
  if (state_ == ForceFsmState::kAborted) {
    return abort(
      "force_stage_sequence_invalid",
      "beginSegment called after the FSM already aborted -- reset() required");
  }

  params_ = params;

  switch (stage) {
    case core_types::ForceStage::kNone:
      return ForceFsmStep{};  // No force obligation for this segment -- today's exact behavior.

    case core_types::ForceStage::kApproach: {
        if (state_ != ForceFsmState::kIdle) {
          return abort("force_stage_sequence_invalid", "APPROACH requested outside kIdle");
        }
        state_ = ForceFsmState::kApproach;
        settle_entry_ns_ = now_ns;
        convergence_count_ = 0;
        ForceFsmStep step;
        step.action = ForceFsmAction::kEngage;
        step.action_target_force_n = params_.target_force_n;
        return step;
      }

    case core_types::ForceStage::kCartesian: {
        if (state_ != ForceFsmState::kApproachContactConfirmed) {
          return abort(
            "force_stage_sequence_invalid",
            "CARTESIAN requested outside kApproachContactConfirmed");
        }
        state_ = ForceFsmState::kCartesianSettle;
        settle_entry_ns_ = now_ns;
        convergence_count_ = 0;
        return ForceFsmStep{}; // Settle convergence is evaluated by subsequent step() calls.
      }

    case core_types::ForceStage::kRetract: {
        if (state_ != ForceFsmState::kCartesianDone) {
          return abort("force_stage_sequence_invalid", "RETRACT requested outside kCartesianDone");
        }
        state_ = ForceFsmState::kRetractSettle;
        settle_entry_ns_ = now_ns;
        convergence_count_ = 0;
        ForceFsmStep step;
        step.action = ForceFsmAction::kSetTargetForce;
        step.action_target_force_n = 0.0;
        return step;
      }
  }
  return abort("force_stage_sequence_invalid", "unreachable force stage value");
}

ForceFsmStep ForceStageFsm::step(const core_types::ComplianceStatus & status)
{
  const ForceFsmStep universal_abort = checkUniversalAborts(status);
  if (universal_abort.action == ForceFsmAction::kAbort) {
    return universal_abort;
  }

  switch (state_) {
    case ForceFsmState::kIdle:
    case ForceFsmState::kApproach:
    case ForceFsmState::kApproachContactConfirmed:
    case ForceFsmState::kCartesianDone:
    case ForceFsmState::kRetractFollow:
    case ForceFsmState::kAborted:
      return ForceFsmStep{};  // Nothing to evaluate here -- these states wait on
    // onTrajectoryFinished() or the next beginSegment().

    case ForceFsmState::kCartesianFollow: {
        if (!status.wrench_valid) {
          return ForceFsmStep{}; // No information -- neither starts nor clears the hold timer.
        }
        const double error = std::abs(
          core_types::normalForceN(
            status.wrench) - params_.target_force_n);
        if (error <= params_.tracking_lost_error_n) {
          tracking_lost_active_ = false;
          return ForceFsmStep{};
        }
        if (!tracking_lost_active_) {
          tracking_lost_active_ = true;
          tracking_lost_since_ns_ = status.stamp_ns;
          return ForceFsmStep{};
        }
        if (elapsedNs(status.stamp_ns, tracking_lost_since_ns_) >=
          static_cast<uint64_t>(params_.tracking_lost_hold_s * kNanosecondsPerSecond))
        {
          return abort(
            "force_tracking_lost",
            "CARTESIAN follow error exceeded tracking_lost_error_n for tracking_lost_hold_s");
        }
        return ForceFsmStep{};
      }

    case ForceFsmState::kCartesianSettle: {
        if (status.wrench_valid) {
          const double error = std::abs(
            core_types::normalForceN(
              status.wrench) - params_.target_force_n);
          convergence_count_ = (error <= params_.force_band_n) ? convergence_count_ + 1 : 0;
        }
        // An invalid sample neither advances nor resets convergence_count_ -- it carries no
        // information -- but the timeout below is still driven by status.stamp_ns regardless.
        if (convergence_count_ >= params_.convergence_ticks) {
          state_ = ForceFsmState::kCartesianFollow;
          convergence_count_ = 0;
          tracking_lost_active_ = false;
          ForceFsmStep step;
          step.action = ForceFsmAction::kReleaseTrajectory;
          return step;
        }
        if (elapsedNs(status.stamp_ns, settle_entry_ns_) >=
          static_cast<uint64_t>(params_.settle_timeout_s * kNanosecondsPerSecond))
        {
          return abort(
            "force_stage_timeout",
            "CARTESIAN settle did not converge within settle_timeout_s");
        }
        return ForceFsmStep{};
      }

    case ForceFsmState::kRetractSettle: {
        if (status.wrench_valid) {
          const double force_mag = std::abs(core_types::normalForceN(status.wrench));
          convergence_count_ =
            (force_mag < params_.force_zero_threshold_n) ? convergence_count_ + 1 : 0;
        }
        if (convergence_count_ >= params_.convergence_ticks) {
          state_ = ForceFsmState::kRetractFollow;
          convergence_count_ = 0;
          ForceFsmStep step;
          step.action = ForceFsmAction::kReleaseTrajectory;
          return step;
        }
        if (elapsedNs(status.stamp_ns, settle_entry_ns_) >=
          static_cast<uint64_t>(params_.settle_timeout_s * kNanosecondsPerSecond))
        {
          return abort("unload_timeout", "RETRACT unload did not converge within settle_timeout_s");
        }
        return ForceFsmStep{};
      }
  }
  return ForceFsmStep{};
}

ForceFsmStep ForceStageFsm::onTrajectoryFinished(
  bool execution_succeeded, const core_types::ComplianceStatus & status)
{
  const ForceFsmStep universal_abort = checkUniversalAborts(status);
  if (universal_abort.action == ForceFsmAction::kAbort) {
    return universal_abort;
  }

  switch (state_) {
    case ForceFsmState::kApproach: {
        if (execution_succeeded && status.wrench_valid &&
          core_types::normalForceN(status.wrench) >= params_.contact_threshold_n)
        {
          state_ = ForceFsmState::kApproachContactConfirmed;
          return ForceFsmStep{};
        }
        return abort(
          "contact_not_detected",
          "APPROACH finished without reaching contact_threshold_n");
      }

    case ForceFsmState::kCartesianFollow: {
        if (!execution_succeeded) {
          return abort("execution_failed", "CARTESIAN follow trajectory execution failed");
        }
        state_ = ForceFsmState::kCartesianDone;
        return ForceFsmStep{};
      }

    case ForceFsmState::kRetractFollow: {
        if (!execution_succeeded) {
          return abort("execution_failed", "RETRACT peel trajectory execution failed");
        }
        if (!status.wrench_valid) {
          return abort("wrench_invalid", "cannot confirm switch-safe force after peel");
        }
        const double force_mag = std::abs(core_types::normalForceN(status.wrench));
        if (force_mag < params_.switch_safe_force_n) {
          state_ = ForceFsmState::kIdle;
          convergence_count_ = 0;
          ForceFsmStep step;
          step.action = ForceFsmAction::kDisengage;
          return step;
        }
        // Peel finished but force is still above the bumpless-transfer bound -- abort WITHOUT
        // emitting kDisengage, so the sequencer never switches controllers while loaded (P3.2).
        return abort(
          "controller_switch_unsafe",
          "peel finished but |F| still >= switch_safe_force_n");
      }

    default:
      return abort(
        "force_stage_sequence_invalid",
        "onTrajectoryFinished called in an unexpected state");
  }
}

void ForceStageFsm::reset()
{
  state_ = ForceFsmState::kIdle;
  params_ = ForceFsmParams{};
  settle_entry_ns_ = 0;
  convergence_count_ = 0;
  tracking_lost_active_ = false;
  tracking_lost_since_ns_ = 0;
}

}  // namespace force_core
