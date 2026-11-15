#include "force_core/force_stage_sequencer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace force_core
{

namespace
{

constexpr auto kDwellTickDuration = std::chrono::milliseconds(20);  // ~50Hz, matches the
                                                                    // sequencer's documented tick rate.
// Matches this codebase's own established bumpless-transfer convention (the engage and wall-approach tests' own
// "max_abs_force_at_switch_n < 3.0" check) -- see handleAbort()'s own doc comment.
constexpr double kAbortSafeDisengageForceN = 3.0;
// Trim doesn't need the dwell loop's own contact-detection responsiveness -- 20Hz is plenty for
// an integral correction whose whole point is slow, steady-state error removal.
constexpr auto kTrimTickDuration = std::chrono::milliseconds(50);

// 0 on ForceIntent::target_force_n means "use the active ToolProfile's own task.target_force_n"
// (see force_stage.hpp). Note this resolution is applied uniformly, including for RETRACT
// segments whose own ForceIntent::target_force_n is conventionally 0.0 -- harmless, because
// ForceStageFsm's kRetract handling never reads ForceFsmParams::target_force_n at all (it always
// commands exactly 0.0 for the unload, hardcoded in beginSegment()); the resolved value here only
// matters for the kCartesianSettle convergence check.
double resolveTargetForceN(
  const core_types::ForceIntent & intent, const core_types::ToolProfile & tool_profile)
{
  return intent.target_force_n > 0.0 ? intent.target_force_n : tool_profile.task.target_force_n;
}

ForceFsmParams buildForceFsmParams(
  const core_types::ForceIntent & intent, const core_types::ToolProfile & tool_profile)
{
  ForceFsmParams params;
  params.target_force_n = resolveTargetForceN(intent, tool_profile);
  params.force_band_n = tool_profile.task.force_band_n;
  params.force_zero_threshold_n = tool_profile.task.force_zero_threshold_n;
  params.settle_timeout_s = tool_profile.task.settle_timeout_s;
  params.overload_abort_n = tool_profile.task.overload_abort_n;
  params.tangential_torque_abort_nm = tool_profile.task.tangential_torque_abort_nm;
  params.unexpected_contact_abort_n = tool_profile.task.unexpected_contact_abort_n;
  params.contact_threshold_n = tool_profile.control.admittance.contact_threshold_n;
  // convergence_ticks / tracking_lost_* / switch_safe_force_n keep ForceFsmParams's own
  // defaults -- FSM-internal tuning constants not yet promoted to the ToolProfile schema, per
  // force_stage_fsm.hpp's own doc comment.
  return params;
}

core_types::ComplianceConfig buildComplianceConfig(
  const core_types::ForceIntent & intent, const core_types::ToolProfile & tool_profile)
{
  core_types::ComplianceConfig config;
  config.selected_axes = tool_profile.control.admittance.selected_axes;
  config.mass = tool_profile.control.admittance.mass;
  config.damping = tool_profile.control.admittance.damping;
  config.stiffness = tool_profile.control.admittance.stiffness;
  config.target_force_n = resolveTargetForceN(intent, tool_profile);
  config.max_offset_m = tool_profile.control.admittance.max_offset_m;
  config.lpf_cutoff_hz = tool_profile.control.ft.lpf_cutoff_hz;
  config.deadband_n = tool_profile.control.ft.deadband_n;
  config.clamp_n = tool_profile.control.ft.clamp_n;
  return config;
}

}  // namespace

uint64_t ForceStageSequencer::defaultNowNs()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch())
    .count());
}

void ForceStageSequencer::defaultSleep()
{
  std::this_thread::sleep_for(kDwellTickDuration);
}

ForceStageSequencer::ForceStageSequencer(
  core_types::CompliancePort & compliance_port, NowNsFn now_ns_fn, SleepFn sleep_fn)
: compliance_port_(compliance_port), now_ns_fn_(std::move(now_ns_fn)),
  sleep_fn_(std::move(sleep_fn))
{
}

ForceStageSequencer::~ForceStageSequencer()
{
  stopTrimLoopIfRunning();
}

void ForceStageSequencer::trimLoop(
  double target_force_n, double trim_ki, double trim_integral_max_n)
{
  while (trim_thread_running_.load()) {
    const auto status = compliance_port_.status();
    if (status.wrench_valid) {
      const double error_n = target_force_n - core_types::normalForceN(status.wrench);
      double integral = trim_integral_n_.load();
      integral += trim_ki * error_n * std::chrono::duration<double>(kTrimTickDuration).count();
      integral = std::clamp(integral, -trim_integral_max_n, trim_integral_max_n);
      trim_integral_n_.store(integral);
      std::string error_message;
      // A failed setTargetForce() here has no clean way to signal an abort mid-kCartesianFollow
      // (nothing else watches that state's own force trace either, today -- onTrajectoryFinished()
      // is what evaluates state, only once the caller's execute() call returns) -- best-effort,
      // not silently pretending this can't fail.
      compliance_port_.setTargetForce(target_force_n + integral, error_message);
    }
    std::this_thread::sleep_for(kTrimTickDuration);
  }
}

void ForceStageSequencer::startTrimLoopIfEnabled(
  double target_force_n, double trim_ki, double trim_integral_max_n)
{
  if (trim_ki <= 0.0) {
    return;  // Byte-identical behavior to before trim existed -- see this file's own doc comment.
  }
  trim_integral_n_.store(0.0);  // Fresh per CARTESIAN segment -- never carries over.
  trim_thread_running_.store(true);
  trim_thread_ = std::thread(
    &ForceStageSequencer::trimLoop, this, target_force_n, trim_ki, trim_integral_max_n);
}

void ForceStageSequencer::stopTrimLoopIfRunning()
{
  trim_thread_running_.store(false);
  if (trim_thread_.joinable()) {
    trim_thread_.join();
  }
  telemetry_.final_trim_integral_n = trim_integral_n_.load();
}

void ForceStageSequencer::noteStateReached()
{
  // kAborted is excluded so an aborted segment's telemetry still shows how far it legitimately
  // got (e.g. "reached kCartesianSettle") rather than just "aborted" -- see the class doc.
  const auto current = fsm_.state();
  if (current == ForceFsmState::kAborted) {
    return;
  }
  const auto current_value = static_cast<uint8_t>(current);
  if (current_value > telemetry_.force_stage_reached) {
    telemetry_.force_stage_reached = current_value;
  }
}

ForceSequenceOutcome ForceStageSequencer::beforeSegment(
  const core_types::ForceIntent & force_intent,
  const core_types::ToolProfile & tool_profile)
{
  telemetry_ = ForceSegmentTelemetry{};  // Per-segment granularity -- see the struct's own doc.

  if (force_intent.stage == core_types::ForceStage::kNone) {
    return ForceSequenceOutcome{};  // proceed=true -- today's exact behavior, untouched.
  }

  force_active_ = true;
  const ForceFsmParams params = buildForceFsmParams(force_intent, tool_profile);
  const auto begin_step = fsm_.beginSegment(force_intent.stage, params, now_ns_fn_());
  noteStateReached();

  if (begin_step.action == ForceFsmAction::kAbort) {
    // Deliberately NOT handleAbort() here -- an illegal-sequence rejection (e.g. kRetract from
    // kIdle) happens BEFORE this call ever touches the port; there is nothing to clean up, and
    // touching the port anyway would be a real behavior change (this exact "rejected before
    // touching the port" guarantee is asserted by IllegalSequenceIsRejectedBeforeTouchingThe
    // CompliancePort).
    force_active_ = false;
    telemetry_.compliance_engaged_at_exit = force_active_;
    return ForceSequenceOutcome{false, begin_step.failure_reason_code, begin_step.message};
  }

  if (begin_step.action == ForceFsmAction::kEngage) {
    std::string error;
    const auto config = buildComplianceConfig(force_intent, tool_profile);
    if (!compliance_port_.engage(config, error)) {
      // Deliberately NOT handleAbort() here -- engage() itself is what would have turned
      // compliance on, and it FAILED, so there is nothing engaged to clean up (asserted by
      // EngageFailureAbortsWithoutEnteringADwellLoop's own zero-status()-calls expectation).
      force_active_ = false;
      telemetry_.compliance_engaged_at_exit = force_active_;
      return ForceSequenceOutcome{false, "controller_switch_failed", "engage() failed: " + error};
    }
    ++telemetry_.compliance_switch_count;
    telemetry_.compliance_engaged_at_exit = force_active_;
    // APPROACH releases its trajectory immediately -- a straight move to the surface, with no
    // settle dwell of its own. Contact is checked later, in afterSegment(), once that move
    // finishes; see force_stage_fsm.hpp's own class doc for why commanding straight to the
    // surface is safe once compliance is already engaged.
    return ForceSequenceOutcome{};
  }

  if (begin_step.action == ForceFsmAction::kSetTargetForce) {
    std::string error;
    if (!compliance_port_.setTargetForce(begin_step.action_target_force_n, error)) {
      const bool disengaged = handleAbort();
      force_active_ = false;
      telemetry_.compliance_engaged_at_exit = !disengaged;
      return ForceSequenceOutcome{
        false, "controller_switch_failed", "setTargetForce() failed: " + error};
    }
  }

  // kSetTargetForce (RETRACT, unloading) and kDwell (CARTESIAN, settling onto target_force_n)
  // both need to converge before the caller's trajectory may be released.
  const auto outcome = dwellUntilReleaseOrAbort();
  telemetry_.compliance_engaged_at_exit = force_active_;

  // Trim only ever applies to CARTESIAN's own "trajectory released, holding force" window -- see
  // this file's own "FORCE TRIM" doc comment. A no-op if trim_ki <= 0.0.
  if (outcome.proceed && force_intent.stage == core_types::ForceStage::kCartesian) {
    startTrimLoopIfEnabled(
      params.target_force_n, tool_profile.control.admittance.trim_ki,
      tool_profile.control.admittance.trim_integral_max_n);
  }
  return outcome;
}

ForceSequenceOutcome ForceStageSequencer::dwellUntilReleaseOrAbort()
{
  const uint64_t dwell_start_ns = now_ns_fn_();
  while (true) {
    const auto status = compliance_port_.status();
    if (!status.wrench_valid) {
      ++telemetry_.invalid_wrench_ticks;
    }
    const auto step = fsm_.step(status);
    noteStateReached();
    if (step.action == ForceFsmAction::kReleaseTrajectory) {
      telemetry_.settle_time_ms = (now_ns_fn_() - dwell_start_ns) / 1000000ULL;
      return ForceSequenceOutcome{};
    }
    if (step.action == ForceFsmAction::kAbort) {
      const bool disengaged = handleAbort();
      force_active_ = false;
      telemetry_.compliance_engaged_at_exit = !disengaged;
      telemetry_.settle_time_ms = (now_ns_fn_() - dwell_start_ns) / 1000000ULL;
      return ForceSequenceOutcome{false, step.failure_reason_code, step.message};
    }
    sleep_fn_();
  }
}

bool ForceStageSequencer::handleAbort()
{
  // Always safe, unconditionally -- removes the ongoing push regardless of current load. See
  // this method's own header doc comment.
  std::string unload_error;
  compliance_port_.setTargetForce(0.0, unload_error);

  const auto status = compliance_port_.status();
  const bool safe_to_disengage =
    status.wrench_valid && std::abs(core_types::normalForceN(status.wrench)) <
    kAbortSafeDisengageForceN;
  if (safe_to_disengage) {
    std::string disengage_error;
    if (compliance_port_.disengage(disengage_error)) {
      ++telemetry_.compliance_switch_count;
      return true;  // Fully disengaged -- caller should report compliance_engaged_at_exit=false.
    }
    // A failed disengage() here is best-effort -- the caller already has an abort outcome to act
    // on; there is no additional outcome slot to report a SECOND failure through. The target
    // force is already zeroed regardless, which is the load-bearing part of this fix.
  }
  return false;  // Left selected but zero-biased (still too loaded to safely switch), or the
                 // disengage() call itself failed -- either way, compliance_engaged_at_exit=true.
}

ForceSequenceOutcome ForceStageSequencer::afterSegment(bool execution_succeeded)
{
  if (!force_active_) {
    telemetry_ = ForceSegmentTelemetry{};  // ForceStage::kNone segment -- nothing was engaged.
    return ForceSequenceOutcome{};
  }

  // Stop trim BEFORE reading status() for the final decision below -- otherwise the trim thread's
  // own concurrent setTargetForce() calls could race this method's own read, and the final
  // decision must see a settled, no-longer-changing setpoint. A no-op if trim was never started
  // for this segment (e.g. RETRACT, or trim_ki == 0.0).
  stopTrimLoopIfRunning();

  const auto status = compliance_port_.status();
  if (!status.wrench_valid) {
    ++telemetry_.invalid_wrench_ticks;
  }
  const auto step = fsm_.onTrajectoryFinished(execution_succeeded, status);
  noteStateReached();

  if (step.action == ForceFsmAction::kAbort) {
    const bool disengaged = handleAbort();
    force_active_ = false;
    telemetry_.compliance_engaged_at_exit = !disengaged;
    return ForceSequenceOutcome{false, step.failure_reason_code, step.message};
  }

  if (step.action == ForceFsmAction::kDisengage) {
    std::string error;
    if (!compliance_port_.disengage(error)) {
      // disengage() FAILED -- compliance is presumably STILL engaged on the real backend, not
      // disengaged. Found live (2026-08-21) that this branch previously hardcoded false here
      // (via force_active_, an internal sequencer flag unrelated to the PORT's own real state),
      // silently misreporting a failed disengage as a successful one. force_active_ itself still
      // correctly reflects "this segment group is done" from the sequencer's own perspective --
      // only the BACKEND-facing telemetry field needed the fix.
      force_active_ = false;
      telemetry_.compliance_engaged_at_exit = true;
      return ForceSequenceOutcome{
        false, "controller_switch_failed", "disengage() failed: " + error};
    }
    ++telemetry_.compliance_switch_count;
    // The P3.2 acceptance metric: force sampled at the moment of this disengage call, from the
    // same status() sample onTrajectoryFinished() already used to decide it was safe to switch.
    telemetry_.max_abs_force_at_switch_n = std::abs(core_types::normalForceN(status.wrench));
    force_active_ = false;  // The whole APPROACH->CARTESIAN->RETRACT group is complete.
  }

  telemetry_.compliance_engaged_at_exit = force_active_;
  return ForceSequenceOutcome{};
}

void ForceStageSequencer::reset()
{
  stopTrimLoopIfRunning();
  fsm_.reset();
  force_active_ = false;
  telemetry_ = ForceSegmentTelemetry{};
}

}  // namespace force_core
