/**
 * @file force_stage_sequencer.hpp
 * @brief Drives ForceStageFsm against a real core_types::CompliancePort.
 * @details The non-portable half of the force-stage design (see force_stage_fsm.hpp's own
 *          portability constraint) -- this file may freely include manager headers, use a clock,
 *          and own the dwell loops the FSM itself is deliberately kept ignorant of.
 *
 *          Wired as explicit beforeSegment()/afterSegment() hooks around the caller's own
 *          ExecutionPort::execute() call, NOT as an ExecutionPort decorator: a decorator fails
 *          open if a future injection site forgets to wrap it, an explicit call fails closed
 *          (the caller cannot accidentally skip force sequencing for a force-staged segment
 *          without the compile-time visibility of the call site itself).
 *
 *          Scope note: tare() is deliberately NOT called here. Per ToolFtHygiene::tare_per_goal,
 *          tare is a mission-level concern (once per CleanWall-equivalent goal, not once per
 *          contact segment group) -- calling it here would tare on every APPROACH, which is both
 *          wrong per-profile semantics and, per this codebase's own force-control landmine list,
 *          actively destructive under some backends (overwrites the payload/load-offset
 *          calibration). The caller above this class owns when tare() happens.
 *
 *          FORCE TRIM (ToolAdmittance::trim_ki/trim_integral_max_n): admittance alone is a
 *          stiffness-based (proportional) regulator -- achieved_force = K_z * penetration -- which
 *          has zero steady-state error only if the surface stays at exactly the depth the
 *          admittance offset already found. Tracing a real, imperfect wall while CARTESIAN holds
 *          the trajectory released (kCartesianFollow) changes that depth continuously, and nothing
 *          watches or corrects for it: beforeSegment(kCartesian) only settles once, BEFORE
 *          releasing the trajectory, and afterSegment() only checks again AFTER the caller's own
 *          execute() call returns -- see force_control_backend_admittance_vs_diffik.md's own
 *          "Control law: direct force (PI) vs. admittance" section for why literature treats this
 *          as the admittance law's one structural blind spot, and why the fix is a small outer
 *          integral trim (this codebase's own trim_ki/trim_integral_max_n fields), not a
 *          replacement of admittance itself.
 *
 *          Implemented as a background thread (see trimLoop()), spawned ONLY when
 *          tool_profile.control.admittance.trim_ki > 0.0, for the exact duration the trajectory is
 *          released in the CARTESIAN stage (from beforeSegment(kCartesian)'s own successful
 *          return until afterSegment() is next called) -- the ONLY window where the caller's own
 *          ExecutionPort::execute() runs concurrently with this class's own code, which is why a
 *          real thread (not another dwell-loop tick) is required here specifically. A thread is
 *          the correct choice ONLY because it stays entirely inside the CompliancePort's own
 *          documented, already-thread-safe seam (status()/setTargetForce(), the same two calls
 *          the dwell loop above already makes) -- no backend-specific code, no new port method:
 *          this keeps the trim policy backend-agnostic, matching CompliancePort's own "carries NO
 *          policy" contract, rather than duplicating it inside each CompliancePort implementation.
 *          trim_ki == 0.0 (every ToolProfile not opting in, including every existing test's own
 *          profile) means no thread is ever spawned -- byte-identical behavior to before this
 *          existed.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

#include "core_types/compliance_port.hpp"
#include "core_types/force_stage.hpp"
#include "core_types/tool_profile.hpp"
#include "force_core/force_stage_fsm.hpp"

namespace force_core
{

/**
 * @struct ForceSegmentTelemetry
 * @brief Per-segment force telemetry, reset at the start of every beforeSegment() call.
 * @details Deliberately does NOT include mean_normal_force_n / max_abs_normal_force_n /
 *          force_in_band_ratio (see MotionSegmentExecMetrics's own doc comment on those three):
 *          this sequencer has no visibility into the force trace DURING the caller's own
 *          blocking ExecutionPort::execute() call, only immediately before and after it, so it
 *          cannot honestly compute a mean/in-band ratio over the trajectory itself.
 */
struct ForceSegmentTelemetry
{
  /// Furthest ForceFsmState reached while handling this segment, as a raw ForceFsmState value --
  /// excludes kAborted itself, so an aborted segment still reports how far it legitimately got
  /// (e.g. "reached kCartesianSettle" rather than just "aborted").
  uint8_t force_stage_reached{0};
  uint64_t settle_time_ms{0};             ///< Wall time spent in the dwell loop, if any.
  uint32_t compliance_switch_count{0};    ///< engage()/disengage() calls made for this segment.
  double max_abs_force_at_switch_n{0.0};  ///< |F| sampled at disengage -- the P3.2 acceptance metric.
  uint32_t invalid_wrench_ticks{0};       ///< wrench_valid==false samples seen this segment.
  bool compliance_engaged_at_exit{false}; ///< True if compliance is still engaged once this
                                          ///< segment's hook returns -- expected true after
                                          ///< APPROACH/CARTESIAN, false after RETRACT/kNone.
  /// The trim integral's accumulated value (newtons, added on top of target_force_n) at the
  /// moment the trim thread was stopped -- 0.0 if trim_ki was 0.0 (trim never ran) or this wasn't
  /// a CARTESIAN segment. See this file's own "FORCE TRIM" doc comment.
  double final_trim_integral_n{0.0};
};

/**
 * @struct ForceSequenceOutcome
 * @brief One result of driving a segment's force obligations, from either hook.
 */
struct ForceSequenceOutcome
{
  /// True: the caller may proceed (call ExecutionPort::execute() from beforeSegment(), or treat
  /// the segment as fully handled from afterSegment()). False: a force-layer abort occurred --
  /// the caller must NOT call execute() (from beforeSegment()) and must record the failure using
  /// failure_reason_code/message instead.
  bool proceed{true};
  std::string failure_reason_code{"none"};
  std::string message;
};

/**
 * @class ForceStageSequencer
 * @brief Sequences one force-staged segment group's compliance engagement around the caller's
 *        own ExecutionPort::execute() calls.
 * @details Owns exactly one ForceStageFsm and drives it against a real CompliancePort, including
 *          the settle/unload dwell loops the FSM's own step() leaves to its caller. ToolProfile
 *          fields are translated into ForceFsmParams/ComplianceConfig here -- neither the FSM nor
 *          any CompliancePort implementation ever reads a ToolProfile directly, so a backend swap
 *          can never re-interpret a profile field differently.
 */
class ForceStageSequencer
{
public:
  using NowNsFn = std::function<uint64_t()>;
  using SleepFn = std::function<void ()>;

  /**
   * @param compliance_port Backend to drive. Not owned; must outlive this sequencer.
   * @param now_ns_fn Monotonic clock, injectable for tests. Defaults to std::chrono::steady_clock.
   * @param sleep_fn Dwell-loop tick delay, injectable for tests (default: a short real sleep).
   */
  explicit ForceStageSequencer(
    core_types::CompliancePort & compliance_port,
    NowNsFn now_ns_fn = defaultNowNs,
    SleepFn sleep_fn = defaultSleep);

  /// Stops and joins the trim thread if one is running -- see this file's own "FORCE TRIM" doc
  /// comment. A no-op (trim_thread_ never joinable) for every ToolProfile with trim_ki == 0.0.
  ~ForceStageSequencer();

  /**
   * @brief Drive this segment's force obligations up to the point where the caller's own
   *        ExecutionPort::execute() should run.
   * @details ForceStage::kNone is a pure no-op (returns {proceed=true} immediately, touching
   *          neither the FSM nor the CompliancePort) -- today's exact behavior for non-force
   *          segments. On {proceed=false}, the caller must NOT call execute() for this segment
   *          and must NOT call afterSegment() either.
   */
  ForceSequenceOutcome beforeSegment(
    const core_types::ForceIntent & force_intent,
    const core_types::ToolProfile & tool_profile);

  /**
   * @brief Feed back the result of the execute() call beforeSegment() authorized.
   * @details Only call this after a beforeSegment() that returned {proceed=true}. A no-op
   *          returning {proceed=true} if the segment carried ForceStage::kNone.
   */
  ForceSequenceOutcome afterSegment(bool execution_succeeded);

  ForceFsmState state() const {return fsm_.state();}

  /// Telemetry for the most recently handled segment -- see ForceSegmentTelemetry's own doc.
  /// Reset at the start of every beforeSegment() call (including a kNone one, to all-zero).
  const ForceSegmentTelemetry & telemetry() const {return telemetry_;}

  /// Mission-level reset -- see ForceStageFsm::reset(). Does not call CompliancePort::disengage();
  /// the caller must ensure the backend is already safely disengaged before calling this.
  void reset();

private:
  ForceSequenceOutcome dwellUntilReleaseOrAbort();
  void noteStateReached();

  /// Called on EVERY abort path (dwellUntilReleaseOrAbort()'s own settle-dwell abort AND
  /// afterSegment()'s own onTrajectoryFinished() abort) -- found live (2026-08-21) that NEITHER
  /// previously did anything to the real backend on abort beyond setting force_active_ = false
  /// and returning a failure outcome, leaving compliance ENGAGED on the REAL CONTROLLER
  /// indefinitely (with whatever target_force_n was last set) -- confirmed live to cause a
  /// robot to drift away from a subsequently-commanded rigid HOME position, entirely without any
  /// caller command, for as long as nothing ever disengaged it (in one case, hours later, across
  /// multiple SEPARATE process invocations). telemetry_.compliance_engaged_at_exit was ALSO
  /// wrongly reporting false (disengaged) in this state -- a doubly dangerous silent lie.
  ///
  /// Fix: unconditionally zero the target force bias first (always safe -- removes the ongoing
  /// PUSH regardless of current load, letting admittance settle toward its own zero-force
  /// equilibrium rather than continuing to chase a stale nonzero target). Then, ONLY if the
  /// CURRENT measured force is already below kAbortSafeDisengageForceN (matching this codebase's
  /// own established bumpless-transfer threshold, see MotionSegmentExecMetrics/T8-T9's own
  /// "< 3.0" convention), fully disengage() for a clean rigid switch. If force is still high,
  /// disengaging immediately would itself be the unsafe "snap back at full stiffness" hazard
  /// RETRACT's own kRetractFollow path is already designed to avoid -- in that case, this
  /// deliberately leaves the axis selected (still nominally "engaged") but zero-biased (a safe,
  /// static hold, not a growing drift) and reports this honestly via compliance_engaged_at_exit.
  /// @return true if fully disengaged (caller should report compliance_engaged_at_exit=false);
  ///         false if left engaged-but-zero-biased or the disengage() call itself failed
  ///         (caller should report compliance_engaged_at_exit=true).
  bool handleAbort();

  /// Runs on trim_thread_ until trim_thread_running_ is cleared -- see this file's own "FORCE
  /// TRIM" doc comment. target_force_n/trim_ki/trim_integral_max_n are captured by value at
  /// thread-start time (ToolProfile-derived, fixed for the segment's own duration).
  void trimLoop(double target_force_n, double trim_ki, double trim_integral_max_n);
  /// Starts trimLoop() on trim_thread_ if trim_ki > 0.0; a no-op otherwise (byte-identical
  /// behavior to before this existed for every ToolProfile not opting in).
  void startTrimLoopIfEnabled(double target_force_n, double trim_ki, double trim_integral_max_n);
  /// Signals trimLoop() to stop and joins it -- safe to call whether or not a thread is running.
  /// Records telemetry_.final_trim_integral_n before clearing the integral state.
  void stopTrimLoopIfRunning();

  static uint64_t defaultNowNs();
  static void defaultSleep();

  core_types::CompliancePort & compliance_port_;
  ForceStageFsm fsm_;
  NowNsFn now_ns_fn_;
  SleepFn sleep_fn_;
  bool force_active_{false};  ///< True from a force-staged beforeSegment() until the group ends.
  ForceSegmentTelemetry telemetry_;

  // -- Force trim (see this file's own "FORCE TRIM" doc comment) --
  std::thread trim_thread_;
  std::atomic<bool> trim_thread_running_{false};
  std::atomic<double> trim_integral_n_{0.0};
};

}  // namespace force_core
