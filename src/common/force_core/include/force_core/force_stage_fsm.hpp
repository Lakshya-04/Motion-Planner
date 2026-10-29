/**
 * @file force_stage_fsm.hpp
 * @brief Pure sequencing layer for one force-staged segment group (APPROACH/CARTESIAN/RETRACT).
 * @details Backend-agnostic by construction: it only ever reads core_types::ComplianceStatus
 *          samples and returns a requested ForceFsmAction -- it never calls a CompliancePort or
 *          ExecutionPort itself. That split is what makes every abort path here unit-testable
 *          against fabricated force traces with synthetic timestamps, with zero ROS and zero
 *          hardware, per force_control_derisking_plan.md's Gate 0 (R12).
 *
 *          PORTABILITY CONSTRAINT (see implementation_plan_manipulation_architecture.md's
 *          decision to split by directory, not by package, until a genuine second consumer
 *          exists): this file and force_stage_fsm.cpp may include only cstdint, string, vector,
 *          array, plus core_types headers. No force_core-external header, no clock, no
 *          thread -- so the eventual split into a standalone force_core package
 *          (triggered when a 100Hz ros2_control-side trim loop needs this FSM, per the
 *          force-control implementation plan) is a plain `git mv`, not a rewrite.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cstdint>
#include <string>

#include "core_types/compliance_port.hpp"
#include "core_types/force_stage.hpp"

namespace force_core
{

/**
 * @enum ForceFsmState
 * @brief Runtime sub-state within one force-staged segment group.
 * @details The wire-level core_types::ForceStage carries per-segment INTENT (four values);
 *          this carries runtime progress through that intent (nine values), including the
 *          "confirmed, waiting for the next segment to begin" states between them.
 */
enum class ForceFsmState : uint8_t
{
  kIdle = 0,                   ///< No force obligation active.
  kApproach,                   ///< Approach trajectory released, watching for contact.
  kApproachContactConfirmed,   ///< Contact detected; waiting for beginSegment(kCartesian).
  kCartesianSettle,            ///< Engaged, dwelling until force converges to target_force_n.
  kCartesianFollow,            ///< Converged; the strip trajectory has been released.
  kCartesianDone,              ///< Strip finished; waiting for beginSegment(kRetract).
  kRetractSettle,              ///< Dwelling until force converges to ~zero (unloaded).
  kRetractFollow,              ///< Unloaded; the peel-off trajectory has been released.
  kAborted,                    ///< Terminal. See ForceFsmStep::failure_reason_code for why.
};

/**
 * @enum ForceFsmAction
 * @brief What the FSM wants the caller (the force-stage sequencer) to do next.
 * @details Exactly one is returned per call. The FSM never performs the action itself -- it has
 *          no CompliancePort or ExecutionPort reference at all.
 */
enum class ForceFsmAction : uint8_t
{
  kDwell = 0,         ///< Keep polling; do NOT release a trajectory or change compliance yet.
  kEngage,            ///< Call CompliancePort::engage() with the tool's ComplianceConfig.
  kSetTargetForce,    ///< Call CompliancePort::setTargetForce(action_target_force_n).
  kReleaseTrajectory, ///< Preconditions met -- the caller may now execute this segment's trajectory.
  kDisengage,         ///< Call CompliancePort::disengage(); only ever legal below the unload threshold.
  kAbort,             ///< Terminal for this segment group; failure_reason_code is set.
};

/**
 * @struct ForceFsmParams
 * @brief Per-segment-group thresholds, extracted verbatim from the active ToolProfile.
 * @details Field-for-field sourced from core_types::ToolTask / ToolAdmittance -- see each
 *          comment for exactly which profile field drives it. Values with no direct profile
 *          source (convergence_ticks, tracking_lost_*, switch_safe_force_n) are FSM-internal
 *          tuning constants, not yet promoted to the profile schema.
 */
struct ForceFsmParams
{
  double target_force_n{0.0};              ///< ToolTask::target_force_n.
  double force_band_n{0.0};                ///< ToolTask::force_band_n.
  double force_zero_threshold_n{0.0};      ///< ToolTask::force_zero_threshold_n.
  double settle_timeout_s{0.0};            ///< ToolTask::settle_timeout_s.
  double overload_abort_n{0.0};            ///< ToolTask::overload_abort_n.
  double tangential_torque_abort_nm{0.0};  ///< ToolTask::tangential_torque_abort_nm.
  double unexpected_contact_abort_n{0.0};  ///< ToolTask::unexpected_contact_abort_n.
  double contact_threshold_n{0.0};         ///< ToolAdmittance::contact_threshold_n.
  int convergence_ticks{5};                ///< Debounce ticks required before settle is accepted.
  double tracking_lost_error_n{5.0};       ///< |error| beyond this for tracking_lost_hold_s aborts.
  double tracking_lost_hold_s{1.0};
  double switch_safe_force_n{3.0};         ///< Never disengage above this -- the P3.2 bumpless-transfer bound.
};

/**
 * @struct ForceFsmStep
 * @brief One decision returned by the FSM, in response to one event.
 */
struct ForceFsmStep
{
  ForceFsmAction action{ForceFsmAction::kDwell};
  double action_target_force_n{0.0};       ///< Meaningful only when action == kSetTargetForce.
  std::string failure_reason_code{"none"}; ///< Set (non-"none") only when action == kAbort.
  std::string message;                     ///< Human-readable detail, for logging only.
};

/**
 * @class ForceStageFsm
 * @brief Sequences one force-staged segment group. No clock, no I/O, no thread.
 * @details Every timeout and debounce is derived from the stamp_ns carried on each
 *          core_types::ComplianceStatus sample the caller supplies -- the FSM itself never
 *          reads a clock. That is deliberate: it is what lets a unit test drive the FSM through
 *          hours of simulated time with fabricated, instantly-returning samples.
 */
class ForceStageFsm
{
public:
  ForceStageFsm() = default;

  /**
   * @brief Begin one segment's force obligations.
   * @details Validates the requested transition against the FSM's current state (see the class
   *          doc's state table); an illegal request (e.g. kRetract from kIdle) aborts rather than
   *          silently proceeding -- fail closed, per this codebase's force-control safety
   *          convention. core_types::ForceStage::kNone is always legal and is a no-op
   *          (returns kDwell), matching today's exact behavior for non-force segments.
   */
  ForceFsmStep beginSegment(
    core_types::ForceStage stage, const ForceFsmParams & params, uint64_t now_ns);

  /**
   * @brief One tick, driven by a fresh compliance sample.
   * @details A sample with wrench_valid == false never advances a convergence counter AND never
   *          resets one -- it carries no information, so it changes nothing except that the
   *          settle_timeout_s watchdog (timed from stamp_ns, not from sample validity) keeps
   *          running regardless.
   */
  ForceFsmStep step(const core_types::ComplianceStatus & status);

  /**
   * @brief Feed back the result of the trajectory the FSM most recently released.
   * @details Drives kApproach -> contact-confirmed-or-abort, kCartesianFollow -> kCartesianDone,
   *          kRetractFollow -> kDisengage-or-abort (the bumpless-transfer safety check).
   */
  ForceFsmStep onTrajectoryFinished(
    bool execution_succeeded, const core_types::ComplianceStatus & status);

  ForceFsmState state() const {return state_;}

  /// Mission-level reset: clears kAborted and all debounce/timing state. Does not call
  /// CompliancePort::disengage() -- that remains the sequencer's responsibility.
  void reset();

private:
  ForceFsmStep checkUniversalAborts(const core_types::ComplianceStatus & status);
  ForceFsmStep abort(const std::string & failure_reason_code, const std::string & message);

  ForceFsmState state_{ForceFsmState::kIdle};
  ForceFsmParams params_;
  uint64_t settle_entry_ns_{0};
  int convergence_count_{0};
  bool tracking_lost_active_{false};  ///< True while kCartesianFollow's error has been
                                      ///< out-of-band continuously since tracking_lost_since_ns_.
  uint64_t tracking_lost_since_ns_{0};
};

}  // namespace force_core
