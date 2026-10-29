/**
 * @file force_stage_sequencer_test.cpp
 * @brief Unit tests for ForceStageSequencer against a ScriptedCompliancePort test double.
 * @details The FSM's own decision logic (convergence debouncing, timeouts, the P3.2
 *          bumpless-transfer refusal, etc.) is already exhaustively covered in
 *          force_stage_fsm_test.cpp against ForceStageFsm directly. This file's job is narrower
 *          and different: verify the SEQUENCER correctly wires those decisions to real
 *          CompliancePort calls -- exact call order, correct ComplianceConfig/target-force
 *          translation from ToolProfile, and that a refused-unsafe-disengage never actually
 *          calls disengage() on the port.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <algorithm>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "force_core/force_stage_sequencer.hpp"

namespace force_core
{
namespace
{

// Records every port call (except status(), whose volume varies with dwell-loop timing and is
// asserted separately via call COUNT where it matters) in order, and lets a test script the
// wrench status() returns and whether each mutating call succeeds.
class ScriptedCompliancePort : public core_types::CompliancePort
{
public:
  bool tare(std::string & /*error_message*/) override
  {
    calls.push_back("tare");
    return tare_result;
  }

  bool engage(
    const core_types::ComplianceConfig & config,
    std::string & /*error_message*/) override
  {
    calls.push_back("engage");
    last_engage_config = config;
    return engage_result;
  }

  bool setTargetForce(double target_force_n, std::string & /*error_message*/) override
  {
    calls.push_back("setTargetForce");
    last_set_target_force_n = target_force_n;
    return set_target_force_result;
  }

  bool disengage(std::string & /*error_message*/) override
  {
    calls.push_back("disengage");
    return disengage_result;
  }

  core_types::ComplianceStatus status() override
  {
    status_call_count++;
    core_types::ComplianceStatus status;
    status.wrench_valid = invalid_samples_remaining <= 0;
    if (!status.wrench_valid) {
      --invalid_samples_remaining;
    }
    status.wrench.force_z = -current_normal_force_n;  // normalForceN() negates force_z.
    status.stamp_ns = stamp_ns_;
    stamp_ns_ += 10000000ULL;  // 10ms per sample -- arbitrary, just monotonic.
    return status;
  }

  bool protectiveStop(std::string & /*error_message*/) override
  {
    calls.push_back("protectiveStop");
    return true;
  }

  std::vector<std::string> calls;  // Every mutating call (tare/engage/setTargetForce/disengage),
                                   // in order. status() is deliberately excluded here.
  int status_call_count{0};
  double current_normal_force_n{0.0};
  int invalid_samples_remaining{0};  // Next N status() calls report wrench_valid=false.
  bool tare_result{true};
  bool engage_result{true};
  bool set_target_force_result{true};
  bool disengage_result{true};
  core_types::ComplianceConfig last_engage_config;
  double last_set_target_force_n{-1.0};

private:
  uint64_t stamp_ns_{1000000000ULL};
};

core_types::ToolProfile makeMopProfile()
{
  core_types::ToolProfile profile;
  profile.name = "mop";
  profile.task.contact = true;
  profile.task.target_force_n = 15.0;
  profile.task.force_band_n = 2.0;
  profile.task.force_zero_threshold_n = 3.0;
  profile.task.settle_timeout_s = 8.0;
  profile.task.overload_abort_n = 40.0;
  profile.task.tangential_torque_abort_nm = 5.0;
  profile.control.admittance.selected_axes = {false, false, true, false, false, false};
  profile.control.admittance.mass = {2.0, 2.0, 2.0};
  profile.control.admittance.damping = {500.0, 500.0, 1000.0};
  profile.control.admittance.stiffness = {1000.0, 1000.0, 300.0};
  profile.control.admittance.contact_threshold_n = 1.0;
  profile.control.admittance.max_offset_m = 0.05;
  profile.control.ft.lpf_cutoff_hz = 20.0;
  profile.control.ft.deadband_n = 2.0;
  profile.control.ft.clamp_n = 200.0;
  return profile;
}

// A no-op sleep so dwell loops in these tests run instantly rather than at 20ms/tick.
ForceStageSequencer::SleepFn instantSleep()
{
  return [] {};
}

// Simple incrementing fake clock so beginSegment()'s settle_entry_ns_ is coherent with the
// ScriptedCompliancePort's own advancing stamp_ns, without depending on wall-clock time.
ForceStageSequencer::NowNsFn incrementingClock()
{
  auto counter = std::make_shared<uint64_t>(0);
  return [counter]() {
           *counter += 10000000ULL;
           return *counter;
         };
}

TEST(ForceStageSequencer, ForceStageNoneNeverTouchesTheCompliancePort)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  core_types::ForceIntent none_intent;  // stage defaults to kNone.
  const auto before = sequencer.beforeSegment(none_intent, profile);
  EXPECT_TRUE(before.proceed);
  const auto after = sequencer.afterSegment(true);
  EXPECT_TRUE(after.proceed);

  EXPECT_TRUE(port.calls.empty());
  EXPECT_EQ(port.status_call_count, 0);
}

TEST(ForceStageSequencer, ApproachEngagesAndReleasesImmediatelyWithoutDwelling)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  core_types::ForceIntent approach_intent;
  approach_intent.stage = core_types::ForceStage::kApproach;
  const auto result = sequencer.beforeSegment(approach_intent, profile);

  EXPECT_TRUE(result.proceed);
  ASSERT_EQ(port.calls.size(), 1u);
  EXPECT_EQ(port.calls[0], "engage");
  EXPECT_EQ(port.status_call_count, 0) << "APPROACH releases immediately, no settle dwell";
}

TEST(ForceStageSequencer, EngageConfigIsTranslatedFromTheActiveToolProfile)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  core_types::ForceIntent approach_intent;
  approach_intent.stage = core_types::ForceStage::kApproach;
  sequencer.beforeSegment(approach_intent, profile);

  EXPECT_EQ(port.last_engage_config.selected_axes, profile.control.admittance.selected_axes);
  EXPECT_EQ(port.last_engage_config.mass, profile.control.admittance.mass);
  EXPECT_EQ(port.last_engage_config.stiffness, profile.control.admittance.stiffness);
  EXPECT_DOUBLE_EQ(port.last_engage_config.target_force_n, 15.0);
  EXPECT_DOUBLE_EQ(port.last_engage_config.max_offset_m, 0.05);
  EXPECT_DOUBLE_EQ(port.last_engage_config.deadband_n, 2.0);
}

TEST(ForceStageSequencer, EngageFailureAbortsWithoutEnteringADwellLoop)
{
  ScriptedCompliancePort port;
  port.engage_result = false;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  core_types::ForceIntent approach_intent;
  approach_intent.stage = core_types::ForceStage::kApproach;
  const auto result = sequencer.beforeSegment(approach_intent, profile);

  EXPECT_FALSE(result.proceed);
  EXPECT_EQ(result.failure_reason_code, "controller_switch_failed");
  EXPECT_EQ(port.status_call_count, 0);
}

TEST(ForceStageSequencer, IllegalSequenceIsRejectedBeforeTouchingTheCompliancePort)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  core_types::ForceIntent retract_intent;
  retract_intent.stage = core_types::ForceStage::kRetract;  // illegal from a fresh sequencer.
  const auto result = sequencer.beforeSegment(retract_intent, profile);

  EXPECT_FALSE(result.proceed);
  EXPECT_EQ(result.failure_reason_code, "force_stage_sequence_invalid");
  EXPECT_TRUE(port.calls.empty());
}

// Drives a sequencer through APPROACH -> confirmed contact -> CARTESIAN settle -> released, so
// later tests can start from kCartesianFollow without repeating the setup each time.
void driveToCartesianFollow(
  ForceStageSequencer & sequencer, ScriptedCompliancePort & port,
  const core_types::ToolProfile & profile)
{
  core_types::ForceIntent approach_intent;
  approach_intent.stage = core_types::ForceStage::kApproach;
  ASSERT_TRUE(sequencer.beforeSegment(approach_intent, profile).proceed);

  port.current_normal_force_n = 2.0;  // above contact_threshold_n (1.0N) -- contact confirmed.
  ASSERT_TRUE(sequencer.afterSegment(true).proceed);

  core_types::ForceIntent cartesian_intent;
  cartesian_intent.stage = core_types::ForceStage::kCartesian;
  port.current_normal_force_n = profile.task.target_force_n;  // held at target throughout settle.
  ASSERT_TRUE(sequencer.beforeSegment(cartesian_intent, profile).proceed);
  ASSERT_EQ(sequencer.state(), ForceFsmState::kCartesianFollow);
}

TEST(ForceStageSequencer, CartesianSettleDwellsThenReleasesWithoutAnyOtherPortCall)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  driveToCartesianFollow(sequencer, port, profile);

  // Only "engage" (from APPROACH) should appear -- CARTESIAN's own settle dwell must never call
  // engage/setTargetForce/disengage, only status().
  EXPECT_EQ(port.calls.size(), 1u);
  EXPECT_EQ(port.calls[0], "engage");
  EXPECT_GT(port.status_call_count, 0) << "the settle dwell must actually poll status()";
}

TEST(ForceStageSequencer, FullApproachCartesianRetractGroupCallsPortsInOrder)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  driveToCartesianFollow(sequencer, port, profile);
  ASSERT_TRUE(sequencer.afterSegment(true).proceed);  // strip finished -> kCartesianDone.

  core_types::ForceIntent retract_intent;
  retract_intent.stage = core_types::ForceStage::kRetract;
  port.current_normal_force_n = 0.5;  // below force_zero_threshold_n (3.0N) throughout unload.
  ASSERT_TRUE(sequencer.beforeSegment(retract_intent, profile).proceed);
  ASSERT_EQ(sequencer.state(), ForceFsmState::kRetractFollow);

  // Peel finished, force still low -> safe to disengage.
  const auto final_result = sequencer.afterSegment(true);
  EXPECT_TRUE(final_result.proceed);

  // Full non-status call log for the whole group: engage (approach) -> setTargetForce (retract
  // unload target) -> disengage (once safely unloaded). tare() is deliberately never called here
  // -- see the class doc for why that's a mission-level, not segment-group-level, concern.
  ASSERT_EQ(port.calls.size(), 3u);
  EXPECT_EQ(port.calls[0], "engage");
  EXPECT_EQ(port.calls[1], "setTargetForce");
  EXPECT_EQ(port.calls[2], "disengage");
  EXPECT_DOUBLE_EQ(port.last_set_target_force_n, 0.0)
    << "RETRACT always commands zero, never the task's own target_force_n";
}

TEST(ForceStageSequencer, RetractNeverDisengagesWhileForceIsStillHigh)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  driveToCartesianFollow(sequencer, port, profile);
  ASSERT_TRUE(sequencer.afterSegment(true).proceed);

  core_types::ForceIntent retract_intent;
  retract_intent.stage = core_types::ForceStage::kRetract;
  port.current_normal_force_n = 0.5;  // converges the unload settle normally.
  ASSERT_TRUE(sequencer.beforeSegment(retract_intent, profile).proceed);

  // Peel trajectory finishes, but the force reading is still well above switch_safe_force_n (3N)
  // -- this is the P3.2 bumpless-transfer bound. disengage() must NEVER be called here.
  port.current_normal_force_n = 5.0;
  const auto result = sequencer.afterSegment(true);

  EXPECT_FALSE(result.proceed);
  EXPECT_EQ(result.failure_reason_code, "controller_switch_unsafe");
  for (const auto & call : port.calls) {
    EXPECT_NE(call, "disengage") << "must never disengage while |F| is still above the safe bound";
  }
}

TEST(ForceStageSequencer, AfterSegmentIsANoOpWhenBeforeSegmentWasNeverCalledWithForce)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());

  // No beforeSegment() call at all -- afterSegment() alone must not touch the port or crash.
  const auto result = sequencer.afterSegment(true);
  EXPECT_TRUE(result.proceed);
  EXPECT_TRUE(port.calls.empty());
}

TEST(ForceStageSequencer, ResetAllowsANewGroupAfterAnAbort)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  core_types::ForceIntent retract_intent;
  retract_intent.stage = core_types::ForceStage::kRetract;
  ASSERT_FALSE(sequencer.beforeSegment(retract_intent, profile).proceed);  // illegal -> aborts.
  ASSERT_EQ(sequencer.state(), ForceFsmState::kAborted);

  sequencer.reset();
  EXPECT_EQ(sequencer.state(), ForceFsmState::kIdle);

  core_types::ForceIntent approach_intent;
  approach_intent.stage = core_types::ForceStage::kApproach;
  const auto result = sequencer.beforeSegment(approach_intent, profile);
  EXPECT_TRUE(result.proceed);
}

TEST(ForceStageSequencer, TelemetryResetsToZeroAtTheStartOfEverySegment)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  core_types::ForceIntent approach_intent;
  approach_intent.stage = core_types::ForceStage::kApproach;
  sequencer.beforeSegment(approach_intent, profile);
  EXPECT_GT(sequencer.telemetry().compliance_switch_count, 0u)
    << "sanity check -- APPROACH's own engage() must have been recorded first";

  // A subsequent kNone segment must see fully-reset telemetry, not APPROACH's leftover values.
  core_types::ForceIntent none_intent;
  sequencer.beforeSegment(none_intent, profile);
  const auto & telemetry = sequencer.telemetry();
  EXPECT_EQ(telemetry.compliance_switch_count, 0u);
  EXPECT_EQ(telemetry.force_stage_reached, 0u);
  EXPECT_EQ(telemetry.settle_time_ms, 0u);
  EXPECT_EQ(telemetry.invalid_wrench_ticks, 0u);
  EXPECT_FALSE(telemetry.compliance_engaged_at_exit);
}

TEST(ForceStageSequencer, TelemetryTracksEngageAsAComplianceSwitch)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  core_types::ForceIntent approach_intent;
  approach_intent.stage = core_types::ForceStage::kApproach;
  sequencer.beforeSegment(approach_intent, profile);

  EXPECT_EQ(sequencer.telemetry().compliance_switch_count, 1u);
  EXPECT_TRUE(sequencer.telemetry().compliance_engaged_at_exit);
}

TEST(ForceStageSequencer, TelemetryTracksForceAtDisengageAndResetsPerSegment)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  driveToCartesianFollow(sequencer, port, profile);
  sequencer.afterSegment(true);

  core_types::ForceIntent retract_intent;
  retract_intent.stage = core_types::ForceStage::kRetract;
  port.current_normal_force_n = 0.5;
  sequencer.beforeSegment(retract_intent, profile);
  ASSERT_EQ(sequencer.state(), ForceFsmState::kRetractFollow);
  // RETRACT's own beforeSegment() call already reset telemetry -- APPROACH's earlier
  // compliance_switch_count==1 must not leak forward into this segment's own count.
  EXPECT_EQ(sequencer.telemetry().compliance_switch_count, 0u)
    << "the RETRACT segment's own settle dwell doesn't engage/disengage anything by itself";

  port.current_normal_force_n = 1.5;  // sampled at the disengage moment.
  const auto result = sequencer.afterSegment(true);
  ASSERT_TRUE(result.proceed);

  EXPECT_EQ(sequencer.telemetry().compliance_switch_count, 1u);
  EXPECT_DOUBLE_EQ(sequencer.telemetry().max_abs_force_at_switch_n, 1.5);
  EXPECT_FALSE(sequencer.telemetry().compliance_engaged_at_exit)
    << "the group is fully complete -- compliance must read as disengaged on exit";
}

TEST(ForceStageSequencer, TelemetryCountsInvalidWrenchTicksDuringTheSettleDwell)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  core_types::ForceIntent approach_intent;
  approach_intent.stage = core_types::ForceStage::kApproach;
  sequencer.beforeSegment(approach_intent, profile);
  port.current_normal_force_n = 2.0;
  sequencer.afterSegment(true);

  core_types::ForceIntent cartesian_intent;
  cartesian_intent.stage = core_types::ForceStage::kCartesian;
  port.current_normal_force_n = profile.task.target_force_n;
  port.invalid_samples_remaining = 3;  // three stale/NaN samples before the trace goes valid.
  sequencer.beforeSegment(cartesian_intent, profile);

  EXPECT_EQ(sequencer.telemetry().invalid_wrench_ticks, 3u);
}

TEST(ForceStageSequencer, TelemetryTracksFurthestStageReachedEvenAfterAnAbort)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  driveToCartesianFollow(sequencer, port, profile);
  const uint8_t reached_before_abort = sequencer.telemetry().force_stage_reached;
  EXPECT_EQ(reached_before_abort, static_cast<uint8_t>(ForceFsmState::kCartesianFollow));

  // An overload abort mid-follow must not erase the fact that kCartesianFollow was reached.
  port.current_normal_force_n = 41.0;  // above overload_abort_n (40N).
  const auto result = sequencer.afterSegment(true);

  EXPECT_FALSE(result.proceed);
  EXPECT_EQ(result.failure_reason_code, "contact_overload");
  EXPECT_EQ(sequencer.state(), ForceFsmState::kAborted);
  EXPECT_EQ(sequencer.telemetry().force_stage_reached, reached_before_abort)
    << "telemetry must report the furthest state legitimately reached, not kAborted itself";
}

TEST(ForceStageSequencer, SettleTimeIsPositiveAfterASettleDwell)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  core_types::ForceIntent approach_intent;
  approach_intent.stage = core_types::ForceStage::kApproach;
  sequencer.beforeSegment(approach_intent, profile);
  EXPECT_EQ(sequencer.telemetry().settle_time_ms, 0u)
    << "APPROACH releases immediately -- there is no settle dwell to time";

  port.current_normal_force_n = 2.0;
  sequencer.afterSegment(true);

  core_types::ForceIntent cartesian_intent;
  cartesian_intent.stage = core_types::ForceStage::kCartesian;
  port.current_normal_force_n = profile.task.target_force_n;
  sequencer.beforeSegment(cartesian_intent, profile);

  EXPECT_GT(sequencer.telemetry().settle_time_ms, 0u)
    << "the CARTESIAN settle dwell took several ticks of the injected clock to converge";
}

// -- Force trim (see force_stage_sequencer.hpp's own "FORCE TRIM" doc comment) --
//
// Unlike every test above, the trim loop runs on a REAL std::thread against REAL wall-clock
// sleeps (see that file's own trimLoop() -- it must interleave with whatever the CALLER's own
// ExecutionPort::execute() is doing concurrently, which incrementingClock()/instantSleep()'s own
// fake-time model cannot represent). These tests therefore take real, short wall-clock delays
// (a few hundred ms) rather than running instantly like the rest of this file -- an accepted,
// deliberate exception, not an oversight. ScriptedCompliancePort itself needs no extra locking
// for this: every read of its state below happens strictly AFTER afterSegment() has already
// stopped and joined the trim thread (std::thread::join() is a happens-before edge), and the
// main test thread touches no port state while the trim thread might still be running.

TEST(ForceStageSequencer, TrimAccumulatesTowardAPersistentForceErrorDuringCartesianFollow)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  auto profile = makeMopProfile();
  profile.control.admittance.trim_ki = 1.0;
  profile.control.admittance.trim_integral_max_n = 20.0;

  driveToCartesianFollow(sequencer, port, profile);  // enters kCartesianFollow at target (15N).

  // Environment shifts (e.g. the real wall bows in slightly) -- force drops 5N below target,
  // with nothing else correcting it (admittance alone would just sit here indefinitely).
  port.current_normal_force_n = profile.task.target_force_n - 5.0;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));  // ~6 trim ticks at 50ms/tick.

  ASSERT_TRUE(sequencer.afterSegment(true).proceed);

  EXPECT_GT(sequencer.telemetry().final_trim_integral_n, 0.0)
    << "a persistent below-target force error must accumulate a POSITIVE trim (push harder)";
  EXPECT_GT(port.last_set_target_force_n, profile.task.target_force_n)
    << "the trim must have raised the commanded setpoint above the profile's own target_force_n";
}

TEST(ForceStageSequencer, TrimIntegralIsClampedToTrimIntegralMaxN)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  auto profile = makeMopProfile();
  profile.control.admittance.trim_ki = 1.0;
  profile.control.admittance.trim_integral_max_n = 0.5;  // deliberately tiny.

  driveToCartesianFollow(sequencer, port, profile);
  port.current_normal_force_n = profile.task.target_force_n - 5.0;
  // Unclamped, ki=1.0 * 5N * ~0.3s would accumulate ~1.5N -- well past the 0.5N ceiling.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  ASSERT_TRUE(sequencer.afterSegment(true).proceed);

  EXPECT_DOUBLE_EQ(sequencer.telemetry().final_trim_integral_n, 0.5)
    << "the integral must be clamped exactly at trim_integral_max_n, not left to grow unbounded";
}

TEST(ForceStageSequencer, TrimNeverStartsWhenTrimKiIsZero)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  auto profile = makeMopProfile();
  profile.control.admittance.trim_ki = 0.0;  // explicit, even though this is ToolAdmittance's
                                             // own default -- documents the exact case this
                                             // guards, not relying on the default silently.

  driveToCartesianFollow(sequencer, port, profile);
  port.current_normal_force_n = profile.task.target_force_n - 5.0;
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  ASSERT_TRUE(sequencer.afterSegment(true).proceed);

  EXPECT_EQ(sequencer.telemetry().final_trim_integral_n, 0.0);
  // Same exact-call-log invariant CartesianSettleDwellsThenReleasesWithoutAnyOtherPortCall
  // already asserts for the settle dwell itself -- trim_ki==0.0 must add NOTHING beyond that:
  // no setTargetForce() calls at all across the whole APPROACH+CARTESIAN group so far.
  EXPECT_EQ(port.calls.size(), 1u);
  EXPECT_EQ(port.calls[0], "engage");
}

// -- Abort-path backend safety (see ForceStageSequencer::handleAbort()'s own doc comment) --
//
// Found live (2026-08-21): an APPROACH that never detected contact left compliance ENGAGED on
// the REAL backend indefinitely -- neither dwellUntilReleaseOrAbort() nor afterSegment() called
// disengage() (or even zeroed the target force) on ANY abort, and telemetry.
// compliance_engaged_at_exit lied, reporting false (disengaged) regardless. Confirmed live to
// drive a robot to drift away from a subsequently-commanded rigid HOME position with no active
// caller command at all, hours after the original test process had already exited.

TEST(ForceStageSequencer, AfterSegmentAbortZerosTargetForceAndDisengagesWhenSafe)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  core_types::ForceIntent approach_intent;
  approach_intent.stage = core_types::ForceStage::kApproach;
  ASSERT_TRUE(sequencer.beforeSegment(approach_intent, profile).proceed);

  // Never reached contact_threshold_n -- exactly the bug's own real-world trigger.
  port.current_normal_force_n = 0.0;
  const auto result = sequencer.afterSegment(true);

  ASSERT_FALSE(result.proceed);
  EXPECT_EQ(result.failure_reason_code, "contact_not_detected");
  // engage, THEN setTargetForce(0) (the always-safe unload), THEN disengage (force is low enough
  // to be a safe switch) -- not left stuck "engaged" on the real backend.
  ASSERT_EQ(port.calls.size(), 3u);
  EXPECT_EQ(port.calls[0], "engage");
  EXPECT_EQ(port.calls[1], "setTargetForce");
  EXPECT_DOUBLE_EQ(port.last_set_target_force_n, 0.0);
  EXPECT_EQ(port.calls[2], "disengage");
  EXPECT_FALSE(sequencer.telemetry().compliance_engaged_at_exit)
    << "successfully disengaged -- telemetry must reflect that truthfully, not hardcode false "
    "regardless of what actually happened (the exact form of the original bug)";
}

TEST(ForceStageSequencer, AfterSegmentAbortLeavesEngagedButZeroBiasedWhenStillLoaded)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  driveToCartesianFollow(sequencer, port, profile);

  // Still heavily loaded at the moment of abort -- disengaging now would be the exact "snap back
  // at full stiffness" hazard RETRACT's own bumpless-transfer check already exists to avoid.
  port.current_normal_force_n = 41.0;  // above overload_abort_n (40N).
  const auto result = sequencer.afterSegment(true);

  ASSERT_FALSE(result.proceed);
  EXPECT_EQ(result.failure_reason_code, "contact_overload");
  // setTargetForce(0) still happens (always safe -- stop pushing harder), but NOT disengage()
  // (unsafe while this loaded).
  const auto disengage_calls =
    std::count(port.calls.begin(), port.calls.end(), "disengage");
  EXPECT_EQ(disengage_calls, 0);
  EXPECT_TRUE(
    std::find(
      port.calls.begin(), port.calls.end(),
      "setTargetForce") != port.calls.end());
  EXPECT_DOUBLE_EQ(port.last_set_target_force_n, 0.0);
  EXPECT_TRUE(sequencer.telemetry().compliance_engaged_at_exit)
    << "still too loaded to safely disengage -- must honestly report still-engaged, not silently "
    "claim disengaged while leaving the real backend loaded and driven";
}

TEST(ForceStageSequencer, DisengageFailureReportsStillEngagedNotFalselyDisengaged)
{
  ScriptedCompliancePort port;
  ForceStageSequencer sequencer(port, incrementingClock(), instantSleep());
  const auto profile = makeMopProfile();

  driveToCartesianFollow(sequencer, port, profile);
  sequencer.afterSegment(true);

  core_types::ForceIntent retract_intent;
  retract_intent.stage = core_types::ForceStage::kRetract;
  port.current_normal_force_n = 0.5;
  sequencer.beforeSegment(retract_intent, profile);

  port.current_normal_force_n = 0.5;
  port.disengage_result = false;  // the REAL backend call itself fails.
  const auto result = sequencer.afterSegment(true);

  EXPECT_FALSE(result.proceed);
  EXPECT_EQ(result.failure_reason_code, "controller_switch_failed");
  EXPECT_TRUE(sequencer.telemetry().compliance_engaged_at_exit)
    << "disengage() FAILED -- the backend is presumably still engaged; must not report false "
    "(disengaged) just because this segment group is done from the sequencer's own side";
}

}  // namespace
}  // namespace force_core
