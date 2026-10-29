/**
 * @file force_stage_fsm_test.cpp
 * @brief R12 (force_control_derisking_plan.md Gate 0): unit tests for ForceStageFsm against
 *        fabricated ComplianceStatus traces with synthetic timestamps. No ROS, no hardware, no
 *        clock -- every timeout/debounce assertion here would otherwise be first exercised
 *        against a real wall at 15 N.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <gtest/gtest.h>

#include "force_core/force_stage_fsm.hpp"

namespace force_core
{
namespace
{

constexpr uint64_t kSecond = 1000000000ULL;

ForceFsmParams makeMopParams()
{
  ForceFsmParams params;
  params.target_force_n = 15.0;
  params.force_band_n = 2.0;
  params.force_zero_threshold_n = 3.0;
  params.settle_timeout_s = 8.0;
  params.overload_abort_n = 40.0;
  params.tangential_torque_abort_nm = 5.0;
  params.unexpected_contact_abort_n = 0.0;  // contact tool -- not used here.
  params.contact_threshold_n = 1.0;
  params.convergence_ticks = 5;
  params.tracking_lost_error_n = 5.0;
  params.tracking_lost_hold_s = 1.0;
  params.switch_safe_force_n = 3.0;
  return params;
}

core_types::ComplianceStatus makeStatus(
  double normal_force_n, uint64_t stamp_ns,
  bool valid = true)
{
  core_types::ComplianceStatus status;
  status.wrench_valid = valid;
  status.wrench.force_z = -normal_force_n;  // normalForceN() negates force_z, see Wrench's convention.
  status.stamp_ns = stamp_ns;
  return status;
}

// Drives beginSegment(kApproach) -> onTrajectoryFinished(contact confirmed) -> beginSegment(kCartesian),
// landing the FSM in kCartesianSettle at the given start time, ready for convergence-tick tests.
ForceStageFsm makeFsmAtCartesianSettle(const ForceFsmParams & params, uint64_t start_ns)
{
  ForceStageFsm fsm;
  fsm.beginSegment(core_types::ForceStage::kApproach, params, start_ns);
  fsm.onTrajectoryFinished(true, makeStatus(2.0, start_ns + kSecond));
  EXPECT_EQ(fsm.state(), ForceFsmState::kApproachContactConfirmed);
  fsm.beginSegment(core_types::ForceStage::kCartesian, params, start_ns + kSecond);
  EXPECT_EQ(fsm.state(), ForceFsmState::kCartesianSettle);
  return fsm;
}

TEST(ForceStageFsm, ConvergesAfterExactlyConvergenceTicksNotBefore)
{
  const auto params = makeMopParams();
  auto fsm = makeFsmAtCartesianSettle(params, 0);
  const uint64_t base_ns = kSecond;

  // 4 in-band samples (convergence_ticks - 1): still dwelling.
  for (int i = 0; i < params.convergence_ticks - 1; ++i) {
    const auto step = fsm.step(makeStatus(15.0, base_ns + i * 10000000ULL));
    EXPECT_EQ(step.action, ForceFsmAction::kDwell) << "tick " << i;
    EXPECT_EQ(fsm.state(), ForceFsmState::kCartesianSettle);
  }

  // The Nth (5th) in-band sample releases the trajectory.
  const auto final_step = fsm.step(
    makeStatus(
      15.0,
      base_ns + params.convergence_ticks * 10000000ULL));
  EXPECT_EQ(final_step.action, ForceFsmAction::kReleaseTrajectory);
  EXPECT_EQ(fsm.state(), ForceFsmState::kCartesianFollow);
}

TEST(ForceStageFsm, SingleOutOfBandSampleResetsDebounce)
{
  const auto params = makeMopParams();
  auto fsm = makeFsmAtCartesianSettle(params, 0);
  const uint64_t base_ns = kSecond;

  for (int i = 0; i < params.convergence_ticks - 1; ++i) {
    fsm.step(makeStatus(15.0, base_ns + i * 10000000ULL));
  }
  EXPECT_EQ(fsm.state(), ForceFsmState::kCartesianSettle);

  // One out-of-band sample resets the counter to zero.
  const uint64_t reset_ns = base_ns + (params.convergence_ticks - 1) * 10000000ULL;
  const auto reset_step = fsm.step(makeStatus(25.0, reset_ns));  // far outside force_band_n
  EXPECT_EQ(reset_step.action, ForceFsmAction::kDwell);
  EXPECT_EQ(fsm.state(), ForceFsmState::kCartesianSettle);

  // convergence_ticks - 1 more in-band samples should NOT be enough now -- the counter restarted.
  for (int i = 0; i < params.convergence_ticks - 1; ++i) {
    const auto step = fsm.step(makeStatus(15.0, reset_ns + (i + 1) * 10000000ULL));
    EXPECT_EQ(step.action, ForceFsmAction::kDwell) << "post-reset tick " << i;
  }
  EXPECT_EQ(fsm.state(), ForceFsmState::kCartesianSettle);
}

TEST(ForceStageFsm, InvalidWrenchNeitherAdvancesNorResetsDebounceButTimeoutStillRuns)
{
  const auto params = makeMopParams();
  auto fsm = makeFsmAtCartesianSettle(params, 0);
  const uint64_t base_ns = kSecond;

  // 2 in-band samples.
  fsm.step(makeStatus(15.0, base_ns));
  fsm.step(makeStatus(15.0, base_ns + 10000000ULL));

  // An invalid sample changes nothing about the debounce count.
  const auto invalid_step = fsm.step(makeStatus(0.0, base_ns + 20000000ULL, /*valid=*/ false));
  EXPECT_EQ(invalid_step.action, ForceFsmAction::kDwell);
  EXPECT_EQ(fsm.state(), ForceFsmState::kCartesianSettle);

  // The remaining 3 in-band samples still complete convergence (count was preserved at 2, not reset).
  fsm.step(makeStatus(15.0, base_ns + 30000000ULL));
  fsm.step(makeStatus(15.0, base_ns + 40000000ULL));
  const auto final_step = fsm.step(makeStatus(15.0, base_ns + 50000000ULL));
  EXPECT_EQ(final_step.action, ForceFsmAction::kReleaseTrajectory);
}

TEST(ForceStageFsm, InvalidWrenchDoesNotStopTheSettleTimeoutWatchdog)
{
  const auto params = makeMopParams();
  auto fsm = makeFsmAtCartesianSettle(params, 0);
  const uint64_t base_ns = kSecond;

  // Only invalid samples for the whole settle_timeout_s window -- the watchdog must still fire.
  const uint64_t timeout_ns = base_ns + static_cast<uint64_t>(params.settle_timeout_s * kSecond);
  const auto step = fsm.step(makeStatus(0.0, timeout_ns, /*valid=*/ false));
  EXPECT_EQ(step.action, ForceFsmAction::kAbort);
  EXPECT_EQ(step.failure_reason_code, "force_stage_timeout");
  EXPECT_EQ(fsm.state(), ForceFsmState::kAborted);
}

TEST(ForceStageFsm, SettleTimeoutFiresAtSettleTimeoutSeconds)
{
  const auto params = makeMopParams();
  auto fsm = makeFsmAtCartesianSettle(params, 0);
  const uint64_t base_ns = kSecond;

  // Force never converges (always out of band).
  const uint64_t timeout_ns = base_ns + static_cast<uint64_t>(params.settle_timeout_s * kSecond);
  const auto step = fsm.step(makeStatus(0.0, timeout_ns));
  EXPECT_EQ(step.action, ForceFsmAction::kAbort);
  EXPECT_EQ(step.failure_reason_code, "force_stage_timeout");
}

// Regression: a status sample whose stamp_ns is OLDER than the settle-entry timestamp -- which
// happens routinely against a REAL clock (a just-cached wrench sample is always some
// publish/subscribe latency behind "now", and a caller can easily capture settle_entry_ns
// moments before that stale sample is even superseded) -- must NOT be treated as an elapsed time
// large enough to trip settle_timeout_s. A naive `status.stamp_ns - settle_entry_ns_` (unsigned
// uint64_t subtraction) UNDERFLOWS to a value near UINT64_MAX here, aborting on the very first
// tick regardless of how small settle_timeout_s actually is. Hit live: JtacControllerCompliancePort
// (compliant_trajectory_controller, T8) aborted CARTESIAN settle in under 100 microseconds
// with the measured force already exactly at target, because ForceStageSequencer's now_ns_fn
// captured settle_entry_ns_ a few milliseconds AFTER the last cached wrench sample's own stamp.
TEST(ForceStageFsm, StaleFirstSampleOlderThanSettleEntryDoesNotFalseTimeout)
{
  const auto params = makeMopParams();
  auto fsm = makeFsmAtCartesianSettle(params, 0);
  const uint64_t settle_entry_ns = kSecond;  // set inside makeFsmAtCartesianSettle().

  // In-band force, but the sample's own stamp predates settle_entry_ns -- exactly the ordering a
  // real wrench subscription produces on the very first poll after a stage transition.
  const auto step = fsm.step(makeStatus(15.0, settle_entry_ns - 5000000ULL));  // 5ms "in the past".
  EXPECT_NE(step.action, ForceFsmAction::kAbort)
    << "a stale-but-in-band sample must not be treated as an elapsed-time timeout; got "
    << step.failure_reason_code;
  EXPECT_EQ(fsm.state(), ForceFsmState::kCartesianSettle);
}

TEST(ForceStageFsm, OverloadAbortsOnFirstSampleFromEveryEngagedState)
{
  const auto params = makeMopParams();

  // kApproach.
  {
    ForceStageFsm fsm;
    fsm.beginSegment(core_types::ForceStage::kApproach, params, 0);
    const auto step = fsm.step(makeStatus(41.0, kSecond));
    EXPECT_EQ(step.action, ForceFsmAction::kAbort);
    EXPECT_EQ(step.failure_reason_code, "contact_overload");
  }

  // kCartesianSettle.
  {
    auto fsm = makeFsmAtCartesianSettle(params, 0);
    const auto step = fsm.step(makeStatus(41.0, 2 * kSecond));
    EXPECT_EQ(step.action, ForceFsmAction::kAbort);
    EXPECT_EQ(step.failure_reason_code, "contact_overload");
  }

  // kCartesianFollow.
  {
    auto fsm = makeFsmAtCartesianSettle(params, 0);
    for (int i = 0; i < params.convergence_ticks; ++i) {
      fsm.step(makeStatus(15.0, 2 * kSecond + i * 10000000ULL));
    }
    EXPECT_EQ(fsm.state(), ForceFsmState::kCartesianFollow);
    const auto step = fsm.step(makeStatus(41.0, 3 * kSecond));
    EXPECT_EQ(step.action, ForceFsmAction::kAbort);
    EXPECT_EQ(step.failure_reason_code, "contact_overload");
  }
}

TEST(ForceStageFsm, RetractFollowRefusesDisengageWhileForceStillHigh)
{
  const auto params = makeMopParams();
  ForceStageFsm fsm;

  // Drive to kRetractFollow: approach -> confirmed -> cartesian settle -> follow -> done -> retract settle -> follow.
  fsm.beginSegment(core_types::ForceStage::kApproach, params, 0);
  fsm.onTrajectoryFinished(true, makeStatus(2.0, kSecond));
  fsm.beginSegment(core_types::ForceStage::kCartesian, params, kSecond);
  for (int i = 0; i < params.convergence_ticks; ++i) {
    fsm.step(makeStatus(15.0, kSecond + i * 10000000ULL));
  }
  ASSERT_EQ(fsm.state(), ForceFsmState::kCartesianFollow);
  fsm.onTrajectoryFinished(true, makeStatus(15.0, 2 * kSecond));
  ASSERT_EQ(fsm.state(), ForceFsmState::kCartesianDone);
  fsm.beginSegment(core_types::ForceStage::kRetract, params, 2 * kSecond);
  ASSERT_EQ(fsm.state(), ForceFsmState::kRetractSettle);
  for (int i = 0; i < params.convergence_ticks; ++i) {
    fsm.step(makeStatus(1.0, 2 * kSecond + i * 10000000ULL));  // below force_zero_threshold_n
  }
  ASSERT_EQ(fsm.state(), ForceFsmState::kRetractFollow);

  // Peel trajectory finishes but force is still at 5N, above switch_safe_force_n (3N).
  const auto step = fsm.onTrajectoryFinished(true, makeStatus(5.0, 3 * kSecond));
  EXPECT_EQ(step.action, ForceFsmAction::kAbort);
  EXPECT_EQ(step.failure_reason_code, "controller_switch_unsafe");
  EXPECT_NE(step.action, ForceFsmAction::kDisengage);
}

TEST(ForceStageFsm, RetractFollowDisengagesOnceBelowSwitchSafeForce)
{
  const auto params = makeMopParams();
  ForceStageFsm fsm;

  fsm.beginSegment(core_types::ForceStage::kApproach, params, 0);
  fsm.onTrajectoryFinished(true, makeStatus(2.0, kSecond));
  fsm.beginSegment(core_types::ForceStage::kCartesian, params, kSecond);
  for (int i = 0; i < params.convergence_ticks; ++i) {
    fsm.step(makeStatus(15.0, kSecond + i * 10000000ULL));
  }
  fsm.onTrajectoryFinished(true, makeStatus(15.0, 2 * kSecond));
  fsm.beginSegment(core_types::ForceStage::kRetract, params, 2 * kSecond);
  for (int i = 0; i < params.convergence_ticks; ++i) {
    fsm.step(makeStatus(1.0, 2 * kSecond + i * 10000000ULL));
  }
  ASSERT_EQ(fsm.state(), ForceFsmState::kRetractFollow);

  const auto step = fsm.onTrajectoryFinished(true, makeStatus(1.0, 3 * kSecond));  // well below 3N
  EXPECT_EQ(step.action, ForceFsmAction::kDisengage);
  EXPECT_EQ(fsm.state(), ForceFsmState::kIdle);
}

TEST(ForceStageFsm, IllegalSequenceRetractFromIdleAbortsRatherThanProceeding)
{
  const auto params = makeMopParams();
  ForceStageFsm fsm;
  ASSERT_EQ(fsm.state(), ForceFsmState::kIdle);

  const auto step = fsm.beginSegment(core_types::ForceStage::kRetract, params, 0);
  EXPECT_EQ(step.action, ForceFsmAction::kAbort);
  EXPECT_EQ(step.failure_reason_code, "force_stage_sequence_invalid");
  EXPECT_EQ(fsm.state(), ForceFsmState::kAborted);
}

TEST(ForceStageFsm, IllegalSequenceCartesianFromIdleAbortsRatherThanProceeding)
{
  const auto params = makeMopParams();
  ForceStageFsm fsm;

  const auto step = fsm.beginSegment(core_types::ForceStage::kCartesian, params, 0);
  EXPECT_EQ(step.action, ForceFsmAction::kAbort);
  EXPECT_EQ(step.failure_reason_code, "force_stage_sequence_invalid");
}

TEST(ForceStageFsm, ForceStageNoneIsAlwaysLegalNoOp)
{
  const auto params = makeMopParams();
  ForceStageFsm fsm;
  const auto step = fsm.beginSegment(core_types::ForceStage::kNone, params, 0);
  EXPECT_EQ(step.action, ForceFsmAction::kDwell);
  EXPECT_EQ(fsm.state(), ForceFsmState::kIdle);
}

TEST(ForceStageFsm, ResetClearsAbortedState)
{
  const auto params = makeMopParams();
  ForceStageFsm fsm;
  fsm.beginSegment(core_types::ForceStage::kRetract, params, 0);  // illegal -> aborts
  ASSERT_EQ(fsm.state(), ForceFsmState::kAborted);

  fsm.reset();
  EXPECT_EQ(fsm.state(), ForceFsmState::kIdle);
  const auto step = fsm.beginSegment(core_types::ForceStage::kApproach, params, 0);
  EXPECT_EQ(step.action, ForceFsmAction::kEngage);
}

}  // namespace
}  // namespace force_core
