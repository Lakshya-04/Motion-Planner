/**
 * @file safety_chain_test.cpp
 * @brief SafetyChain: every transition, latching, stale inputs, reset rules, event log.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "safety_monitor/safety_chain.hpp"

using namespace safety_monitor;

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

ChainParams params()
{
  ChainParams p;
  p.force_abort_n = 60.0;
  p.torque_abort_nm = 8.0;
  p.ft_stale_s = 0.1;
  p.joint_stale_s = 0.1;
  return p;
}

// Feeds healthy data at time t and steps.
ChainOutputs healthy(SafetyChain & c, double t)
{
  c.updateFt(10.0, 1.0, t);
  c.updateJointState(t);
  return c.step(t);
}

bool logged(const SafetyChain & c, ChainEventKind k, ChainCause cause)
{
  for (const auto & e : c.events()) {if (e.kind == k && e.cause == cause) {return true;}}
  return false;
}
}  // namespace

TEST(SafetyChain, StartsOkAndStaysOkWithHealthyInputs)
{
  SafetyChain c(params());
  for (int i = 0; i < 20; ++i) {
    const auto o = healthy(c, 0.01 * i);
    EXPECT_EQ(o.state, ChainState::kOk);
    EXPECT_FALSE(o.arm_stop || o.base_stop || o.tool_off || o.power_removal_request);
  }
  EXPECT_TRUE(c.events().empty());
}

TEST(SafetyChain, NeverFedInputsAreStale)
{
  SafetyChain c(params());
  EXPECT_EQ(c.step(1.0).state, ChainState::kEstop);   // joint state never seen: ESTOP
  EXPECT_EQ(c.latchedCause(), ChainCause::kStaleJointState);
}

TEST(SafetyChain, ForceOverloadTripsEstopWithAllOutputs)
{
  SafetyChain c(params());
  healthy(c, 0.0);
  c.updateFt(61.0, 1.0, 0.01);
  c.updateJointState(0.01);
  const auto o = c.step(0.01);
  EXPECT_EQ(o.state, ChainState::kEstop);
  EXPECT_TRUE(o.arm_stop && o.base_stop && o.tool_off && o.power_removal_request);
  EXPECT_EQ(c.latchedCause(), ChainCause::kForceOverload);
}

TEST(SafetyChain, ExactThresholdIsNotAnOverload)
{
  SafetyChain c(params());
  c.updateFt(60.0, 8.0, 0.0);
  c.updateJointState(0.0);
  EXPECT_EQ(c.step(0.0).state, ChainState::kOk);
}

TEST(SafetyChain, NegativeForceUsesMagnitude)
{
  SafetyChain c(params());
  c.updateFt(-61.0, 0.0, 0.0);
  c.updateJointState(0.0);
  EXPECT_EQ(c.step(0.0).state, ChainState::kEstop);
}

TEST(SafetyChain, TorqueOverloadTripsEstop)
{
  SafetyChain c(params());
  c.updateFt(1.0, 8.5, 0.0);
  c.updateJointState(0.0);
  c.step(0.0);
  EXPECT_EQ(c.state(), ChainState::kEstop);
  EXPECT_EQ(c.latchedCause(), ChainCause::kTorqueOverload);
}

TEST(SafetyChain, SpikeBetweenStepsIsNotLost)
{
  SafetyChain c(params());
  c.updateFt(100.0, 0.0, 0.00);   // spike
  c.updateFt(5.0, 0.0, 0.005);    // back to normal before the next step
  c.updateJointState(0.005);
  EXPECT_EQ(c.step(0.005).state, ChainState::kEstop);
}

TEST(SafetyChain, NaNWrenchIsAnOverload)
{
  SafetyChain c(params());
  c.updateFt(kNaN, 0.0, 0.0);
  c.updateJointState(0.0);
  EXPECT_EQ(c.step(0.0).state, ChainState::kEstop);
}

TEST(SafetyChain, OverloadChecksOffWhenThresholdZero)
{
  ChainParams p = params();
  p.force_abort_n = 0.0;
  p.torque_abort_nm = 0.0;
  SafetyChain c(p);
  c.updateFt(1e6, 1e6, 0.0);
  c.updateJointState(0.0);
  EXPECT_EQ(c.step(0.0).state, ChainState::kOk);
}

TEST(SafetyChain, StaleFtGivesHoldOnly)
{
  SafetyChain c(params());
  healthy(c, 0.0);
  c.updateJointState(0.2);
  const auto o = c.step(0.2);   // F/T is 0.2 s old, limit 0.1 s
  EXPECT_EQ(o.state, ChainState::kHold);
  EXPECT_TRUE(o.arm_stop && o.base_stop);
  EXPECT_FALSE(o.tool_off || o.power_removal_request);
  EXPECT_EQ(c.latchedCause(), ChainCause::kStaleFt);
}

TEST(SafetyChain, StaleFtNotRequiredForNonContactTool)
{
  ChainParams p = params();
  p.require_ft = false;
  SafetyChain c(p);
  c.updateJointState(0.0);
  EXPECT_EQ(c.step(0.0).state, ChainState::kOk);
  c.updateJointState(5.0);
  EXPECT_EQ(c.step(5.0).state, ChainState::kOk);
}

TEST(SafetyChain, StaleJointStateGivesEstop)
{
  SafetyChain c(params());
  healthy(c, 0.0);
  c.updateFt(1.0, 0.0, 0.2);
  EXPECT_EQ(c.step(0.2).state, ChainState::kEstop);
  EXPECT_EQ(c.latchedCause(), ChainCause::kStaleJointState);
}

TEST(SafetyChain, FutureAndNonFiniteStampsAreStale)
{
  SafetyChain c(params());
  c.updateFt(1.0, 0.0, 5.0);       // 5 s in the future
  c.updateJointState(0.0);
  EXPECT_EQ(c.step(0.0).state, ChainState::kHold);

  SafetyChain d(params());
  d.updateFt(1.0, 0.0, kNaN);      // unstamped: never fresh
  d.updateJointState(0.0);
  EXPECT_EQ(d.step(0.0).state, ChainState::kHold);
}

TEST(SafetyChain, BaseObstacleHoldAndAbort)
{
  SafetyChain c(params());
  healthy(c, 0.0);
  c.setBaseObstacle(BaseObstacle::kHold);
  EXPECT_EQ(healthy(c, 0.01).state, ChainState::kHold);
  c.setBaseObstacle(BaseObstacle::kAbort);
  EXPECT_EQ(healthy(c, 0.02).state, ChainState::kEstop);   // escalates HOLD -> ESTOP
  EXPECT_EQ(c.latchedCause(), ChainCause::kBaseObstacleHold);   // first cause stays
}

TEST(SafetyChain, OperatorEstopLatchesAndPulseIsNotLost)
{
  SafetyChain c(params());
  healthy(c, 0.0);
  c.setOperatorEstop(true);
  c.setOperatorEstop(false);   // released before the next step
  EXPECT_EQ(healthy(c, 0.01).state, ChainState::kEstop);
  EXPECT_EQ(healthy(c, 0.02).state, ChainState::kEstop);   // still latched
  EXPECT_EQ(c.latchedCause(), ChainCause::kOperatorEstop);
}

TEST(SafetyChain, ControllerFaultTripsEstop)
{
  SafetyChain c(params());
  healthy(c, 0.0);
  c.setControllerFault(true);
  EXPECT_EQ(healthy(c, 0.01).state, ChainState::kEstop);
  EXPECT_EQ(c.latchedCause(), ChainCause::kControllerFault);
}

TEST(SafetyChain, EstopNeverAutoClears)
{
  ChainParams p = params();
  p.hold_auto_clear = true;
  p.hold_clear_time_s = 0.05;
  SafetyChain c(p);
  c.updateFt(100.0, 0.0, 0.0);
  c.updateJointState(0.0);
  c.step(0.0);
  for (int i = 1; i <= 500; ++i) {EXPECT_EQ(healthy(c, 0.01 * i).state, ChainState::kEstop);}
}

TEST(SafetyChain, HoldLatchesByDefault)
{
  SafetyChain c(params());
  healthy(c, 0.0);
  c.setBaseObstacle(BaseObstacle::kHold);
  healthy(c, 0.01);
  c.setBaseObstacle(BaseObstacle::kNone);
  for (int i = 2; i < 500; ++i) {EXPECT_EQ(healthy(c, 0.01 * i).state, ChainState::kHold);}
}

TEST(SafetyChain, HoldAutoClearsAfterClearTimeWhenEnabled)
{
  ChainParams p = params();
  p.hold_auto_clear = true;
  p.hold_clear_time_s = 0.5;
  SafetyChain c(p);
  healthy(c, 0.0);
  c.setBaseObstacle(BaseObstacle::kHold);
  healthy(c, 0.01);
  c.setBaseObstacle(BaseObstacle::kNone);
  EXPECT_EQ(healthy(c, 0.02).state, ChainState::kHold);   // clear timer starts
  EXPECT_EQ(healthy(c, 0.30).state, ChainState::kHold);
  EXPECT_EQ(healthy(c, 0.60).state, ChainState::kOk);
  // a new hazard during the clear time restarts it
  c.setBaseObstacle(BaseObstacle::kHold);
  healthy(c, 0.70);
  c.setBaseObstacle(BaseObstacle::kNone);
  healthy(c, 0.71);
  c.setBaseObstacle(BaseObstacle::kHold);
  healthy(c, 1.0);
  c.setBaseObstacle(BaseObstacle::kNone);
  EXPECT_EQ(healthy(c, 1.1).state, ChainState::kHold);
}

TEST(SafetyChain, ResetRefusedWhileCauseActive)
{
  SafetyChain c(params());
  healthy(c, 0.0);
  c.setOperatorEstop(true);
  healthy(c, 0.01);
  EXPECT_FALSE(c.reset(0.02));   // e-stop still pressed
  EXPECT_EQ(c.state(), ChainState::kEstop);
  EXPECT_TRUE(logged(c, ChainEventKind::kResetRefused, ChainCause::kOperatorEstop));
  c.setOperatorEstop(false);
  healthy(c, 0.03);              // the pressed event is consumed
  EXPECT_TRUE(c.reset(0.03));
  EXPECT_EQ(c.state(), ChainState::kOk);
}

TEST(SafetyChain, ResetRefusedWithUnconsumedPulse)
{
  SafetyChain c(params());
  healthy(c, 0.0);
  c.setControllerFault(true);
  healthy(c, 0.01);
  c.setControllerFault(false);
  c.setControllerFault(true);    // new pulse not yet stepped
  c.setControllerFault(false);
  EXPECT_FALSE(c.reset(0.02));
}

TEST(SafetyChain, ResetRefusedWhileInputsStale)
{
  SafetyChain c(params());
  healthy(c, 0.0);
  c.setOperatorEstop(true);
  healthy(c, 0.01);
  c.setOperatorEstop(false);
  healthy(c, 0.02);
  EXPECT_FALSE(c.reset(1.0));    // no fresh data for a second
  EXPECT_EQ(c.state(), ChainState::kEstop);
}

TEST(SafetyChain, ResetRefusedWhileOverloadStillMeasured)
{
  SafetyChain c(params());
  c.updateFt(100.0, 0.0, 0.0);
  c.updateJointState(0.0);
  c.step(0.0);
  c.updateFt(100.0, 0.0, 0.01);
  c.updateJointState(0.01);
  c.step(0.01);
  EXPECT_FALSE(c.reset(0.01));
  healthy(c, 0.02);
  EXPECT_TRUE(c.reset(0.02));
}

TEST(SafetyChain, ResetInOkIsANoOpReturningFalse)
{
  SafetyChain c(params());
  healthy(c, 0.0);
  EXPECT_FALSE(c.reset(0.0));
}

TEST(SafetyChain, ResetFromHoldAndThenRetripsIfHazardReturns)
{
  SafetyChain c(params());
  healthy(c, 0.0);
  c.setBaseObstacle(BaseObstacle::kHold);
  healthy(c, 0.01);
  c.setBaseObstacle(BaseObstacle::kNone);
  healthy(c, 0.02);
  ASSERT_TRUE(c.reset(0.02));
  EXPECT_EQ(healthy(c, 0.03).state, ChainState::kOk);
  c.setBaseObstacle(BaseObstacle::kHold);
  EXPECT_EQ(healthy(c, 0.04).state, ChainState::kHold);
}

TEST(SafetyChain, IgnoredSeverityDoesNotTripOrBlockReset)
{
  ChainParams p = params();
  p.severity[static_cast<std::size_t>(ChainCause::kBaseObstacleHold)] = ChainSeverity::kNone;
  SafetyChain c(p);
  healthy(c, 0.0);
  c.setBaseObstacle(BaseObstacle::kHold);
  EXPECT_EQ(healthy(c, 0.01).state, ChainState::kOk);
  EXPECT_TRUE(logged(c, ChainEventKind::kCauseAsserted, ChainCause::kBaseObstacleHold));
}

TEST(SafetyChain, EventLogIsTimestampedAndOrdered)
{
  SafetyChain c(params());
  healthy(c, 1.0);
  c.setBaseObstacle(BaseObstacle::kHold);
  healthy(c, 2.0);
  c.setBaseObstacle(BaseObstacle::kNone);
  healthy(c, 3.0);
  healthy(c, 4.0);
  c.reset(4.0);
  ASSERT_EQ(c.events().size(), 3u);
  EXPECT_EQ(c.events()[0].kind, ChainEventKind::kCauseAsserted);
  EXPECT_DOUBLE_EQ(c.events()[0].t_s, 2.0);
  EXPECT_EQ(c.events()[1].kind, ChainEventKind::kStateChange);
  EXPECT_EQ(c.events()[1].from, ChainState::kOk);
  EXPECT_EQ(c.events()[1].to, ChainState::kHold);
  EXPECT_EQ(c.events()[2].kind, ChainEventKind::kResetAccepted);
  EXPECT_DOUBLE_EQ(c.events()[2].t_s, 4.0);
}

TEST(SafetyChain, EventLogIsBounded)
{
  ChainParams p = params();
  p.max_events = 4;
  SafetyChain c(p);
  healthy(c, 0.0);
  for (int i = 0; i < 10; ++i) {
    c.setBaseObstacle(BaseObstacle::kHold);
    healthy(c, 1.0 + i);
    c.setBaseObstacle(BaseObstacle::kNone);
    healthy(c, 1.5 + i);
    c.reset(1.6 + i);
  }
  EXPECT_EQ(c.events().size(), 4u);
  EXPECT_GT(c.droppedEvents(), 0u);
}

TEST(SafetyChain, ToStringNames)
{
  EXPECT_STREQ(toString(ChainState::kEstop), "ESTOP");
  EXPECT_STREQ(toString(ChainCause::kStaleFt), "stale_ft");
}
