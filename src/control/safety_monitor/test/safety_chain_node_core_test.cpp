/**
 * @file safety_chain_node_core_test.cpp
 * @brief Helpers of safety_chain_node: wrench magnitudes, severity parsing, state text, cause naming,
 *        refusal reasons, all against a real SafetyChain.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "safety_monitor/safety_chain_node_core.hpp"

using namespace safety_monitor;

namespace
{
ChainParams params()
{
  ChainParams p;
  p.force_abort_n = 60.0;
  p.torque_abort_nm = 8.0;
  return p;
}

void feed(SafetyChain & c, double t, double f = 10.0, double tau = 1.0)
{
  c.updateFt(f, tau, t);
  c.updateJointState(t);
  c.step(t);
}
}  // namespace

TEST(NodeCore, WrenchMagnitudes)
{
  double f = 0, t = 0;
  wrenchMagnitudes(3, 4, 0, 0, 0, 2, f, t);
  EXPECT_DOUBLE_EQ(f, 5.0);
  EXPECT_DOUBLE_EQ(t, 2.0);
}

TEST(NodeCore, NonFiniteWrenchGivesNaNAndTripsTheChain)
{
  double f = 0, t = 0;
  wrenchMagnitudes(std::nan(""), 0, 0, 0, 0, 0, f, t);
  EXPECT_TRUE(std::isnan(f));
  EXPECT_DOUBLE_EQ(t, 0.0);
  wrenchMagnitudes(0, 0, 0, std::numeric_limits<double>::infinity(), 0, 0, f, t);
  EXPECT_TRUE(std::isnan(t));

  SafetyChain c(params());
  feed(c, 0.0);
  c.updateFt(f, t, 0.01);
  c.updateJointState(0.01);
  c.step(0.01);
  EXPECT_NE(c.state(), ChainState::kOk);
}

TEST(NodeCore, ParseSeverity)
{
  ChainSeverity s = ChainSeverity::kEstop;
  EXPECT_TRUE(parseSeverity("none", s));
  EXPECT_EQ(s, ChainSeverity::kNone);
  EXPECT_TRUE(parseSeverity("HOLD", s));
  EXPECT_EQ(s, ChainSeverity::kHold);
  EXPECT_TRUE(parseSeverity("EStop", s));
  EXPECT_EQ(s, ChainSeverity::kEstop);
  s = ChainSeverity::kHold;
  EXPECT_FALSE(parseSeverity("stop", s));
  EXPECT_FALSE(parseSeverity("", s));
  EXPECT_EQ(s, ChainSeverity::kHold);   // untouched on failure
  EXPECT_STREQ(toString(ChainSeverity::kEstop), "estop");
}

TEST(NodeCore, StateLabels)
{
  EXPECT_STREQ(stateLabel(ChainState::kOk), "OK");
  EXPECT_STREQ(stateLabel(ChainState::kHold), "HOLD");
  EXPECT_STREQ(stateLabel(ChainState::kEstop), "ESTOP");
}

TEST(NodeCore, OkHasNoCauses)
{
  SafetyChain c(params());
  feed(c, 0.0);
  EXPECT_TRUE(activeCauseNames(c.events(), c.state()).empty());
  EXPECT_EQ(formatState(c.state(), {}), "OK");
}

TEST(NodeCore, CausesAreNamedAndDeduplicated)
{
  SafetyChain c(params());
  feed(c, 0.0);
  c.updateFt(80.0, 20.0, 0.02);   // force and torque overload together
  c.updateJointState(0.02);
  c.step(0.02);
  c.updateFt(90.0, 1.0, 0.04);    // force again
  c.updateJointState(0.04);
  c.step(0.04);
  ASSERT_EQ(c.state(), ChainState::kEstop);
  const auto causes = activeCauseNames(c.events(), c.state());
  ASSERT_EQ(causes.size(), 2u);
  EXPECT_EQ(causes[0], "force_overload");
  EXPECT_EQ(causes[1], "torque_overload");
  EXPECT_EQ(formatState(c.state(), causes), "ESTOP force_overload,torque_overload");
}

TEST(NodeCore, CausesRestartAfterAnAcceptedReset)
{
  SafetyChain c(params());
  feed(c, 0.0);
  c.setOperatorEstop(true);
  c.step(0.02);
  c.setOperatorEstop(false);
  feed(c, 0.04);
  ASSERT_TRUE(c.reset(0.06));
  feed(c, 0.08);
  c.setControllerFault(true);
  c.step(0.10);
  const auto causes = activeCauseNames(c.events(), c.state());
  ASSERT_EQ(causes.size(), 1u);
  EXPECT_EQ(causes[0], "controller_fault");
}

TEST(NodeCore, RefusalReasonsNameTheActiveCause)
{
  SafetyChain c(params());
  feed(c, 0.0);
  c.setOperatorEstop(true);
  c.step(0.02);
  EXPECT_TRUE(refusalReasons(c.events(), 0.03).empty());
  EXPECT_FALSE(c.reset(0.03));
  const std::string why = refusalReasons(c.events(), 0.03);
  EXPECT_NE(why.find("operator_estop"), std::string::npos);
  EXPECT_TRUE(refusalReasons(c.events(), 0.5).empty());   // a different call
}
