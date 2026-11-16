#include <gtest/gtest.h>

#include "manipulation_manager/core/tool_actuation.hpp"

using manipulation_manager::ToolActuation;
using manipulation_manager::ToolActuationConfig;
using manipulation_manager::ToolResult;

namespace
{
ToolActuationConfig valve(double lead = 0.0)
{
  ToolActuationConfig c;
  c.tool_command = "valve";
  c.off_lead_s = lead;
  return c;
}
}  // namespace

TEST(ToolActuation, OpensAtWindowStartClosesWithLead)
{
  ToolActuation t(valve(0.5));
  t.start(1.0, 5.0);
  EXPECT_FALSE(t.tick(0.5, 0.5, false, 1.0, false, false).send);
  auto o = t.tick(1.0, 1.0, false, 1.0, false, false);
  EXPECT_TRUE(o.send);
  EXPECT_TRUE(o.command);
  EXPECT_FALSE(t.tick(4.4, 4.4, false, 1.0, false, false).send);
  o = t.tick(4.5, 4.5, false, 1.0, false, false);  // end - lead
  EXPECT_TRUE(o.send);
  EXPECT_FALSE(o.command);
  EXPECT_EQ(t.tick(5.0, 5.0, false, 1.0, false, false).result, ToolResult::OK);
}

TEST(ToolActuation, LeadLongerThanWindowNeverOpens)
{
  ToolActuation t(valve(10.0));
  t.start(1.0, 3.0);
  EXPECT_FALSE(t.tick(1.5, 1.5, false, 1.0, false, false).command);
  EXPECT_EQ(t.tick(3.0, 3.0, false, 1.0, false, false).result, ToolResult::OK);
}

TEST(ToolActuation, PauseClosesAndResumeReopens)
{
  ToolActuation t(valve());
  t.start(0.0, 10.0);
  EXPECT_TRUE(t.tick(1.0, 1.0, false, 1.0, false, false).command);
  auto o = t.tick(2.0, 2.0, true, 1.0, false, false);
  EXPECT_TRUE(o.send);
  EXPECT_FALSE(o.command);
  // Trajectory time stands still while paused; the schedule resumes there.
  EXPECT_FALSE(t.tick(2.0, 5.0, true, 1.0, false, false).send);
  o = t.tick(2.0, 6.0, false, 1.0, false, false);
  EXPECT_TRUE(o.send);
  EXPECT_TRUE(o.command);
}

TEST(ToolActuation, PauseKeepsStateWhenAsked)
{
  auto c = valve();
  c.pause_keeps_state = true;
  ToolActuation t(c);
  t.start(0.0, 10.0);
  t.tick(1.0, 1.0, false, 1.0, false, false);
  EXPECT_FALSE(t.tick(2.0, 2.0, true, 1.0, false, false).send);
  EXPECT_TRUE(t.commanded());
}

TEST(ToolActuation, PumpWaitsForSpeedRatio)
{
  ToolActuationConfig c;
  c.tool_command = "pump";
  c.on_at_speed_ratio = 0.8;
  ToolActuation t(c);
  t.start(0.0, 10.0);
  EXPECT_FALSE(t.tick(1.0, 1.0, false, 0.5, false, false).command);
  EXPECT_TRUE(t.tick(1.1, 1.1, false, 0.9, false, false).command);
}

TEST(ToolActuation, NoneNeverSwitches)
{
  ToolActuationConfig c;
  c.tool_command = "none";
  ToolActuation t(c);
  t.start(0.0, 1.0);
  EXPECT_FALSE(t.tick(0.5, 0.5, false, 1.0, false, false).send);
  t.cancel();
  EXPECT_FALSE(t.tick(0.6, 0.6, false, 1.0, false, false).send);
}

TEST(ToolActuation, VerifiedAgainstEchoWithLatency)
{
  ToolActuation t(valve());
  t.start(0.0, 10.0);
  EXPECT_TRUE(t.tick(0.0, 100.0, false, 1.0, true, false).send);
  t.tick(0.1, 100.2, false, 1.0, true, true);  // echo on after 0.2 s
  EXPECT_NEAR(t.onLatencyS(), 0.2, 1e-9);
  EXPECT_EQ(t.result(), ToolResult::RUNNING);
}

TEST(ToolActuation, UnconfirmedCommandIsResentThenNotResponsive)
{
  ToolActuation t(valve());
  t.start(0.0, 10.0);
  t.tick(0.0, 0.0, false, 1.0, true, false);
  EXPECT_FALSE(t.tick(0.1, 0.1, false, 1.0, true, false).send);
  auto o = t.tick(0.3, 0.3, false, 1.0, true, false);
  EXPECT_TRUE(o.send);
  EXPECT_TRUE(o.command);
  o = t.tick(1.2, 1.2, false, 1.0, true, false);
  EXPECT_EQ(o.result, ToolResult::TOOL_NOT_RESPONSIVE);
}

TEST(ToolActuation, CancelForcesOffUntilConfirmed)
{
  ToolActuation t(valve());
  t.start(0.0, 10.0);
  t.tick(1.0, 1.0, false, 1.0, true, false);
  t.tick(1.1, 1.1, false, 1.0, true, true);
  t.cancel();
  auto o = t.tick(1.2, 1.2, false, 1.0, true, true);
  EXPECT_TRUE(o.send);
  EXPECT_FALSE(o.command);
  EXPECT_EQ(o.result, ToolResult::RUNNING);  // off not yet echoed
  o = t.tick(1.3, 1.3, false, 1.0, true, false);
  EXPECT_EQ(o.result, ToolResult::CANCELLED);
}

TEST(ToolActuation, CancelWithoutEchoFinishesAfterOneOff)
{
  ToolActuation t(valve());
  t.start(0.0, 10.0);
  t.tick(1.0, 1.0, false, 1.0, false, false);
  t.cancel();
  auto o = t.tick(1.1, 1.1, false, 1.0, false, false);
  EXPECT_TRUE(o.send);
  EXPECT_FALSE(o.command);
  EXPECT_EQ(o.result, ToolResult::CANCELLED);
}

TEST(ToolActuation, CancelBeforeOpenedStillSendsOff)
{
  ToolActuation t(valve());
  t.start(5.0, 10.0);
  t.cancel();
  auto o = t.tick(0.0, 0.0, false, 1.0, false, false);
  EXPECT_TRUE(o.send);
  EXPECT_FALSE(o.command);
}

TEST(ToolActuation, FaultForcesOffAndReportsToolFault)
{
  ToolActuation t(valve());
  t.start(0.0, 10.0);
  t.tick(1.0, 1.0, false, 1.0, false, false);
  t.fault();
  auto o = t.tick(1.1, 1.1, false, 1.0, false, false);
  EXPECT_FALSE(o.command);
  EXPECT_EQ(o.result, ToolResult::TOOL_FAULT);
}

TEST(ToolActuation, EchoDisagreeingAfterConfirmIsToolFault)
{
  ToolActuation t(valve());
  t.start(0.0, 10.0);
  t.tick(1.0, 1.0, false, 1.0, true, false);
  t.tick(1.1, 1.1, false, 1.0, true, true);  // confirmed on
  auto o = t.tick(1.2, 1.2, false, 1.0, true, false);  // dropped by itself
  EXPECT_FALSE(o.command);
  EXPECT_TRUE(o.send);
  o = t.tick(1.3, 1.3, false, 1.0, true, false);
  EXPECT_EQ(o.result, ToolResult::TOOL_FAULT);
}

TEST(ToolActuation, StuckOnAtCancelIsNotResponsive)
{
  ToolActuation t(valve());
  t.start(0.0, 10.0);
  t.tick(1.0, 1.0, false, 1.0, true, false);
  t.tick(1.1, 1.1, false, 1.0, true, true);
  t.cancel();
  t.tick(2.0, 2.0, false, 1.0, true, true);
  auto o = t.tick(3.5, 3.5, false, 1.0, true, true);  // still on
  EXPECT_EQ(o.result, ToolResult::TOOL_NOT_RESPONSIVE);
}
