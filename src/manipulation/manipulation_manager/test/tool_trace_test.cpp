#include <gtest/gtest.h>

#include "manipulation_manager/core/tool_trace.hpp"

using namespace manipulation_manager;

TEST(ToolTrace, KeepsSamplesOnlyWhileStroking)
{
  ToolTrace t;
  EXPECT_FALSE(t.addTool({1, 2, 3}));
  EXPECT_FALSE(t.addJoints({0.1, 0.2}));
  t.setStroking(true, 4);
  t.setForce(12.0);
  EXPECT_TRUE(t.addTool({1, 2, 3}));
  EXPECT_TRUE(t.addJoints({0.1, 0.2}));
  t.setStroking(false, 4);
  EXPECT_FALSE(t.addTool({4, 5, 6}));
  const auto s = t.samples();
  ASSERT_EQ(s.size(), 1u);
  EXPECT_EQ(s[0].column, 4u);
  EXPECT_DOUBLE_EQ(s[0].force_n, 12.0);
  ASSERT_EQ(t.jointStrokes().size(), 1u);
  EXPECT_EQ(t.jointStrokes()[0].column, 4u);
}

TEST(ToolTrace, SamplesCarryTheColumnAndTheLatestForce)
{
  ToolTrace t;
  t.setStroking(true, 0);
  t.setForce(5.0);
  t.addTool({0, 0, 0});
  t.setForce(9.0);
  t.addTool({0, 0, 0.01});
  t.setStroking(true, 1);
  t.addTool({1, 0, 0});
  const auto s = t.samples();
  ASSERT_EQ(s.size(), 3u);
  EXPECT_DOUBLE_EQ(s[0].force_n, 5.0);
  EXPECT_DOUBLE_EQ(s[1].force_n, 9.0);
  EXPECT_EQ(s[2].column, 1u);
}

TEST(ToolTrace, CapDropsLaterSamplesAndCountsThem)
{
  ToolTrace t(3);
  t.setStroking(true, 0);
  for (int i = 0; i < 5; ++i) {t.addTool({0, 0, 0.01 * i});}
  EXPECT_EQ(t.samples().size(), 3u);
  EXPECT_EQ(t.dropped(), 2u);
}

TEST(ToolTrace, ResetClearsEverythingButTheCap)
{
  ToolTrace t;
  t.setPlan({PlannedStroke{0, {{0, 0, 0}, {0, 0, 1}}}});
  t.setStroking(true, 0);
  t.addTool({0, 0, 0});
  t.reset();
  EXPECT_TRUE(t.plan().empty());
  EXPECT_TRUE(t.samples().empty());
  EXPECT_FALSE(t.stroking());
  EXPECT_FALSE(t.addTool({0, 0, 0}));
}

TEST(ToolTrace, JointStrokesAreOnePerColumnAscending)
{
  ToolTrace t;
  t.setStroking(true, 2);
  t.addJoints({1.0});
  t.addJoints({1.1});
  t.setStroking(true, 0);
  t.addJoints({2.0});
  const auto j = t.jointStrokes();
  ASSERT_EQ(j.size(), 2u);
  EXPECT_EQ(j[0].column, 0u);
  EXPECT_EQ(j[1].column, 2u);
  EXPECT_EQ(j[1].waypoints.size(), 2u);
}
