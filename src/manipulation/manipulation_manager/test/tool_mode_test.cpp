#include <gtest/gtest.h>

#include "manipulation_manager/core/tool_mode.hpp"

namespace manipulation_manager
{

TEST(ToolModeTest, ContactToolKeepsTheOriginalColumn)
{
  const auto s = columnSteps(true, 0.0015, 0.2);
  EXPECT_TRUE(s.engage_force);
  EXPECT_TRUE(s.press);
  EXPECT_TRUE(s.allow_wall_contact);
  EXPECT_FALSE(s.valve);
  EXPECT_DOUBLE_EQ(s.stroke_shift_m, 0.0015);   // the extra stand-off is ignored for a contact tool
}

TEST(ToolModeTest, SprayToolNeverEngagesPressesOrTouches)
{
  const auto s = columnSteps(false, 0.0015, 0.0);
  EXPECT_FALSE(s.engage_force);
  EXPECT_FALSE(s.press);
  EXPECT_FALSE(s.allow_wall_contact);
  EXPECT_TRUE(s.valve);
  EXPECT_DOUBLE_EQ(s.stroke_shift_m, 0.0);   // no overshoot into the wall
}

TEST(ToolModeTest, SprayExtraStandoffMovesAwayFromTheWallOnly)
{
  EXPECT_DOUBLE_EQ(columnSteps(false, 0.0, 0.05).stroke_shift_m, -0.05);
  EXPECT_DOUBLE_EQ(columnSteps(false, 0.0, -0.05).stroke_shift_m, 0.0);
}

TEST(ToolModeTest, ValveGateReportsOnlyChanges)
{
  ValveGate g;
  EXPECT_FALSE(g.on());
  EXPECT_FALSE(g.update(false));
  EXPECT_TRUE(g.update(true));
  EXPECT_TRUE(g.on());
  EXPECT_FALSE(g.update(true));
  EXPECT_TRUE(g.update(false));
  EXPECT_FALSE(g.on());
}

}  // namespace manipulation_manager
