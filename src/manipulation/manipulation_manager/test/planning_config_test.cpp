// The shipped default planning config must select exactly what the manager hard-wired before
// backends were selectable: MoveIt Cartesian first, OMPL for free space, no segment overrides.
#include <gtest/gtest.h>

#include <string>

#include "motion_planner/ros/planning_config.hpp"

TEST(PlanningConfigTest, ShippedDefaultMatchesTheBuiltInDefaults)
{
  motion_planner::PlanningSelection shipped;
  std::string error;
  ASSERT_TRUE(motion_planner::loadPlanningSelection(PLANNING_YAML_PATH, shipped, error)) << error;
  const motion_planner::PlanningSelection defaults;
  EXPECT_EQ(shipped.ik, defaults.ik);
  EXPECT_EQ(shipped.free_space, defaults.free_space);
  EXPECT_EQ(shipped.cartesian, defaults.cartesian);
  EXPECT_EQ(shipped.collision, defaults.collision);
  EXPECT_EQ(shipped.cartesian, "moveit_cartesian");
  EXPECT_EQ(shipped.free_space, "moveit_ompl");
  EXPECT_TRUE(shipped.scene.empty());
  EXPECT_TRUE(shipped.segments.empty());
}
