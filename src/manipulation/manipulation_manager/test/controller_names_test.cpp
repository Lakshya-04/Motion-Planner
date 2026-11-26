#include <gtest/gtest.h>

#include "manipulation_manager/core/controller_names.hpp"

namespace manipulation_manager
{

TEST(ControllerNames, DefaultsMatchTheCurrentHardCodedNames)
{
  EXPECT_EQ(
    resolveInterfaceName("", "combined_arm_lift_controller", "follow_joint_trajectory"),
    "/combined_arm_lift_controller/follow_joint_trajectory");
  EXPECT_EQ(
    resolveInterfaceName("", "combined_arm_lift_controller", "target_force_n"),
    "/combined_arm_lift_controller/target_force_n");
}

TEST(ControllerNames, LeadingAndTrailingSlashesAreNotDoubled)
{
  EXPECT_EQ(controllerInterfaceName("/ns/ctrl/", "target_force_n"), "/ns/ctrl/target_force_n");
  EXPECT_EQ(controllerInterfaceName("ctrl", "x"), "/ctrl/x");
}

TEST(ControllerNames, ExplicitOverrideWinsVerbatim)
{
  EXPECT_EQ(resolveInterfaceName("/other/force", "ctrl", "target_force_n"), "/other/force");
}

}  // namespace manipulation_manager
