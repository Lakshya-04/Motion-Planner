/**
 * @file test_tool_profile.cpp
 * @brief Unit tests for core_types::ToolProfile and the sprayer swath placeholder.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <gtest/gtest.h>

#include "core_types/sprayer_effective_swath.hpp"
#include "core_types/tool_profile.hpp"

TEST(ToolProfile, DefaultsAreSafeForContactTools)
{
  core_types::ToolProfile profile;
  EXPECT_FALSE(profile.task.contact);
  EXPECT_DOUBLE_EQ(profile.task.reachability_tilt_deg_max, 0.0);
  EXPECT_EQ(profile.task.tool_normal_policy, "constant");
  EXPECT_EQ(profile.control.controller, "jtc_only");
}

TEST(ToolProfile, DefaultCoverageHasExactlyOnePass)
{
  core_types::ToolProfile profile;
  ASSERT_EQ(profile.coverage.passes.size(), 1u);
  EXPECT_DOUBLE_EQ(profile.coverage.passes[0].standoff_delta_m, 0.0);
  EXPECT_FALSE(profile.coverage.passes[0].reverse_direction);
}

TEST(ToolProfile, PassListGeneralizesToMultiStandoffStrategy)
{
  core_types::ToolProfile profile;
  profile.coverage.passes = {
    {-0.05, false},
    {0.0, false},
    {0.05, false},
  };
  ASSERT_EQ(profile.coverage.passes.size(), 3u);
  EXPECT_DOUBLE_EQ(profile.coverage.passes[0].standoff_delta_m, -0.05);
  EXPECT_DOUBLE_EQ(profile.coverage.passes[2].standoff_delta_m, 0.05);
}

TEST(ToolProfile, MopStyleDoublePassIsForwardThenReversedSameStandoff)
{
  core_types::ToolProfile profile;
  profile.coverage.passes = {
    {0.0, false},
    {0.0, true},
  };
  ASSERT_EQ(profile.coverage.passes.size(), 2u);
  EXPECT_FALSE(profile.coverage.passes[0].reverse_direction);
  EXPECT_TRUE(profile.coverage.passes[1].reverse_direction);
}

TEST(SprayerEffectiveSwath, IsMonotonicInStandoff)
{
  const double swath_near = core_types::sprayerEffectiveSwath(0.10);
  const double swath_far = core_types::sprayerEffectiveSwath(0.20);
  EXPECT_LT(swath_near, swath_far);
}
