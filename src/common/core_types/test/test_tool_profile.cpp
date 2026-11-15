/**
 * @file test_tool_profile.cpp
 * @brief Unit tests for core_types::ToolProfile and the sprayer swath model.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <cmath>
#include <stdexcept>

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

TEST(SprayerEffectiveSwath, MatchesConeFormula)
{
  core_types::SpraySwathModel model;
  model.half_angle_deg = 30.0;
  model.effective_width_fraction = 0.5;
  const double expected = 0.5 * 2.0 * 0.2 * std::tan(30.0 * M_PI / 180.0);
  EXPECT_NEAR(core_types::sprayerEffectiveSwath(0.2, model), expected, 1e-12);
  EXPECT_NEAR(core_types::sprayGeometricWidth(0.2, 45.0), 0.4, 1e-12);
}

TEST(SprayerEffectiveSwath, IsLinearAndStrictlyMonotonicInStandoff)
{
  double previous = 0.0;
  for (double d = 0.05; d < 0.6; d += 0.05) {
    const double w = core_types::sprayerEffectiveSwath(d);
    EXPECT_GT(w, previous);
    EXPECT_NEAR(w / d, core_types::sprayerEffectiveSwath(0.1) / 0.1, 1e-12);
    previous = w;
  }
}

TEST(SprayerEffectiveSwath, RejectsNonPositiveOrOutOfRangeInputs)
{
  EXPECT_THROW(core_types::sprayerEffectiveSwath(0.0), std::invalid_argument);
  EXPECT_THROW(core_types::sprayerEffectiveSwath(-0.1), std::invalid_argument);
  core_types::SpraySwathModel bad;
  bad.half_angle_deg = 0.0;
  EXPECT_THROW(core_types::sprayerEffectiveSwath(0.2, bad), std::invalid_argument);
  bad.half_angle_deg = 90.0;
  EXPECT_THROW(core_types::sprayerEffectiveSwath(0.2, bad), std::invalid_argument);
  bad = {};
  bad.effective_width_fraction = 0.0;
  EXPECT_THROW(core_types::sprayerEffectiveSwath(0.2, bad), std::invalid_argument);
  bad.effective_width_fraction = 1.5;
  EXPECT_THROW(core_types::sprayerEffectiveSwath(0.2, bad), std::invalid_argument);
}

TEST(LineSpacing, FollowsSwathAndOverlap)
{
  EXPECT_NEAR(core_types::lineSpacingFromSwath(0.2, 0.0), 0.2, 1e-12);
  EXPECT_NEAR(core_types::lineSpacingFromSwath(0.2, 30.0), 0.14, 1e-12);
  EXPECT_THROW(core_types::lineSpacingFromSwath(0.0, 0.0), std::invalid_argument);
  EXPECT_THROW(core_types::lineSpacingFromSwath(-0.2, 10.0), std::invalid_argument);
  EXPECT_THROW(core_types::lineSpacingFromSwath(0.2, -1.0), std::invalid_argument);
  EXPECT_THROW(core_types::lineSpacingFromSwath(0.2, 100.0), std::invalid_argument);
}
