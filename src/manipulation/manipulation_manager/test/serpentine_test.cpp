#include <gtest/gtest.h>

#include <string>

#include "manipulation_manager/core/serpentine.hpp"

using namespace manipulation_manager;

namespace
{
core_types::Pose at(double x, double y, double z)
{
  core_types::Pose p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.qw = 1.0;
  return p;
}
}  // namespace

TEST(Serpentine, RetractIsAtTheStrokeEndOnlyWhenSerpentine)
{
  const auto standoff = at(0.5, 0.2, 1.4);
  const std::vector<core_types::Pose> up{at(0.6, 0.2, 1.2), at(0.6, 0.2, 1.8)};
  const std::vector<core_types::Pose> down{at(0.6, 0.2, 1.8), at(0.6, 0.2, 1.2)};
  EXPECT_DOUBLE_EQ(chooseRetractPose(true, standoff, up).z, 1.8);
  EXPECT_DOUBLE_EQ(chooseRetractPose(true, standoff, down).z, 1.2);
  EXPECT_DOUBLE_EQ(chooseRetractPose(false, standoff, up).z, 1.4);
  const auto r = chooseRetractPose(true, standoff, up);
  EXPECT_DOUBLE_EQ(r.x, 0.5);
  EXPECT_DOUBLE_EQ(r.y, 0.2);
  EXPECT_DOUBLE_EQ(r.qw, 1.0);
}

TEST(Serpentine, EmptyStrokeKeepsTheStandoff)
{
  EXPECT_DOUBLE_EQ(chooseRetractPose(true, at(0, 0, 1.4), {}).z, 1.4);
}

TEST(Serpentine, RetractHints)
{
  EXPECT_EQ(std::string(retractModeHint(true)), "cartesian");
  EXPECT_EQ(std::string(retractModeHint(false)), "fsp");
  EXPECT_EQ(std::string(retractFallbackHint()), "fsp");
}

TEST(Serpentine, StowLowersOnlyWhenSerpentineValidAndHigh)
{
  const auto last = at(0.5, 0.2, 1.8);
  const auto d = decideStowLower(true, true, last, 1.2);
  EXPECT_TRUE(d.lower);
  EXPECT_DOUBLE_EQ(d.target.z, 1.2);
  EXPECT_DOUBLE_EQ(d.target.x, 0.5);
  EXPECT_FALSE(decideStowLower(false, true, last, 1.2).lower);
  EXPECT_FALSE(decideStowLower(true, false, last, 1.2).lower);
}

TEST(Serpentine, StowDoesNotLowerWhenAlreadyLow)
{
  EXPECT_FALSE(decideStowLower(true, true, at(0, 0, 1.205), 1.2).lower);
  EXPECT_FALSE(decideStowLower(true, true, at(0, 0, 1.1), 1.2).lower);
  EXPECT_TRUE(decideStowLower(true, true, at(0, 0, 1.25), 1.2).lower);
}
