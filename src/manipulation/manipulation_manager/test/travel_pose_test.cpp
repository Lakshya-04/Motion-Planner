#include <vector>

#include <gtest/gtest.h>

#include "manipulation_manager/core/travel_pose.hpp"

namespace manipulation_manager
{

TEST(TravelPoseTest, LiftAtTheLowerStopMovesInByTheMargin)
{
  const auto p = keepLiftOffEndStops({0.0, 1.0, 2.0}, 0.0, 1.5, 0.01);
  ASSERT_EQ(p.size(), 3u);
  EXPECT_DOUBLE_EQ(p[0], 0.01);
  EXPECT_DOUBLE_EQ(p[1], 1.0);   // the arm joints are not touched
  EXPECT_DOUBLE_EQ(p[2], 2.0);
}

TEST(TravelPoseTest, LiftAtTheUpperStopMovesInByTheMargin)
{
  const auto p = keepLiftOffEndStops({1.5, 0.0}, 0.0, 1.5, 0.02);
  EXPECT_DOUBLE_EQ(p[0], 1.48);
}

TEST(TravelPoseTest, LiftInsideTheRangeIsUnchanged)
{
  const auto p = keepLiftOffEndStops({0.8, 0.0}, 0.0, 1.5, 0.01);
  EXPECT_DOUBLE_EQ(p[0], 0.8);
}

TEST(TravelPoseTest, UnknownUpperEndOnlyKeepsTheLowerSideOff)
{
  const auto p = keepLiftOffEndStops({5.0, 0.0}, 0.0, 0.0, 0.01);
  EXPECT_DOUBLE_EQ(p[0], 5.0);
  const auto q = keepLiftOffEndStops({-0.002, 0.0}, 0.0, 0.0, 0.01);
  EXPECT_DOUBLE_EQ(q[0], 0.01);
}

TEST(TravelPoseTest, EmptyPoseAndZeroMarginPassThrough)
{
  EXPECT_TRUE(keepLiftOffEndStops({}, 0.0, 1.5, 0.01).empty());
  const auto p = keepLiftOffEndStops({0.0, 1.0}, 0.0, 1.5, 0.0);
  EXPECT_DOUBLE_EQ(p[0], 0.0);
}

TEST(TravelPoseTest, RangeTooNarrowForTheMarginStillGivesAValue)
{
  // hi - lo < 2 * margin: the upper clamp is skipped, the lower one wins.
  const auto p = keepLiftOffEndStops({0.0}, 0.0, 0.01, 0.01);
  EXPECT_DOUBLE_EQ(p[0], 0.01);
}

}  // namespace manipulation_manager
