#include <cmath>

#include <gtest/gtest.h>

#include "manipulation_manager/core/base_stance_check.hpp"

namespace manipulation_manager
{

namespace
{
CoverageSegmentIntent column(double x, double y, double z0, double z1)
{
  CoverageSegmentIntent c;
  core_types::Pose a;
  a.x = x; a.y = y; a.z = z0;
  core_types::Pose b = a;
  b.z = z1;
  c.poses = {a, b};
  return c;
}
}  // namespace

TEST(StanceWallTest, FlattensEveryPoseAndTakesTheMeanWallDepth)
{
  const auto wall = stanceWall({column(-0.2, 0.8, 0.5, 1.5), column(0.2, 1.0, 0.5, 1.5)});
  EXPECT_EQ(wall.targets.size(), 4U);
  EXPECT_NEAR(wall.wall_y, 0.9, 1e-12);
  EXPECT_NEAR(wall.wall_x, 0.0, 1e-12);
  EXPECT_NEAR(wall.wall_z, 1.0, 1e-12);
}

TEST(StanceWallTest, EmptyPlanHasNoTargets)
{
  EXPECT_TRUE(stanceWall({}).targets.empty());
}

TEST(StanceVerdictTest, OneStandWhereTheBaseIsPasses)
{
  const auto v = evaluateStance({{0.05, -0.03, 0.02}}, 0, 0);
  EXPECT_TRUE(v.ok) << v.message;
}

TEST(StanceVerdictTest, OneStandElsewhereSaysWhereTheBaseWouldHaveToBe)
{
  const auto v = evaluateStance({{0.6, 0.0, 0.0}}, 0, 0);
  EXPECT_FALSE(v.ok);
  EXPECT_NE(v.message.find("would have to move"), std::string::npos) << v.message;
}

TEST(StanceVerdictTest, WrongYawFailsEvenWhenInPlace)
{
  EXPECT_FALSE(evaluateStance({{0.0, 0.0, 0.5}}, 0, 0).ok);
  EXPECT_TRUE(evaluateStance({{0.0, 0.0, 2.0 * M_PI + 0.01}}, 0, 0).ok);  // wraps around
}

TEST(StanceVerdictTest, TheBaseCanBeAnywhereInAWorldFrame)
{
  const StanceTolerance tol;
  const StanceStand here{-0.12, -0.25, 1.57};
  EXPECT_TRUE(evaluateStance({{-0.1, -0.24, 1.55}}, 0, 0, tol, here).ok);
  EXPECT_FALSE(evaluateStance({{-0.1, -0.24, 1.55}}, 0, 0, tol).ok);  // the origin is the default
  EXPECT_FALSE(evaluateStance({{-0.12, 0.3, 1.57}}, 0, 0, tol, here).ok);
  EXPECT_FALSE(evaluateStance({{-0.12, -0.25, 0.0}}, 0, 0, tol, here).ok);
}

TEST(StanceVerdictTest, SeveralStandsMeanOneStanceIsNotEnough)
{
  const auto v = evaluateStance({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}, 0, 0);
  EXPECT_FALSE(v.ok);
  EXPECT_NE(v.message.find("2 stands"), std::string::npos) << v.message;
}

TEST(StanceVerdictTest, UnreachableOrUnplannedTargetsFail)
{
  EXPECT_FALSE(evaluateStance({{0.0, 0.0, 0.0}}, 1, 0).ok);
  EXPECT_FALSE(evaluateStance({{0.0, 0.0, 0.0}}, 0, 2).ok);
  EXPECT_FALSE(evaluateStance({}, 0, 0).ok);
}

}  // namespace manipulation_manager
