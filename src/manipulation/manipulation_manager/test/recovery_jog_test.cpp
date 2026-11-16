#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "manipulation_manager/core/recovery_jog.hpp"

namespace manipulation_manager
{

TEST(RecoveryJogTest, AlreadyAtTheGoodStateNeedsNothing)
{
  const auto jog = planRecoveryJog({0.1, 0.2}, {0.1, 0.205}, RecoveryJogParams{});
  EXPECT_TRUE(jog.ok);
  EXPECT_FALSE(jog.needed);
  EXPECT_TRUE(jog.points.empty());
}

TEST(RecoveryJogTest, StepsAreSmallAndEndAtTheTarget)
{
  RecoveryJogParams p;
  const std::vector<double> cur{0.0, 0.0, 1.0};
  const std::vector<double> good{0.5, -0.2, 1.0};
  const auto jog = planRecoveryJog(cur, good, p);
  ASSERT_TRUE(jog.ok);
  ASSERT_TRUE(jog.needed);
  std::vector<double> prev = cur;
  double prev_t = 0.0;
  for (const auto & pt : jog.points) {
    for (size_t i = 0; i < cur.size(); ++i) {
      EXPECT_LE(std::fabs(pt.positions[i] - prev[i]), p.max_step_rad * 1.51);   // smoothstep peak slope 1.5
    }
    EXPECT_GT(pt.time_from_start_s, prev_t);
    prev = pt.positions;
    prev_t = pt.time_from_start_s;
  }
  for (size_t i = 0; i < cur.size(); ++i) {
    EXPECT_NEAR(jog.points.back().positions[i], good[i], 1e-12);
  }
}

TEST(RecoveryJogTest, SpeedStaysUnderTheLimit)
{
  RecoveryJogParams p;
  p.max_speed = 0.1;
  const auto jog = planRecoveryJog({0.0, 0.0}, {1.0, 0.0}, p);
  ASSERT_TRUE(jog.ok);
  double prev_t = 0.0;
  double prev_q = 0.0;
  for (const auto & pt : jog.points) {
    const double v = std::fabs(pt.positions[0] - prev_q) / (pt.time_from_start_s - prev_t);
    EXPECT_LE(v, p.max_speed * 1.6);   // per step; the peak slope of the ramp is 1.5 times the mean
    prev_t = pt.time_from_start_s;
    prev_q = pt.positions[0];
  }
}

TEST(RecoveryJogTest, TargetIsKeptInsideTheLimits)
{
  RecoveryJogParams p;
  p.lower = {0.0, -3.0};
  p.upper = {1.5, 3.0};
  p.limit_margin = 0.02;
  const auto jog = planRecoveryJog({-0.0015, 0.0}, {0.0, 0.0}, p);   // lift sat below its lower stop
  ASSERT_TRUE(jog.ok);
  ASSERT_TRUE(jog.needed);
  EXPECT_DOUBLE_EQ(jog.target[0], 0.02);
}

TEST(RecoveryJogTest, TooFarIsRefused)
{
  const auto jog = planRecoveryJog({0.0}, {3.0}, RecoveryJogParams{});
  EXPECT_FALSE(jog.ok);
  EXPECT_FALSE(jog.reason.empty());
}

TEST(RecoveryJogTest, SizeMismatchAndEmptyAreRefused)
{
  EXPECT_FALSE(planRecoveryJog({}, {}, RecoveryJogParams{}).ok);
  EXPECT_FALSE(planRecoveryJog({0.0}, {0.0, 1.0}, RecoveryJogParams{}).ok);
}

TEST(RecoveryJogTest, NearJointLimits)
{
  EXPECT_TRUE(nearJointLimits({0.005}, {0.0}, {1.5}, 0.01));
  EXPECT_TRUE(nearJointLimits({1.499}, {0.0}, {1.5}, 0.01));
  EXPECT_FALSE(nearJointLimits({0.7}, {0.0}, {1.5}, 0.01));
  EXPECT_FALSE(nearJointLimits({0.7}, {}, {}, 0.01));
}

}  // namespace manipulation_manager
