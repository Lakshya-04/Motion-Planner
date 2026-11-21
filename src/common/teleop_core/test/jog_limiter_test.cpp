#include <gtest/gtest.h>

#include <cmath>

#include "teleop_core/jog_limiter.hpp"

using namespace teleop_core;  // NOLINT

namespace
{
Vector6d tw()
{
  Vector6d v;
  v << 0.1, -0.2, 0.05, 0.3, 0.0, -0.1;
  return v;
}
}  // namespace

TEST(JogLimiter, NoScaleReceivedFailsSafe)
{
  JogLimiter l;
  auto o = l.limit(tw(), 0.0);
  EXPECT_EQ(o.twist.norm(), 0.0);
  EXPECT_EQ(o.reason, LimitReason::kStaleScale);
}

TEST(JogLimiter, FullScalePassesThrough)
{
  JogLimiter l;
  l.setSpeedScale(1.0, 0.0);
  auto o = l.limit(tw(), 0.1);
  EXPECT_EQ(o.twist, tw());
  EXPECT_EQ(o.reason, LimitReason::kNone);
}

TEST(JogLimiter, UniformScalingPreservesDirection)
{
  JogLimiter l;
  l.setSpeedScale(0.4, 0.0);
  auto o = l.limit(tw(), 0.1);
  EXPECT_EQ(o.reason, LimitReason::kScaled);
  EXPECT_NEAR(o.twist.norm(), 0.4 * tw().norm(), 1e-12);
  EXPECT_NEAR(o.twist.normalized().dot(tw().normalized()), 1.0, 1e-12);
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(o.twist[i], 0.4 * tw()[i], 1e-15);
  }
}

TEST(JogLimiter, ZeroScaleGivesZero)
{
  JogLimiter l;
  l.setSpeedScale(0.0, 0.0);
  EXPECT_EQ(l.limit(tw(), 0.1).twist.norm(), 0.0);
}

TEST(JogLimiter, ScaleIsClampedToUnitInterval)
{
  JogLimiter l;
  l.setSpeedScale(3.0, 0.0);
  EXPECT_EQ(l.limit(tw(), 0.1).twist, tw());
  l.setSpeedScale(-1.0, 0.0);
  EXPECT_EQ(l.limit(tw(), 0.1).twist.norm(), 0.0);
}

TEST(JogLimiter, StopForcesZeroEvenWithFreshFullScale)
{
  JogLimiter l;
  l.setSpeedScale(1.0, 0.0);
  l.setStop(true);
  auto o = l.limit(tw(), 0.1);
  EXPECT_EQ(o.twist.norm(), 0.0);
  EXPECT_EQ(o.reason, LimitReason::kStop);
  l.setStop(false);
  EXPECT_EQ(l.limit(tw(), 0.1).twist, tw());
}

TEST(JogLimiter, StaleScaleFailsSafe)
{
  JogLimiterConfig c;
  c.stale_timeout_s = 0.5;
  JogLimiter l(c);
  l.setSpeedScale(1.0, 0.0);
  EXPECT_EQ(l.limit(tw(), 0.4).twist, tw());
  auto o = l.limit(tw(), 0.6);
  EXPECT_EQ(o.twist.norm(), 0.0);
  EXPECT_EQ(o.reason, LimitReason::kStaleScale);
  l.setSpeedScale(1.0, 0.6);
  EXPECT_EQ(l.limit(tw(), 0.7).twist, tw());
}

TEST(JogLimiter, NanScaleInvalidatesPreviousGoodScale)
{
  JogLimiter l;
  l.setSpeedScale(1.0, 0.0);
  l.setSpeedScale(std::nan(""), 0.1);
  EXPECT_EQ(l.limit(tw(), 0.15).twist.norm(), 0.0);
}

TEST(JogLimiter, NonFiniteTwistOrTimeIsZero)
{
  JogLimiter l;
  l.setSpeedScale(1.0, 0.0);
  Vector6d bad = tw();
  bad[2] = std::nan("");
  auto o = l.limit(bad, 0.1);
  EXPECT_EQ(o.twist.norm(), 0.0);
  EXPECT_EQ(o.reason, LimitReason::kInvalidTwist);
  EXPECT_EQ(l.limit(tw(), std::nan("")).twist.norm(), 0.0);
}

TEST(JogLimiter, TimeGoingBackwardsFailsSafe)
{
  JogLimiter l;
  l.setSpeedScale(1.0, 10.0);
  EXPECT_EQ(l.limit(tw(), 5.0).twist.norm(), 0.0);
}

TEST(JogLimiter, ShaperThenLimiterEndToEndAndDeterministic)
{
  auto run = []() {
      JogShaperConfig c;
      c.max_linear_accel = 0.0;
      JogShaper s(c);
      JogLimiter l;
      l.setSpeedScale(0.5, 0.0);
      Vector6d in = Vector6d::Zero();
      in[0] = 1.0;
      s.setInput(in, 0.0);
      return l.limit(s.update(0.01, 0.01).twist, 0.01).twist;
    };
  EXPECT_EQ(run(), run());
  EXPECT_NEAR(run()[0], 0.05, 1e-12);
}
