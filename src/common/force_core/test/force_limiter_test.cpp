#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "force_core/force_limiter.hpp"

using force_core::ForceLimiter;
using force_core::ForceLimiterParams;

namespace
{
struct Wall
{
  double x0{0.10};    // wall surface, m
  double k{8000.0};   // N/m
  double force(double x) const {return x > x0 ? k * (x - x0) : 0.0;}
};

// Drive the tool towards the wall at v_cmd for `steps` ms with or without the limiter; returns the peak force.
double drive(const Wall & wall, double v_cmd, int steps, ForceLimiter * limiter, double * final_force = nullptr)
{
  const double dt = 0.001;
  double x = 0.0, peak = 0.0, f = 0.0;
  for (int i = 0; i < steps; ++i) {
    f = wall.force(x);
    if (limiter) {limiter->update(x, f);}
    const double v = limiter ? limiter->limit(v_cmd, f) : v_cmd;
    x += v * dt;
    peak = std::max(peak, wall.force(x));
  }
  if (final_force) {*final_force = wall.force(x);}
  return peak;
}
}  // namespace

TEST(ForceLimiter, WithoutItAFastApproachBlowsPastTheLimit)
{
  EXPECT_GT(drive(Wall{}, 0.2, 2000, nullptr), 100.0);
}

TEST(ForceLimiter, ForceNeverCrossesTheLimitAndSettlesBelowIt)
{
  ForceLimiterParams p;
  p.f_max_n = 15.0;
  ForceLimiter limiter(p);
  double final_f = 0.0;
  const double peak = drive(Wall{}, 0.2, 9000, &limiter, &final_f);
  EXPECT_LE(peak, 15.0 + 1e-6);
  EXPECT_GT(final_f, 13.0);   // it does press, it is not just frozen
}

TEST(ForceLimiter, HoldsForVeryStiffAndVerySoftWalls)
{
  for (double k : {1000.0, 8000.0, 40000.0}) {
    ForceLimiter limiter;
    Wall w;
    w.k = k;
    EXPECT_LE(drive(w, 0.3, 9000, &limiter), 15.0 + 1e-6) << "k=" << k;
  }
}

TEST(ForceLimiter, StiffnessEstimateConvergesOnAMovingTool)
{
  ForceLimiter limiter;
  EXPECT_FALSE(limiter.hasEstimate());
  EXPECT_DOUBLE_EQ(limiter.stiffness(), ForceLimiterParams{}.k_max_n_per_m);   // conservative prior
  Wall w;
  w.k = 12000.0;
  double x = 0.099;
  for (int i = 0; i < 200; ++i) {
    limiter.update(x, w.force(x));
    x += 0.0001;   // 0.1 mm steps into the wall
  }
  EXPECT_TRUE(limiter.hasEstimate());
  EXPECT_NEAR(limiter.stiffness(), 12000.0, 120.0);
}

TEST(ForceLimiter, AboveTheLimitItForcesARetreat)
{
  ForceLimiter limiter;
  const double v = limiter.limit(0.05, 20.0);   // 5 N over the limit, asked to push further
  EXPECT_LT(v, 0.0);
  EXPECT_LT(limiter.maxApproachVelocity(20.0), 0.0);
}

TEST(ForceLimiter, RetreatAndFreeSpaceMotionPassThrough)
{
  ForceLimiter limiter;
  EXPECT_DOUBLE_EQ(limiter.limit(-0.1, 5.0), -0.1);   // retreating is never held back
  EXPECT_DOUBLE_EQ(limiter.limit(0.01, 0.0), 0.01);   // slow approach in free space is allowed
  EXPECT_DOUBLE_EQ(limiter.limit(5.0, 0.0), ForceLimiterParams{}.free_space_speed_mps);   // a huge command is capped
}

TEST(ForceLimiter, ResetForgetsTheEstimate)
{
  ForceLimiter limiter;
  Wall w;
  double x = 0.099;
  for (int i = 0; i < 50; ++i) {limiter.update(x, w.force(x)); x += 0.0001;}
  ASSERT_TRUE(limiter.hasEstimate());
  limiter.reset();
  EXPECT_FALSE(limiter.hasEstimate());
}

TEST(ForceLimiter, RejectsBadParameters)
{
  ForceLimiterParams p;
  p.f_max_n = 0.0;
  EXPECT_THROW(ForceLimiter{p}, std::invalid_argument);
  p = {};
  p.forgetting = 1.5;
  EXPECT_THROW(ForceLimiter{p}, std::invalid_argument);
  p = {};
  p.free_space_speed_mps = 0.0;
  EXPECT_THROW(ForceLimiter{p}, std::invalid_argument);
  p = {};
  p.k_max_n_per_m = 100.0;   // below k_min
  EXPECT_THROW(ForceLimiter{p}, std::invalid_argument);
}
