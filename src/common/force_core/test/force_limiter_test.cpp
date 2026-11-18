#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <vector>
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

// ---------------------------------------------------------------------------------------------
// NormalAxisLimiter at the JTAC loop rate (100 Hz): a spring wall behind a position-controlled arm,
// the ros2_controllers admittance rule on the normal axis (offset integrates (F - D v - K x) / M with
// x = measured - reference), the 10 cm offset clamp seen in the MuJoCo retreat run. "Without" is that plant with only the
// clamp; "with" adds the limiter (F_max 15 N, 5 cm offset clamp, back-calculation, reset on retreat); the "without" clamp is 10 cm.
// ---------------------------------------------------------------------------------------------
using force_core::NormalAxisLimiter;
using force_core::NormalAxisLimiterParams;

namespace
{
struct LoopResult
{
  double peak{0.0};          // true peak force, N
  double steady{0.0};        // mean true force over the last second, N
  double steady_sd{0.0};
  double release_s{-1.0};    // time from the retreat start until the true force is below 0.5 N
};

struct Scenario
{
  double wall_k{8000.0};
  double arm_k{50000.0};          // servo stiffness of the position-controlled arm
  double noise_n{0.0};            // uniform noise amplitude on the measured force
  double contact_n{0.5};          // limiter contact threshold (above the noise amplitude)
  double target_n{0.0};           // force target (biased into the admittance input as JTAC does)
  double duration_s{6.0};
  double retreat_at_s{-1.0};      // < 0: no retreat
  // reference position into the wall, m, wall surface at 0
  std::function<double(double)> ref;
};

LoopResult runLoop(const Scenario & s, bool with_limiter)
{
  const double dt = 0.01, tau = 0.04, wall = 0.0;
  const double M = 8.0, K = 300.0, D = 2.0 * 6.0 * std::sqrt(M * K), clamp = 0.10;
  NormalAxisLimiterParams lp;
  lp.limiter.f_max_n = 15.0;
  lp.limiter.free_space_speed_mps = 0.02;
  lp.limiter.contact_force_n = s.contact_n;
  NormalAxisLimiter lim(lp);

  double p = s.ref(0.0), off = 0.0, off_v = 0.0, force = 0.0;
  unsigned seed = 12345;
  auto rnd = [&seed]() {seed = seed * 1664525u + 1013904223u; return (seed >> 8) / 8388608.0 - 1.0;};
  LoopResult r;
  std::vector<double> tail;
  double windup = 0.0;
  const int n = static_cast<int>(s.duration_s / dt);
  for (int i = 0; i < n; ++i) {
    const double t = i * dt, ref = s.ref(t);
    force = std::max(0.0, s.wall_k * (p - wall));
    const double f_meas = force + s.noise_n * rnd();
    // admittance on the normal axis; pressing reads negative, the target biases it as in JTAC
    const double f_in = -f_meas + s.target_n - (with_limiter ? windup : 0.0);
    const double acc = (f_in - D * off_v - K * (p - ref)) / M;
    off_v += acc * dt;
    off += off_v * dt;
    double desired = ref + std::clamp(off, -clamp, clamp);
    if (with_limiter) {
      desired = lim.step(dt, desired, ref, p, f_meas);
      windup = lim.windupForce();
      if (lim.resetRequested()) {off = 0.0; off_v = 0.0;}
    }
    // arm: p_dot = ((cmd - p) - F / arm_k) / tau
    // the plant runs at 1 kHz under a zero-order-hold 100 Hz command (a stiff wall needs the fine step)
    for (int sub = 0; sub < 10; ++sub) {
      force = std::max(0.0, s.wall_k * (p - wall));
      p += ((desired - p) - force / s.arm_k) / tau * (dt / 10.0);
      force = std::max(0.0, s.wall_k * (p - wall));
      r.peak = std::max(r.peak, force);
    }
    if (std::getenv("DBG") && i % std::atoi(std::getenv("DBG")) == 0) {
      std::printf("t %.2f ref %.4f p %.4f off %.4f F %.1f\n", t, ref, p, off, force);
    }
    if (t >= s.duration_s - 1.0) {tail.push_back(force);}
    if (s.retreat_at_s >= 0.0 && t >= s.retreat_at_s && r.release_s < 0.0 && force < 0.5) {
      r.release_s = t - s.retreat_at_s;
    }
  }
  double sum = 0.0;
  for (double f : tail) {sum += f;}
  r.steady = tail.empty() ? 0.0 : sum / tail.size();
  for (double f : tail) {r.steady_sd += (f - r.steady) * (f - r.steady);}
  r.steady_sd = tail.empty() ? 0.0 : std::sqrt(r.steady_sd / tail.size());
  return r;
}
}  // namespace

// The arm's servo stiffness against a force: a stiff industrial arm and a soft one (the MuJoCo UR10e
// position actuators give about 300 N/m: 47 N with the command 15 cm past the wall).
constexpr double kStiffArm = 50000.0;
constexpr double kSoftArm = 320.0;

TEST(NormalAxisLimiter, StepApproachPeakIsBounded)
{
  for (double arm_k : {kStiffArm, kSoftArm}) {
    Scenario s;
    s.arm_k = arm_k;
    s.ref = [](double t) {return t < 0.5 ? -0.05 : 0.10;};  // 15 cm step, ends 10 cm into the wall
    const auto off = runLoop(s, false), on = runLoop(s, true);
    std::printf("step approach, arm %.0f N/m: peak without %.1f N, with %.1f N\n", arm_k, off.peak, on.peak);
    EXPECT_GT(off.peak, 25.0);
    EXPECT_LT(on.peak, 16.5);
  }
}

TEST(NormalAxisLimiter, RampApproachPeakIsBounded)
{
  for (double arm_k : {kStiffArm, kSoftArm}) {
    Scenario s;
    s.arm_k = arm_k;
    s.ref = [](double t) {return -0.05 + 0.05 * t;};  // 5 cm/s through the wall
    s.duration_s = 3.0;
    const auto off = runLoop(s, false), on = runLoop(s, true);
    std::printf("ramp approach, arm %.0f N/m: peak without %.1f N, with %.1f N\n", arm_k, off.peak, on.peak);
    EXPECT_GT(off.peak, 30.0);
    EXPECT_LT(on.peak, 16.5);
  }
}

// Force noise and a stiff wall. The planned approach speed is passed through before contact, so the first
// contact costs about k_eff * v * dt on top of the limit: 20 kN/m (14 kN/m in series with the stiff arm)
// at 5 cm/s is 7 N a step. At the limiter's own 2 cm/s the bound holds within 2.5 N.
TEST(NormalAxisLimiter, BoundHoldsWithForceNoiseAndAStiffWall)
{
  for (double arm_k : {kStiffArm, kSoftArm}) {
    for (double speed : {0.05, 0.02}) {
      Scenario s;
      s.arm_k = arm_k;
      s.noise_n = 0.6;
      s.contact_n = 1.5;
      s.wall_k = 20000.0;
      s.ref = [speed](double t) {return -0.05 + speed * t;};
      s.duration_s = 0.05 / speed + 2.0;
      const auto off = runLoop(s, false), on = runLoop(s, true);
      std::printf(
        "noise 0.6 N, 20 kN/m wall, arm %.0f N/m, %.0f cm/s: peak without %.1f N, with %.1f N\n",
        arm_k, speed * 100.0, off.peak, on.peak);
      EXPECT_LT(on.peak, speed > 0.03 ? 28.0 : 17.5);
      EXPECT_LE(on.peak, off.peak + 0.5);
      if (speed > 0.03 && arm_k > 1000.0) {EXPECT_LT(on.peak, 0.6 * off.peak);}
    }
  }
}

TEST(NormalAxisLimiter, TenNewtonHoldIsNotDisturbed)
{
  for (double arm_k : {kStiffArm, kSoftArm}) {
    Scenario s;
    s.arm_k = arm_k;
    s.target_n = 10.0;
    s.noise_n = 0.3;
    s.ref = [](double t) {return t < 1.0 ? -0.02 + 0.02 * t : 0.0;};  // arrive at the wall, hold the reference there
    s.duration_s = 8.0;
    const auto off = runLoop(s, false), on = runLoop(s, true);
    std::printf(
      "10 N hold, arm %.0f N/m: without peak %.1f steady %.2f sd %.2f; with peak %.1f steady %.2f sd %.2f\n",
      arm_k, off.peak, off.steady, off.steady_sd, on.peak, on.steady, on.steady_sd);
    EXPECT_NEAR(on.steady, off.steady, 1.0);
    EXPECT_LT(on.peak, 16.5);
  }
}

TEST(NormalAxisLimiter, RetreatReleasesFasterThanTheCompliantOffset)
{
  Scenario s;
  s.arm_k = kSoftArm;
  s.target_n = 32.0;  // the MuJoCo run: a 47 N push, the offset ending 0.1 m past the reference
  s.ref = [](double t) {return t < 2.0 ? -0.05 + 0.05 * t : (t < 3.0 ? 0.05 : -0.03);};  // 5 cm past, then 3 cm short
  s.retreat_at_s = 3.0;
  s.duration_s = 8.0;
  const auto off = runLoop(s, false), on = runLoop(s, true);
  std::printf("retreat: peak without %.1f N (release %.2f s), with %.1f N (release %.2f s)\n",
    off.peak, off.release_s, on.peak, on.release_s);
  ASSERT_GE(on.release_s, 0.0);
  EXPECT_LT(on.release_s, 0.5);
  EXPECT_TRUE(off.release_s < 0.0 || off.release_s > 1.0);
}

TEST(NormalAxisLimiter, RejectsBadParameters)
{
  NormalAxisLimiterParams p;
  p.max_offset_m = 0.0;
  EXPECT_THROW(NormalAxisLimiter{p}, std::invalid_argument);
}
