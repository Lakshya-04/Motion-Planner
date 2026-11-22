#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "lift_hardware/core/profile.hpp"

namespace lift_hardware
{
namespace
{

constexpr double kVmax = 0.1;
constexpr double kAmax = 0.2;
constexpr double kDt = 0.01;

struct Trace
{
  double peak_velocity{0.0};
  double peak_acceleration{0.0};
  double overshoot{0.0};
  double time_s{0.0};
  Kinematics last;
};

Trace run(Kinematics state, double target, double seconds = 30.0)
{
  Trace t;
  const double direction = target >= state.position ? 1.0 : -1.0;
  for (double time = 0.0; time < seconds; time += kDt) {
    const Kinematics next = advance_toward(state, target, kVmax, kAmax, kDt);
    t.peak_velocity = std::max(t.peak_velocity, std::abs(next.velocity));
    t.peak_acceleration =
      std::max(t.peak_acceleration, std::abs(next.velocity - state.velocity) / kDt);
    t.overshoot = std::max(t.overshoot, (next.position - target) * direction);
    state = next;
    t.time_s = time + kDt;
    if (state.position == target && state.velocity == 0.0) {
      break;
    }
  }
  t.last = state;
  return t;
}

TEST(AdvanceToward, LongMoveCruisesAtTheVelocityLimitAndLandsExactly)
{
  const Trace t = run({0.0, 0.0}, 0.6);
  EXPECT_EQ(t.last.position, 0.6);
  EXPECT_EQ(t.last.velocity, 0.0);
  EXPECT_NEAR(t.peak_velocity, kVmax, 1e-9);
  // 0.5 s up, 0.5 s down, 0.6 - 0.05 m at 0.1 m/s in between, about 6.5 s in all.
  EXPECT_NEAR(t.time_s, 6.5, 0.15);
}

TEST(AdvanceToward, StaysNearTheLimitsForShortAndLongMovesBothWays)
{
  for (double target : {0.6, 0.02, -0.3, 0.004}) {
    const Trace t = run({0.0, 0.0}, target);
    // only the last step, which lands on the target, may exceed the limit (here by under 2x)
    EXPECT_LE(t.peak_acceleration, 2.0 * kAmax) << target;
    EXPECT_LE(t.peak_velocity, kVmax + 1e-9) << target;
  }
}

TEST(AdvanceToward, AccelerationStaysWithinTheLimitWhileTravelling)
{
  Kinematics s{0.0, 0.0};
  for (int i = 0; i < 400; ++i) {
    const Kinematics n = advance_toward(s, 0.6, kVmax, kAmax, kDt);
    EXPECT_LE(std::abs(n.velocity - s.velocity), kAmax * kDt + 1e-12);
    s = n;
  }
}

TEST(AdvanceToward, ShortMoveIsATriangleBelowTheVelocityLimit)
{
  const Trace t = run({0.0, 0.0}, 0.02);
  EXPECT_LT(t.peak_velocity, kVmax);
  EXPECT_EQ(t.last.position, 0.02);
}

TEST(AdvanceToward, DoesNotOvershootEvenWhenFastAndCloseToTheTarget)
{
  // Moving at the limit with only 1 cm left: the brake needs 2.5 cm, so it cannot stop in time,
  // but it must still never pass the target.
  const Trace t = run({0.59, kVmax}, 0.6);
  EXPECT_LE(t.overshoot, 1e-12);
  EXPECT_EQ(t.last.position, 0.6);
}

TEST(AdvanceToward, ReversalBrakesThroughZeroFirst)
{
  Kinematics s{0.3, kVmax};
  bool crossed_zero = false;
  for (int i = 0; i < 100 && !crossed_zero; ++i) {
    s = advance_toward(s, 0.0, kVmax, kAmax, kDt);
    EXPECT_LE(std::abs(s.velocity), kVmax + 1e-12);
    if (s.velocity <= 0.0) {
      crossed_zero = true;
    }
  }
  EXPECT_TRUE(crossed_zero);
  EXPECT_GT(s.position, 0.3);  // it kept moving up while braking
}

TEST(AdvanceToward, InvalidLimitsOrStepLeaveTheStateAlone)
{
  const Kinematics s{0.1, 0.02};
  EXPECT_EQ(advance_toward(s, 0.5, 0.0, kAmax, kDt).position, 0.1);
  EXPECT_EQ(advance_toward(s, 0.5, kVmax, -1.0, kDt).velocity, 0.02);
  EXPECT_EQ(advance_toward(s, 0.5, kVmax, kAmax, 0.0).position, 0.1);
}

TEST(AdvanceToward, AtRestOnTheTargetStaysThere)
{
  const Kinematics s = advance_toward({0.4, 0.0}, 0.4, kVmax, kAmax, kDt);
  EXPECT_EQ(s.position, 0.4);
  EXPECT_EQ(s.velocity, 0.0);
}

}  // namespace
}  // namespace lift_hardware
