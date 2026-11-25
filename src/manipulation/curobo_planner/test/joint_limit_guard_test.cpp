// The joint limit guard with hand-made trajectories (no ROS, no cuRobo).
#include <gtest/gtest.h>

#include <cmath>

#include "curobo_planner/joint_limit_guard.hpp"

namespace cp = curobo_planner;
namespace mp = motion_planner;

namespace
{
mp::JointLimit lim(double v, double a)
{
  mp::JointLimit l;
  l.velocity = v;
  l.acceleration = a;
  return l;
}

// n points, dt apart, joint 0 moving `step` per point, joint 1 still
core_types::Trajectory ramp(int n, double dt, double step)
{
  core_types::Trajectory t;
  t.joint_names = {"a", "b"};
  for (int i = 0; i < n; ++i) {
    core_types::TrajectoryPoint p;
    p.positions = {i * step, 0.3};
    p.time_from_start_sec = i * dt;
    t.points.push_back(p);
  }
  return t;
}

cp::JointLimitPolicy policy(double factor = 1.0)
{
  cp::JointLimitPolicy p;
  p.limits = {{"a", lim(1.0, 100.0)}, {"b", lim(1.0, 100.0)}};
  p.safety_factor = factor;
  return p;
}
}  // namespace

TEST(JointLimitGuard, SlowTrajectoryIsUntouched)
{
  auto t = ramp(5, 0.1, 0.05);  // 0.5 rad/s, constant
  const auto before = t;
  const auto c = cp::enforceJointLimits(t, policy(0.9));
  EXPECT_TRUE(c.ok) << c.message;
  EXPECT_DOUBLE_EQ(c.time_scale, 1.0);
  EXPECT_NEAR(c.max_velocity_ratio, 0.5, 1e-9);
  for (size_t i = 0; i < t.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(t.points[i].time_from_start_sec, before.points[i].time_from_start_sec);
  }
}

TEST(JointLimitGuard, FastTrajectoryIsRetimedToTheSafetyFactor)
{
  auto t = ramp(5, 0.025, 0.05);  // 2 rad/s against 1 rad/s
  const auto c = cp::enforceJointLimits(t, policy(0.9));
  ASSERT_TRUE(c.ok) << c.message;
  EXPECT_NEAR(c.time_scale, 2.0 / 0.9, 1e-9);
  EXPECT_NEAR(t.points.back().time_from_start_sec, 0.1 * 2.0 / 0.9, 1e-9);
  EXPECT_NEAR(c.duration_after_sec, t.points.back().time_from_start_sec, 1e-12);
  // positions untouched: only the time axis moves
  EXPECT_DOUBLE_EQ(t.points[4].positions[0], 0.2);
  // re-checking the result: it now sits at the factor, nothing more to do
  const auto again = cp::enforceJointLimits(t, policy(0.9));
  EXPECT_TRUE(again.ok);
  EXPECT_DOUBLE_EQ(again.time_scale, 1.0);
  EXPECT_NEAR(again.max_velocity_ratio, 0.9, 1e-9);
}

TEST(JointLimitGuard, AccelerationLimitsTheScaleAsASquareRoot)
{
  // rest, then a jump of velocity: v = 0, 1, 1 over dt 0.1 => a = 10 rad/s^2 against 4
  core_types::Trajectory t;
  t.joint_names = {"a"};
  const double q[] = {0.0, 0.0, 0.1, 0.2};
  for (int i = 0; i < 4; ++i) {
    core_types::TrajectoryPoint p;
    p.positions = {q[i]};
    p.time_from_start_sec = i * 0.1;
    t.points.push_back(p);
  }
  cp::JointLimitPolicy p;
  p.limits = {{"a", lim(5.0, 4.0)}};
  p.safety_factor = 1.0;
  const auto c = cp::enforceJointLimits(t, p);
  ASSERT_TRUE(c.ok) << c.message;
  EXPECT_NEAR(c.max_acceleration_ratio, 10.0 / 4.0, 1e-9);
  EXPECT_NEAR(c.time_scale, std::sqrt(2.5), 1e-9);
  // the stretched trajectory meets the acceleration limit exactly
  const auto after = cp::enforceJointLimits(t, p);
  EXPECT_NEAR(after.max_acceleration_ratio, 1.0, 1e-9);
}

TEST(JointLimitGuard, RejectModeFailsWithAReasonCodeAndKeepsTheTrajectory)
{
  auto t = ramp(5, 0.025, 0.05);
  const auto before = t;
  auto p = policy(0.9);
  p.mode = cp::LimitMode::kReject;
  const auto c = cp::enforceJointLimits(t, p);
  EXPECT_FALSE(c.ok);
  EXPECT_EQ(c.reason_code, "limit_violation");
  EXPECT_DOUBLE_EQ(t.points[4].time_from_start_sec, before.points[4].time_from_start_sec);
  // reject mode lets a conforming trajectory through
  auto slow = ramp(5, 0.1, 0.05);
  EXPECT_TRUE(cp::enforceJointLimits(slow, p).ok);
}

TEST(JointLimitGuard, LimitsAreMatchedByNameNotByPosition)
{
  auto t = ramp(5, 0.025, 0.05);  // joint "a" moves at 2 rad/s
  t.joint_names = {"b", "a"};     // the moving joint is now listed second... and is named "b" at index 0
  // index 0 (moving) is "b": give "b" a generous limit and "a" the tight one => untouched
  cp::JointLimitPolicy p;
  p.limits = {{"a", lim(0.1, 100.0)}, {"b", lim(5.0, 100.0)}};
  p.safety_factor = 1.0;
  auto c = cp::enforceJointLimits(t, p);
  EXPECT_TRUE(c.ok);
  EXPECT_DOUBLE_EQ(c.time_scale, 1.0);
  // swap the limits: now the moving joint is the tight one
  p.limits = {{"a", lim(5.0, 100.0)}, {"b", lim(1.0, 100.0)}};
  c = cp::enforceJointLimits(t, p);
  EXPECT_NEAR(c.time_scale, 2.0, 1e-9);
}

TEST(JointLimitGuard, AJointWithoutALimitIsAnError)
{
  auto t = ramp(5, 0.1, 0.01);
  cp::JointLimitPolicy p;
  p.limits = {{"a", lim(1.0, 1.0)}};
  const auto c = cp::enforceJointLimits(t, p);
  EXPECT_FALSE(c.ok);
  EXPECT_EQ(c.reason_code, "limit_missing");
  EXPECT_NE(c.message.find("'b'"), std::string::npos);
}

TEST(JointLimitGuard, BadInputsAreReported)
{
  auto t = ramp(3, 0.1, 0.01);
  auto p = policy();
  p.safety_factor = 0.0;
  EXPECT_EQ(cp::enforceJointLimits(t, p).reason_code, "limits_invalid");
  p = policy();
  p.limits["a"].velocity = 0.0;
  EXPECT_EQ(cp::enforceJointLimits(t, p).reason_code, "limits_invalid");
  t.points[2].time_from_start_sec = t.points[1].time_from_start_sec;
  EXPECT_EQ(cp::enforceJointLimits(t, policy()).reason_code, "invalid_trajectory");
  auto ragged = ramp(3, 0.1, 0.01);
  ragged.points[1].positions.pop_back();
  EXPECT_EQ(cp::enforceJointLimits(ragged, policy()).reason_code, "invalid_trajectory");
}

TEST(JointLimitGuard, NoLimitsIsANoOp)
{
  auto t = ramp(5, 0.025, 0.5);
  cp::JointLimitPolicy none;
  EXPECT_FALSE(none.enabled());
  const auto c = cp::enforceJointLimits(t, none);
  EXPECT_TRUE(c.ok);
  EXPECT_DOUBLE_EQ(t.points[4].time_from_start_sec, 0.1);
}

TEST(JointLimitGuard, ProvidedVelocityVectorsAreScaledWithTime)
{
  auto t = ramp(3, 0.1, 0.01);  // slow by differences
  for (auto & p : t.points) {
    p.velocities = {3.0, 0.0};  // but the planner says 3 rad/s
    p.accelerations = {0.0, 0.0};
  }
  const auto c = cp::enforceJointLimits(t, policy(1.0));
  ASSERT_TRUE(c.ok);
  EXPECT_NEAR(c.time_scale, 3.0, 1e-9);
  EXPECT_NEAR(t.points[1].velocities[0], 1.0, 1e-9);
}

TEST(MoveItJointLimits, ParsesNamesAndSkipsUnlimitedJoints)
{
  const std::string yaml =
    "default_velocity_scaling_factor: 0.5\n"
    "default_acceleration_scaling_factor: 0.25\n"
    "joint_limits:\n"
    "  lift_joint: {has_velocity_limits: true, max_velocity: 0.2, has_acceleration_limits: true, max_acceleration: 0.5}\n"
    "  arm_1: &f {has_velocity_limits: true, max_velocity: 2.0, has_acceleration_limits: true, max_acceleration: 3.0}\n"
    "  arm_2: *f\n"
    "  half: {has_velocity_limits: true, max_velocity: 2.0, has_acceleration_limits: false}\n";
  std::map<std::string, mp::JointLimit> limits;
  std::string error;
  double vs = 0, as = 0;
  ASSERT_TRUE(cp::parseMoveItJointLimits(yaml, limits, error, &vs, &as)) << error;
  EXPECT_EQ(limits.size(), 3U);
  EXPECT_DOUBLE_EQ(limits.at("lift_joint").velocity, 0.2);
  EXPECT_DOUBLE_EQ(limits.at("arm_2").acceleration, 3.0);
  EXPECT_EQ(limits.count("half"), 0U);  // not unlimited: absent, so the guard reports it
  EXPECT_DOUBLE_EQ(vs, 0.5);
  EXPECT_DOUBLE_EQ(as, 0.25);
  EXPECT_FALSE(cp::parseMoveItJointLimits("foo: 1\n", limits, error));
  EXPECT_FALSE(cp::loadMoveItJointLimits("/nonexistent/joint_limits.yaml", limits, error));
}

TEST(MoveItJointLimits, ShippedDescriptionFileCoversTheLiftArmJoints)
{
  std::map<std::string, mp::JointLimit> limits;
  std::string error;
  ASSERT_TRUE(cp::loadMoveItJointLimits(SHIPPED_JOINT_LIMITS_YAML, limits, error)) << error;
  for (const char * j : {"lift_joint", "arm_shoulder_pan_joint", "arm_wrist_3_joint"}) {
    EXPECT_EQ(limits.count(j), 1U) << j;
  }
}

TEST(InlineJointLimits, ParsesAndRejectsMalformedEntries)
{
  std::map<std::string, mp::JointLimit> limits;
  std::string error;
  ASSERT_TRUE(cp::parseInlineJointLimits("j1:1.5:3, j2 : 0.2 : 0.5", limits, error)) << error;
  EXPECT_DOUBLE_EQ(limits.at("j2").velocity, 0.2);
  EXPECT_FALSE(cp::parseInlineJointLimits("j1:1.5", limits, error));
  EXPECT_FALSE(cp::parseInlineJointLimits("j1:abc:3", limits, error));
  EXPECT_FALSE(cp::parseInlineJointLimits("j1:0:3", limits, error));
  EXPECT_FALSE(cp::parseInlineJointLimits("j1:1:3:4:5", limits, error));  // (j1:1:3:4 = jerk 4, valid)
}

namespace
{
// n points dt apart, joint "a" follows `pos(i)`, joint "b" still
template<class F>
core_types::Trajectory shaped(int n, double dt, F pos)
{
  core_types::Trajectory t;
  t.joint_names = {"a", "b"};
  for (int i = 0; i < n; ++i) {
    core_types::TrajectoryPoint p;
    p.positions = {pos(i), 0.3};
    p.time_from_start_sec = i * dt;
    t.points.push_back(p);
  }
  return t;
}

cp::JointLimitPolicy jerkPolicy(double jerk, double factor = 1.0)
{
  auto p = policy(factor);
  p.limits["a"].jerk = jerk;
  return p;
}

// smooth cubic-ish path: 0 .. 1 over 21 points, small jerk
double smooth(int i)
{
  const double u = i / 20.0;
  return 3 * u * u - 2 * u * u * u;
}
// same path with a one-sample kink (the spike) at point 10
double spiky(int i) {return smooth(i) + (i == 10 ? 0.02 : 0.0);}
}  // namespace

TEST(JointLimitGuardJerk, JerkFreePathPasses)
{
  auto t = shaped(21, 0.1, smooth);  // peak jerk 6 / T^3 = 6 / 8 = 0.75 rad/s^3
  const auto before = t;
  const auto c = cp::enforceJointLimits(t, jerkPolicy(5.0));
  EXPECT_TRUE(c.ok) << c.message;
  EXPECT_DOUBLE_EQ(c.time_scale, 1.0);
  EXPECT_LT(c.max_jerk_ratio, 0.5);
  EXPECT_EQ(t.points.back().time_from_start_sec, before.points.back().time_from_start_sec);
}

TEST(JointLimitGuardJerk, SpikeIsRejectedWithJointAndWaypoint)
{
  auto t = shaped(21, 0.1, spiky);
  const auto before = t;
  auto p = jerkPolicy(50.0);
  p.mode = cp::LimitMode::kReject;
  const auto c = cp::enforceJointLimits(t, p);
  EXPECT_FALSE(c.ok);
  EXPECT_EQ(c.reason_code, "jerk_violation");
  EXPECT_EQ(c.jerk_joint, "a");
  EXPECT_GE(c.jerk_waypoint, 6);
  EXPECT_LE(c.jerk_waypoint, 10);
  EXPECT_NE(c.message.find("'a'"), std::string::npos) << c.message;
  EXPECT_NE(c.message.find("waypoint " + std::to_string(c.jerk_waypoint)), std::string::npos) << c.message;
  EXPECT_GT(c.max_jerk_ratio, 1.0);
  EXPECT_EQ(t.points[10].positions[0], before.points[10].positions[0]);  // untouched
  EXPECT_EQ(t.points[10].time_from_start_sec, before.points[10].time_from_start_sec);
}

TEST(JointLimitGuardJerk, RetimeScalesTimeAndEndsCompliant)
{
  auto t = shaped(21, 0.1, spiky);
  const double d0 = t.points.back().time_from_start_sec;
  const auto c = cp::enforceJointLimits(t, jerkPolicy(50.0, 0.9));
  ASSERT_TRUE(c.ok) << c.message;
  EXPECT_GT(c.time_scale, 1.0);
  EXPECT_NEAR(c.max_jerk_ratio / (c.time_scale * c.time_scale * c.time_scale), 0.9, 1e-6);  // jerk binds
  EXPECT_NEAR(t.points.back().time_from_start_sec, d0 * c.time_scale, 1e-9);
  // the guard run again on its own output finds nothing to do
  const auto again = cp::enforceJointLimits(t, jerkPolicy(50.0, 0.9));
  EXPECT_TRUE(again.ok);
  EXPECT_DOUBLE_EQ(again.time_scale, 1.0);
  EXPECT_LE(again.max_jerk_ratio, 0.9 + 1e-9);
  // positions unchanged
  EXPECT_DOUBLE_EQ(t.points[10].positions[0], spiky(10));
}

TEST(JointLimitGuardJerk, NeverShortens)
{
  auto t = shaped(21, 0.1, smooth);
  const auto c = cp::enforceJointLimits(t, jerkPolicy(1e6));
  EXPECT_TRUE(c.ok);
  EXPECT_GE(c.time_scale, 1.0);
  EXPECT_DOUBLE_EQ(t.points.back().time_from_start_sec, 2.0);
}

TEST(JointLimitGuardJerk, UnconfiguredJerkChangesNothing)
{
  // spiky path, huge v/a limits, jerk 0 (default): untouched even though the jerk is large
  auto t = shaped(21, 0.1, spiky);
  const auto before = t;
  auto p = policy(0.9);
  p.limits["a"].acceleration = 1e6;
  p.limits["a"].velocity = 10.0;
  const auto c = cp::enforceJointLimits(t, p);
  EXPECT_TRUE(c.ok) << c.message;
  EXPECT_DOUBLE_EQ(c.time_scale, 1.0);
  EXPECT_DOUBLE_EQ(c.max_jerk_ratio, 0.0);
  EXPECT_EQ(c.jerk_waypoint, -1);
  for (size_t i = 0; i < t.points.size(); ++i) {
    EXPECT_EQ(t.points[i].time_from_start_sec, before.points[i].time_from_start_sec);
  }
}

TEST(JointLimitGuardJerk, VelocityViolationKeepsItsReasonAndJerkOnlyShortTrajectoriesPass)
{
  auto fast = ramp(5, 0.1, 0.5);  // 5 rad/s, v limit 1
  auto p = jerkPolicy(1.0);
  p.mode = cp::LimitMode::kReject;
  EXPECT_EQ(cp::enforceJointLimits(fast, p).reason_code, "limit_violation");
  auto short_traj = ramp(3, 0.1, 0.01);  // fewer than 4 points: no jerk estimate
  EXPECT_TRUE(cp::enforceJointLimits(short_traj, jerkPolicy(1e-9)).ok);
}

TEST(JointLimitGuardJerk, NegativeJerkLimitIsInvalid)
{
  auto t = shaped(21, 0.1, smooth);
  EXPECT_EQ(cp::enforceJointLimits(t, jerkPolicy(-1.0)).reason_code, "limits_invalid");
}

TEST(InlineJointLimitsJerk, OptionalFourthFieldIsParsed)
{
  std::map<std::string, mp::JointLimit> m;
  std::string err;
  ASSERT_TRUE(cp::parseInlineJointLimits("a:1:2:30, b:1:2", m, err)) << err;
  EXPECT_DOUBLE_EQ(m["a"].jerk, 30.0);
  EXPECT_DOUBLE_EQ(m["b"].jerk, 0.0);
  EXPECT_FALSE(cp::parseInlineJointLimits("c:1:2:0", m, err));
  EXPECT_FALSE(cp::parseInlineJointLimits("c:1:2:3:4", m, err));
}

TEST(MoveItJointLimitsJerk, JerkReadOnlyWhenEnabled)
{
  std::map<std::string, mp::JointLimit> m;
  std::string err;
  const std::string y =
    "joint_limits:\n"
    "  x: {has_velocity_limits: true, max_velocity: 1, has_acceleration_limits: true, max_acceleration: 2,"
    " has_jerk_limits: true, max_jerk: 9}\n"
    "  y: {has_velocity_limits: true, max_velocity: 1, has_acceleration_limits: true, max_acceleration: 2,"
    " has_jerk_limits: false, max_jerk: 9}\n";
  ASSERT_TRUE(cp::parseMoveItJointLimits(y, m, err)) << err;
  EXPECT_DOUBLE_EQ(m["x"].jerk, 9.0);
  EXPECT_DOUBLE_EQ(m["y"].jerk, 0.0);
}
