#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "teleop_core/jog_shaper.hpp"

using namespace teleop_core;  // NOLINT

namespace
{
JogShaperConfig plain()
{
  JogShaperConfig c;
  c.deadband.fill(0.1);
  c.expo = 0.0;
  c.max_linear_mps = 0.2;
  c.max_angular_rps = 1.0;
  c.max_linear_accel = 0.0;
  c.max_angular_accel = 0.0;
  c.watchdog_s = 0.25;
  return c;
}
Vector6d in(double x, double y = 0, double z = 0, double rx = 0, double ry = 0, double rz = 0)
{
  Vector6d v;
  v << x, y, z, rx, ry, rz;
  return v;
}
}  // namespace

TEST(JogShaper, DeadbandZeroesSmallInput)
{
  JogShaper s(plain());
  s.setInput(in(0.09, -0.1), 0.0);
  EXPECT_EQ(s.update(0.01, 0.01).twist.norm(), 0.0);
}

TEST(JogShaper, DeadbandRescalesSoFullDeflectionIsMax)
{
  JogShaper s(plain());
  s.setInput(in(1.0, 0.55), 0.0);
  auto o = s.update(0.01, 0.01);
  // Per-axis shaped values are 1.0 and (0.55-0.1)/0.9 = 0.5; the linear norm is then clamped to 0.2.
  const double k = 0.2 / std::sqrt(1.0 + 0.25);
  EXPECT_NEAR(o.twist[0], k, 1e-12);
  EXPECT_NEAR(o.twist[1], k * 0.5, 1e-12);
  EXPECT_NEAR(o.twist.head<3>().norm(), 0.2, 1e-12);
}

TEST(JogShaper, ExpoCurveNumbers)
{
  auto c = plain();
  c.deadband.fill(0.0);
  c.expo = 0.5;
  JogShaper s(c);
  s.setInput(in(0.5), 0.0);
  EXPECT_NEAR(s.update(0.01, 0.01).twist[0], 0.2 * (0.5 * 0.5 + 0.5 * 0.125), 1e-12);
}

TEST(JogShaper, ExpoIsOddSymmetric)
{
  auto c = plain();
  c.expo = 0.7;
  JogShaper a(c), b(c);
  a.setInput(in(0.6), 0.0);
  b.setInput(in(-0.6), 0.0);
  EXPECT_NEAR(a.update(0.01, 0.01).twist[0], -b.update(0.01, 0.01).twist[0], 1e-15);
}

TEST(JogShaper, SaturationClampsAndFlags)
{
  JogShaper s(plain());
  s.setInput(in(5.0, 0, 0, 0, 0, -3.0), 0.0);
  auto o = s.update(0.01, 0.01);
  EXPECT_TRUE(o.saturated);
  EXPECT_NEAR(o.twist[0], 0.2, 1e-12);
  EXPECT_NEAR(o.twist[5], -1.0, 1e-12);
}

TEST(JogShaper, AngularAndLinearUseTheirOwnMax)
{
  JogShaper s(plain());
  s.setInput(in(1, 0, 0, 1, 0, 0), 0.0);
  auto o = s.update(0.01, 0.01);
  EXPECT_NEAR(o.twist[0], 0.2, 1e-12);
  EXPECT_NEAR(o.twist[3], 1.0, 1e-12);
}

TEST(JogShaper, MaskedAxisIsZeroOthersUnchanged)
{
  auto c = plain();
  c.enabled[2] = false;  // contact normal
  JogShaper s(c);
  s.setInput(in(1, 1, 1), 0.0);
  auto o = s.update(0.01, 0.01);
  EXPECT_EQ(o.twist[2], 0.0);
  EXPECT_NEAR(o.twist[0], 0.2 / std::sqrt(2.0), 1e-12);  // norm clamp keeps the direction
  EXPECT_NEAR(o.twist[1], 0.2 / std::sqrt(2.0), 1e-12);
}

TEST(JogShaper, MaskCanChangeAtRunTime)
{
  JogShaper s(plain());
  s.setInput(in(0, 0, 1), 0.0);
  EXPECT_NEAR(s.update(0.01, 0.01).twist[2], 0.2, 1e-12);
  s.setEnabled({true, true, false, true, true, true});
  EXPECT_EQ(s.update(0.02, 0.01).twist[2], 0.0);
}

TEST(JogShaper, SlewLimitsLinearRamp)
{
  auto c = plain();
  c.max_linear_accel = 1.0;  // 0.01 m/s per 10 ms
  JogShaper s(c);
  s.setInput(in(1.0), 0.0);
  EXPECT_NEAR(s.update(0.01, 0.01).twist[0], 0.01, 1e-12);
  EXPECT_NEAR(s.update(0.02, 0.01).twist[0], 0.02, 1e-12);
  double v = 0;
  for (int i = 3; i < 200; ++i) {
    s.setInput(in(1.0), 0.01 * i);
    v = s.update(0.01 * i, 0.01).twist[0];
  }
  EXPECT_NEAR(v, 0.2, 1e-12);  // reaches the target and stops there
}

TEST(JogShaper, SlewAppliesToDecelerationToo)
{
  auto c = plain();
  c.max_linear_accel = 1.0;
  JogShaper s(c);
  s.setInput(in(1.0), 0.0);
  for (int i = 1; i <= 100; ++i) {
    s.setInput(in(1.0), 0.01 * i);
    s.update(0.01 * i, 0.01);
  }
  s.setInput(in(0.0), 1.01);
  EXPECT_NEAR(s.update(1.01, 0.01).twist[0], 0.19, 1e-12);
}

TEST(JogShaper, SlewKeepsDirection)
{
  auto c = plain();
  c.max_linear_accel = 1.0;
  JogShaper s(c);
  s.setInput(in(1.0, 1.0), 0.0);
  auto o = s.update(0.01, 0.01);
  EXPECT_NEAR(o.twist[0], o.twist[1], 1e-15);
  EXPECT_NEAR(o.twist.head<3>().norm(), 0.01, 1e-12);
}

TEST(JogShaper, WatchdogZerosImmediatelyAndFlags)
{
  JogShaper s(plain());
  s.setInput(in(1.0), 0.0);
  EXPECT_FALSE(s.update(0.2, 0.01).watchdog_tripped);
  auto o = s.update(0.3, 0.01);
  EXPECT_TRUE(o.watchdog_tripped);
  EXPECT_EQ(o.twist.norm(), 0.0);
}

TEST(JogShaper, WatchdogWithNoInputEver)
{
  JogShaper s(plain());
  EXPECT_TRUE(s.update(0.0, 0.01).watchdog_tripped);
}

TEST(JogShaper, WatchdogResetsSlewAndRecovers)
{
  auto c = plain();
  c.max_linear_accel = 1.0;
  JogShaper s(c);
  s.setInput(in(1.0), 0.0);
  for (int i = 1; i <= 20; ++i) {
    s.setInput(in(1.0), 0.01 * i);
    s.update(0.01 * i, 0.01);
  }
  EXPECT_TRUE(s.update(1.0, 0.01).watchdog_tripped);
  s.setInput(in(1.0), 1.01);
  auto o = s.update(1.01, 0.01);
  EXPECT_FALSE(o.watchdog_tripped);
  EXPECT_NEAR(o.twist[0], 0.01, 1e-12);  // ramps again from zero
}

TEST(JogShaper, NanInputIsZeroInvalidAndDoesNotFeedWatchdog)
{
  JogShaper s(plain());
  s.setInput(in(1.0), 0.0);
  s.setInput(in(std::nan("")), 0.2);
  auto o = s.update(0.3, 0.01);
  EXPECT_TRUE(o.invalid_input);
  EXPECT_TRUE(o.watchdog_tripped);  // the NaN sample at 0.2 did not refresh; last valid was at 0.0
  EXPECT_EQ(o.twist.norm(), 0.0);
}

TEST(JogShaper, ToolFrameRotatesIntoBase)
{
  auto c = plain();
  c.frame = JogFrame::kTool;
  JogShaper s(c);
  FrameRotations r;
  r.tool_in_base = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  s.setInput(in(1.0), 0.0);
  auto o = s.update(0.01, 0.01, r);
  EXPECT_NEAR(o.twist[0], 0.0, 1e-12);
  EXPECT_NEAR(o.twist[1], 0.2, 1e-12);
}

TEST(JogShaper, WorldFrameUsesWorldRotationAndBaseIgnoresRotations)
{
  FrameRotations r;
  r.tool_in_base = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  r.world_in_base = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto c = plain();
  c.frame = JogFrame::kWorld;
  JogShaper w(c);
  w.setInput(in(1.0), 0.0);
  EXPECT_NEAR(w.update(0.01, 0.01, r).twist[0], -0.2, 1e-12);
  c.frame = JogFrame::kBase;
  JogShaper b(c);
  b.setInput(in(1.0), 0.0);
  EXPECT_NEAR(b.update(0.01, 0.01, r).twist[0], 0.2, 1e-12);
}

TEST(JogShaper, DeterministicForSameInputs)
{
  auto run = []() {
      auto c = plain();
      c.max_linear_accel = 0.7;
      c.expo = 0.4;
      JogShaper s(c);
      Vector6d acc = Vector6d::Zero();
      for (int i = 0; i < 100; ++i) {
        s.setInput(in(std::sin(i * 0.1), std::cos(i * 0.2), 0.3, 0, 0, std::sin(i * 0.05)), 0.01 * i);
        acc += s.update(0.01 * i, 0.01).twist;
      }
      return acc;
    };
  EXPECT_EQ(run(), run());
}

// ---- review fixes

namespace
{
JogShaperConfig slewed()
{
  JogShaperConfig c = plain();
  c.deadband.fill(0.0);
  c.max_linear_mps = 0.1;
  c.max_linear_accel = 0.5;
  c.max_angular_accel = 1.5;
  return c;
}
}  // namespace

TEST(JogShaper, BackwardsClockTripsWatchdog)
{
  JogShaper s(plain());
  s.setInput(in(1.0), 100.0);
  const auto o = s.update(1.0, 0.01);
  EXPECT_TRUE(o.watchdog_tripped);
  EXPECT_EQ(o.twist.norm(), 0.0);
}

TEST(JogShaper, MaskingAxisZeroesItImmediatelyWithSlewEnabled)
{
  JogShaper s(slewed());
  double t = 0.0;
  s.setInput(in(0, 0, 1.0), t);
  for (int i = 0; i < 400; ++i) {  // ramp to the full 0.1 m/s
    t += 0.01;
    s.setInput(in(0, 0, 1.0), t);
    s.update(t, 0.01);
  }
  t += 0.01;
  s.setInput(in(0, 0, 1.0), t);
  ASSERT_NEAR(s.update(t, 0.01).twist[2], 0.1, 1e-9);
  s.setEnabled({true, true, false, true, true, true});
  t += 0.01;
  s.setInput(in(0, 0, 1.0), t);
  EXPECT_EQ(s.update(t, 0.01).twist[2], 0.0);
}

TEST(JogShaper, MaskInToolFrameZeroesToolAxisImmediately)
{
  JogShaperConfig c = slewed();
  c.frame = JogFrame::kTool;
  JogShaper s(c);
  FrameRotations rot;
  rot.tool_in_base = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()).toRotationMatrix();
  double t = 0.0;
  for (int i = 0; i < 400; ++i) {
    t += 0.01;
    s.setInput(in(0.5, 0, 1.0), t);
    s.update(t, 0.01, rot);
  }
  s.setEnabled({true, true, false, true, true, true});
  t += 0.01;
  s.setInput(in(0.5, 0, 1.0), t);
  const auto o = s.update(t, 0.01, rot);
  const Eigen::Vector3d tool_v = rot.tool_in_base.transpose() * o.twist.head<3>();
  EXPECT_NEAR(tool_v[2], 0.0, 1e-12);
  EXPECT_GT(tool_v[0], 0.0);
}

TEST(JogShaper, BadConfigThrowsAndLeavesConfigUnchanged)
{
  const double nan = std::nan("");
  const double inf = std::numeric_limits<double>::infinity();
  for (double bad : {nan, inf, 0.0, -1.0}) {
    JogShaperConfig c = plain();
    c.watchdog_s = bad;
    EXPECT_THROW(JogShaper{c}, std::invalid_argument);
  }
  for (double bad : {nan, inf}) {
    JogShaperConfig c = plain();
    c.max_linear_accel = bad;
    EXPECT_THROW(JogShaper{c}, std::invalid_argument);
    c = plain();
    c.max_angular_accel = bad;
    EXPECT_THROW(JogShaper{c}, std::invalid_argument);
  }
  JogShaper s(plain());
  JogShaperConfig c = plain();
  c.watchdog_s = nan;
  EXPECT_THROW(s.setConfig(c), std::invalid_argument);
  s.setInput(in(1.0), 0.0);
  EXPECT_FALSE(s.update(0.01, 0.01).watchdog_tripped);
}

TEST(JogShaper, NanRotationZeroesAndRecovers)
{
  JogShaperConfig c = slewed();
  c.frame = JogFrame::kTool;
  JogShaper s(c);
  FrameRotations good;
  FrameRotations bad;
  bad.tool_in_base(1, 1) = std::nan("");
  double t = 0.0;
  for (int i = 0; i < 50; ++i) {
    t += 0.01;
    s.setInput(in(1.0), t);
    s.update(t, 0.01, good);
  }
  t += 0.01;
  s.setInput(in(1.0), t);
  auto o = s.update(t, 0.01, bad);
  EXPECT_TRUE(o.invalid_rotation);
  EXPECT_EQ(o.twist.norm(), 0.0);
  t += 0.01;
  s.setInput(in(1.0), t);
  o = s.update(t, 0.01, good);
  EXPECT_FALSE(o.invalid_rotation);
  EXPECT_TRUE(o.twist.allFinite());
  EXPECT_GT(o.twist[0], 0.0);  // ramps up again from rest
  EXPECT_LT(o.twist[0], 0.01);
}

TEST(JogShaper, LinearSpeedNormIsClamped)
{
  JogShaperConfig c = plain();
  c.deadband.fill(0.0);
  c.max_linear_mps = 0.1;
  c.max_angular_rps = 0.3;
  JogShaper s(c);
  s.setInput(in(1, 1, 1, 1, 1, 1), 0.0);
  const auto o = s.update(0.01, 0.01);
  EXPECT_NEAR(o.twist.head<3>().norm(), 0.1, 1e-12);
  EXPECT_NEAR(o.twist.tail<3>().norm(), 0.3, 1e-12);
  EXPECT_NEAR(o.twist[0], o.twist[1], 1e-15);  // direction kept
  EXPECT_NEAR(o.twist[3], o.twist[4], 1e-15);
}

TEST(JogShaper, SingleAxisStillReachesMax)
{
  JogShaperConfig c = plain();
  c.deadband.fill(0.0);
  c.max_linear_mps = 0.1;
  JogShaper s(c);
  s.setInput(in(0, 1.0), 0.0);
  EXPECT_NEAR(s.update(0.01, 0.01).twist[1], 0.1, 1e-12);
}
