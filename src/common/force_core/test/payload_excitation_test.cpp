#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "force_core/payload_excitation.hpp"
#include "force_core/wrench_conditioning.hpp"

using force_core::CalibrationSample;
using force_core::Mat3;
using force_core::Payload;
using force_core::Vec3;

namespace
{
Mat3 rotZ(double a)
{
  Mat3 r;
  r.m = {std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a), 0, 0, 0, 1};
  return r;
}
Mat3 rotY(double a)
{
  Mat3 r;
  r.m = {std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a)};
  return r;
}
Mat3 rotX(double a)
{
  Mat3 r;
  r.m = {1, 0, 0, 0, std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a)};
  return r;
}
Mat3 mul(const Mat3 & a, const Mat3 & b)
{
  Mat3 r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double s = 0;
      for (int k = 0; k < 3; ++k) {s += a.at(i, k) * b.at(k, j);}
      r.m[3 * i + j] = s;
    }
  }
  return r;
}

// A 6-joint wrist-ish chain: Rz(q0) Ry(q1) Ry(q2) Rz(q3) Ry(q4) Rz(q5), orientation only.
Mat3 fk6(const std::vector<double> & q)
{
  Mat3 r = rotZ(q[0]);
  r = mul(r, rotY(q[1]));
  r = mul(r, rotY(q[2]));
  r = mul(r, rotZ(q[3]));
  r = mul(r, rotY(q[4]));
  return mul(r, rotZ(q[5]));
}

force_core::ExcitationLimits limits6()
{
  force_core::ExcitationLimits l;
  l.q_min = std::vector<double>(6, -2.8);
  l.q_max = std::vector<double>(6, 2.8);
  l.v_max = std::vector<double>(6, 1.5);
  l.a_max = std::vector<double>(6, 5.0);
  return l;
}

const Vec3 kGravity{0, 0, -9.80665};
}  // namespace

TEST(RegressorCondition, WellSpreadIsLowAndPlanarSweepIsWorse)
{
  std::vector<Mat3> spread;
  for (const double a : {0.0, 1.5, 3.0, -1.5}) {spread.push_back(mul(rotX(a), rotY(0.4 * a - 0.3)));}
  spread.push_back(rotY(1.6));
  spread.push_back(rotX(-1.6));
  const double good = force_core::regressorConditionNumber(spread, kGravity);
  EXPECT_LT(good, 10.0);

  std::vector<Mat3> planar;
  for (const double a : {0.0, 0.2, 0.4, 0.6}) {planar.push_back(rotX(a));}
  EXPECT_GT(force_core::regressorConditionNumber(planar, kGravity), 5.0 * good);

  std::vector<Mat3> same(6);
  EXPECT_TRUE(std::isinf(force_core::regressorConditionNumber(same, kGravity)));
}

TEST(PayloadIdentification, ReportsConditionAndRejectsAboveLimit)
{
  const Payload truth{1.0, {0.0, 0.0, 0.05}};
  std::vector<CalibrationSample> s;
  for (const double a : {0.0, 0.1, 0.2, 0.3, 0.4, 0.5}) {
    CalibrationSample p;
    p.r_world_sensor = rotX(a);
    const auto g = force_core::gravityWrench(truth, p.r_world_sensor, kGravity);
    p.raw = g;
    s.push_back(p);
  }
  const auto loose = force_core::identifyPayload(s, kGravity, 1.0, 1e9);
  ASSERT_TRUE(loose.ok) << loose.message;
  EXPECT_GT(loose.condition_number, 10.0);
  const auto tight = force_core::identifyPayload(s, kGravity, 1.0, 10.0);
  EXPECT_FALSE(tight.ok);
  EXPECT_NE(tight.message.find("condition number"), std::string::npos);
}

TEST(ExcitationPlan, SelectsSpreadPosesInsideBounds)
{
  const auto lim = limits6();
  force_core::ExcitationOptions opt;
  opt.num_poses = 8;
  const std::vector<double> q0{0.1, 0.3, -0.4, 0.2, 0.5, 0.0};
  const auto plan = force_core::generateExcitationPoses(q0, lim, fk6, {}, kGravity, opt);
  ASSERT_TRUE(plan.ok) << plan.message;
  EXPECT_EQ(plan.waypoints.size(), 8u);
  EXPECT_LT(plan.condition_number, 10.0);
  EXPECT_GT(plan.min_pair_angle_rad, 0.3);
  for (const auto & w : plan.waypoints) {
    for (std::size_t j = 0; j < 6; ++j) {
      EXPECT_GE(w.q[j], lim.q_min[j] + lim.margin_rad);
      EXPECT_LE(w.q[j], lim.q_max[j] - lim.margin_rad);
    }
    EXPECT_LE(w.peak_velocity_ratio, lim.velocity_scale + 1e-9);
    EXPECT_LE(w.peak_accel_ratio, lim.velocity_scale + 1e-9);
  }
  std::printf("[ excitation ] poses %zu, cond %.2f, min pair angle %.1f deg, duration %.1f s\n",
    plan.waypoints.size(), plan.condition_number, plan.min_pair_angle_rad * 180.0 / M_PI, plan.duration_s);
}

TEST(ExcitationPlan, RespectsSafetyProxyAlongTheMotion)
{
  const auto lim = limits6();
  // Proxy: joint 1 must stay below 0.8 (a stand-in for a self-collision bound).
  const force_core::SafetyProxy proxy = [](const std::vector<double> & q) {return q[1] < 0.8;};
  const std::vector<double> q0{0.1, 0.3, -0.4, 0.2, 0.5, 0.0};
  force_core::ExcitationOptions opt;
  const auto plan = force_core::generateExcitationPoses(q0, lim, fk6, proxy, kGravity, opt);
  ASSERT_TRUE(plan.ok) << plan.message;
  for (double t = 0.0; t <= plan.waypoints.back().t_leave_s; t += 0.05) {
    EXPECT_TRUE(proxy(force_core::sampleExcitationPlan(plan, t))) << "t = " << t;
  }
}

TEST(ExcitationPlan, SampledVelocityAndAccelerationStayInBounds)
{
  const auto lim = limits6();
  const std::vector<double> q0{0.1, 0.3, -0.4, 0.2, 0.5, 0.0};
  const auto plan = force_core::generateExcitationPoses(q0, lim, fk6, {}, kGravity);
  ASSERT_TRUE(plan.ok) << plan.message;
  const double dt = 0.01;
  auto prev = force_core::sampleExcitationPlan(plan, 0.0);
  std::vector<double> prev_v(6, 0.0);
  for (double t = dt; t <= plan.duration_s; t += dt) {
    const auto q = force_core::sampleExcitationPlan(plan, t);
    for (std::size_t j = 0; j < 6; ++j) {
      const double v = (q[j] - prev[j]) / dt;
      EXPECT_LE(std::abs(v), lim.v_max[j] * 1.01);
      EXPECT_LE(std::abs((v - prev_v[j]) / dt), lim.a_max[j] * 1.05);
      prev_v[j] = v;
    }
    prev = q;
  }
}

TEST(ExcitationPlan, FailsWhenNothingIsSafe)
{
  const auto lim = limits6();
  const std::vector<double> q0{0.1, 0.3, -0.4, 0.2, 0.5, 0.0};
  const force_core::SafetyProxy proxy = [&](const std::vector<double> & q) {return q[0] == q0[0] && q[1] == q0[1];};
  const auto plan = force_core::generateExcitationPoses(q0, lim, fk6, proxy, kGravity);
  EXPECT_FALSE(plan.ok);
  const auto bad_start = force_core::generateExcitationPoses({9, 0, 0, 0, 0, 0}, lim, fk6, {}, kGravity);
  EXPECT_FALSE(bad_start.ok);
}

TEST(PayloadIdentification, RecoversMassAndCentreWithPointOneNewtonNoiseFromThePlan)
{
  const Payload truth{2.0, {0.015, -0.010, 0.060}};
  const core_types::Wrench bias = []() {
      core_types::Wrench b;
      b.force_x = 0.5; b.force_y = -0.4; b.force_z = 0.3;
      b.torque_x = 0.03; b.torque_y = -0.02; b.torque_z = 0.01;
      return b;
    }();
  const std::vector<double> q0{0.1, 0.3, -0.4, 0.2, 0.5, 0.0};
  force_core::ExcitationOptions opt;
  opt.num_poses = 10;
  const auto plan = force_core::generateExcitationPoses(q0, limits6(), fk6, {}, kGravity, opt);
  ASSERT_TRUE(plan.ok) << plan.message;

  std::mt19937 rng(7);
  std::normal_distribution<double> nf(0.0, 0.1), nt(0.0, 0.01);
  double worst_mass = 0.0, worst_cog = 0.0, worst_bias = 0.0;
  for (int trial = 0; trial < 50; ++trial) {
    std::vector<CalibrationSample> s;
    for (const auto & wp : plan.waypoints) {
      CalibrationSample p;
      p.r_world_sensor = fk6(wp.q);
      auto g = force_core::gravityWrench(truth, p.r_world_sensor, kGravity);
      g.force_x += bias.force_x + nf(rng); g.force_y += bias.force_y + nf(rng); g.force_z += bias.force_z + nf(rng);
      g.torque_x += bias.torque_x + nt(rng); g.torque_y += bias.torque_y + nt(rng);
      g.torque_z += bias.torque_z + nt(rng);
      p.raw = g;
      s.push_back(p);
    }
    const auto r = force_core::identifyPayload(s, kGravity);
    ASSERT_TRUE(r.ok) << r.message;
    worst_mass = std::max(worst_mass, std::abs(r.payload.mass_kg - truth.mass_kg));
    worst_cog = std::max({worst_cog, std::abs(r.payload.cog.x - truth.cog.x),
        std::abs(r.payload.cog.y - truth.cog.y), std::abs(r.payload.cog.z - truth.cog.z)});
    worst_bias = std::max({worst_bias, std::abs(r.bias.force_x - bias.force_x),
        std::abs(r.bias.force_y - bias.force_y), std::abs(r.bias.force_z - bias.force_z)});
    EXPECT_NEAR(r.rms_force_n, 0.1, 0.06);
  }
  std::printf("[ payload id ] 50 trials, 10 poses, 0.1 N noise: worst mass err %.4f kg, worst cog err %.2f mm, "
    "worst force-bias err %.3f N, cond %.2f\n", worst_mass, worst_cog * 1000.0, worst_bias, plan.condition_number);
  EXPECT_LT(worst_mass, 0.1);
  EXPECT_LT(worst_cog, 0.004);
  EXPECT_LT(worst_bias, 0.25);
}
