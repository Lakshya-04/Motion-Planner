#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "core_types/pivot_calibration.hpp"

using core_types::Pose;

namespace
{
Pose fromAxisAngle(double ax, double ay, double az, double angle)
{
  const double n = std::sqrt(ax * ax + ay * ay + az * az);
  const double s = std::sin(angle / 2) / n;
  Pose o;
  o.qx = ax * s;
  o.qy = ay * s;
  o.qz = az * s;
  o.qw = std::cos(angle / 2);
  return o;
}

// Flange pose that puts the tip t exactly on pivot q with the given orientation.
Pose poseForPivot(Pose orient, const std::array<double, 3> & t, const std::array<double, 3> & q)
{
  std::array<double, 9> r;
  core_types::detail::quatToMatrix(orient, r);
  orient.x = q[0] - (r[0] * t[0] + r[1] * t[1] + r[2] * t[2]);
  orient.y = q[1] - (r[3] * t[0] + r[4] * t[1] + r[5] * t[2]);
  orient.z = q[2] - (r[6] * t[0] + r[7] * t[1] + r[8] * t[2]);
  return orient;
}

std::vector<Pose> makePoses(
  const std::array<double, 3> & t, const std::array<double, 3> & q, double noise_m, unsigned seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> g(0.0, 1.0);
  std::uniform_real_distribution<double> u(-0.6, 0.6);
  std::vector<Pose> v;
  for (int i = 0; i < 12; ++i) {
    Pose p = poseForPivot(fromAxisAngle(u(rng), u(rng), 1.0 + u(rng), u(rng) + 0.2), t, q);
    p.x += noise_m * g(rng);
    p.y += noise_m * g(rng);
    p.z += noise_m * g(rng);
    v.push_back(p);
  }
  return v;
}
}  // namespace

TEST(PivotCalibration, ExactDataRecoversTcpAndPivot)
{
  const std::array<double, 3> t{0.02, -0.01, 0.25}, q{0.6, 0.1, 0.9};
  const auto r = core_types::pivotCalibration(makePoses(t, q, 0.0, 1));
  ASSERT_TRUE(r.ok) << r.error;
  for (int k = 0; k < 3; ++k) {
    EXPECT_NEAR(r.tcp_xyz[k], t[k], 1e-9);
    EXPECT_NEAR(r.pivot_xyz[k], q[k], 1e-9);
  }
  EXPECT_LT(r.rms_m, 1e-9);
}

TEST(PivotCalibration, SimulatedNoiseStaysWithinAFewTimesTheNoise)
{
  const std::array<double, 3> t{0.02, -0.01, 0.25}, q{0.6, 0.1, 0.9};
  const double sigma = 0.0005;  // 0.5 mm per coordinate
  const auto r = core_types::pivotCalibration(makePoses(t, q, sigma, 7));
  ASSERT_TRUE(r.ok) << r.error;
  for (int k = 0; k < 3; ++k) {
    EXPECT_NEAR(r.tcp_xyz[k], t[k], 3 * sigma);
    EXPECT_NEAR(r.pivot_xyz[k], q[k], 3 * sigma);
  }
  EXPECT_LT(r.rms_m, 2.0 * sigma * std::sqrt(3.0));
  EXPECT_GT(r.rms_m, 0.0);
}

TEST(PivotCalibration, TooFewPosesRefused)
{
  const auto poses = makePoses({0, 0, 0.2}, {1, 0, 1}, 0.0, 3);
  const auto r = core_types::pivotCalibration({poses[0], poses[1]});
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("at least"), std::string::npos);
}

TEST(PivotCalibration, RotationsAboutOneAxisAreDegenerate)
{
  const std::array<double, 3> t{0.0, 0.0, 0.2}, q{0.5, 0.0, 0.8};
  std::vector<Pose> v;
  for (int i = 0; i < 8; ++i) {
    v.push_back(poseForPivot(fromAxisAngle(0, 0, 1, 0.2 * i), t, q));
  }
  const auto r = core_types::pivotCalibration(v);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("degenerate"), std::string::npos);
}

TEST(PivotCalibration, ZeroQuaternionRefused)
{
  auto poses = makePoses({0, 0, 0.2}, {1, 0, 1}, 0.0, 3);
  poses[2].qx = poses[2].qy = poses[2].qz = poses[2].qw = 0.0;
  EXPECT_FALSE(core_types::pivotCalibration(poses).ok);
}
