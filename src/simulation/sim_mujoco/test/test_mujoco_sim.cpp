#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>

#include "sim_mujoco/mujoco_sim.hpp"

namespace
{
const std::string kModel = std::string(SIM_MUJOCO_TEST_DATA) + "/slide_wall.xml";
}

using sim_mujoco::MujocoSim;

// static sag of the 2.001 kg carriage on a kp = 1000 N/m actuator
constexpr double kSag = 2.001 * 9.81 / 1000.0;

TEST(MujocoSim, ThrowsOnMissingJointOrSite)
{
  EXPECT_THROW(MujocoSim(kModel, {"nope"}), std::runtime_error);
  EXPECT_THROW(MujocoSim(kModel, {"slide"}, "no_site"), std::runtime_error);
  EXPECT_THROW(MujocoSim("/does/not/exist.xml", {"slide"}), std::runtime_error);
}

TEST(MujocoSim, TracksPositionTargetInFreeSpace)
{
  MujocoSim sim(kModel, {"slide"});
  sim.setJointPositions({0.0});
  sim.setCommand(0, 0.1, 0.0);
  sim.advance(3.0);
  EXPECT_NEAR(sim.position(0), 0.1 - kSag, 1e-3);  // the carriage is not gravity compensated
  EXPECT_NEAR(sim.velocity(0), 0.0, 1e-3);
  // free, no contact: after the tare the sensor reads nothing
  sim.tareWrench();
  sim.advance(0.5);
  EXPECT_NEAR(sim.wrench()[2], 0.0, 0.5);
}

TEST(MujocoSim, MimicFollowerIsPlacedOnItsConstraint)
{
  MujocoSim sim(kModel, {"slide"});
  sim.setJointPositions({0.2});
  const mjModel * m = sim.model();
  const int slave = mj_name2id(m, mjOBJ_JOINT, "slave");
  EXPECT_NEAR(sim.data()->qpos[m->jnt_qposadr[slave]], 0.1, 1e-9);
}

TEST(MujocoSim, NaNCommandHoldsTargetAndNaNVelocityIsZero)
{
  MujocoSim sim(kModel, {"slide"});
  sim.setJointPositions({0.0});
  sim.setCommand(0, 0.05, 0.0);
  sim.advance(2.0);
  sim.setCommand(0, std::nan(""), std::nan(""));
  sim.advance(1.0);
  EXPECT_NEAR(sim.position(0), 0.05 - kSag, 1e-3);
}

TEST(MujocoSim, VelocityFeedForwardReducesRampLag)
{
  auto lag = [](bool feedforward) {
      MujocoSim sim(kModel, {"slide"});
      sim.setJointPositions({-0.2});
      const double v = 0.05;
      double t = 0.0;
      double worst = 0.0;
      for (int k = 0; k < 1000; ++k) {  // 10 ms control period, 10 s
        t += 0.01;
        sim.setCommand(0, -0.2 + v * t, feedforward ? v : 0.0);
        sim.advance(0.01);
        if (t > 4.0 && t < 5.0) {
          worst = std::max(worst, std::abs(-0.2 + v * t - sim.position(0)) - kSag);
        }
      }
      return worst;
    };
  const double without = lag(false);
  const double with = lag(true);
  EXPECT_GT(without, 1e-3);
  EXPECT_LT(with, 0.5 * without);
}

TEST(MujocoSim, WeightAndPush)
{
  // carriage: 2 kg with its centre of mass 0.05 m in +x of the sensor, follower 1 g; z is up.
  const double m = 2.001;
  const double g = 9.81;
  MujocoSim sim(kModel, {"slide"});
  sim.setJointPositions({0.0});
  sim.setCommand(0, 0.0, 0.0);
  sim.advance(4.0);
  ASSERT_EQ(sim.contactCount(), 0);

  // 1. Known weight hanging: the tool side loads the sensor with (0, 0, -m g), torque r x W.
  const auto w0 = sim.wrench();
  EXPECT_NEAR(w0[0], 0.0, 0.2);
  EXPECT_NEAR(w0[2], -m * g, 0.3);
  EXPECT_NEAR(w0[4], 0.05 * 2.0 * g, 0.1);  // ty = rz Wx - rx Wz = +0.05 * 2 g

  // 2. Known push: pad face (tcp) is at z = 0.05 + sag, wall face at 0.25; command through it.
  sim.setCommand(0, 0.30, 0.0);
  sim.advance(4.0);
  ASSERT_GT(sim.contactCount(), 0);
  const auto raw = sim.wrench();
  const auto truth = sim.groundTruthWrench();
  EXPECT_LT(truth[2], -20.0);   // the wall pushes the tool back along -z
  EXPECT_GT(truth[2], -400.0);
  EXPECT_NEAR(truth[0], 0.0, 1.0);
  // raw = truth + weight at the sensor; torque about the sensor = torque at tcp + r_tcp x F + weight torque
  const double r[3] = {0.1, 0.0, 0.05};
  const double f[3] = {truth[0], truth[1], truth[2]};
  const double expected_f[3] = {f[0] + w0[0], f[1] + w0[1], f[2] + w0[2]};
  const double expected_t[3] = {
    truth[3] + (r[1] * f[2] - r[2] * f[1]) + w0[3],
    truth[4] + (r[2] * f[0] - r[0] * f[2]) + w0[4],
    truth[5] + (r[0] * f[1] - r[1] * f[0]) + w0[5]};
  for (int k = 0; k < 3; ++k) {
    EXPECT_NEAR(raw[k], expected_f[k], 0.03 * std::abs(truth[2]) + 0.3) << "force " << k;
    EXPECT_NEAR(raw[3 + k], expected_t[k], 0.03 * std::abs(truth[2]) * 0.15 + 0.3) << "torque " << k;
  }
}

TEST(MujocoSim, ReleasingTheCommandReleasesTheForce)
{
  MujocoSim sim(kModel, {"slide"});
  sim.setJointPositions({0.0});
  sim.advance(0.5);
  sim.advance(3.5);
  sim.tareWrench();
  sim.setCommand(0, 0.30, 0.0);
  sim.advance(2.0);
  ASSERT_LT(sim.wrench()[2], -20.0);
  sim.setCommand(0, 0.0, 0.0);
  sim.advance(3.0);
  EXPECT_NEAR(sim.wrench()[2], 0.0, 1.0);
}

// ------------------------------------------------------------------ depth sensor

namespace
{
const std::string kDepthModel = std::string(SIM_MUJOCO_TEST_DATA) + "/depth_wall.xml";
}

TEST(MujocoSimDepth, AbsentWithoutTheSiteAndNumeric)
{
  MujocoSim sim(kModel, {"slide"});
  EXPECT_FALSE(sim.hasDepth());
  EXPECT_TRUE(sim.depthScan().empty());
}

TEST(MujocoSimDepth, ScansTheWallPlaneAndNotTheRobot)
{
  MujocoSim sim(kDepthModel, {"slide"});
  ASSERT_TRUE(sim.hasDepth());
  EXPECT_EQ(sim.depthConfig().width, 21);
  sim.setJointPositions({0.0});
  const auto points = sim.depthScan();
  ASSERT_EQ(points.size(), 21u * 15u);  // every ray hits the wall; the pad (group 0) is not scanned
  // wall front face z = 0.25 is the sensor's x axis; y spans +-tan(hfov/2) x range, z +-tan(vfov/2)
  double ymax = 0.0, zmax = 0.0;
  for (const auto & p : points) {
    EXPECT_NEAR(p[0], 0.25, 1e-5);
    ymax = std::max(ymax, static_cast<double>(std::abs(p[1])));
    zmax = std::max(zmax, static_cast<double>(std::abs(p[2])));
  }
  EXPECT_NEAR(ymax, 0.25 * std::tan(0.5235988 / 2) * (1.0 - 1.0 / 21), 2e-3);
  EXPECT_NEAR(zmax, 0.25 * std::tan(0.3490659 / 2) * (1.0 - 1.0 / 15), 2e-3);
}

TEST(MujocoSimDepth, FollowsTheBodyAndDropsOutOfRange)
{
  MujocoSim sim(kDepthModel, {"slide"});
  sim.setJointPositions({0.1});  // carriage (and sensor) 0.1 m closer to the wall
  const auto near_points = sim.depthScan();
  ASSERT_FALSE(near_points.empty());
  EXPECT_NEAR(near_points[0][0], 0.15, 1e-5);
  sim.setJointPositions({0.22});  // wall 0.03 away, below the 0.05 minimum range: nothing returned
  EXPECT_TRUE(sim.depthScan().empty());
}

TEST(MujocoSimDepth, RangeNoiseIsAppliedAlongTheRay)
{
  std::string text;
  {
    std::ifstream in(kDepthModel);
    text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  }
  const std::string from = "5.0 0.0\"/>";
  const auto pos = text.find(from);
  ASSERT_NE(pos, std::string::npos);
  text.replace(pos, from.size(), "5.0 0.005\"/>");
  const std::string path = ::testing::TempDir() + "/depth_wall_noise.xml";
  {
    std::ofstream out(path);
    out << text;
  }
  MujocoSim sim(path, {"slide"});
  sim.setJointPositions({0.0});
  const auto points = sim.depthScan();
  double sum = 0.0, sum2 = 0.0;
  for (const auto & p : points) {
    const double r = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    const double d = r - 0.25 / (p[0] / r);  // measured minus true range along this ray
    sum += d;
    sum2 += d * d;
  }
  const double n = static_cast<double>(points.size());
  EXPECT_NEAR(sum / n, 0.0, 0.002);
  EXPECT_NEAR(std::sqrt(sum2 / n), 0.005, 0.0015);
}
