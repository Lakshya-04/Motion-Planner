#include <gtest/gtest.h>

#include <cmath>

#include "sim_mujoco/mujoco_sim.hpp"

namespace
{
const std::string kModel = std::string(SIM_MUJOCO_TEST_DATA) + "/slide_wall.xml";
}

using sim_mujoco::MujocoSim;

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
  EXPECT_NEAR(sim.position(0), 0.1, 1e-3);
  EXPECT_NEAR(sim.velocity(0), 0.0, 1e-3);
  // gravity compensated and free: no contact, so almost no force on the sensor after the tare
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
  EXPECT_NEAR(sim.position(0), 0.05, 1e-3);
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
          worst = std::max(worst, std::abs(-0.2 + v * t - sim.position(0)));
        }
      }
      return worst;
    };
  const double without = lag(false);
  const double with = lag(true);
  EXPECT_GT(without, 1e-3);
  EXPECT_LT(with, 0.5 * without);
}

TEST(MujocoSim, SignOfWallReaction)
{
  MujocoSim sim(kModel, {"slide"});
  sim.setJointPositions({0.0});
  sim.advance(0.5);
  sim.tareWrench();
  // Pad top is at z = 0.05; wall face at z = 0.25. Command 0.30: about 0.05 m of "push" through
  // the wall, so the actuator pushes with up to kp * (0.30 - z) ~ 50 N against the contact.
  sim.setCommand(0, 0.30, 0.0);
  sim.advance(3.0);
  EXPECT_GT(sim.contactCount(), 0);
  const auto w = sim.wrench();
  // the tool presses along +z, so the environment pushes back along -z
  EXPECT_LT(w[2], -20.0);
  EXPECT_GT(w[2], -300.0);
  EXPECT_NEAR(w[0], 0.0, 1.0);
  EXPECT_NEAR(w[1], 0.0, 1.0);
  EXPECT_NEAR(std::abs(w[2]), std::abs(sim.effort(0)), 0.2 * std::abs(sim.effort(0)));
}

TEST(MujocoSim, ReleasingTheCommandReleasesTheForce)
{
  MujocoSim sim(kModel, {"slide"});
  sim.setJointPositions({0.0});
  sim.advance(0.5);
  sim.tareWrench();
  sim.setCommand(0, 0.30, 0.0);
  sim.advance(2.0);
  ASSERT_LT(sim.wrench()[2], -20.0);
  sim.setCommand(0, 0.0, 0.0);
  sim.advance(3.0);
  EXPECT_NEAR(sim.wrench()[2], 0.0, 1.0);
}
