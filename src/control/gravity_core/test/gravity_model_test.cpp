#include <gtest/gtest.h>

#include <cmath>

#include "gravity_core/urdf_loader.hpp"

using gravity_core::loadUrdf;

namespace {

// Planar pendulum: shoulder about -y at the origin (positive q raises the arm), links along +x, masses on the link frames.
// Hanging flat (q = 0), each point mass m at horizontal distance d gives torque m g d about y.
const char* kTwoLink = R"(
<robot name="p">
  <link name="base"/>
  <link name="l1"><inertial><origin xyz="0.5 0 0"/><mass value="2.0"/></inertial></link>
  <link name="l2"><inertial><origin xyz="0.25 0 0"/><mass value="1.0"/></inertial></link>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/>
    <axis xyz="0 -1 0"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/>
    <origin xyz="1 0 0"/><axis xyz="0 -1 0"/></joint>
</robot>)";

}  // namespace

TEST(GravityModel, TwoLinkStretchedOut) {
  auto m = loadUrdf(kTwoLink, {"j1", "j2"});
  Eigen::VectorXd tau;
  m.compute(Eigen::Vector2d(0.0, 0.0), tau);
  // j2 holds 1 kg at 0.25 m; j1 holds 2 kg at 0.5 m and 1 kg at 1.25 m. Rotation about -y at q=0
  // lifts +x toward +z, so the arm sags and the holding torque is positive.
  EXPECT_NEAR(tau[1], 1.0 * 9.81 * 0.25, 1e-9);
  EXPECT_NEAR(tau[0], 9.81 * (2.0 * 0.5 + 1.0 * 1.25), 1e-9);
}

TEST(GravityModel, TwoLinkFollowsCosine) {
  auto m = loadUrdf(kTwoLink, {"j1", "j2"});
  Eigen::VectorXd tau;
  const double q1 = 0.7, q2 = -0.4;
  m.compute(Eigen::Vector2d(q1, q2), tau);
  const double expect2 = 1.0 * 9.81 * 0.25 * std::cos(q1 + q2);
  const double expect1 = 9.81 * (2.0 * 0.5 * std::cos(q1) + 1.0 * (1.0 * std::cos(q1) + 0.25 * std::cos(q1 + q2)));
  EXPECT_NEAR(tau[1], expect2, 1e-9);
  EXPECT_NEAR(tau[0], expect1, 1e-9);
}

TEST(GravityModel, VerticalPoseNeedsNoTorque) {
  auto m = loadUrdf(kTwoLink, {"j1", "j2"});
  Eigen::VectorXd tau;
  m.compute(Eigen::Vector2d(M_PI / 2, 0.0), tau);  // pointing straight up or down
  EXPECT_NEAR(tau[0], 0.0, 1e-9);
  EXPECT_NEAR(tau[1], 0.0, 1e-9);
}

TEST(GravityModel, PayloadAddsAndClears) {
  auto m = loadUrdf(kTwoLink, {"j1", "j2"});
  Eigen::VectorXd base, loaded;
  m.compute(Eigen::Vector2d(0.0, 0.0), base);
  m.setPayload("l2", 0.5, Eigen::Vector3d(0.5, 0, 0));
  m.compute(Eigen::Vector2d(0.0, 0.0), loaded);
  EXPECT_NEAR(loaded[1] - base[1], 0.5 * 9.81 * 0.5, 1e-9);
  EXPECT_NEAR(loaded[0] - base[0], 0.5 * 9.81 * 1.5, 1e-9);
  m.setPayload("l2", 0.0, Eigen::Vector3d::Zero());
  m.compute(Eigen::Vector2d(0.0, 0.0), loaded);
  EXPECT_NEAR(loaded[0], base[0], 1e-12);
}

TEST(GravityModel, TiltedBaseRotatesGravity) {
  auto m = loadUrdf(kTwoLink, {"j1", "j2"});
  m.setGravity(Eigen::Vector3d(0, 0, 0));
  Eigen::VectorXd tau;
  m.compute(Eigen::Vector2d(0.3, 0.2), tau);
  EXPECT_NEAR(tau.norm(), 0.0, 1e-12);
}

TEST(GravityModel, PrismaticLiftCarriesEverything) {
  const char* urdf = R"(
<robot name="lift">
  <link name="base"/>
  <link name="carriage"><inertial><mass value="3.0"/></inertial></link>
  <link name="arm"><inertial><origin xyz="0.4 0 0"/><mass value="2.0"/></inertial></link>
  <joint name="lift" type="prismatic"><parent link="base"/><child link="carriage"/><axis xyz="0 0 1"/></joint>
  <joint name="shoulder" type="revolute"><parent link="carriage"/><child link="arm"/><axis xyz="0 -1 0"/></joint>
</robot>)";
  auto m = loadUrdf(urdf, {"lift", "shoulder"});
  Eigen::VectorXd tau;
  m.compute(Eigen::Vector2d(0.2, 0.0), tau);
  EXPECT_NEAR(tau[0], 5.0 * 9.81, 1e-9);        // N, independent of height
  EXPECT_NEAR(tau[1], 2.0 * 9.81 * 0.4, 1e-9);  // N m
}

TEST(GravityModel, MimicJointFoldsIntoDriver) {
  // Two stages on one actuated slider: stage 2 moves twice as far as stage 1.
  const char* urdf = R"(
<robot name="tele">
  <link name="base"/>
  <link name="s1"><inertial><mass value="1.0"/></inertial></link>
  <link name="s2"><inertial><mass value="4.0"/></inertial></link>
  <joint name="drive" type="prismatic"><parent link="base"/><child link="s1"/><axis xyz="0 0 1"/></joint>
  <joint name="follow" type="prismatic"><parent link="s1"/><child link="s2"/><axis xyz="0 0 1"/>
    <mimic joint="drive" multiplier="2.0"/></joint>
</robot>)";
  auto m = loadUrdf(urdf, {"drive"});
  ASSERT_EQ(m.dof(), 1u);
  Eigen::VectorXd tau;
  m.compute(Eigen::VectorXd::Constant(1, 0.1), tau);
  // Moving drive by dq raises s1 by dq and s2 by 3 dq: (1 + 4*3) * g.
  EXPECT_NEAR(tau[0], (1.0 + 4.0 * 3.0) * 9.81, 1e-9);
}

TEST(GravityModel, BadInputThrows) {
  EXPECT_THROW(loadUrdf("<robot", {}), std::runtime_error);
  EXPECT_THROW(loadUrdf(kTwoLink, {"nope"}), std::runtime_error);
  auto m = loadUrdf(kTwoLink, {"j1"});
  Eigen::VectorXd tau;
  EXPECT_THROW(m.compute(Eigen::Vector2d(0, 0), tau), std::runtime_error);
}
