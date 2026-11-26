#include <cstdlib>
#include <filesystem>

#include "gtest/gtest.h"
#include "mobile_manipulator_profile/profile.hpp"

using mobile_manipulator_profile::loadProfile;

class ProfileTest : public ::testing::Test
{
protected:
  void SetUp() override {setenv("MOBILE_MANIPULATOR_PROFILE_DIR", PROFILE_DIR, 1);}
};

TEST_F(ProfileTest, Ur10eHasSixArmJoints)
{
  const auto p = loadProfile("ur10e_telescopic");
  EXPECT_EQ(p.arm, "ur10e");
  EXPECT_EQ(p.armDof(), 6u);
  EXPECT_EQ(p.dof(), 7u);
  EXPECT_EQ(p.joints().front(), "lift_joint");
  EXPECT_EQ(p.joints()[1], "arm_shoulder_pan_joint");
  EXPECT_EQ(p.whole_body_group, "lift_arm");
  EXPECT_EQ(p.arm_group, "arm");
  EXPECT_EQ(p.tcpLink(), "tcp_link");
  EXPECT_EQ(p.homePositions().size(), 7u);
}

TEST_F(ProfileTest, PandaHasSevenArmJoints)
{
  const auto p = loadProfile("panda_telescopic");
  EXPECT_EQ(p.armDof(), 7u);
  EXPECT_EQ(p.dof(), 8u);
  EXPECT_EQ(p.joints().back(), "arm_joint7");
  EXPECT_EQ(p.controller_family, "panda");
}

TEST_F(ProfileTest, GenericArmLoadsByPath)
{
  const auto p = loadProfile(std::string(PROFILE_DIR) + "/generic_6dof_telescopic.yaml");
  EXPECT_EQ(p.name, "generic_6dof_telescopic");
  EXPECT_EQ(p.armDof(), 6u);
}

TEST_F(ProfileTest, MissingProfileThrows)
{
  EXPECT_THROW(loadProfile("no_such_profile"), std::runtime_error);
}

TEST_F(ProfileTest, DofMismatchThrows)
{
  auto root = YAML::LoadFile(std::string(PROFILE_DIR) + "/panda_telescopic.yaml");
  root["arm"]["dof"] = 6;
  EXPECT_THROW(mobile_manipulator_profile::parseProfile(root, "bad"), std::runtime_error);
}

TEST_F(ProfileTest, HomeMustCoverEveryJoint)
{
  auto root = YAML::LoadFile(std::string(PROFILE_DIR) + "/panda_telescopic.yaml");
  root["home"].remove("arm_joint7");
  EXPECT_THROW(mobile_manipulator_profile::parseProfile(root, "bad"), std::runtime_error);
}

TEST_F(ProfileTest, NoLiftProfileHasNoLiftJoint)
{
  const auto p = loadProfile("ur10e_nolift");
  EXPECT_TRUE(p.lift_joint.empty());
  EXPECT_EQ(p.dof(), p.armDof());
  EXPECT_EQ(p.homePositions().size(), p.dof());
}

TEST_F(ProfileTest, NullMimicIsEmpty)
{
  auto root = YAML::LoadFile(std::string(PROFILE_DIR) + "/panda_telescopic.yaml");
  root["joints"]["mimic"] = YAML::Node(YAML::NodeType::Null);
  EXPECT_TRUE(mobile_manipulator_profile::parseProfile(root, "p").mimic_joints.empty());
}

TEST_F(ProfileTest, EveryShippedProfileLoads)
{
  for (const auto & e : std::filesystem::directory_iterator(PROFILE_DIR)) {
    if (e.path().extension() != ".yaml") {continue;}
    EXPECT_NO_THROW(loadProfile(e.path().string())) << e.path();
  }
}

namespace
{
// A minimal profile: lift (optional) plus a 2-joint arm.
std::string miniProfile(bool lift, const std::string & extra)
{
  std::string y =
    "robot:\n  xacro_args:\n    arm: generic_6dof\n"
    "joints:\n";
  y += lift ? "  lift: lift_joint\n" : "  lift: null\n";
  y += "  arm: [j1, j2]\nhome:\n";
  if (lift) {y += "  lift_joint: 0.0\n";}
  y += "  j1: 0.0\n  j2: 0.0\n"
    "frames: {world_base: a, base: b, arm_base: c, flange: d, tool_mount: e, tcp: f}\n" + extra;
  return y;
}
}  // namespace

TEST(ProfileJointCosts, DefaultIsLiftTwoArmOne)
{
  const auto p = mobile_manipulator_profile::parseProfile(YAML::Load(miniProfile(true, "")), "mini");
  EXPECT_TRUE(p.joint_costs.empty());
  EXPECT_EQ(p.jointCosts(), (std::vector<double>{2.0, 1.0, 1.0}));
  const auto q = mobile_manipulator_profile::parseProfile(YAML::Load(miniProfile(false, "")), "mini");
  EXPECT_EQ(q.jointCosts(), (std::vector<double>{1.0, 1.0}));
}

TEST(ProfileJointCosts, ExplicitArrayInJointsOrder)
{
  const auto p = mobile_manipulator_profile::parseProfile(
    YAML::Load(miniProfile(true, "joint_costs: [3.0, 1.0, 0.5]\n")), "mini");
  EXPECT_EQ(p.jointCosts(), (std::vector<double>{3.0, 1.0, 0.5}));
}

TEST(ProfileJointCosts, RejectsWrongSizeAndBadValues)
{
  using mobile_manipulator_profile::parseProfile;
  EXPECT_THROW(parseProfile(YAML::Load(miniProfile(true, "joint_costs: [2.0, 1.0]\n")), "mini"), std::runtime_error);
  EXPECT_THROW(parseProfile(YAML::Load(miniProfile(true, "joint_costs: [2.0, 0.0, 1.0]\n")), "mini"), std::runtime_error);
  EXPECT_THROW(parseProfile(YAML::Load(miniProfile(true, "joint_costs: [2.0, -1.0, 1.0]\n")), "mini"), std::runtime_error);
}

TEST_F(ProfileTest, ShippedProfilesDefaultToTheCurrentWeights)
{
  const auto p = loadProfile("ur10e_telescopic");
  EXPECT_EQ(p.jointCosts().size(), p.dof());
  EXPECT_DOUBLE_EQ(p.jointCosts().front(), 2.0);
}
