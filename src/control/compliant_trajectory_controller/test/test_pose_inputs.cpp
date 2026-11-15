#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "compliant_trajectory_controller/pose_inputs.hpp"

namespace
{
using compliant_trajectory_controller::checkPoseChain;
using compliant_trajectory_controller::resolveJointInputs;
using mobile_manipulator_profile::Profile;

Profile makeProfile(bool lift)
{
  Profile p;
  p.lift_joint = lift ? "lift" : "";
  p.arm_joints = {"j1", "j2"};
  p.home = {{"lift", 0.5}, {"j1", 0.1}, {"j2", 0.2}};
  return p;
}
}  // namespace

TEST(ResolveJointInputs, LiftProfileTakesHome)
{
  std::vector<std::string> names;
  std::vector<double> pose;
  EXPECT_EQ(resolveJointInputs(makeProfile(true), names, pose), "");
  EXPECT_EQ(names.size(), 3u);
  EXPECT_EQ(pose.size(), 3u);
}

TEST(ResolveJointInputs, NoLiftProfileStillTakesHome)
{
  std::vector<std::string> names;
  std::vector<double> pose;
  EXPECT_EQ(resolveJointInputs(makeProfile(false), names, pose), "");
  EXPECT_EQ(names, (std::vector<std::string>{"j1", "j2"}));
  EXPECT_EQ(pose, (std::vector<double>{0.1, 0.2}));
}

TEST(ResolveJointInputs, EmptyProfileFailsLoudly)
{
  std::vector<std::string> names;
  std::vector<double> pose;
  EXPECT_NE(resolveJointInputs(Profile{}, names, pose), "");
}

TEST(CheckPoseChain, AcceptsMatchingChain)
{
  const std::vector<std::string> n{"a", "b"};
  EXPECT_EQ(checkPoseChain(n, n, {0.0, 1.0}), "");
}

TEST(CheckPoseChain, RejectsOtherArm)
{
  const std::vector<std::string> expected{"a", "b"};
  EXPECT_NE(checkPoseChain({"a", "b", "c"}, expected, {0.0, 1.0}), "");
  EXPECT_NE(checkPoseChain({"a", "x"}, expected, {0.0, 1.0}), "");
}
