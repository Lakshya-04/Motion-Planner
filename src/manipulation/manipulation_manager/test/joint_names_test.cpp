#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "manipulation_manager/core/joint_names.hpp"

namespace manipulation_manager
{

namespace
{
const std::vector<std::string> kLegacy = {"lift_prismatic_joint", "joint1", "joint2"};
const std::vector<std::string> kUr = {"lift_joint", "arm_shoulder_pan_joint", "arm_elbow_joint"};
}  // namespace

TEST(JointNamesTest, PicksTheListTheRobotPublishes)
{
  const std::vector<std::string> available = {
    "lift_joint", "lift_stage_1_joint", "arm_shoulder_pan_joint", "arm_elbow_joint", "wheel"};
  EXPECT_EQ(selectJointNames({kLegacy, kUr}, available), kUr);
}

TEST(JointNamesTest, FirstMatchingCandidateWins)
{
  const std::vector<std::string> available = {
    "lift_prismatic_joint", "joint1", "joint2", "lift_joint", "arm_shoulder_pan_joint", "arm_elbow_joint"};
  EXPECT_EQ(selectJointNames({kLegacy, kUr}, available), kLegacy);
  EXPECT_EQ(selectJointNames({kUr, kLegacy}, available), kUr);
}

TEST(JointNamesTest, PartialMatchIsNoMatch)
{
  const std::vector<std::string> available = {"lift_joint", "arm_shoulder_pan_joint"};
  EXPECT_TRUE(selectJointNames({kLegacy, kUr}, available).empty());
}

TEST(JointNamesTest, NothingAvailableOrNoCandidates)
{
  EXPECT_TRUE(selectJointNames({kUr}, {}).empty());
  EXPECT_TRUE(selectJointNames({}, {"lift_joint"}).empty());
  EXPECT_TRUE(selectJointNames({{}}, {"lift_joint"}).empty());
}

TEST(JointNamesTest, GroupJointsFollowTheProfile)
{
  const std::vector<std::string> arm = {"arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5",
    "arm_joint6", "arm_joint7"};
  auto whole = groupJointNames("lift_arm", "lift_arm", "arm", "lift_joint", arm);
  ASSERT_EQ(whole.size(), 8u);
  EXPECT_EQ(whole.front(), "lift_joint");
  EXPECT_EQ(whole.back(), "arm_joint7");
  EXPECT_EQ(groupJointNames("arm", "lift_arm", "arm", "lift_joint", arm), arm);
  // no actuated lift: the whole-body group is the arm
  EXPECT_EQ(groupJointNames("lift_arm", "lift_arm", "arm", "", arm), arm);
  EXPECT_TRUE(groupJointNames("tool", "lift_arm", "arm", "lift_joint", arm).empty());
  // the derived list is a candidate like any other: it matches a 7-DOF /joint_states
  std::vector<std::string> available = whole;
  available.push_back("lift_stage_1_joint");
  EXPECT_EQ(selectJointNames({whole, kUr}, available), whole);
}

}  // namespace manipulation_manager
