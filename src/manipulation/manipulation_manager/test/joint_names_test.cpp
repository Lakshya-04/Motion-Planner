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

}  // namespace manipulation_manager
