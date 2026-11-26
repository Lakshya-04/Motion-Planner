#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "manipulation_manager/core/recovery_branch.hpp"

namespace manipulation_manager
{
namespace
{
using V = std::vector<double>;

DockIkSelectorParams params()
{
  DockIkSelectorParams p;
  p.linear_joints = {0};
  p.branch_joints = {1, 3};
  p.wrist_joint = 4;
  return p;
}

// live state of the Gazebo run: pan -0.62, elbow +0.87 (joint 0 is the lift)
const V kLive{0.46, -0.62, -0.10, 0.87, 0.2};
const V kFlipped{0.09, 2.49, -1.77, -0.94, 0.2};
const V kOnBranch{0.40, -0.55, -0.20, 0.80, 0.1};
}  // namespace

TEST(RecoveryBranchTest, JogTargetIsLastGoodOnTheLiveBranch)
{
  const auto t = selectRecoveryJogTarget(kLive, kOnBranch, kFlipped, params());
  ASSERT_TRUE(t.ok);
  EXPECT_EQ(t.q, kOnBranch);
  EXPECT_FALSE(t.from_branch_seed);
}

TEST(RecoveryBranchTest, FlippedLastGoodIsRejectedBranchSeedUsed)
{
  const auto t = selectRecoveryJogTarget(kLive, kFlipped, kOnBranch, params());
  ASSERT_TRUE(t.ok);
  EXPECT_EQ(t.q, kOnBranch);
  EXPECT_TRUE(t.from_branch_seed);
  EXPECT_TRUE(t.last_good_rejected);
}

TEST(RecoveryBranchTest, NoCandidateOnTheBranchMeansNoTarget)
{
  const auto t = selectRecoveryJogTarget(kLive, kFlipped, kFlipped, params());
  EXPECT_FALSE(t.ok);
  EXPECT_TRUE(t.q.empty());
  EXPECT_FALSE(selectRecoveryJogTarget(kLive, V{}, V{}, params()).ok);
  EXPECT_FALSE(selectRecoveryJogTarget(V{}, kOnBranch, kOnBranch, params()).ok);
}

TEST(RecoveryBranchTest, WindingWristAndSizeMismatchAreRejected)
{
  V wound = kOnBranch;
  wound[4] = -4.6;
  EXPECT_FALSE(recoveryConfigOnBranch(wound, kLive, params()));
  EXPECT_FALSE(recoveryConfigOnBranch(V{0.1, 0.2}, kLive, params()));
  EXPECT_TRUE(recoveryConfigOnBranch(kOnBranch, kLive, params()));
}

TEST(RecoveryBranchTest, GoalChosenOnTheLiveBranch)
{
  // from the live seed the IK stays on the branch, from the branch seed it flips
  const auto ik = [](const V & seed) -> std::optional<V> {
      return seed == kLive ? kOnBranch : kFlipped;
    };
  const auto c = selectRecoveryGoal(ik, kLive, kFlipped, params());
  ASSERT_TRUE(c.ok);
  EXPECT_EQ(c.q, kOnBranch);
  EXPECT_EQ(c.seed_index, 0u);
}

TEST(RecoveryBranchTest, FlipRejectedFallsBackWhenNothingAcceptable)
{
  const auto ik = [](const V &) -> std::optional<V> {return kFlipped;};
  const auto c = selectRecoveryGoal(ik, kLive, kOnBranch, params());
  EXPECT_FALSE(c.ok);
  EXPECT_EQ(c.rejected_branch, 2u);
  const auto none = [](const V &) -> std::optional<V> {return std::nullopt;};
  EXPECT_FALSE(selectRecoveryGoal(none, kLive, kOnBranch, params()).ok);
}

TEST(RecoveryBranchTest, LiftMetresAreNotAnArmJumpAgainstLive)
{
  auto p = params();
  p.max_dq_rad = 0.1;
  V q = kLive;
  q[0] += 0.06;   // 0.06 m of lift
  EXPECT_TRUE(recoveryConfigOnBranch(q, kLive, p));
  q[2] += 0.2;    // 0.2 rad of arm
  EXPECT_FALSE(recoveryConfigOnBranch(q, kLive, p));
}

}  // namespace manipulation_manager
