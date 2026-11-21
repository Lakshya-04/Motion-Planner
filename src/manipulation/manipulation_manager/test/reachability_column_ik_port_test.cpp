#include <gtest/gtest.h>

#include <cmath>

#include "manipulation_manager/adapters/reachability_column_ik_port.hpp"

using namespace manipulation_manager;

namespace
{
// Fake IK for a lift + 2 joints. q = {lift, 0.5 z, +-0.8}: poses with z > 0.15 only have branch B (-0.8); below that both exist
// and the one chosen follows the seed's lift (lift above 0.3 -> B, else A), which then persists along the chain.
std::optional<std::vector<double>> fakeIk(const core_types::Pose & p, const std::vector<double> & seed)
{
  const double lift = seed[0];
  const bool only_b = p.z > 0.15;
  const bool b = only_b || lift > 0.3;
  return std::vector<double>{lift, 0.5 * p.z, b ? -0.8 : 0.8};
}

core_types::Pose at(double z)
{
  core_types::Pose p;
  p.x = 0.98;
  p.y = 0.0;
  p.z = z;
  p.qw = 1.0;
  return p;
}

ReachabilityColumnIkConfig config()
{
  ReachabilityColumnIkConfig c;
  c.sequence.standoff_m = 0.12;
  c.sequence.approach_extra_m = 0.0;
  c.sequence.press_depth_m = 0.0;
  c.chain.branch_joints = {2};
  return c;
}

const std::vector<core_types::Pose> kStrokes{at(0.0), at(0.1), at(0.2), at(0.3)};
const std::array<double, 3> kNormal{-1.0, 0.0, 0.0};
}  // namespace

TEST(ReachabilityColumnIkPort, ASeedOnTheRightBranchIsContinuous)
{
  ReachabilityColumnIkPort port(config(), fakeIk);
  const auto v = port.check(kStrokes, kNormal, {0.5, 0.0, 0.0});
  EXPECT_TRUE(v.solved);
  EXPECT_TRUE(v.continuous);
  EXPECT_EQ(v.split_count, 0u);
  EXPECT_TRUE(v.failure.empty());
}

TEST(ReachabilityColumnIkPort, ABranchFlipIsReportedWithItsPoseAndRole)
{
  ReachabilityColumnIkPort port(config(), fakeIk);
  const auto v = port.check(kStrokes, kNormal, {0.1, 0.0, 0.0});
  EXPECT_TRUE(v.solved);
  EXPECT_FALSE(v.continuous);
  EXPECT_GE(v.split_count, 1u);
  EXPECT_EQ(v.failure, "branch_break");
  EXPECT_NE(v.failure_pose, kNoIkPose);
  EXPECT_EQ(v.failure_role, "stroke");
  EXPECT_GT(v.max_continuity_ratio, 6.0);
}

TEST(ReachabilityColumnIkPort, GateRescuesTheBreakWithAnotherLiftHeight)
{
  ReachabilityColumnIkPort port(config(), fakeIk);
  ColumnIkGateConfig g;
  g.lift_candidates = {0.2, 0.5};
  const auto d = ColumnIkGate(g).decide(&port, kStrokes, kNormal, {0.1, 0.0, 0.0});
  EXPECT_EQ(d.action, ColumnIkAction::kGoWithSeed);
  EXPECT_DOUBLE_EQ(d.seed[0], 0.5);
  EXPECT_EQ(d.attempts, 3u);
}

TEST(ReachabilityColumnIkPort, NoIkSolutionIsRefusedAtThatPose)
{
  ReachabilityColumnIkPort port(
    config(), [](const core_types::Pose & p, const std::vector<double> & s) -> std::optional<std::vector<double>> {
      if (p.z > 0.25) {return std::nullopt;}
      return fakeIk(p, s);
    });
  const auto v = port.check(kStrokes, kNormal, {0.5, 0.0, 0.0});
  EXPECT_FALSE(v.solved);
  EXPECT_EQ(v.failure, "refused");
  EXPECT_EQ(v.failure_role, "stroke");
}

TEST(ReachabilityColumnIkPort, WrongSizedIkAnswerIsRefusedNotACrash)
{
  ReachabilityColumnIkPort port(
    config(), [](const core_types::Pose &, const std::vector<double> &) -> std::optional<std::vector<double>> {
      return std::vector<double>{1.0};
    });
  const auto v = port.check(kStrokes, kNormal, {0.5, 0.0, 0.0});
  EXPECT_EQ(v.failure, "refused");
}

TEST(ReachabilityColumnIkPort, EmptyInputsGiveAnUncheckedVerdict)
{
  ReachabilityColumnIkPort port(config(), fakeIk);
  EXPECT_FALSE(port.check({}, kNormal, {0.5, 0.0, 0.0}).continuous);
  EXPECT_FALSE(port.check(kStrokes, kNormal, {}).continuous);
  ReachabilityColumnIkPort noik(config(), nullptr);
  EXPECT_FALSE(noik.check(kStrokes, kNormal, {0.5, 0.0, 0.0}).solved);
}
