#include <gtest/gtest.h>

#include <functional>

#include "manipulation_manager/core/column_ik_gate.hpp"

using namespace manipulation_manager;

namespace
{
// A port whose verdict is a function of the seed's lift (entry 0), recording the lifts it was asked about.
class FakePort : public ColumnIkCheckPort
{
public:
  std::function<ColumnIkVerdict(double lift)> verdict;
  std::vector<double> asked;
  int calls{0};
  ColumnIkVerdict check(
    const std::vector<core_types::Pose> &, const std::array<double, 3> &, const std::vector<double> & seed) override
  {
    ++calls;
    asked.push_back(seed[0]);
    return verdict(seed[0]);
  }
};

ColumnIkVerdict ok()
{
  ColumnIkVerdict v;
  v.solved = true;
  v.continuous = true;
  return v;
}

ColumnIkVerdict broken(size_t pose = 3, double ratio = 12.0)
{
  ColumnIkVerdict v;
  v.solved = true;
  v.split_count = 1;
  v.failure = "branch_break";
  v.failure_pose = pose;
  v.failure_role = "stroke";
  v.max_continuity_ratio = ratio;
  return v;
}

ColumnIkVerdict refused(size_t pose = 2)
{
  ColumnIkVerdict v;
  v.failure = "refused";
  v.failure_pose = pose;
  v.failure_role = "stroke";
  return v;
}

const std::vector<core_types::Pose> kStrokes(3);
const std::array<double, 3> kNormal{-1.0, 0.0, 0.0};
const std::vector<double> kSeed{0.1, 0.0, 1.0};
}  // namespace

TEST(ColumnIkGate, ContinuousFromTheGivenSeedIsGoWithOneCall)
{
  FakePort p;
  p.verdict = [](double) {return ok();};
  const auto d = ColumnIkGate(ColumnIkGateConfig{}).decide(&p, kStrokes, kNormal, kSeed);
  EXPECT_EQ(d.action, ColumnIkAction::kGo);
  EXPECT_EQ(d.attempts, 1u);
  EXPECT_EQ(d.seed, kSeed);
  EXPECT_EQ(p.calls, 1);
}

TEST(ColumnIkGate, ABreakIsRescuedByAnotherLiftHeight)
{
  FakePort p;
  p.verdict = [](double lift) {return lift > 0.4 ? ok() : broken();};
  ColumnIkGateConfig c;
  c.lift_candidates = {0.3, 0.5, 0.7};
  const auto d = ColumnIkGate(c).decide(&p, kStrokes, kNormal, kSeed);
  EXPECT_EQ(d.action, ColumnIkAction::kGoWithSeed);
  EXPECT_DOUBLE_EQ(d.seed[0], 0.5);
  EXPECT_DOUBLE_EQ(d.seed[1], 0.0);   // the rest of the seed is untouched
  EXPECT_EQ(d.attempts, 3u);          // given seed, 0.3, 0.5: stops at the first continuous one
  EXPECT_NE(d.reason.find("lift 0.5"), std::string::npos);
}

TEST(ColumnIkGate, EveryAttemptBreakingRejectsWithTheBestVerdict)
{
  FakePort p;
  p.verdict = [](double lift) {return broken(3, 20.0 - 10.0 * lift);};   // a higher lift breaks less badly
  ColumnIkGateConfig c;
  c.lift_candidates = {0.3, 0.6};
  const auto d = ColumnIkGate(c).decide(&p, kStrokes, kNormal, kSeed);
  EXPECT_EQ(d.action, ColumnIkAction::kReject);
  EXPECT_EQ(d.attempts, 3u);
  EXPECT_NEAR(d.verdict.max_continuity_ratio, 14.0, 1e-9);   // the lift 0.6 attempt
  EXPECT_NE(d.reason.find("pose 3"), std::string::npos);
}

TEST(ColumnIkGate, RejectOnBreakCanBeSwitchedOff)
{
  FakePort p;
  p.verdict = [](double) {return broken();};
  ColumnIkGateConfig c;
  c.reject_on_break = false;
  EXPECT_EQ(ColumnIkGate(c).decide(&p, kStrokes, kNormal, kSeed).action, ColumnIkAction::kGo);
}

TEST(ColumnIkGate, RefusedPosesAreLetThroughUnlessAskedToReject)
{
  FakePort p;
  p.verdict = [](double) {return refused();};
  const auto pass = ColumnIkGate(ColumnIkGateConfig{}).decide(&p, kStrokes, kNormal, kSeed);
  EXPECT_EQ(pass.action, ColumnIkAction::kGo);
  EXPECT_NE(pass.reason.find("refused"), std::string::npos);
  ColumnIkGateConfig c;
  c.reject_on_refused = true;
  EXPECT_EQ(ColumnIkGate(c).decide(&p, kStrokes, kNormal, kSeed).action, ColumnIkAction::kReject);
}

TEST(ColumnIkGate, RefusedFromTheSeedButSolvedElsewhereUsesThatSeed)
{
  FakePort p;
  p.verdict = [](double lift) {return lift > 0.4 ? ok() : refused();};
  ColumnIkGateConfig c;
  c.lift_candidates = {0.5};
  const auto d = ColumnIkGate(c).decide(&p, kStrokes, kNormal, kSeed);
  EXPECT_EQ(d.action, ColumnIkAction::kGoWithSeed);
}

TEST(ColumnIkGate, MixedRefusedAndBrokenCountsAsABreak)
{
  FakePort p;
  p.verdict = [](double lift) {return lift > 0.4 ? broken() : refused();};
  ColumnIkGateConfig c;
  c.lift_candidates = {0.5};
  EXPECT_EQ(ColumnIkGate(c).decide(&p, kStrokes, kNormal, kSeed).action, ColumnIkAction::kReject);
}

TEST(ColumnIkGate, MissingInputsAreNotChecked)
{
  FakePort p;
  p.verdict = [](double) {return ok();};
  EXPECT_EQ(ColumnIkGate(ColumnIkGateConfig{}).decide(nullptr, kStrokes, kNormal, kSeed).action, ColumnIkAction::kNotChecked);
  EXPECT_EQ(ColumnIkGate(ColumnIkGateConfig{}).decide(&p, {}, kNormal, kSeed).action, ColumnIkAction::kNotChecked);
  EXPECT_EQ(ColumnIkGate(ColumnIkGateConfig{}).decide(&p, kStrokes, kNormal, {}).action, ColumnIkAction::kNotChecked);
  EXPECT_EQ(p.calls, 0);
}

TEST(ColumnIkGate, LiftJointOutsideTheSeedSkipsTheCandidates)
{
  FakePort p;
  p.verdict = [](double) {return broken();};
  ColumnIkGateConfig c;
  c.lift_joint = 7;
  c.lift_candidates = {0.3, 0.5};
  const auto d = ColumnIkGate(c).decide(&p, kStrokes, kNormal, kSeed);
  EXPECT_EQ(d.action, ColumnIkAction::kReject);
  EXPECT_EQ(d.attempts, 1u);
}
