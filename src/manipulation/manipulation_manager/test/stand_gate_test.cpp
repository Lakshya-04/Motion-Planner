#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "manipulation_manager/core/stand_gate.hpp"

using namespace manipulation_manager;

namespace
{
/// Fake navigator: each drive "arrives" with the next scripted error; a drive can also fail.
class FakeNav : public StandNavigatorPort
{
public:
  FakeNav(StandAchieved first, std::vector<StandAchieved> redock_arrivals = {})
  : current_(first), script_(std::move(redock_arrivals)) {}
  bool navigate(std::string & m) override
  {
    ++drives;
    if (fail_drive) {
      m = "nav goal aborted";
      return false;
    }
    if (next_ < script_.size()) {
      current_ = script_[next_++];
    }
    return true;
  }
  StandAchieved achieved() const override {return current_;}
  bool stopRequested() const override {return stop;}
  int drives{0};
  bool fail_drive{false};
  bool stop{false};

private:
  StandAchieved current_;
  std::vector<StandAchieved> script_;
  size_t next_{0};
};

StandCheck check(int max_redocks)
{
  StandCheckConfig c;
  c.max_redocks = max_redocks;
  return StandCheck(c);
}
const StandAchieved kGood{0.005, 0.01};
const StandAchieved kBad{0.30, 0.01};   // beyond max_tol_m 0.15
}  // namespace

TEST(StandGate, WithinToleranceGoesWithoutADrive)
{
  FakeNav nav(kGood);
  const auto o = runStandGate(nav, check(1), 0.1);
  EXPECT_TRUE(o.ok);
  EXPECT_FALSE(o.abort);
  EXPECT_EQ(o.redocks, 0);
  EXPECT_EQ(nav.drives, 0);
}

TEST(StandGate, MissThenRedockThenGo)
{
  FakeNav nav(kBad, {kGood});
  const auto o = runStandGate(nav, check(1), 0.1);
  EXPECT_TRUE(o.ok);
  EXPECT_EQ(o.redocks, 1);
  EXPECT_EQ(nav.drives, 1);
}

TEST(StandGate, MissAfterTheRedockBudgetAbortsTheStand)
{
  FakeNav nav(kBad, {kBad});
  const auto o = runStandGate(nav, check(1), 0.1);
  EXPECT_FALSE(o.ok);
  EXPECT_TRUE(o.abort);
  EXPECT_EQ(o.reason, StandReason::kRedocksExhausted);
  EXPECT_EQ(nav.drives, 1);
  EXPECT_NE(o.message.find("stand check"), std::string::npos);
}

TEST(StandGate, ZeroRedocksAbortsOnTheFirstMiss)
{
  FakeNav nav(kBad);
  const auto o = runStandGate(nav, check(0), 0.1);
  EXPECT_TRUE(o.abort);
  EXPECT_EQ(nav.drives, 0);
}

TEST(StandGate, InvalidMeasurementAborts)
{
  FakeNav nav({std::nan(""), 0.0});
  const auto o = runStandGate(nav, check(2), 0.1);
  EXPECT_TRUE(o.abort);
  EXPECT_EQ(o.reason, StandReason::kInvalidMeasurement);
  EXPECT_EQ(nav.drives, 0);
}

TEST(StandGate, FailedRedockDriveIsARetryNotAnAbort)
{
  FakeNav nav(kBad);
  nav.fail_drive = true;
  const auto o = runStandGate(nav, check(2), 0.1);
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.abort);
  EXPECT_EQ(o.message, "nav goal aborted");
  EXPECT_EQ(o.redocks, 1);
}

TEST(StandGate, CancelDuringAMissDoesNotDrive)
{
  FakeNav nav(kBad);
  nav.stop = true;
  const auto o = runStandGate(nav, check(2), 0.1);
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.abort);
  EXPECT_EQ(o.message, "canceled");
  EXPECT_EQ(nav.drives, 0);
}

TEST(StandGate, TwoRedocksBeforeGo)
{
  FakeNav nav(kBad, {kBad, kGood});
  const auto o = runStandGate(nav, check(2), 0.1);
  EXPECT_TRUE(o.ok);
  EXPECT_EQ(o.redocks, 2);
}

TEST(StandGate, UnknownMarginUsesTheFallbackTolerance)
{
  FakeNav nav({0.10, 0.0});   // inside the 0.15 fallback, outside 0.5 * 0.1 = 0.05
  EXPECT_TRUE(runStandGate(nav, check(0), -1.0).ok);
  FakeNav nav2({0.10, 0.0});
  EXPECT_TRUE(runStandGate(nav2, check(0), 0.1).abort);
}

TEST(ResolveNavigation, PlainFailureRetriesWhileAttemptsRemain)
{
  TaskStateMachine m(2);
  ASSERT_TRUE(m.startMulti());
  ASSERT_TRUE(m.standsPlanned(2));
  EXPECT_EQ(resolveNavigation(m, false, false), NavVerdict::kRetry);
  EXPECT_EQ(m.stand(), 0u);
}

TEST(ResolveNavigation, GateAbortSkipsToTheNextStandAtOnce)
{
  TaskStateMachine m(3);
  ASSERT_TRUE(m.startMulti());
  ASSERT_TRUE(m.standsPlanned(2));
  EXPECT_EQ(resolveNavigation(m, false, true), NavVerdict::kSkip);
  EXPECT_EQ(m.stand(), 1u);
  EXPECT_EQ(m.state(), TaskState::kNavigating);
}

TEST(ResolveNavigation, GateAbortOnTheLastStandFinishes)
{
  TaskStateMachine m(3);
  ASSERT_TRUE(m.startMulti());
  ASSERT_TRUE(m.standsPlanned(1));
  EXPECT_EQ(resolveNavigation(m, false, true), NavVerdict::kFinish);
  EXPECT_EQ(m.state(), TaskState::kSucceeded);
}

TEST(ResolveNavigation, ArrivalRunsTheColumns)
{
  TaskStateMachine m(2);
  ASSERT_TRUE(m.startMulti());
  ASSERT_TRUE(m.standsPlanned(1));
  EXPECT_EQ(resolveNavigation(m, true, false), NavVerdict::kRun);
  EXPECT_EQ(m.state(), TaskState::kRunningColumns);
}

TEST(ResolveNavigation, GateGoThenColumnsThenNextStandViaTheMachine)
{
  // Whole path for two stands: stand 0 misses and is given up, stand 1 arrives within tolerance.
  TaskStateMachine m(2);
  ASSERT_TRUE(m.startMulti());
  ASSERT_TRUE(m.standsPlanned(2));
  FakeNav bad(kBad);
  const auto o0 = runStandGate(bad, check(0), 0.1);
  EXPECT_EQ(resolveNavigation(m, o0.ok, o0.abort), NavVerdict::kSkip);
  FakeNav good(kGood);
  const auto o1 = runStandGate(good, check(0), 0.1);
  EXPECT_EQ(resolveNavigation(m, o1.ok, o1.abort), NavVerdict::kRun);
  EXPECT_EQ(m.stand(), 1u);
}
