#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "manipulation_manager/core/wall_dock_gate.hpp"

using namespace manipulation_manager;

namespace
{
class FakeDock : public WallDockPort
{
public:
  explicit FakeDock(std::vector<WallDockMeasured> s) : script_(std::move(s)) {}
  WallDockMeasured dock() override
  {
    ++goals;
    const auto m = script_[std::min(next_, script_.size() - 1)];
    ++next_;
    return m;
  }
  bool stopRequested() const override {return stop;}
  int goals{0};
  bool stop{false};

private:
  std::vector<WallDockMeasured> script_;
  size_t next_{0};
};
WallDockMeasured good() {return {true, 0.004, 0.01, ""};}
WallDockMeasured farOff() {return {true, -0.08, 0.01, ""};}
WallDockMeasured skewed() {return {true, 0.0, 0.3, ""};}
WallDockMeasured failed() {return {false, 0.0, 0.0, "no fresh plane"};}
WallDockConfig cfg(int attempts)
{
  WallDockConfig c;
  c.max_attempts = attempts;
  return c;
}
}  // namespace

TEST(WallDockGate, WithinToleranceGoesWithOneGoal)
{
  FakeDock d({good()});
  const auto o = runWallDock(d, cfg(2));
  EXPECT_TRUE(o.ok);
  EXPECT_EQ(o.attempts, 1);
  EXPECT_TRUE(o.measured);
  EXPECT_DOUBLE_EQ(o.standoff_error_m, 0.004);
}

TEST(WallDockGate, MissThenRedockThenGo)
{
  FakeDock d({farOff(), good()});
  const auto o = runWallDock(d, cfg(2));
  EXPECT_TRUE(o.ok);
  EXPECT_EQ(o.attempts, 2);
  EXPECT_DOUBLE_EQ(o.standoff_error_m, 0.004);   // the last measurement is kept
}

TEST(WallDockGate, MissOutOfBudgetAborts)
{
  FakeDock d({farOff(), skewed()});
  const auto o = runWallDock(d, cfg(2));
  EXPECT_FALSE(o.ok);
  EXPECT_TRUE(o.abort);
  EXPECT_EQ(d.goals, 2);
  EXPECT_DOUBLE_EQ(o.yaw_error_rad, 0.3);
  EXPECT_NE(o.message.find("wall dock check"), std::string::npos);
}

TEST(WallDockGate, SingleAttemptAbortsOnTheFirstMiss)
{
  FakeDock d({skewed()});
  const auto o = runWallDock(d, cfg(1));
  EXPECT_TRUE(o.abort);
  EXPECT_EQ(d.goals, 1);
}

TEST(WallDockGate, ZeroAttemptsStillSendsOneGoal)
{
  FakeDock d({good()});
  EXPECT_TRUE(runWallDock(d, cfg(0)).ok);
  EXPECT_EQ(d.goals, 1);
}

TEST(WallDockGate, FailedDockIsRetriedThenNotAnAbort)
{
  FakeDock d({failed()});
  const auto o = runWallDock(d, cfg(3));
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.abort);
  EXPECT_FALSE(o.measured);
  EXPECT_EQ(d.goals, 3);
  EXPECT_EQ(o.message, "no fresh plane");
}

TEST(WallDockGate, FailedDockThenGood)
{
  FakeDock d({failed(), good()});
  const auto o = runWallDock(d, cfg(2));
  EXPECT_TRUE(o.ok);
  EXPECT_EQ(o.attempts, 2);
}

TEST(WallDockGate, InvalidMeasurementAborts)
{
  FakeDock d({{true, std::nan(""), 0.0, ""}});
  const auto o = runWallDock(d, cfg(3));
  EXPECT_TRUE(o.abort);
  EXPECT_EQ(d.goals, 1);
}

TEST(WallDockGate, CancelSendsNoGoal)
{
  FakeDock d({good()});
  d.stop = true;
  const auto o = runWallDock(d, cfg(2));
  EXPECT_FALSE(o.ok);
  EXPECT_FALSE(o.abort);
  EXPECT_EQ(o.message, "canceled");
  EXPECT_EQ(d.goals, 0);
}

TEST(WallDockGate, PlannedStandoffIsTheDistanceToThePlanAlongTheWallDirection)
{
  // stand at x 0.08, wall at x 0.94, wall on +x
  EXPECT_NEAR(plannedStandoff(0.94, 0.5, 0.08, 0.01, 1.0, 0.0), 0.86, 1e-12);
  // wall on +y: only y counts
  EXPECT_NEAR(plannedStandoff(3.0, 1.2, -1.0, 0.2, 0.0, 1.0), 1.0, 1e-12);
  // slanted direction projects both
  EXPECT_NEAR(plannedStandoff(1.0, 1.0, 0.0, 0.0, std::sqrt(0.5), std::sqrt(0.5)), std::sqrt(2.0), 1e-12);
}
