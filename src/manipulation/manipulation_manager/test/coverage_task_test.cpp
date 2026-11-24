#include <gtest/gtest.h>

#include "manipulation_manager/core/coverage_task.hpp"

using namespace manipulation_manager;

TEST(AssignColumns, EachColumnGoesToTheStandThatReachesMostOfIt)
{
  // three columns of 4 poses: targets 0-3, 4-7, 8-11
  const std::vector<size_t> sizes = {4, 4, 4};
  const std::vector<std::vector<size_t>> stands = {{0, 1, 2, 3, 4, 5}, {6, 7, 8, 9, 10, 11}};
  const auto a = assignColumnsToStands(sizes, stands);
  ASSERT_EQ(a.columns_per_stand.size(), 2u);
  EXPECT_EQ(a.columns_per_stand[0], (std::vector<size_t>{0, 1}));   // column 1: 2 poses at stand 0, 2 at stand 1: tie -> first
  EXPECT_EQ(a.columns_per_stand[1], (std::vector<size_t>{2}));
  EXPECT_TRUE(a.unassigned.empty());
}

TEST(AssignColumns, MajorityBeatsOrder)
{
  const std::vector<size_t> sizes = {4};
  const auto a = assignColumnsToStands(sizes, {{0}, {1, 2, 3}});
  EXPECT_TRUE(a.columns_per_stand[0].empty());
  EXPECT_EQ(a.columns_per_stand[1], (std::vector<size_t>{0}));
}

TEST(AssignColumns, UncoveredColumnsAreUnassigned)
{
  const auto a = assignColumnsToStands({2, 2}, {{0, 1}});
  EXPECT_EQ(a.columns_per_stand[0], (std::vector<size_t>{0}));
  EXPECT_EQ(a.unassigned, (std::vector<size_t>{1}));
}

TEST(AssignColumns, WantedSubsetAndOutOfRangeIndices)
{
  const auto a = assignColumnsToStands({2, 2, 2}, {{0, 1, 2, 3, 4, 5, 99}}, {2, 0, 7});
  EXPECT_EQ(a.columns_per_stand[0], (std::vector<size_t>{0, 2}));   // ascending, 7 ignored
  EXPECT_TRUE(a.unassigned.empty());
}

TEST(TaskTracker, ResumedColumnsSurviveBeginMulti)
{
  TaskTracker t;
  t.beginSingle();
  t.columnResumed(0);
  t.columnResumed(1);
  t.columnSkipped(2);   // skipped for an overload on the earlier run
  t.beginMulti(1);
  t.standReached(0, true, "");
  t.columnFinished(0, 3, ColumnOutcome::kDone);
  EXPECT_EQ(t.done(), (std::vector<size_t>{0, 1, 3}));
  EXPECT_EQ(t.failed(), (std::vector<size_t>{2}));
  EXPECT_FALSE(t.success());
}

TEST(AssignColumns, NoStands)
{
  const auto a = assignColumnsToStands({2}, {});
  EXPECT_TRUE(a.columns_per_stand.empty());
  EXPECT_EQ(a.unassigned, (std::vector<size_t>{0}));
}

TEST(GroupByLateralSpan, SplitsAWideWallAndKeepsAscendingOrder)
{
  // 0.0 .. 0.8 in steps of 0.1, given out of order
  const std::vector<double> lat = {0.5, 0.0, 0.1, 0.8, 0.3, 0.2, 0.4, 0.6, 0.7};
  const auto g = groupByLateralSpan(lat, 0.45);
  ASSERT_EQ(g.size(), 2u);
  EXPECT_EQ(g[0], (std::vector<size_t>{1, 2, 5, 4, 6}));   // 0.0 .. 0.4
  EXPECT_EQ(g[1], (std::vector<size_t>{0, 7, 8, 3}));      // 0.5 .. 0.8
}

TEST(GroupByLateralSpan, OffOrNarrowWallIsOneGroup)
{
  const std::vector<double> lat = {0.0, 0.2, 0.4};
  EXPECT_EQ(groupByLateralSpan(lat, 0.0).size(), 1u);
  EXPECT_EQ(groupByLateralSpan(lat, 0.4).size(), 1u);
  EXPECT_EQ(groupByLateralSpan(lat, 0.39).size(), 2u);
  EXPECT_TRUE(groupByLateralSpan({}, 0.5).empty());
}

TEST(StateMachine, SingleStandRun)
{
  TaskStateMachine m;
  EXPECT_TRUE(m.startSingle());
  EXPECT_EQ(m.state(), TaskState::kRunningColumns);
  EXPECT_FALSE(m.startMulti());
  EXPECT_TRUE(m.singleFinished());
  EXPECT_EQ(m.state(), TaskState::kSucceeded);
  EXPECT_TRUE(m.terminal());
  EXPECT_FALSE(m.cancel());
}

TEST(StateMachine, MultiStandRunsEveryStandInOrder)
{
  TaskStateMachine m(2);
  ASSERT_TRUE(m.startMulti());
  EXPECT_EQ(m.state(), TaskState::kPlanningStands);
  ASSERT_TRUE(m.standsPlanned(2));
  EXPECT_EQ(m.state(), TaskState::kNavigating);
  EXPECT_EQ(m.navigationResult(true), NavVerdict::kRun);
  EXPECT_EQ(m.state(), TaskState::kRunningColumns);
  ASSERT_TRUE(m.standFinished());
  EXPECT_EQ(m.state(), TaskState::kNavigating);
  EXPECT_EQ(m.stand(), 1u);
  EXPECT_EQ(m.navigationResult(true), NavVerdict::kRun);
  ASSERT_TRUE(m.standFinished());
  EXPECT_EQ(m.state(), TaskState::kSucceeded);
}

TEST(StateMachine, FailedNavigationRetriesThenSkipsToTheNextStand)
{
  TaskStateMachine m(2);
  m.startMulti();
  m.standsPlanned(2);
  EXPECT_EQ(m.navigationResult(false), NavVerdict::kRetry);
  EXPECT_EQ(m.stand(), 0u);
  EXPECT_EQ(m.navigationResult(false), NavVerdict::kSkip);
  EXPECT_EQ(m.stand(), 1u);
  EXPECT_EQ(m.state(), TaskState::kNavigating);
  // the attempt counter is per stand: the second stand gets its two tries
  EXPECT_EQ(m.navigationResult(false), NavVerdict::kRetry);
  EXPECT_EQ(m.navigationResult(true), NavVerdict::kRun);
}

TEST(StateMachine, SkippingTheLastStandFinishes)
{
  TaskStateMachine m(1);
  m.startMulti();
  m.standsPlanned(1);
  EXPECT_EQ(m.navigationResult(false), NavVerdict::kFinish);
  EXPECT_EQ(m.state(), TaskState::kSucceeded);
}

TEST(StateMachine, NoStandsFails)
{
  TaskStateMachine m;
  m.startMulti();
  EXPECT_TRUE(m.standsPlanned(0));
  EXPECT_EQ(m.state(), TaskState::kFailed);

  TaskStateMachine n;
  n.startMulti();
  EXPECT_TRUE(n.standsFailed());
  EXPECT_EQ(n.state(), TaskState::kFailed);
}

TEST(StateMachine, CancelFromAnyLiveStateAndOutOfOrderCallsAreRefused)
{
  TaskStateMachine m;
  EXPECT_FALSE(m.standFinished());
  EXPECT_FALSE(m.standsPlanned(2));
  m.startMulti();
  m.standsPlanned(2);
  EXPECT_TRUE(m.cancel());
  EXPECT_EQ(m.state(), TaskState::kCanceled);
  EXPECT_FALSE(m.standFinished());
  EXPECT_EQ(m.navigationResult(true), NavVerdict::kFinish);   // not navigating any more
  EXPECT_EQ(m.state(), TaskState::kCanceled);
}

TEST(Tracker, RecordsPerStandAndTotals)
{
  TaskTracker t;
  t.beginMulti(3);
  t.standReached(0, true, "");
  t.columnFinished(0, 0, ColumnOutcome::kDone);
  t.columnFinished(0, 1, ColumnOutcome::kPartial);
  t.standReached(1, false, "navigation failed");
  t.standReached(2, true, "");
  t.columnFinished(2, 4, ColumnOutcome::kFailed);
  t.columnSkipped(3);
  EXPECT_EQ(t.done(), (std::vector<size_t>{0}));
  EXPECT_EQ(t.partial(), (std::vector<size_t>{1}));
  EXPECT_EQ(t.failed(), (std::vector<size_t>{4, 3}));
  EXPECT_EQ(t.stands()[0].partial, (std::vector<size_t>{1}));
  EXPECT_EQ(t.stands()[2].failed, (std::vector<size_t>{4}));
  EXPECT_FALSE(t.success());
  EXPECT_NE(t.summary().find("stand 1 not reached (navigation failed)"), std::string::npos);
}

TEST(Tracker, RetriedColumnReplacesItsEarlierOutcome)
{
  TaskTracker t;
  t.beginMulti(2);
  t.standReached(0, true, "");
  t.columnFinished(0, 0, ColumnOutcome::kDone);
  t.columnFinished(0, 1, ColumnOutcome::kFailed);
  t.columnFinished(0, 2, ColumnOutcome::kPartial);
  t.columnRetried(0, 1, ColumnOutcome::kDone);
  t.columnRetried(0, 2, ColumnOutcome::kFailed);
  EXPECT_EQ(t.done(), (std::vector<size_t>{0, 1}));
  EXPECT_TRUE(t.partial().empty());
  EXPECT_EQ(t.failed(), (std::vector<size_t>{2}));
  EXPECT_EQ(t.stands()[0].done, (std::vector<size_t>{0, 1}));
  EXPECT_EQ(t.stands()[0].failed, (std::vector<size_t>{2}));
}

TEST(Tracker, SuccessNeedsEveryColumnDoneAndEveryStandReached)
{
  TaskTracker t;
  t.beginMulti(2);
  t.standReached(0, true, "");
  t.columnFinished(0, 0, ColumnOutcome::kDone);
  t.standReached(1, true, "");
  t.columnFinished(1, 1, ColumnOutcome::kDone);
  EXPECT_TRUE(t.success());

  TaskTracker u;
  u.beginMulti(1);
  u.standReached(0, false, "no path");
  EXPECT_FALSE(u.success());

  TaskTracker s;
  s.beginSingle();
  s.columnFinished(0, 0, ColumnOutcome::kDone);
  EXPECT_TRUE(s.success());
  EXPECT_TRUE(s.stands().empty());
  s.columnFinished(0, 1, ColumnOutcome::kPartial);
  EXPECT_FALSE(s.success());
}

TEST(MergeNearDuplicateStands, MergesCloseNeighboursWhenOneStandServesBoth)
{
  std::vector<MergeableStand> stands = {{0.38, -0.61, 0.0, {0, 1}}, {0.38, -0.57, 0.0, {2, 3}},
    {0.38, -0.53, 0.0, {4}}, {0.38, 0.59, 0.0, {5, 6}}};
  std::vector<std::vector<size_t>> asked;
  const auto n = mergeNearDuplicateStands(stands, 0.15, [&](const std::vector<size_t> & t) {
      asked.push_back(t);
      return std::optional<MergeableStand>(MergeableStand{0.38, -0.57, 0.0, {}});
    });
  EXPECT_EQ(n, 2u);
  ASSERT_EQ(stands.size(), 2u);
  EXPECT_EQ(stands[0].targets, (std::vector<size_t>{0, 1, 2, 3, 4}));   // coverage identical
  EXPECT_EQ(stands[1].targets, (std::vector<size_t>{5, 6}));
  ASSERT_EQ(asked.size(), 2u);
  EXPECT_EQ(asked[1], (std::vector<size_t>{0, 1, 2, 3, 4}));
}

TEST(MergeNearDuplicateStands, RefusedPairAndFarStandsAreKept)
{
  std::vector<MergeableStand> stands = {{0.0, 0.0, 0.0, {0}}, {0.0, 0.04, 0.0, {1}}, {0.0, 1.0, 0.0, {2}}};
  EXPECT_EQ(mergeNearDuplicateStands(stands, 0.15, [](const std::vector<size_t> &) {return std::nullopt;}), 0u);
  EXPECT_EQ(stands.size(), 3u);
  size_t calls = 0;
  EXPECT_EQ(mergeNearDuplicateStands(stands, 0.15, [&](const std::vector<size_t> &) {
        ++calls;
        return std::optional<MergeableStand>(MergeableStand{});
      }), 1u);
  EXPECT_EQ(calls, 1u);   // the 1 m neighbour is never offered
}

TEST(MergeNearDuplicateStands, ZeroToleranceIsOff)
{
  std::vector<MergeableStand> stands = {{0.0, 0.0, 0.0, {0}}, {0.0, 0.01, 0.0, {1}}};
  EXPECT_EQ(mergeNearDuplicateStands(stands, 0.0, [](const std::vector<size_t> &) {
        return std::optional<MergeableStand>(MergeableStand{});
      }), 0u);
  EXPECT_EQ(stands.size(), 2u);
}
