/**
 * @file mission_tree_test.cpp
 * @brief gtests of the full mission tree (trees/mission.xml) over the fake ports. No ROS, no simulator.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <functional>
#include <regex>
#include <sstream>

#include "manipulation_bt_prototype/mission_fakes.hpp"
#include "manipulation_bt_prototype/mission_nodes.hpp"

using namespace manipulation_bt::mission;
using manipulation_manager::MissionState;

namespace
{

std::string readFile(const char * path)
{
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

FakeArm::Key K(ArmPhase p, size_t column) {return FakeArm::Key(p, column);}

Script failing(Reason r = Reason::kPlanFailed) {return Script{0, true, r, 0.0};}
Script slow(int polls) {return Script{polls, false, Reason::kNone, 0.0};}

const std::vector<std::string> kCleanupOrder{"cancel_trajectory", "tool_off", "admittance_off", "force_zero", "valve_closed"};

struct Fix
{
  BT::BehaviorTreeFactory factory;
  MissionCtxPtr ctx = std::make_shared<MissionCtx>();
  FakeWorld w;
  BT::Tree tree;
  int tick_no{0};

  explicit Fix(
    size_t stands = 1, size_t columns = 4, const std::function<std::string(std::string)> & edit = nullptr)
  {
    w = makeFakeWorld(*ctx, stands, columns);
    ctx->clock = [this]() {return tick_no * 0.1;};
    registerMissionNodes(factory, ctx);
    std::string xml = readFile(MISSION_TREE_FILE);
    if (edit) {xml = edit(xml);}
    tree = factory.createTreeFromText(xml);
  }

  /// Ticks until the tree stops running; `hook(tick)` runs before each tick (commands and sensor values arrive
  /// between ticks).
  BT::NodeStatus run(const std::function<void(int)> & hook = nullptr, int max_ticks = 3000)
  {
    BT::NodeStatus st = BT::NodeStatus::RUNNING;
    for (int i = 0; i < max_ticks && st == BT::NodeStatus::RUNNING; ++i) {
      tick_no = i;
      if (hook) {hook(i);}
      st = tree.tickOnce();
    }
    return st;
  }

  /// Ticks until `event` is in the log (or the tree ends); the tree is left running when it was seen.
  BT::NodeStatus runUntil(const std::string & event, int max_ticks = 3000)
  {
    BT::NodeStatus st = BT::NodeStatus::RUNNING;
    for (int i = 0; i < max_ticks && st == BT::NodeStatus::RUNNING; ++i) {
      tick_no = i;
      if (has(event)) {break;}
      st = tree.tickOnce();
    }
    return st;
  }

  int idx(const std::string & e, size_t from = 0) const
  {
    for (size_t i = from; i < ctx->events.size(); ++i) {
      if (ctx->events[i] == e) {return static_cast<int>(i);}
    }
    return -1;
  }
  bool has(const std::string & e) const {return idx(e) >= 0;}
  size_t count(const std::string & e) const {return std::count(ctx->events.begin(), ctx->events.end(), e);}
  size_t countPrefix(const std::string & p) const
  {
    return std::count_if(
      ctx->events.begin(), ctx->events.end(), [&](const std::string & e) {return e.rfind(p, 0) == 0;});
  }
  const MissionReport & report() const {return w.sink->reports.back();}

  void expectCleanupAfter(int halt_index) const
  {
    ASSERT_GE(halt_index, 0);
    ASSERT_LT(static_cast<size_t>(halt_index) + kCleanupOrder.size(), ctx->events.size());
    for (size_t k = 0; k < kCleanupOrder.size(); ++k) {
      EXPECT_EQ(ctx->events[halt_index + 1 + k], kCleanupOrder[k]) << "cleanup step " << k;
    }
  }
};

}  // namespace

// ------------------------------------------------------------------------------------------------ structure

TEST(MissionTree, EveryNodeOfTheModelIsRegistered)
{
  Fix f;
  const std::string xml = readFile(MISSION_TREE_FILE);
  const std::regex re("<(Action|Condition|Control|Decorator)\\s+ID=\"([A-Za-z]+)\"");
  size_t n = 0;
  for (auto it = std::sregex_iterator(xml.begin(), xml.end(), re); it != std::sregex_iterator(); ++it) {
    EXPECT_EQ(f.factory.manifests().count((*it)[2].str()), 1u) << (*it)[2];
    ++n;
  }
  EXPECT_GE(n, 27u);
}

TEST(MissionTree, ReasonVocabularyMapsOntoTheManagersSkipReasons)
{
  EXPECT_EQ(toSkipReason(Reason::kNavFailed), SkipReason::kNavFailed);
  EXPECT_EQ(toSkipReason(Reason::kDockFailed), SkipReason::kDockFailed);
  EXPECT_EQ(toSkipReason(Reason::kStandCheckFailed), SkipReason::kDockFailed);
  EXPECT_EQ(toSkipReason(Reason::kContactOverload), SkipReason::kContactOverload);
  EXPECT_EQ(toSkipReason(Reason::kPlanFailed), SkipReason::kPlanFailed);
  EXPECT_EQ(toSkipReason(Reason::kVerifyFailed), SkipReason::kPlanFailed);
  EXPECT_EQ(toSkipReason(Reason::kSafetyTimeout), SkipReason::kPlanFailed);
  EXPECT_STREQ(toString(Reason::kNavFailed), "NAV_FAILED");
  EXPECT_STREQ(toString(Reason::kStandCheckFailed), "STAND_CHECK_FAILED");
  EXPECT_TRUE(isRetryable(Reason::kPlanFailed));
  EXPECT_TRUE(isRetryable(Reason::kVerifyFailed));
  EXPECT_FALSE(isRetryable(Reason::kContactOverload));
  EXPECT_FALSE(isRetryable(Reason::kSafetyTimeout));
  EXPECT_FALSE(isRetryable(Reason::kCalibrationInvalid));
}

// ------------------------------------------------------------------------------------------------ happy path

TEST(MissionTree, HappyPathDoesEveryColumnAndReportsComplete)
{
  Fix f;
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(f.w.sink->reports.size(), 1u);
  const auto & r = f.report();
  EXPECT_EQ(r.outcome, "finished");
  EXPECT_TRUE(r.success);
  EXPECT_EQ(r.message, "ok");
  EXPECT_EQ(r.columns_done, (std::vector<size_t>{0, 1, 2, 3}));
  EXPECT_TRUE(r.columns_skipped.empty());
  EXPECT_TRUE(r.columns_failed.empty());
  ASSERT_EQ(r.stands.size(), 1u);
  EXPECT_TRUE(r.stands[0].reached);
  for (size_t c = 0; c < 4; ++c) {
    EXPECT_TRUE(f.ctx->ledger.isDone(c));
    EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kApproach, c), 1u);
    EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kPress, c), 1u);
    EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kStroke, c), 1u);
    EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kRetract, c), 1u);
  }
  EXPECT_EQ(f.countPrefix("verify:"), 4u);
}

TEST(MissionTree, HappyPathEndsWithStowToolOffThenTheOnlyReport)
{
  Fix f;
  f.run();
  const int stow = f.idx("arm.start:stow:3:0");   // stow carries the last column index
  const int off = f.idx("tool_off", stow);
  const int rep = f.idx("report:finished");
  EXPECT_GE(stow, 0);
  EXPECT_GT(off, stow);
  EXPECT_GT(rep, off);
  EXPECT_EQ(f.count("report:finished"), 1u);
  EXPECT_EQ(f.ctx->events.back(), "report:finished");
}

TEST(MissionTree, StagesRunInTheDocumentedOrder)
{
  Fix f;
  f.run();
  const int scan = f.idx("perception.scan");
  const int plan = f.idx("planner.coverage");
  const int stands = f.idx("planner.stands");
  const int nav = f.idx("nav.go:0");
  const int dock = f.idx("dock.go:0");
  const int arm = f.idx("arm.start:approach:0:0");
  EXPECT_GE(scan, 0);
  EXPECT_LT(scan, plan);
  EXPECT_LT(plan, stands);
  EXPECT_LT(stands, nav);
  EXPECT_LT(nav, dock);
  EXPECT_LT(dock, arm);
}

TEST(MissionTree, TwoStandsAreVisitedInOrderAndAllColumnsDone)
{
  Fix f(2, 2);
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.navigation->starts, (std::vector<int>{0, 1}));
  EXPECT_EQ(f.w.dock->starts, (std::vector<int>{0, 1}));
  EXPECT_EQ(f.report().columns_done, (std::vector<size_t>{0, 1, 2, 3}));
  ASSERT_EQ(f.report().stands.size(), 2u);
  EXPECT_TRUE(f.report().stands[0].reached);
  EXPECT_TRUE(f.report().stands[1].reached);
}

TEST(MissionTree, FeedbackStatesFollowTheFlow)
{
  Fix f;
  f.run();
  const auto & s = f.w.sink->states;
  const auto pos = [&](const std::string & x) {return std::find(s.begin(), s.end(), x) - s.begin();};
  EXPECT_LT(pos("perceiving"), pos("planning_coverage"));
  EXPECT_LT(pos("planning_stands"), pos("navigating"));
  EXPECT_LT(pos("navigating"), pos("docking"));
  EXPECT_LT(pos("docking"), pos("pressing"));
  EXPECT_LT(pos("pressing"), pos("stroking"));
  EXPECT_LT(pos("stroking"), pos("retracting"));
  EXPECT_LT(pos("retracting"), pos("verifying"));
}

TEST(MissionTree, WantedColumnsReachThePlannerAndPeakForceTheReport)
{
  Fix f;
  f.ctx->wanted_columns = {1, 2};
  f.w.arm->peak_force = 12.5;
  f.run();
  EXPECT_EQ(f.w.planner->wanted_seen, (std::vector<size_t>{1, 2}));
  EXPECT_DOUBLE_EQ(f.report().peak_force_n, 12.5);
}

TEST(MissionTree, NonContactToolSkipsThePressAndNeedsNoCalibration)
{
  Fix f;
  f.w.tool->is_contact = false;
  f.w.calibration->rec.valid = false;
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.calibration->touches, 0);
  EXPECT_EQ(f.count("skip:press"), 4u);
  for (size_t c = 0; c < 4; ++c) {EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kPress, c), 0u);}
  EXPECT_TRUE(f.report().success);
}

TEST(MissionTree, SameScriptGivesTheSameRunTwice)
{
  const auto scenario = [](Fix & f) {
      f.w.arm->script.add(K(ArmPhase::kStroke, 1), failing(), 2);
      f.w.navigation->script.add(1, failing());
      f.w.verify->script.add(2, failing(Reason::kVerifyFailed));
      f.run();
    };
  Fix a(2, 2);
  Fix b(2, 2);
  scenario(a);
  scenario(b);
  EXPECT_EQ(a.ctx->events, b.ctx->events);
  EXPECT_EQ(a.report().columns_done, b.report().columns_done);
  EXPECT_EQ(a.report().columns_skipped, b.report().columns_skipped);
}

// ------------------------------------------------------------------------------------------------ mission-level failures

TEST(MissionTree, ScanFailureIsRetriedOnce)
{
  Fix f;
  f.w.perception->script.add(0, failing(Reason::kPerceptionFailed));
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.perception->starts, 2);
  EXPECT_TRUE(f.report().success);
}

TEST(MissionTree, ScanFailingTwiceAbortsButStillStowsToolOffAndReportsOnce)
{
  Fix f;
  f.w.perception->script.failAlways(0, Reason::kPerceptionFailed);
  EXPECT_EQ(f.run(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(f.w.perception->starts, 2);
  EXPECT_FALSE(f.has("nav.go:0"));
  ASSERT_EQ(f.w.sink->reports.size(), 1u);
  EXPECT_EQ(f.report().outcome, "aborted");
  EXPECT_FALSE(f.report().success);
  EXPECT_EQ(f.report().message, "aborted: PERCEPTION_FAILED");
  const int stow = f.idx("arm.start:stow:0:0");
  EXPECT_GE(stow, 0);
  EXPECT_GT(f.idx("report:aborted"), f.idx("tool_off", stow));
}

TEST(MissionTree, CoveragePlanFailureAborts)
{
  Fix f;
  f.w.planner->plan_ok = false;
  EXPECT_EQ(f.run(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(f.report().message, "aborted: PLAN_FAILED");
  EXPECT_FALSE(f.has("nav.go:0"));
  EXPECT_EQ(f.count("report:aborted"), 1u);
}

TEST(MissionTree, NoStandAbortsWithNoStands)
{
  Fix f;
  f.w.planner->stands_ok = false;
  EXPECT_EQ(f.run(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(f.report().message, "aborted: NO_STANDS");
  EXPECT_TRUE(f.w.navigation->starts.empty());
}

// ------------------------------------------------------------------------------------------------ navigation and docking

TEST(MissionTree, NavigationFailureIsRetried)
{
  Fix f;
  f.w.navigation->script.add(0, failing(Reason::kNavFailed));
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.navigation->starts, (std::vector<int>{0, 0}));
  EXPECT_TRUE(f.report().success);
}

TEST(MissionTree, StandThatCannotBeReachedIsSkippedAndTheLoopContinues)
{
  Fix f(2, 2);
  f.w.navigation->script.failAlways(0, Reason::kNavFailed);
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.navigation->starts, (std::vector<int>{0, 0, 1}));
  EXPECT_EQ(f.w.planner->restand_calls, 1);   // a ReStand was tried, none offered
  for (size_t c : {0u, 1u}) {
    EXPECT_EQ(f.ctx->ledger.get(c).state, LedgerState::kSkipped);
    EXPECT_EQ(f.ctx->ledger.get(c).reason, SkipReason::kNavFailed);
  }
  EXPECT_TRUE(f.ctx->ledger.isDone(2));
  EXPECT_TRUE(f.ctx->ledger.isDone(3));
  const auto & r = f.report();
  EXPECT_EQ(r.outcome, "finished");
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.columns_done, (std::vector<size_t>{2, 3}));
  ASSERT_EQ(r.columns_skipped.size(), 2u);
  EXPECT_EQ(r.columns_skipped[0], (std::pair<size_t, Reason>{0, Reason::kNavFailed}));
  ASSERT_EQ(r.stands.size(), 2u);
  EXPECT_FALSE(r.stands[0].reached);
  EXPECT_EQ(r.stands[0].reason, Reason::kNavFailed);
  EXPECT_TRUE(r.stands[1].reached);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kApproach, 0), 0u);
}

TEST(MissionTree, ReStandRescuesAStandWhoseNavigationFailed)
{
  Fix f;
  f.w.navigation->script.failAlways(0, Reason::kNavFailed);
  StandPlan alt;
  alt.id = 10;
  f.w.planner->alternate_for_stand[0] = alt;
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.navigation->starts, (std::vector<int>{0, 0, 10}));
  EXPECT_EQ(f.report().columns_done, (std::vector<size_t>{0, 1, 2, 3}));
  EXPECT_EQ(f.report().restands, 1u);
  EXPECT_TRUE(f.report().success);
}

TEST(MissionTree, ReStandThatFailsTooLeavesTheStandSkipped)
{
  Fix f;
  f.w.navigation->script.failAlways(0, Reason::kNavFailed);
  f.w.navigation->script.failAlways(10, Reason::kNavFailed);
  StandPlan alt;
  alt.id = 10;
  f.w.planner->alternate_for_stand[0] = alt;
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.navigation->starts, (std::vector<int>{0, 0, 10, 10}));
  EXPECT_TRUE(f.report().columns_done.empty());
  EXPECT_EQ(f.report().columns_skipped.size(), 4u);
}

TEST(MissionTree, StandPoseCheckFailureIsRedockedOnce)
{
  Fix f;
  f.w.dock->script.add(0, Script{0, false, Reason::kNone, 0.20});   // 0.20 m off, margin 0.05 m
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.dock->starts, (std::vector<int>{0, 0}));
  EXPECT_TRUE(f.report().success);
}

TEST(MissionTree, StandPoseOutsideTheMarginSkipsTheStandBeforeAnyArmMotion)
{
  Fix f(2, 2);
  f.w.dock->script.add(0, Script{0, false, Reason::kNone, 0.20}, 6);
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kApproach, 0), 0u);
  EXPECT_EQ(f.ctx->ledger.get(0).reason, SkipReason::kDockFailed);   // ledger vocabulary
  ASSERT_FALSE(f.report().columns_skipped.empty());
  EXPECT_EQ(f.report().columns_skipped[0].second, Reason::kStandCheckFailed);   // full reason in the report
  EXPECT_EQ(f.report().stands[0].reason, Reason::kStandCheckFailed);
  EXPECT_TRUE(f.ctx->ledger.isDone(2));
}

// ------------------------------------------------------------------------------------------------ column policy

TEST(MissionTree, ColumnFailureIsRetriedOnceAfterTheArmIsFreed)
{
  Fix f;
  f.w.arm->script.add(K(ArmPhase::kStroke, 1), failing());
  bool saw_recovery = false;
  EXPECT_EQ(f.run([&](int) {saw_recovery = saw_recovery || f.ctx->fsm.state() == MissionState::kRecovery;}),
    BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(saw_recovery);
  EXPECT_EQ(f.ctx->fsm.state(), MissionState::kActive);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kStroke, 1), 2u);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kRecover, 1), 1u);
  EXPECT_TRUE(f.ctx->ledger.isDone(1));
  EXPECT_EQ(f.report().retries, 1u);
  EXPECT_TRUE(f.report().success);
}

TEST(MissionTree, FailedLeafLeavesTheArmInTheSafeStateInOrder)
{
  Fix f;
  f.w.arm->script.add(K(ArmPhase::kStroke, 0), failing());
  f.run();
  const int failed = f.idx("failed:stroke:PLAN_FAILED");
  f.expectCleanupAfter(failed);
}

TEST(MissionTree, VerifyFailureRetriesTheWholeColumn)
{
  Fix f;
  f.w.verify->script.add(0, failing(Reason::kVerifyFailed));
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kApproach, 0), 2u);
  EXPECT_TRUE(f.ctx->ledger.isDone(0));
}

TEST(MissionTree, ContactOverloadIsNotRetriedAndTheColumnIsSkippedWithItsReason)
{
  Fix f;
  f.w.arm->script.add(K(ArmPhase::kStroke, 1), failing(Reason::kContactOverload));
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kStroke, 1), 1u);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kRecover, 1), 0u);
  EXPECT_EQ(f.ctx->ledger.get(1).state, LedgerState::kSkipped);
  EXPECT_EQ(f.ctx->ledger.get(1).reason, SkipReason::kContactOverload);
  EXPECT_TRUE(f.ctx->ledger.isDone(2));
  EXPECT_TRUE(f.ctx->ledger.isDone(3));
  EXPECT_FALSE(f.report().success);
}

TEST(MissionTree, FailingColumnEscalatesToReStandAfterTheRetry)
{
  Fix f;
  f.w.arm->script.add(K(ArmPhase::kStroke, 2), failing(), 2);
  StandPlan alt;
  alt.id = 7;
  f.w.planner->stand_for_column[2] = alt;
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kStroke, 2), 3u);
  EXPECT_EQ(f.w.navigation->starts, (std::vector<int>{0, 7}));
  EXPECT_EQ(f.w.dock->starts, (std::vector<int>{0, 7}));
  EXPECT_EQ(f.report().restands, 1u);
  EXPECT_TRUE(f.ctx->ledger.isDone(2));
}

TEST(MissionTree, FailingColumnIsReorderedBehindTheOthersThenSucceeds)
{
  Fix f;
  f.w.arm->script.add(K(ArmPhase::kStroke, 0), failing(), 2);
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.count("column:0"), 2u);
  EXPECT_EQ(f.count("defer:0"), 1u);
  EXPECT_LT(f.idx("column:3"), f.idx("column:0", f.idx("column:0") + 1));   // 0 is retried after 3
  EXPECT_EQ(f.report().reorders, 1u);
  EXPECT_TRUE(f.ctx->ledger.isDone(0));
  EXPECT_TRUE(f.report().success);
}

TEST(MissionTree, LastColumnCannotBeReorderedSoItIsSplit)
{
  Fix f;
  f.w.arm->script.add(K(ArmPhase::kStroke, 3), failing(), 2);
  f.w.planner->split_pieces[3] = 2;
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.count("split:3:2"), 1u);
  EXPECT_EQ(f.report().splits, 1u);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kApproach, 3), 4u);   // try, retry, piece 0, piece 1
  EXPECT_TRUE(f.has("arm.start:approach:3:1"));
  EXPECT_TRUE(f.ctx->ledger.isDone(3));
}

TEST(MissionTree, ColumnThatDefeatsEveryStepIsSkippedAsTheLastResort)
{
  Fix f;
  f.w.arm->script.failAlways(K(ArmPhase::kStroke, 1), Reason::kNone);   // a failure that gives no reason
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kStroke, 1), 4u);   // two visits, each a try and a retry
  EXPECT_EQ(f.ctx->ledger.get(1).state, LedgerState::kSkipped);
  EXPECT_EQ(f.ctx->ledger.get(1).reason, SkipReason::kPlanFailed);   // default of the manager
  ASSERT_EQ(f.report().columns_skipped.size(), 1u);
  EXPECT_EQ(f.report().columns_skipped[0], (std::pair<size_t, Reason>{1, Reason::kPlanFailed}));
  EXPECT_EQ(f.report().columns_done, (std::vector<size_t>{0, 2, 3}));
  EXPECT_EQ(f.report().outcome, "finished");
  EXPECT_FALSE(f.report().success);
  EXPECT_EQ(f.count("report:finished"), 1u);
}

// ------------------------------------------------------------------------------------------------ pause

TEST(MissionTree, PauseTakesEffectAtTheColumnBoundaryWithoutCancelling)
{
  Fix f;
  f.w.arm->script.add(K(ArmPhase::kStroke, 1), slow(5));
  int paused_at = -1;
  bool saw_paused = false;
  EXPECT_EQ(f.run([&](int i) {
      if (!f.ctx->fsm.pauseRequested() && f.ctx->fsm.state() == MissionState::kActive &&
      f.has("arm.start:stroke:1:0") && paused_at < 0)
      {
        f.ctx->fsm.requestPause();
        paused_at = i;
      }
      if (f.has("pause:wait")) {
        saw_paused = saw_paused || f.ctx->fsm.state() == MissionState::kPaused;
        if (f.count("pause:resume") == 0 && i > paused_at + 40) {f.ctx->fsm.resume();}
      }
    }), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(saw_paused);
  EXPECT_TRUE(f.ctx->ledger.isDone(1));   // the column in flight finished
  EXPECT_GT(f.idx("arm.start:approach:2:0"), f.idx("pause:resume"));
  EXPECT_GT(f.idx("pause:resume"), f.idx("pause:wait"));
  EXPECT_EQ(f.countPrefix("halt:"), 0u);
  EXPECT_TRUE(f.report().success);
}

// ------------------------------------------------------------------------------------------------ safety gate

TEST(MissionTree, SafetyStopMidColumnHoldsThenResumesTheSameMotion)
{
  Fix f;
  f.w.arm->script.add(K(ArmPhase::kStroke, 1), slow(6));
  int stop_at = -1;
  EXPECT_EQ(f.run([&](int i) {
      if (stop_at < 0 && f.has("arm.start:stroke:1:0")) {
        stop_at = i;
        f.w.safety->state.stop_request = true;
      }
      if (stop_at >= 0 && i == stop_at + 5) {f.w.safety->state.stop_request = false;}
    }), BT::NodeStatus::SUCCESS);
  const int started = f.idx("arm.start:stroke:1:0");
  const int stop = f.idx("safety:stop");
  const int resume = f.idx("safety:resume");
  const int done = f.idx("done:stroke", started);
  EXPECT_GT(stop, started);
  EXPECT_GT(resume, stop);
  EXPECT_GT(done, resume);   // the motion never finished under the stop
  EXPECT_EQ(f.w.arm->holds, 1);
  EXPECT_EQ(f.w.arm->resumes, 1);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kStroke, 1), 1u);   // resumed, not restarted
  EXPECT_EQ(f.report().safety_stops, 1u);
  EXPECT_TRUE(f.report().success);
}

TEST(MissionTree, SafetyStopLongerThanTheTimeoutHaltsCleanlyAndAbortsTheMission)
{
  Fix f;
  f.w.arm->script.add(K(ArmPhase::kStroke, 1), slow(6));
  EXPECT_EQ(f.run([&](int) {
      if (f.has("arm.start:stroke:1:0")) {f.w.safety->state.stop_request = true;}
    }), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(f.has("safety:timeout"));
  f.expectCleanupAfter(f.idx("halt:stroke"));
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kApproach, 2), 0u);   // nothing started after the abort
  EXPECT_TRUE(f.has("skip:stow:safety"));                    // no motion while the stop stands
  EXPECT_EQ(f.report().outcome, "aborted");
  EXPECT_EQ(f.report().abort_reason, Reason::kSafetyTimeout);
  EXPECT_EQ(f.report().message, "aborted: SAFETY_TIMEOUT");
  EXPECT_EQ(f.report().columns_done, (std::vector<size_t>{0}));
  EXPECT_EQ(f.ctx->ledger.get(1).state, LedgerState::kRunning);   // not skipped: a resume reruns it
  EXPECT_EQ(f.count("report:aborted"), 1u);
  EXPECT_GT(f.idx("report:aborted"), f.idx("tool_off", f.idx("safety:timeout")));
}

TEST(MissionTree, NothingStartsWhileAStopStandsAtTheBeginningOfAColumn)
{
  Fix f;
  f.w.safety->state.stop_request = true;
  EXPECT_EQ(f.run([&](int i) {if (i == 30) {f.w.safety->state.stop_request = false;}}), BT::NodeStatus::SUCCESS);
  EXPECT_GE(f.idx("safety:stop"), 0);
  EXPECT_GT(f.idx("arm.start:approach:0:0"), f.idx("safety:resume"));
}

TEST(MissionTree, StaleSafetyDataIsTreatedAsAStop)
{
  Fix f;
  f.w.safety->state.fresh = false;
  EXPECT_EQ(f.run([&](int i) {if (i == 30) {f.w.safety->state.fresh = true;}}), BT::NodeStatus::SUCCESS);
  EXPECT_GE(f.idx("safety:stale"), 0);
  EXPECT_GT(f.idx("arm.start:approach:0:0"), f.idx("safety:resume"));
  EXPECT_GE(f.w.arm->holds, 1);
}

TEST(MissionTree, SpeedScaleIsPassedToTheArmAndClamped)
{
  Fix f;
  f.w.safety->state.speed_scale = 0.5;
  f.run();
  ASSERT_FALSE(f.w.arm->speed_scales.empty());
  for (double s : f.w.arm->speed_scales) {EXPECT_DOUBLE_EQ(s, 0.5);}

  Fix g;
  g.w.safety->state.speed_scale = 2.0;
  g.run();
  ASSERT_FALSE(g.w.arm->speed_scales.empty());
  for (double s : g.w.arm->speed_scales) {EXPECT_DOUBLE_EQ(s, 1.0);}
}

// ------------------------------------------------------------------------------------------------ teleop handover

TEST(MissionTree, TeleopHandoverHaltsTheLeafWaitsForConfirmAndReplansFromThePose)
{
  Fix f;
  f.w.arm->script.add(K(ArmPhase::kStroke, 1), slow(6));
  int phase = 0;
  int t0 = 0;
  EXPECT_EQ(f.run([&](int i) {
      if (phase == 0 && f.has("arm.start:stroke:1:0")) {
        f.w.teleop->current = TeleopMode::kTeleop;
        phase = 1;
        t0 = i;
      } else if (phase == 1 && i == t0 + 10) {
        f.w.teleop->current = TeleopMode::kAuto;   // back in AUTO, but no confirm yet
        phase = 2;
        t0 = i;
      } else if (phase == 2 && i == t0 + 10) {
        f.w.teleop->confirm = true;
        phase = 3;
      }
    }), BT::NodeStatus::SUCCESS);
  const int halt = f.idx("halt:stroke");
  f.expectCleanupAfter(halt);
  EXPECT_GT(f.idx("teleop:halt"), halt);
  EXPECT_GT(f.idx("teleop:await_confirm"), f.idx("teleop:halt"));
  EXPECT_GT(f.idx("teleop:replan"), f.idx("teleop:await_confirm"));
  // nothing of column 1 moved between the halt and the confirm
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kApproach, 1), 2u);
  EXPECT_GT(f.idx("arm.start:approach:1:0", f.idx("teleop:halt")), f.idx("teleop:replan"));
  ASSERT_EQ(f.w.planner->replans.size(), 1u);
  EXPECT_EQ(f.w.planner->replans[0].first, 1u);
  EXPECT_DOUBLE_EQ(f.w.planner->replans[0].second.x, 0.1);
  EXPECT_EQ(f.report().teleop_handovers, 1u);
  EXPECT_EQ(f.report().replans, 1u);
  EXPECT_TRUE(f.report().success);
}

TEST(MissionTree, HoldModeHaltsLikeTeleop)
{
  Fix f;
  f.w.arm->script.add(K(ArmPhase::kStroke, 0), slow(6));
  int phase = 0;
  f.run([&](int i) {
      static int t0 = 0;
      if (phase == 0 && f.has("arm.start:stroke:0:0")) {
        f.w.teleop->current = TeleopMode::kHold;
        phase = 1;
        t0 = i;
      } else if (phase == 1 && i == t0 + 6) {
        f.w.teleop->current = TeleopMode::kAuto;
        f.w.teleop->confirm = true;
        phase = 2;
      }
    });
  EXPECT_TRUE(f.has("teleop:halt"));
  EXPECT_TRUE(f.has("halt:stroke"));
  EXPECT_TRUE(f.has("teleop:replan"));
  EXPECT_TRUE(f.report().success);
}

TEST(MissionTree, ConfirmGivenBeforeAutoIsReturnedDoesNotCount)
{
  Fix f;
  f.w.arm->script.add(K(ArmPhase::kStroke, 0), slow(6));
  int phase = 0;
  int t0 = 0;
  f.run([&](int i) {
      if (phase == 0 && f.has("arm.start:stroke:0:0")) {
        f.w.teleop->current = TeleopMode::kTeleop;
        phase = 1;
        t0 = i;
      } else if (phase == 1 && i == t0 + 3) {
        f.w.teleop->confirm = true;   // still in TELEOP: ignored
        phase = 2;
      } else if (phase == 2 && i == t0 + 8) {
        f.w.teleop->current = TeleopMode::kAuto;
        phase = 3;
        t0 = i;
      } else if (phase == 3 && i == t0 + 10) {
        EXPECT_FALSE(f.has("teleop:replan"));   // 10 ticks in AUTO without a fresh confirm
        f.w.teleop->confirm = true;
        phase = 4;
      }
    });
  EXPECT_TRUE(f.has("teleop:replan"));
  EXPECT_EQ(f.w.teleop->handovers, 1);
}

TEST(MissionTree, TeleopHandoverLongerThanItsTimeoutAbortsTheMission)
{
  Fix f(1, 4, [](std::string xml) {
      const std::string from = "<TeleopGuard timeout_s=\"0\">";
      const auto pos = xml.find(from);
      EXPECT_NE(pos, std::string::npos);
      return xml.replace(pos, from.size(), "<TeleopGuard timeout_s=\"2\">");
    });
  f.w.arm->script.add(K(ArmPhase::kStroke, 1), slow(6));
  EXPECT_EQ(f.run([&](int) {
      if (f.has("arm.start:stroke:1:0")) {f.w.teleop->current = TeleopMode::kTeleop;}
    }), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(f.has("teleop:timeout"));
  f.expectCleanupAfter(f.idx("halt:stroke"));
  EXPECT_EQ(f.report().abort_reason, Reason::kTeleopTimeout);
  EXPECT_EQ(f.report().outcome, "aborted");
}

// ------------------------------------------------------------------------------------------------ cancel

TEST(MissionTree, CancelHaltsTheRunningLeafWithTheCleanupInOrderThenReportsCanceled)
{
  Fix f;
  f.w.arm->script.add(K(ArmPhase::kStroke, 1), slow(50));
  ASSERT_EQ(f.runUntil("arm.start:stroke:1:0"), BT::NodeStatus::RUNNING);
  f.tree.haltTree();
  const int halt = f.idx("halt:stroke");
  f.expectCleanupAfter(halt);
  EXPECT_EQ(f.idx("report:canceled"), halt + 1 + static_cast<int>(kCleanupOrder.size()));
  ASSERT_EQ(f.w.sink->reports.size(), 1u);
  EXPECT_EQ(f.report().outcome, "canceled");
  EXPECT_EQ(f.report().message, "canceled");
  EXPECT_FALSE(f.report().success);
  EXPECT_EQ(f.report().columns_done, (std::vector<size_t>{0}));
  EXPECT_EQ(f.ctx->fsm.state(), MissionState::kStopped);
  EXPECT_EQ(f.ctx->ledger.get(1).state, LedgerState::kRunning);   // a resume reruns it
  EXPECT_EQ(f.countPrefix("arm.start:approach:2"), 0u);
}

TEST(MissionTree, CancelWhileNavigatingCancelsTheNavigationOnly)
{
  Fix f;
  f.w.navigation->script.add(0, slow(50));
  ASSERT_EQ(f.runUntil("nav.go:0"), BT::NodeStatus::RUNNING);
  f.tree.haltTree();
  EXPECT_TRUE(f.has("halt:navigate"));
  EXPECT_TRUE(f.has("nav.cancel"));
  EXPECT_FALSE(f.has("cancel_trajectory"));
  EXPECT_EQ(f.report().outcome, "canceled");
  EXPECT_EQ(f.w.sink->reports.size(), 1u);
}

TEST(MissionTree, CancelDuringASafetyStopStillCleansUpOnce)
{
  Fix f;
  f.w.arm->script.add(K(ArmPhase::kStroke, 0), slow(50));
  ASSERT_EQ(f.runUntil("arm.start:stroke:0:0"), BT::NodeStatus::RUNNING);
  f.w.safety->state.stop_request = true;
  for (int i = 0; i < 5; ++i) {f.tick_no = 100 + i; f.tree.tickOnce();}
  ASSERT_TRUE(f.has("safety:stop"));
  f.tree.haltTree();
  EXPECT_EQ(f.count("halt:stroke"), 1u);
  f.expectCleanupAfter(f.idx("halt:stroke"));
  EXPECT_EQ(f.report().outcome, "canceled");
}

TEST(MissionTree, CancelWhileTeleopIsActiveDoesNotCleanUpTwice)
{
  Fix f;
  f.w.arm->script.add(K(ArmPhase::kStroke, 0), slow(50));
  ASSERT_EQ(f.runUntil("arm.start:stroke:0:0"), BT::NodeStatus::RUNNING);
  f.w.teleop->current = TeleopMode::kTeleop;
  for (int i = 0; i < 5; ++i) {f.tick_no = 100 + i; f.tree.tickOnce();}
  ASSERT_TRUE(f.has("teleop:halt"));
  f.tree.haltTree();
  EXPECT_EQ(f.count("halt:stroke"), 1u);   // the guard had halted it; the cancel finds nothing running
  EXPECT_EQ(f.count("report:canceled"), 1u);
}

TEST(MissionTree, HaltAfterTheMissionEndedDoesNotReportAgain)
{
  Fix f;
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  f.tree.haltTree();
  EXPECT_EQ(f.w.sink->reports.size(), 1u);
  EXPECT_EQ(f.report().outcome, "finished");
}

// ------------------------------------------------------------------------------------------------ resume

TEST(MissionTree, ResumeAfterCancelRerunsOnlyTheColumnsTheLedgerDoesNotHoldAsDone)
{
  Fix f;
  f.w.arm->script.add(K(ArmPhase::kStroke, 2), slow(50));
  ASSERT_EQ(f.runUntil("arm.start:stroke:2:0"), BT::NodeStatus::RUNNING);
  f.tree.haltTree();
  ASSERT_EQ(f.w.sink->reports.size(), 1u);
  f.w.arm->starts.clear();
  f.ctx->resume = true;
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kApproach, 0), 0u);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kApproach, 1), 0u);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kApproach, 2), 1u);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kApproach, 3), 1u);
  ASSERT_EQ(f.w.sink->reports.size(), 2u);
  EXPECT_EQ(f.report().outcome, "finished");
  EXPECT_EQ(f.report().columns_done, (std::vector<size_t>{0, 1, 2, 3}));
  EXPECT_TRUE(f.report().success);
}

TEST(MissionTree, ResumeSkipsDoneAndContactOverloadButRerunsOtherSkips)
{
  Fix f;
  f.ctx->ledger.begin("mission", 4, false);
  f.ctx->ledger.markDone(0, 0);
  f.ctx->ledger.markSkipped(1, 0, SkipReason::kContactOverload);
  f.ctx->ledger.markSkipped(2, 0, SkipReason::kDockFailed);
  f.ctx->resume = true;
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kApproach, 0), 0u);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kApproach, 1), 0u);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kApproach, 2), 1u);
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kApproach, 3), 1u);
  EXPECT_TRUE(f.has("resume_skip:column:0"));
  EXPECT_TRUE(f.has("resume_skip:column:1"));
}

TEST(MissionTree, ResumeDoesNotDriveToAStandWhoseColumnsAreAllDone)
{
  Fix f(2, 2);
  f.ctx->ledger.begin("mission", 4, false);
  f.ctx->ledger.markDone(0, 0);
  f.ctx->ledger.markDone(1, 0);
  f.ctx->resume = true;
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.navigation->starts, (std::vector<int>{1}));
  EXPECT_TRUE(f.has("resume_skip:stand:0"));
  EXPECT_TRUE(f.report().success);
}

TEST(MissionTree, FreshRunIgnoresAnOldLedger)
{
  Fix f;
  f.ctx->ledger.begin("mission", 4, false);
  f.ctx->ledger.markDone(0, 0);
  f.ctx->ledger.markDone(1, 0);
  f.ctx->resume = false;
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  for (size_t c = 0; c < 4; ++c) {EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kApproach, c), 1u);}
}

// ------------------------------------------------------------------------------------------------ calibration gate

TEST(MissionTree, ValidCalibrationRecordPassesWithoutATouch)
{
  Fix f;
  f.w.calibration->rec.valid = true;
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.calibration->touches, 0);
  EXPECT_TRUE(f.has("calibration:valid"));
}

TEST(MissionTree, UncalibratedContactToolIsCalibratedByTheTouchSequenceThenAllowed)
{
  Fix f;
  f.w.calibration->rec.valid = false;
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.w.calibration->touches, 1);
  EXPECT_LT(f.idx("calibration.touch"), f.idx("planner.coverage"));
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kPress, 0), 1u);
  EXPECT_TRUE(f.report().success);
}

TEST(MissionTree, ContactToolWhoseTouchSequenceFailsIsRefusedBeforeAnyColumn)
{
  Fix f;
  f.w.calibration->rec.valid = false;
  f.w.calibration->script.add(0, failing(Reason::kCalibrationInvalid));
  EXPECT_EQ(f.run(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(f.has("nav.go:0"));
  EXPECT_EQ(f.w.arm->startsOf(ArmPhase::kApproach, 0), 0u);
  EXPECT_EQ(f.report().outcome, "aborted");
  EXPECT_EQ(f.report().message, "aborted: CALIBRATION_INVALID");
  EXPECT_TRUE(f.has("calibration.cancel"));
}

TEST(MissionTree, TouchThatLeavesTheRecordInvalidIsRefused)
{
  Fix f;
  f.w.calibration->rec.valid = false;
  f.w.calibration->touch_makes_valid = false;
  EXPECT_EQ(f.run(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(f.w.calibration->touches, 1);
  EXPECT_EQ(f.report().message, "aborted: CALIBRATION_INVALID");
  EXPECT_EQ(f.w.sink->reports.size(), 1u);
}

TEST(MissionTree, UncalibratedNonContactToolIsAllowed)
{
  Fix f;
  f.w.tool->is_contact = false;
  f.w.calibration->rec.valid = false;
  EXPECT_EQ(f.run(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(f.has("calibration:not_required"));
  EXPECT_EQ(f.w.calibration->touches, 0);
}

TEST(MissionTree, CancelDuringTheTouchSequenceCancelsItAndCleansUp)
{
  Fix f;
  f.w.calibration->rec.valid = false;
  f.w.calibration->script.add(0, slow(50));
  ASSERT_EQ(f.runUntil("calibration.touch"), BT::NodeStatus::RUNNING);
  f.tree.haltTree();
  EXPECT_TRUE(f.has("calibration.cancel"));
  f.expectCleanupAfter(f.idx("calibration.cancel"));
  EXPECT_EQ(f.report().outcome, "canceled");
}
