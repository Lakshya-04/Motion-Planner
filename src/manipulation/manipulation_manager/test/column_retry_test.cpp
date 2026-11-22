#include <map>
#include <vector>

#include <gtest/gtest.h>

#include "manipulation_manager/core/column_retry.hpp"
#include "manipulation_manager/core/dock_ik_selector.hpp"

namespace manipulation_manager
{

namespace
{
ColumnRetryConfig on(int retries = 1)
{
  ColumnRetryConfig c;
  c.enabled = true;
  c.max_retries = retries;
  return c;
}

ColumnFailure fail(size_t col, int stand, RetryFailure k, ColumnOutcome o = ColumnOutcome::kFailed)
{
  return {col, stand, o, k};
}
}  // namespace

TEST(ColumnRetryTest, OffByDefaultRetriesNothing)
{
  EXPECT_TRUE(planColumnRetries({fail(1, 0, RetryFailure::kDock)}, ColumnRetryConfig{}).empty());
  auto c = on(0);
  EXPECT_TRUE(planColumnRetries({fail(1, 0, RetryFailure::kDock)}, c).empty());
}

TEST(ColumnRetryTest, OverloadIsNeverRetriedAndRecorded)
{
  std::vector<std::pair<size_t, std::string>> skipped;
  const auto plan = planColumnRetries(
    {fail(2, 0, RetryFailure::kOverload), fail(3, 0, RetryFailure::kDock)}, on(), &skipped);
  ASSERT_EQ(plan.size(), 1u);
  EXPECT_EQ(plan[0].column, 3u);
  ASSERT_EQ(skipped.size(), 1u);
  EXPECT_EQ(skipped[0].first, 2u);
}

TEST(ColumnRetryTest, OrderIsByStandThenColumn)
{
  const auto plan = planColumnRetries(
    {fail(7, 1, RetryFailure::kPlan), fail(5, 1, RetryFailure::kPlan), fail(9, 0, RetryFailure::kNav),
      fail(4, 0, RetryFailure::kDock, ColumnOutcome::kPartial)}, on());
  ASSERT_EQ(plan.size(), 4u);
  EXPECT_EQ(plan[0].column, 4u);
  EXPECT_EQ(plan[1].column, 9u);
  EXPECT_EQ(plan[2].column, 5u);
  EXPECT_EQ(plan[3].column, 7u);
}

TEST(ColumnRetryTest, VariantPerFailureKind)
{
  auto cfg = on(2);
  cfg.replan_stand = true;
  cfg.allow_branch_change = true;
  const auto nav = chooseVariant(RetryFailure::kNav, 1, cfg);
  EXPECT_TRUE(nav.restand);
  EXPECT_FALSE(nav.vary_ik);
  EXPECT_TRUE(nav.fresh_stow);
  const auto d1 = chooseVariant(RetryFailure::kDock, 1, cfg);
  EXPECT_TRUE(d1.vary_ik);
  EXPECT_FALSE(d1.allow_branch_change);
  EXPECT_FALSE(d1.restand);
  const auto d2 = chooseVariant(RetryFailure::kDock, 2, cfg);
  EXPECT_TRUE(d2.allow_branch_change);
  EXPECT_TRUE(d2.restand);
  EXPECT_FALSE(chooseVariant(RetryFailure::kPlan, 2, cfg).restand);   // not reach related
  cfg.replan_stand = false;
  EXPECT_FALSE(chooseVariant(RetryFailure::kNav, 1, cfg).restand);
  EXPECT_EQ(d2.label(), "fresh_stow+next_ik_any_branch+restand");
}

TEST(ColumnRetryTest, RecoversOnSecondAttemptAndRecords)
{
  auto cfg = on(2);
  const auto plan = planColumnRetries({fail(3, 0, RetryFailure::kDock)}, cfg);
  int calls = 0;
  const auto rep = runColumnRetries(plan, cfg, [&](const ColumnRetryPlan &, int a, const RetryVariant &) {
    ++calls;
    RetryAttemptResult r;
    r.outcome = a == 2 ? ColumnOutcome::kDone : ColumnOutcome::kFailed;
    r.kind = RetryFailure::kDock;
    return r;
  });
  EXPECT_EQ(calls, 2);
  ASSERT_EQ(rep.attempts.size(), 2u);
  EXPECT_EQ(rep.attempts[0].variant, "fresh_stow+next_ik");
  EXPECT_EQ(rep.attempts[1].outcome, ColumnOutcome::kDone);
  EXPECT_EQ(rep.final_outcome.at(3), ColumnOutcome::kDone);
  const auto j = columnRetryToJson(rep);
  EXPECT_TRUE(j["columns"][0]["recovered"].get<bool>());
  EXPECT_EQ(j["attempts"].size(), 2u);
}

TEST(ColumnRetryTest, MaxRetriesBoundsAttempts)
{
  auto cfg = on(1);
  const auto plan = planColumnRetries({fail(3, 0, RetryFailure::kPlan)}, cfg);
  int calls = 0;
  const auto rep = runColumnRetries(plan, cfg, [&](const ColumnRetryPlan &, int, const RetryVariant &) {
    ++calls;
    return RetryAttemptResult{};
  });
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(rep.final_outcome.at(3), ColumnOutcome::kFailed);
}

TEST(ColumnRetryTest, OverloadOnRetryEndsThatColumn)
{
  auto cfg = on(3);
  const auto plan = planColumnRetries({fail(3, 0, RetryFailure::kDock)}, cfg);
  int calls = 0;
  const auto rep = runColumnRetries(plan, cfg, [&](const ColumnRetryPlan &, int, const RetryVariant &) {
    ++calls;
    RetryAttemptResult r;
    r.kind = RetryFailure::kOverload;
    return r;
  });
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(rep.attempts[0].kind, RetryFailure::kOverload);
}

TEST(ColumnRetryTest, StopsAfterColumnsInARowThatDoNotRecover)
{
  auto cfg = on(1);
  cfg.stop_after_failed_columns = 2;
  std::vector<ColumnFailure> f;
  for (size_t c = 0; c < 5; ++c) {
    f.push_back(fail(c, 0, RetryFailure::kDock));
  }
  const auto plan = planColumnRetries(f, cfg);
  int calls = 0;
  const auto rep = runColumnRetries(plan, cfg, [&](const ColumnRetryPlan &, int, const RetryVariant &) {
    ++calls;
    return RetryAttemptResult{};
  });
  EXPECT_EQ(calls, 2);
  EXPECT_TRUE(rep.stopped_early);
  EXPECT_EQ(rep.not_retried.size(), 3u);
}

TEST(ColumnRetryTest, ARecoveryResetsTheRun)
{
  auto cfg = on(1);
  cfg.stop_after_failed_columns = 2;
  std::vector<ColumnFailure> f;
  for (size_t c = 0; c < 4; ++c) {
    f.push_back(fail(c, 0, RetryFailure::kDock));
  }
  const auto plan = planColumnRetries(f, cfg);
  const auto rep = runColumnRetries(plan, cfg, [&](const ColumnRetryPlan & p, int, const RetryVariant &) {
    RetryAttemptResult r;
    r.outcome = p.column == 1 ? ColumnOutcome::kDone : ColumnOutcome::kFailed;
    return r;
  });
  EXPECT_FALSE(rep.stopped_early);
  EXPECT_EQ(rep.attempts.size(), 4u);
}

TEST(ColumnRetryTest, CancelAndFatalStop)
{
  auto cfg = on(1);
  const auto plan = planColumnRetries({fail(1, 0, RetryFailure::kDock), fail(2, 0, RetryFailure::kDock)}, cfg);
  int calls = 0;
  auto rep = runColumnRetries(
    plan, cfg, [&](const ColumnRetryPlan &, int, const RetryVariant &) {++calls; return RetryAttemptResult{};},
    [&]() {return calls >= 1;});
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(rep.stop_reason, "canceled");
  calls = 0;
  rep = runColumnRetries(plan, cfg, [&](const ColumnRetryPlan &, int, const RetryVariant &) {
    ++calls;
    RetryAttemptResult r;
    r.fatal = true;
    return r;
  });
  EXPECT_EQ(calls, 1);
  EXPECT_TRUE(rep.stopped_early);
  ASSERT_EQ(rep.not_retried.size(), 1u);
  EXPECT_EQ(rep.not_retried[0].first, 2u);
}

// next-best distinct dock IK candidate
TEST(DockIkExclusionTest, SkipsTheUsedSolutionAndTakesTheNextBest)
{
  using V = std::vector<double>;
  const V ref{0.5, 1.0, -0.6, 0.0};
  const std::vector<V> seeds{{1, 0, 0, 0}, {2, 0, 0, 0}, {3, 0, 0, 0}};
  const std::vector<V> sols{{0.5, 1.0, -0.6, 0.1}, {0.5, 1.0, -0.6, 0.3}, {0.5, 1.0, -0.6, 0.1001}};
  const DockIkFn ik = [&](const V & s) -> std::optional<V> {
    for (size_t i = 0; i < seeds.size(); ++i) {
      if (s == seeds[i]) {return sols[i];}
    }
    return std::nullopt;
  };
  DockIkSelectorParams p;
  p.branch_joints = {2};
  EXPECT_NEAR(selectDockIk(ik, seeds, ref, p).cost, 0.1, 1e-9);
  DockIkExclusion ex;
  ex.used = {sols[0]};
  const auto c = selectDockIk(ik, seeds, ref, p, nullptr, &ex);
  ASSERT_TRUE(c.ok);
  EXPECT_NEAR(c.cost, 0.3, 1e-9);
  EXPECT_EQ(c.rejected_excluded, 2u);   // the used one and its near twin
  ex.used.push_back(sols[1]);
  EXPECT_FALSE(selectDockIk(ik, seeds, ref, p, nullptr, &ex).ok);
}

TEST(DockIkExclusionTest, OtherBranchOnlyWhenAllowed)
{
  using V = std::vector<double>;
  const V ref{0.5, 1.0, -0.6, 0.0};
  const std::vector<V> seeds{{1, 0, 0, 0}};
  const DockIkFn ik = [&](const V &) -> std::optional<V> {return V{0.5, 1.0, 0.7, 0.0};};
  DockIkSelectorParams p;
  p.branch_joints = {2};
  DockIkExclusion ex;
  EXPECT_FALSE(selectDockIk(ik, seeds, ref, p, nullptr, &ex).ok);
  ex.allow_branch_change = true;
  EXPECT_TRUE(selectDockIk(ik, seeds, ref, p, nullptr, &ex).ok);
}

}  // namespace manipulation_manager
