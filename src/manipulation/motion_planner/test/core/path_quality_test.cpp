#include <gtest/gtest.h>

#include <cmath>

#include "motion_planner/core/path_quality.hpp"

using namespace motion_planner;

namespace
{
PathQualityConfig cfg()
{
  PathQualityConfig c;
  c.weights = weightsFromRanges({2.0, 4.0});  // lift 2 m, arm 4 rad
  c.velocity_limits = {0.5, 1.0};
  c.acceleration_limits = {1.0, 2.0};
  return c;
}
}  // namespace

TEST(PathQuality, WeightsNormaliseUnits)
{
  const auto c = cfg();
  // 1 m of lift (half its range) equals 2 rad of arm (half its range).
  EXPECT_NEAR(weightedDistance({0, 0}, {1, 0}, c), weightedDistance({0, 0}, {0, 2}, c), 1e-12);
  EXPECT_NEAR(weightedPathLength({{0, 0}, {1, 0}, {1, 2}}, c), 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(weightsFromRanges({0.0})[0], 1.0);
}

TEST(PathQuality, StraightBaselineAndEfficiency)
{
  const auto c = cfg();
  const std::vector<std::vector<double>> detour{{0, 0}, {1, 0}, {1, 2}};
  EXPECT_NEAR(straightLineLength(detour, c), std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(efficiencyRatio(straightLineLength(detour, c), weightedPathLength(detour, c)), std::sqrt(0.5), 1e-12);
  EXPECT_DOUBLE_EQ(efficiencyRatio(0.0, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(efficiencyRatio(1.0, 0.5), 1.0);  // capped
}

TEST(PathQuality, DurationEstimateRatesDetourLonger)
{
  const auto c = cfg();
  const std::vector<std::vector<double>> straight{{0, 0}, {1, 2}};
  const std::vector<std::vector<double>> detour{{0, 0}, {1, 0}, {1, 2}};
  EXPECT_GT(estimatePathDuration(detour, c), estimatePathDuration(straight, c));
  // straight: max(1/0.5, 2/1) = 2 s + ramp max(0.5, 0.5) = 2.5 s
  EXPECT_NEAR(estimatePathDuration(straight, c), 2.5, 1e-12);
  EXPECT_GE(straightLineDuration(straight, c), 0.0);
}

TEST(PathQuality, SelectsShortestAndStraightWinsAndTiesKeepFirst)
{
  const auto c = cfg();
  PathCandidate direct{"direct", {{0, 0}, {1, 2}}, -1.0};
  PathCandidate wiggly{"RRTstar", {{0, 0}, {1, 0}, {1, 2}}, -1.0};
  auto sel = selectBestCandidate({wiggly, direct}, FreeSpaceObjective::kLength, c);
  EXPECT_EQ(sel.best_index, 1);
  EXPECT_NEAR(sel.efficiency, 1.0, 1e-12);

  PathCandidate same = direct;
  same.source = "other";
  EXPECT_EQ(selectBestCandidate({direct, same}, FreeSpaceObjective::kLength, c).best_index, 0);
  EXPECT_EQ(selectBestCandidate({}, FreeSpaceObjective::kLength, c).best_index, -1);
}

TEST(PathQuality, DurationObjectiveUsesMeasuredTime)
{
  const auto c = cfg();
  PathCandidate shortPath{"a", {{0, 0}, {0.1, 0.1}}, 9.0};
  PathCandidate longerFaster{"b", {{0, 0}, {1, 0}, {1, 2}}, 3.0};
  EXPECT_EQ(selectBestCandidate({shortPath, longerFaster}, FreeSpaceObjective::kLength, c).best_index, 0);
  const auto sel = selectBestCandidate({shortPath, longerFaster}, FreeSpaceObjective::kDuration, c);
  EXPECT_EQ(sel.best_index, 1);
  EXPECT_DOUBLE_EQ(sel.duration, 3.0);
}

TEST(PathQuality, ObjectiveParseAndLog)
{
  EXPECT_EQ(parseObjective("duration"), FreeSpaceObjective::kDuration);
  EXPECT_EQ(parseObjective("length"), FreeSpaceObjective::kLength);
  EXPECT_EQ(parseObjective("junk"), FreeSpaceObjective::kLength);
  CandidateSelection s;
  s.value = 1.5;
  s.baseline = 1.0;
  s.efficiency = 0.667;
  EXPECT_EQ(
    formatFreeSpaceLog(5, FreeSpaceObjective::kLength, s, "BITstar"),
    "free-space plan: candidates 5, best length=1.500 (baseline straight 1.000, efficiency 0.67), BITstar used");
}

TEST(PathQuality, CartesianCandidateWinsOnlyWhenShorterAndTiesKeepOrder)
{
  const auto c = cfg();
  PathCandidate direct{"direct", {{0, 0}, {1, 2}}, -1.0};
  PathCandidate cartesian{"cartesian", {{0, 0}, {0.5, 1.5}, {1, 2}}, -1.0};
  PathCandidate ompl{"RRTstar", {{0, 0}, {1, 0}, {1, 2}}, -1.0};
  // The straight joint line is the shortest possible: direct wins, the Cartesian line and OMPL lose.
  auto sel = selectBestCandidate({direct, cartesian, ompl}, FreeSpaceObjective::kLength, c);
  EXPECT_EQ(sel.best_index, 0);
  // Direct blocked: the Cartesian line (almost straight) beats the OMPL detour.
  sel = selectBestCandidate({cartesian, ompl}, FreeSpaceObjective::kLength, c);
  EXPECT_EQ(sel.best_index, 0);
  EXPECT_LT(sel.length, weightedPathLength(ompl.points, c));
  // An equal-length Cartesian candidate listed after direct does not displace it.
  PathCandidate cart_same{"cartesian", direct.points, -1.0};
  sel = selectBestCandidate({direct, cart_same}, FreeSpaceObjective::kLength, c);
  EXPECT_EQ(sel.best_index, 0);
}

TEST(PathQuality, DurationObjectiveUsesMeasuredDurationOfCartesianCandidate)
{
  const auto c = cfg();
  PathCandidate cartesian{"cartesian", {{0, 0}, {0.2, 1.0}, {1, 2}}, 9.0};   // slow: tool-speed limited
  PathCandidate ompl{"BITstar", {{0, 0}, {1, 0}, {1, 2}}, 4.0};             // longer but fast
  auto by_length = selectBestCandidate({cartesian, ompl}, FreeSpaceObjective::kLength, c);
  auto by_duration = selectBestCandidate({cartesian, ompl}, FreeSpaceObjective::kDuration, c);
  EXPECT_EQ(by_length.best_index, 0);
  EXPECT_EQ(by_duration.best_index, 1);
  EXPECT_DOUBLE_EQ(by_duration.duration, 4.0);
}

TEST(PathQuality, JointGoalBranchCheck)
{
  EXPECT_TRUE(reachesJointGoal({0.0, 1.0}, {0.01, 1.02}, 0.05));
  EXPECT_FALSE(reachesJointGoal({0.0, 1.0}, {0.0, 1.0 + 6.28}, 0.05));  // same pose, wrist wound once
  EXPECT_FALSE(reachesJointGoal({0.0}, {0.0, 1.0}, 0.05));
  EXPECT_FALSE(reachesJointGoal({}, {}, 0.05));
  EXPECT_NEAR(maxJointStep({{0, 0}, {0.1, 0.3}, {0.1, 3.5}}), 3.2, 1e-12);
}

// ---- lift cost weight: lift travel is `w` times as costly as arm travel ----

TEST(PathQualityLiftWeight, WeightOneReproducesThePlainWeighting)
{
  const std::vector<double> base = weightsFromRanges({2.0, 4.0, 6.0});
  EXPECT_EQ(applyLiftCostWeight(base, {true, false, false}, 1.0), base);
  const auto w2 = applyLiftCostWeight(base, {true, false, false}, 2.0);
  EXPECT_DOUBLE_EQ(w2[0], 2.0 * base[0]);
  EXPECT_DOUBLE_EQ(w2[1], base[1]);
  EXPECT_DOUBLE_EQ(w2[2], base[2]);
  EXPECT_EQ(applyLiftCostWeight(base, {true, false, false}, -3.0), base);  // invalid weight = off
}

TEST(PathQualityLiftWeight, LiftMoveCostsTwiceTheEqualArmMove)
{
  PathQualityConfig c;
  c.weights = applyLiftCostWeight({1.0, 1.0}, {true, false}, 2.0);
  EXPECT_NEAR(weightedDistance({0, 0}, {0.5, 0}, c), 2.0 * weightedDistance({0, 0}, {0, 0.5}, c), 1e-12);
}

TEST(PathQualityLiftWeight, LessLiftBeatsLessTotalTravelWhenWeightedCostSaysSo)
{
  // A: lift 0.9, arm 0.1 (total raw travel 1.0, but 0.9 on the lift).
  // B: lift 0.1, arm 1.1 (more raw travel, mostly arm).
  const std::vector<std::vector<double>> a{{0.0, 0.0}, {0.9, 0.1}};
  const std::vector<std::vector<double>> b{{0.0, 0.0}, {0.1, 1.1}};
  PathCandidate ca{"A", a, -1.0};
  PathCandidate cb{"B", b, -1.0};
  PathQualityConfig plain;   // weight 1: A (0.906) beats B (1.105)
  EXPECT_EQ(selectBestCandidate({ca, cb}, FreeSpaceObjective::kLength, plain).best_index, 0);
  PathQualityConfig w2;
  w2.weights = applyLiftCostWeight({1.0, 1.0}, {true, false}, 2.0);  // A 1.81, B 1.11: B wins
  const auto sel = selectBestCandidate({ca, cb}, FreeSpaceObjective::kLength, w2);
  EXPECT_EQ(sel.best_index, 1);
  EXPECT_NEAR(sel.length, std::sqrt(0.2 * 0.2 + 1.1 * 1.1), 1e-12);
  // list order does not matter
  EXPECT_EQ(selectBestCandidate({cb, ca}, FreeSpaceObjective::kLength, w2).best_index, 0);
}

TEST(PathQualityLiftWeight, WeightOneSelectionMatchesDefaultConfigExactly)
{
  const std::vector<std::vector<double>> a{{0.0, 0.0}, {0.9, 0.1}};
  const std::vector<std::vector<double>> b{{0.0, 0.0}, {0.1, 1.1}};
  PathQualityConfig base;
  base.weights = weightsFromRanges({1.5, 6.28});
  PathQualityConfig one;
  one.weights = applyLiftCostWeight(base.weights, {true, false}, 1.0);
  const auto s0 = selectBestCandidate({{"A", a, -1.0}, {"B", b, -1.0}}, FreeSpaceObjective::kLength, base);
  const auto s1 = selectBestCandidate({{"A", a, -1.0}, {"B", b, -1.0}}, FreeSpaceObjective::kLength, one);
  EXPECT_EQ(s0.best_index, s1.best_index);
  EXPECT_DOUBLE_EQ(s0.length, s1.length);
}

TEST(PathQualityJointCosts, MultipliesEachJointByItsCost)
{
  const std::vector<double> base = weightsFromRanges({2.0, 4.0, 6.0});
  const auto w = applyJointCosts(base, {3.0, 1.0, 0.5});
  EXPECT_DOUBLE_EQ(w[0], 3.0 * base[0]);
  EXPECT_DOUBLE_EQ(w[1], base[1]);
  EXPECT_DOUBLE_EQ(w[2], 0.5 * base[2]);
}

TEST(PathQualityJointCosts, WrongSizeOrBadEntryLeavesTheWeights)
{
  const std::vector<double> base = weightsFromRanges({2.0, 4.0});
  EXPECT_EQ(applyJointCosts(base, {2.0}), base);
  EXPECT_EQ(applyJointCosts(base, {}), base);
  const auto w = applyJointCosts(base, {-1.0, 2.0});   // the bad entry counts as 1
  EXPECT_DOUBLE_EQ(w[0], base[0]);
  EXPECT_DOUBLE_EQ(w[1], 2.0 * base[1]);
}

TEST(PathQualityJointCosts, LiftTwoArmOneIsTheLiftWeight)
{
  const std::vector<double> base = weightsFromRanges({1.5, 6.28, 6.28});
  EXPECT_EQ(applyJointCosts(base, {2.0, 1.0, 1.0}), applyLiftCostWeight(base, {true, false, false}, 2.0));
}

TEST(PathQualityJointCosts, AnyJointCanBeTheCostlyOne)
{
  PathQualityConfig c;
  c.weights = applyJointCosts({1.0, 1.0, 1.0}, {1.0, 4.0, 1.0});
  EXPECT_NEAR(weightedDistance({0, 0, 0}, {0, 0.5, 0}, c), 4.0 * weightedDistance({0, 0, 0}, {0, 0, 0.5}, c), 1e-12);
}
