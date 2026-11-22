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
