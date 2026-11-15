#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "coverage_planner/core/run_scorecard.hpp"

using namespace coverage_planner;  // NOLINT

namespace
{
// A 2 m x 1 m wall at the origin with u = x, v = y (identity pose).
WallPlane wall()
{
  WallPlane w;
  w.id = 1;
  w.pose.pose.orientation.w = 1.0;
  w.u_min = 0.0; w.u_max = 2.0; w.v_min = 0.0; w.v_max = 1.0;
  return w;
}

// Vertical strokes every `spacing`, tool pressing at `force`, tool lifted (force 0) between them.
std::vector<RunSample> run(double spacing, double force, double swath_unused = 0.0)
{
  (void)swath_unused;
  std::vector<RunSample> out;
  double t = 0.0;
  for (double u = 0.1; u <= 1.9001; u += spacing) {
    for (double y = 0.0; y <= 1.0001; y += 0.05) {
      RunSample s;
      s.t_s = t; t += 0.1;
      s.x = u; s.y = y; s.z = 0.0;
      s.force_n = force;
      out.push_back(s);
    }
    RunSample lift;   // retract between strokes
    lift.t_s = t; t += 0.1;
    lift.x = u; lift.y = 1.0; lift.force_n = 0.0;
    out.push_back(lift);
  }
  return out;
}

ScorecardOptions contactOptions()
{
  ScorecardOptions o;
  o.contact = true; o.swath_m = 0.2; o.resolution_m = 0.01;
  o.engaged_force_n = 1.0; o.target_force_n = 10.0; o.force_band_n = 1.0;
  return o;
}
}  // namespace

TEST(RunScorecard, GoodRunPasses)
{
  ScorecardLimits limits;
  limits.min_covered_fraction = 0.95; limits.max_force_rms_error_n = 0.5; limits.max_force_peak_n = 15.0;
  limits.min_in_band_fraction = 0.9;
  const auto card = scoreRun(wall(), {}, run(0.16, 10.1), contactOptions(), limits);
  EXPECT_TRUE(card.passed) << (card.failures.empty() ? "" : card.failures.front());
  EXPECT_GT(card.coverage.covered_fraction, 0.95);
  EXPECT_NEAR(card.force_rms_error_n, 0.1, 1e-9);
  EXPECT_DOUBLE_EQ(card.in_band_fraction, 1.0);
  EXPECT_EQ(card.strokes, 12U);   // 0.1 .. 1.86: one stroke per vertical pass, split by the lifts
}

TEST(RunScorecard, GapsInTheWallFailCoverage)
{
  const auto card = scoreRun(wall(), {}, run(0.5, 10.0), contactOptions());   // spacing 2.5x the swath
  EXPECT_FALSE(card.passed);
  EXPECT_LT(card.coverage.covered_fraction, 0.6);
  ASSERT_FALSE(card.failures.empty());
  EXPECT_NE(card.failures.front().find("coverage"), std::string::npos);
}

TEST(RunScorecard, ForceOffTargetAndOverPeakFail)
{
  ScorecardLimits limits;
  limits.max_force_rms_error_n = 0.5; limits.max_force_peak_n = 12.0;
  const auto card = scoreRun(wall(), {}, run(0.16, 14.0), contactOptions(), limits);
  EXPECT_FALSE(card.passed);
  EXPECT_NEAR(card.force_rms_error_n, 4.0, 1e-9);
  EXPECT_DOUBLE_EQ(card.force_peak_n, 14.0);
  EXPECT_GE(card.failures.size(), 2U);
}

TEST(RunScorecard, ObstaclesAreExcludedFromTheTarget)
{
  ObstacleRegion hole;
  hole.parent_wall = 1; hole.u_min = 0.9; hole.u_max = 1.1; hole.v_min = 0.0; hole.v_max = 1.0;
  const auto without = scoreRun(wall(), {}, run(0.5, 10.0), contactOptions());
  const auto with = scoreRun(wall(), {hole}, run(0.5, 10.0), contactOptions());
  EXPECT_LT(with.coverage.target_cells, without.coverage.target_cells);
  EXPECT_NEAR(static_cast<double>(with.coverage.target_cells), 180.0 * 100.0, 1.0);
}

TEST(RunScorecard, NonContactCountsWhileTheValveIsOpen)
{
  auto samples = run(0.16, 0.0);   // no force at all: a spray tool does not press
  for (auto & s : samples) {s.tool_active = (s.y < 1.0);}   // valve closed during the lifts
  ScorecardOptions o = contactOptions();
  o.contact = false;
  const auto card = scoreRun(wall(), {}, samples, o);
  EXPECT_GT(card.coverage.covered_fraction, 0.95);
  EXPECT_DOUBLE_EQ(card.force_rms_error_n, 0.0);   // force statistics are for contact tools
  EXPECT_DOUBLE_EQ(card.in_band_fraction, 0.0);
}

TEST(RunScorecard, DurationAndEngagedTime)
{
  const auto samples = run(0.16, 10.0);
  const auto card = scoreRun(wall(), {}, samples, contactOptions());
  EXPECT_NEAR(card.duration_s, samples.back().t_s - samples.front().t_s, 1e-9);
  EXPECT_GT(card.engaged_s, 0.0);
  EXPECT_LT(card.engaged_s, card.duration_s);
}

TEST(RunScorecard, EmptyRunCoversNothingAndRejectsBadOptions)
{
  const auto card = scoreRun(wall(), {}, {}, contactOptions());
  EXPECT_FALSE(card.passed);
  EXPECT_DOUBLE_EQ(card.coverage.covered_fraction, 0.0);
  ScorecardOptions bad = contactOptions();
  bad.swath_m = 0.0;
  EXPECT_THROW(scoreRun(wall(), {}, {}, bad), std::invalid_argument);
}
