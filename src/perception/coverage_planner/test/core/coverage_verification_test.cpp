#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "coverage_planner/core/coverage_verification.hpp"

using coverage_planner::CoverageVerifier;
using coverage_planner::ToolSample;
using coverage_planner::ToolStroke;

namespace
{
// A 2 m x 1 m wall, 1 cm cells; vertical strokes (along v) spaced `spacing` apart starting at u0.
ToolStroke vertical(double u, double swath, bool engaged = true)
{
  ToolStroke s;
  s.swath_m = swath;
  for (double v = 0.0; v <= 1.0001; v += 0.05) {s.samples.push_back({u, v, engaged});}
  return s;
}
CoverageVerifier wall() {return CoverageVerifier(0.0, 2.0, 0.0, 1.0, 0.01);}
}  // namespace

TEST(CoverageVerification, StrokesAtTheToolSpacingCoverTheWall)
{
  auto verifier = wall();
  const double swath = 0.2, spacing = swath * 0.8;   // 20 % overlap, as the contact wiper plans
  for (double u = 0.1; u < 2.0 + spacing; u += spacing) {verifier.addStroke(vertical(std::min(u, 1.9), swath));}
  const auto r = verifier.report();
  EXPECT_EQ(r.target_cells, 200U * 100U);
  EXPECT_DOUBLE_EQ(r.covered_fraction, 1.0);
  EXPECT_TRUE(r.missed.empty());
  EXPECT_GT(r.mean_overlap, 1.1);   // adjacent strokes overlap
  EXPECT_LT(r.mean_overlap, 1.6);
}

TEST(CoverageVerification, AGapBetweenStrokesIsReportedAsMissed)
{
  auto verifier = wall();
  const double swath = 0.1;
  for (double u : {0.05, 0.35, 0.65, 0.95, 1.25, 1.55, 1.85}) {   // spacing 0.3 = 3x the swath
    verifier.addStroke(vertical(u, swath));
  }
  const auto r = verifier.report();
  EXPECT_NEAR(r.covered_fraction, 0.1 / 0.3, 0.04);
  EXPECT_FALSE(r.missed.empty());
  EXPECT_EQ(r.covered_cells + r.missed.size(), r.target_cells);
}

TEST(CoverageVerification, DisengagedSamplesNeverCount)
{
  auto verifier = wall();
  verifier.addStroke(vertical(1.0, 0.2, /*engaged=*/false));   // transit with the tool off
  const auto r = verifier.report();
  EXPECT_EQ(r.covered_cells, 0U);
  EXPECT_DOUBLE_EQ(r.covered_fraction, 0.0);
}

TEST(CoverageVerification, ToolOffInTheMiddleLeavesAGap)
{
  auto verifier = wall();
  ToolStroke s;
  s.swath_m = 0.1;
  for (double v = 0.0; v <= 1.0001; v += 0.05) {
    s.samples.push_back({1.0, v, !(v > 0.4 && v < 0.6)});   // valve closed between 0.4 and 0.6
  }
  verifier.addStroke(s);
  EXPECT_EQ(verifier.hits(100, 50), 0);    // inside the closed stretch
  EXPECT_EQ(verifier.hits(100, 10), 1);
  EXPECT_EQ(verifier.hits(100, 90), 1);
}

TEST(CoverageVerification, OverlapCountsStrokesNotSamples)
{
  auto verifier = wall();
  ToolStroke dense;
  dense.swath_m = 0.1;
  for (double v = 0.0; v <= 1.0001; v += 0.005) {dense.samples.push_back({1.0, v, true});}
  verifier.addStroke(dense);
  EXPECT_EQ(verifier.hits(100, 50), 1);   // 200 samples, one stroke
  verifier.addStroke(vertical(1.0, 0.1));
  EXPECT_EQ(verifier.hits(100, 50), 2);
}

TEST(CoverageVerification, TargetMaskExcludesObstaclesAndCountsOverspray)
{
  const int w = 200, h = 100;
  std::vector<bool> target(static_cast<size_t>(w) * h, true);
  for (int r = 0; r < h; ++r) {
    for (int c = 90; c < 110; ++c) {target[static_cast<size_t>(r) * w + c] = false;}   // a 20 cm obstacle
  }
  CoverageVerifier verifier(0.0, 2.0, 0.0, 1.0, 0.01, target);
  verifier.addStroke(vertical(1.0, 0.4));   // reaches the obstacle columns and 10 cm beyond each side
  const auto r = verifier.report();
  EXPECT_EQ(r.target_cells, static_cast<size_t>(w * h - 20 * h));
  EXPECT_NEAR(static_cast<double>(r.covered_cells), 20.0 * h, 2.0 * h);   // 10 cm each side of the obstacle
  EXPECT_GT(r.outside_cells, 0U);
}

TEST(CoverageVerification, ALoneEngagedSampleMarksADisc)
{
  auto verifier = wall();
  ToolStroke s;
  s.swath_m = 0.2;
  s.samples.push_back({1.0, 0.5, true});
  verifier.addStroke(s);
  const auto r = verifier.report();
  EXPECT_NEAR(static_cast<double>(r.covered_cells), M_PI * 0.1 * 0.1 / (0.01 * 0.01), 12.0);
}

TEST(CoverageVerification, RejectsBadInput)
{
  EXPECT_THROW(CoverageVerifier(0.0, 1.0, 0.0, 1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(CoverageVerifier(1.0, 1.0, 0.0, 1.0, 0.01), std::invalid_argument);
  EXPECT_THROW(CoverageVerifier(0.0, 1.0, 0.0, 1.0, 0.01, std::vector<bool>(3, true)), std::invalid_argument);
  auto verifier = wall();
  EXPECT_THROW(verifier.addStroke(ToolStroke{}), std::invalid_argument);
}

TEST(CoverageVerification, NoTargetMeansFullyCovered)
{
  CoverageVerifier verifier(0.0, 1.0, 0.0, 1.0, 0.1, std::vector<bool>(100, false));
  EXPECT_DOUBLE_EQ(verifier.report().covered_fraction, 1.0);
}
