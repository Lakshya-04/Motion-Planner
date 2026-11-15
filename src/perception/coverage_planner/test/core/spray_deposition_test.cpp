#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "coverage_planner/core/coverage_verification.hpp"
#include "coverage_planner/core/spray_deposition.hpp"

using namespace coverage_planner;  // NOLINT

TEST(SprayFootprint, WidthIsTwoDTanHalfAngle)
{
  const double theta = 20.0, d = 0.30;
  const auto fp = SprayFootprint::fromCone(d, theta);
  EXPECT_NEAR(2.0 * fp.half_width_m, 2.0 * d * std::tan(theta * M_PI / 180.0), 1e-12);
  // linear in the stand-off
  EXPECT_NEAR(SprayFootprint::fromCone(2 * d, theta).half_width_m, 2.0 * fp.half_width_m, 1e-12);
  EXPECT_THROW(SprayFootprint::fromCone(0.0, theta), std::invalid_argument);
  EXPECT_THROW(SprayFootprint::fromCone(d, 90.0), std::invalid_argument);
  EXPECT_THROW(SprayFootprint::fromCone(d, theta, 0.0), std::invalid_argument);
}

TEST(SprayFootprint, DensityIsNormalisedSymmetricAndPeaksOnAxis)
{
  const auto fp = SprayFootprint::fromCone(0.3, 20.0);
  double integral = 0.0;
  const double dx = 1e-5;
  for (double x = -fp.half_width_m; x < fp.half_width_m; x += dx) {integral += fp.density(x + 0.5 * dx) * dx;}
  EXPECT_NEAR(integral, 1.0, 1e-3);
  EXPECT_DOUBLE_EQ(fp.density(0.03), fp.density(-0.03));
  EXPECT_GT(fp.density(0.0), fp.density(0.03));
  EXPECT_GT(fp.density(0.03), fp.density(0.06));
  EXPECT_DOUBLE_EQ(fp.density(fp.half_width_m * 1.01), 0.0);
  // edge at two sigma: 13.5 % of the peak
  EXPECT_NEAR(fp.density(fp.half_width_m) / fp.density(0.0), std::exp(-2.0), 1e-9);
}

namespace
{
// Vertical strokes (along v) at u = k * spacing across a 1 m x 1 m wall, 1 cm cells.
CoverageVerifier sprayWall(double spacing, double standoff, bool valve_on = true)
{
  CoverageVerifier verifier(0.0, 1.0, 0.0, 1.0, 0.01);
  const auto fp = SprayFootprint::fromCone(standoff, 20.0);
  const double swath = 0.8 * 2.0 * fp.half_width_m;   // the effective swath the planner spaces by
  for (double u = -0.4; u <= 1.4 + 1e-9; u += spacing) {   // strokes run past the wall: no edge fall-off inside
    ToolStroke s;
    s.swath_m = swath;
    for (double v = -0.1; v <= 1.1001; v += 0.02) {s.samples.push_back({u, v, valve_on});}
    verifier.addSprayStroke(s, fp);
  }
  return verifier;
}
}  // namespace

TEST(SprayThickness, SinglePassProfileFollowsTheFootprint)
{
  CoverageVerifier verifier(0.0, 1.0, 0.0, 1.0, 0.01);
  const auto fp = SprayFootprint::fromCone(0.3, 20.0);
  ToolStroke s;
  s.swath_m = 2.0 * fp.half_width_m;
  for (double v = 0.0; v <= 1.0001; v += 0.02) {s.samples.push_back({0.5, v, true});}
  verifier.addSprayStroke(s, fp);
  const int row = 50;
  // along a row the thickness is g(u - 0.5) times the stroke length that row sees (a 2 cm segment
  // covers the row, deposit g * 0.02)
  EXPECT_NEAR(verifier.thickness(50, row), fp.density(0.005) * 0.02, 0.05 * fp.density(0.005) * 0.02);
  EXPECT_GT(verifier.thickness(50, row), verifier.thickness(60, row));
  EXPECT_DOUBLE_EQ(verifier.thickness(10, row), 0.0);   // outside the strip
  // summed over the column of cells along the stroke, the profile integrates to the stroke length
  double total = 0.0;
  for (int c = 0; c < verifier.width(); ++c) {total += verifier.thickness(c, row) * 0.01;}
  EXPECT_NEAR(total, 0.02, 0.002);
}

TEST(SprayThickness, ClosedValveDepositsNothing)
{
  auto verifier = sprayWall(0.1, 0.3, /*valve_on=*/false);
  const auto r = verifier.report();
  EXPECT_EQ(r.covered_cells, 0U);
  EXPECT_DOUBLE_EQ(r.thickness.mean, 0.0);
  EXPECT_DOUBLE_EQ(verifier.thickness(50, 50), 0.0);
}

TEST(SprayThickness, CloserSpacingIsMoreUniform)
{
  const double w_eff = 0.8 * 2.0 * 0.3 * std::tan(20.0 * M_PI / 180.0);
  const auto tight = sprayWall(0.5 * w_eff, 0.3).report();
  const auto planned = sprayWall(0.8 * w_eff, 0.3).report();   // 20 % overlap
  const auto sparse = sprayWall(1.6 * w_eff, 0.3).report();    // gaps between the passes
  ASSERT_TRUE(tight.thickness.valid);
  EXPECT_LT(tight.thickness.cov, planned.thickness.cov);
  EXPECT_LT(planned.thickness.cov, sparse.thickness.cov);
  EXPECT_LT(sparse.covered_fraction, 1.0);   // the gaps also show up as missed cells
  EXPECT_GT(planned.thickness.mean, 0.0);
  EXPECT_LE(planned.thickness.min, planned.thickness.mean);
  EXPECT_GE(planned.thickness.max, planned.thickness.mean);
}

TEST(SprayThickness, NoSprayStrokesLeavesTheStatsInvalid)
{
  CoverageVerifier verifier(0.0, 1.0, 0.0, 1.0, 0.01);
  ToolStroke s;
  s.swath_m = 0.1;
  s.samples = {{0.5, 0.0, true}, {0.5, 1.0, true}};
  verifier.addStroke(s);
  EXPECT_FALSE(verifier.report().thickness.valid);
}
