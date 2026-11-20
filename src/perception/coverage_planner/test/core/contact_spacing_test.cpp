// Contact-area model -> swath and line spacing, edge spacing adaptation, scored on a raster.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "coverage_planner/core/contact_spacing.hpp"
#include "coverage_planner/core/coverage_verification.hpp"
#include "coverage_planner/core/tool_profile_loading.hpp"
#include "ros_adapters/tool_profile_yaml_loader.hpp"

using namespace coverage_planner;  // NOLINT

namespace
{
core_types::ToolProfile padProfile(double force_n)
{
  core_types::ToolProfile p;
  p.name = "pad";
  p.task.contact = true;
  p.task.target_force_n = force_n;
  p.coverage.overlap_pct = 20.0;
  p.coverage.swath_m = 0.20;
  p.coverage.contact_area.pad_width_m = 0.20;
  p.coverage.contact_area.reference_force_n = 10.0;
  p.coverage.contact_area.width_gain_per_n = 0.01;   // +1 % width per N
  return p;
}

// Raster 2.4 x 1.0 m, 1 cm cells; the target is u in [0.2, 2.2]. Vertical strokes at `offsets`.
struct Scored
{
  VerificationReport report;
};

VerificationReport score(const std::vector<double> & offsets, double swath)
{
  const double res = 0.01;
  const int w = 240, h = 100;
  std::vector<bool> target(static_cast<size_t>(w * h), false);
  for (int r = 0; r < h; ++r) {
    for (int c = 20; c < 220; ++c) {target[static_cast<size_t>(r * w + c)] = true;}
  }
  CoverageVerifier verifier(0.0, 2.4, 0.0, 1.0, res, target);
  for (double u : offsets) {
    ToolStroke s;
    s.swath_m = swath;
    for (double v = 0.0; v <= 1.0001; v += 0.05) {s.samples.push_back({u, v, true});}
    verifier.addStroke(s);
  }
  return verifier.report();
}
}  // namespace

TEST(ContactSpacing, DefaultsKeepTheFixedSwath)
{
  auto p = padProfile(20.0);
  p.coverage.contact_area = {};   // model off
  const auto t = toCoverageProfile(p);
  EXPECT_DOUBLE_EQ(toolSwath(t), 0.20);
  EXPECT_NEAR(toolLineSpacing(t), 0.16, 1e-12);
  EXPECT_DOUBLE_EQ(t.coverage.edge_spacing_scale, 1.0);
}

TEST(ContactSpacing, SwathAndSpacingFollowTheTargetForce)
{
  const auto at10 = toCoverageProfile(padProfile(10.0));
  const auto at20 = toCoverageProfile(padProfile(20.0));
  const auto at30 = toCoverageProfile(padProfile(30.0));
  EXPECT_NEAR(toolSwath(at10), 0.20, 1e-12);
  EXPECT_NEAR(toolSwath(at20), 0.22, 1e-12);
  EXPECT_NEAR(toolSwath(at30), 0.24, 1e-12);
  EXPECT_NEAR(toolLineSpacing(at20), 0.22 * 0.8, 1e-12);
  EXPECT_LT(toolLineSpacing(at10), toolLineSpacing(at30));
}

TEST(ContactSpacing, ExplicitSwathOverrideDisablesTheModel)
{
  auto t = toCoverageProfile(padProfile(30.0));
  ToolOverrides o;
  o.swath_m = 0.15;
  applyToolOverrides(t, o);
  EXPECT_DOUBLE_EQ(toolSwath(t), 0.15);
  EXPECT_NEAR(t.coverage.obstacle_inflation_radius_m, 0.075, 1e-12);
}

TEST(ContactSpacing, InflationRadiusUsesTheModelSwath)
{
  auto t = toCoverageProfile(padProfile(30.0));
  applyToolOverrides(t, {});
  EXPECT_NEAR(t.coverage.obstacle_inflation_radius_m, 0.12, 1e-12);
}

TEST(ContactSpacing, ModelSpacingGivesEvenCoverageAcrossForces)
{
  std::cout << "force_n  swath  spacing  strips  covered  mean_overlap  outside\n";
  for (double force : {5.0, 10.0, 20.0, 30.0}) {
    const auto t = toCoverageProfile(padProfile(force));
    const double swath = toolSwath(t);
    const auto offsets = contactStripOffsets(t, 0.2 + swath / 2.0, 2.2 - swath / 2.0);
    const auto r = score(offsets, swath);
    std::cout << force << "  " << swath << "  " << toolLineSpacing(t) << "  " << offsets.size()
              << "  " << r.covered_fraction << "  " << r.mean_overlap << "  " << r.outside_cells
              << "\n";
    EXPECT_DOUBLE_EQ(r.covered_fraction, 1.0) << force;
    EXPECT_TRUE(r.missed.empty());
    EXPECT_EQ(r.outside_cells, 0U) << "overspray at " << force;
    // 20 % planned overlap: ~1.25 strokes per cell however wide the pad is.
    EXPECT_GT(r.mean_overlap, 1.15) << force;
    EXPECT_LT(r.mean_overlap, 1.45) << force;
  }
}

TEST(ContactSpacing, FixedSpacingIgnoringForceOverlapsMoreThanTheModel)
{
  const auto t = toCoverageProfile(padProfile(40.0));    // pad grows to 0.26 m
  const double swath = toolSwath(t);
  const double fixed_spacing = 0.16;                     // the old fixed 0.20 m swath spacing
  std::vector<double> fixed;
  for (double u = 0.2 + swath / 2.0; u < 2.2 - swath / 2.0; u += fixed_spacing) {fixed.push_back(u);}
  fixed.push_back(2.2 - swath / 2.0);
  const auto rf = score(fixed, swath);
  const auto rm = score(contactStripOffsets(t, 0.2 + swath / 2.0, 2.2 - swath / 2.0), swath);
  std::cout << "fixed spacing overlap " << rf.mean_overlap << ", model spacing overlap "
            << rm.mean_overlap << "\n";
  EXPECT_DOUBLE_EQ(rf.covered_fraction, 1.0);
  EXPECT_GT(rf.mean_overlap, rm.mean_overlap + 0.1);     // wasted passes
}

TEST(ContactSpacing, WiderSpacingThanTheModelLeavesGaps)
{
  const auto t = toCoverageProfile(padProfile(0.0));     // pad shrinks to 0.18 m
  const double swath = toolSwath(t);
  std::vector<double> offsets;
  for (double u = 0.2 + swath / 2.0; u < 2.2 - swath / 2.0; u += 0.22) {offsets.push_back(u);}
  const auto r = score(offsets, swath);
  EXPECT_LT(r.covered_fraction, 1.0);
  EXPECT_FALSE(r.missed.empty());
}

TEST(ContactSpacing, EdgeZoneNarrowsSpacingNearEdgesOnly)
{
  const auto uniform = contactStripOffsets(0.0, 2.0, 0.16, 0.0, 1.0);
  const auto edge = contactStripOffsets(0.0, 2.0, 0.16, 0.3, 0.5, {1.0});
  EXPECT_DOUBLE_EQ(uniform.front(), 0.0);
  EXPECT_DOUBLE_EQ(uniform.back(), 2.0);
  EXPECT_DOUBLE_EQ(edge.back(), 2.0);
  EXPECT_GT(edge.size(), uniform.size());
  double max_gap = 0.0, min_gap = 1e9, gap_near_obstacle = 1e9;
  for (size_t i = 1; i < edge.size(); ++i) {
    const double g = edge[i] - edge[i - 1];
    max_gap = std::max(max_gap, g);
    min_gap = std::min(min_gap, g);
    if (std::abs(edge[i - 1] - 1.0) < 0.1) {gap_near_obstacle = std::min(gap_near_obstacle, g);}
  }
  EXPECT_LE(max_gap, 0.16 + 1e-9);                      // never wider than uniform
  EXPECT_NEAR(min_gap, 0.08, 1e-9);                     // scale 0.5 in the zones
  EXPECT_NEAR(gap_near_obstacle, 0.08, 1e-9);           // the obstacle edge opened a zone
}

TEST(ContactSpacing, EdgeAdaptationIsDrivenByTheProfile)
{
  auto p = padProfile(10.0);
  p.coverage.edge_zone_m = 0.3;
  p.coverage.edge_spacing_scale = 0.5;
  const auto t = toCoverageProfile(p);
  const auto adaptive = contactStripOffsets(t, 0.1, 2.1);
  auto q = padProfile(10.0);
  const auto flat = contactStripOffsets(toCoverageProfile(q), 0.1, 2.1);
  EXPECT_GT(adaptive.size(), flat.size());
  const auto r = score(adaptive, toolSwath(t));
  EXPECT_DOUBLE_EQ(r.covered_fraction, 1.0);
}

TEST(ContactSpacing, EdgeZoneKeepsCoverageWhenThePadNarrowsNearTheEdge)
{
  // Near edges the pad only reaches 0.14 m of its 0.20 m (tilt/curvature). Uniform 0.16 m spacing
  // then leaves gaps; the edge zone (spacing 0.16 * 0.8 = 0.128) closes them.
  const double narrow = 0.14, wide = 0.20;
  const auto uniform = contactStripOffsets(0.3, 2.1, 0.16, 0.0, 1.0);
  const auto adaptive = contactStripOffsets(0.3, 2.1, 0.16, 0.5, 0.8);
  const auto coverage = [&](const std::vector<double> & offsets) {
      const double res = 0.01;
      CoverageVerifier verifier(0.0, 2.4, 0.0, 1.0, res);
      for (double u : offsets) {
        const bool edge = u < 0.8 || u > 1.6;
        ToolStroke s;
        s.swath_m = edge ? narrow : wide;
        for (double v = 0.0; v <= 1.0001; v += 0.05) {s.samples.push_back({u, v, true});}
        verifier.addStroke(s);
      }
      // Score only the span the strips must cover: [0.4, 2.0].
      size_t covered = 0, total = 0;
      for (int c = 40; c < 200; ++c) {
        ++total;
        if (verifier.hits(c, 50) > 0) {++covered;}
      }
      return static_cast<double>(covered) / static_cast<double>(total);
    };
  const double cu = coverage(uniform), ca = coverage(adaptive);
  std::cout << "narrow-edge pad: uniform coverage " << cu << ", edge-adapted " << ca << "\n";
  EXPECT_LT(cu, 1.0);
  EXPECT_DOUBLE_EQ(ca, 1.0);
}

TEST(ContactSpacing, RejectsBadArguments)
{
  EXPECT_THROW(contactStripOffsets(1.0, 0.0, 0.1, 0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(contactStripOffsets(0.0, 1.0, 0.0, 0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(contactStripOffsets(0.0, 1.0, 0.1, 0.1, 0.0), std::invalid_argument);
  EXPECT_THROW(contactStripOffsets(0.0, 1.0, 0.1, 0.1, 1.5), std::invalid_argument);
  EXPECT_THROW(contactStripOffsets(0.0, 1.0, 0.1, -0.1, 1.0), std::invalid_argument);
  EXPECT_EQ(contactStripOffsets(0.5, 0.5, 0.1, 0.0, 1.0).size(), 1U);
}

TEST(ContactSpacing, ShippedContactWiperIsUnchanged)
{
  const auto t = toCoverageProfile(ros_adapters::loadToolProfile(TOOL_PROFILES_PATH, "contact_wiper"));
  EXPECT_FALSE(t.coverage.contact_area.enabled());
  EXPECT_DOUBLE_EQ(toolSwath(t), 0.20);
  EXPECT_NEAR(toolLineSpacing(t), 0.16, 1e-12);
  EXPECT_EQ(contactStripOffsets(t, 0.1, 1.9).size(), contactStripOffsets(0.1, 1.9, 0.16, 0.0, 1.0).size());
}

TEST(ObstacleEdgeCells, FindsTheStepoverExtentOfEveryBlockedRegion)
{
  // a in [0, 19], b in [0, 9]; two blocks: a 4..6 (b 2..3) and a 12..12 (b 5..8)
  const auto blocked = [](int a, int b) {
      return (a >= 4 && a <= 6 && b >= 2 && b <= 3) || (a == 12 && b >= 5 && b <= 8);
    };
  EXPECT_EQ(obstacleEdgeCells(0, 19, 0, 9, blocked), (std::vector<int>{4, 6, 12}));
  EXPECT_TRUE(obstacleEdgeCells(0, 19, 0, 9, [](int, int) {return false;}).empty());
  EXPECT_TRUE(obstacleEdgeCells(5, 4, 0, 9, blocked).empty());
  // a region is cut at the box: only a in [5, 19] looks at the first block's cells 5..6
  EXPECT_EQ(obstacleEdgeCells(5, 19, 0, 9, blocked), (std::vector<int>{5, 6, 12}));
}

TEST(StripCellPositions, ObstacleEdgesOpenAZoneBothWays)
{
  // step 4, fine step 2, zone 3 cells, edge at 20 (range ends are far enough not to matter there)
  const auto up = stripCellPositions(0, 40, true, 4, 3, 0.5, {20});
  const std::vector<int> expect_up = {0, 2, 4, 8, 12, 16, 20, 22, 24, 28, 32, 36, 40};
  EXPECT_EQ(up, expect_up);
  const auto down = stripCellPositions(0, 40, false, 4, 3, 0.5, {20});
  const std::vector<int> expect_down = {40, 38, 36, 32, 28, 24, 20, 18, 16, 12, 8, 4, 0};
  EXPECT_EQ(down, expect_down);
  // no edges, no zone: the old uniform loop
  EXPECT_EQ(stripCellPositions(0, 20, true, 4, 0, 0.5, {10}), (std::vector<int>{0, 4, 8, 12, 16, 20}));
}

TEST(ResolveEdgeSpacing, RequestWinsOnlyWhenItIsSet)
{
  using P = std::pair<double, double>;
  EXPECT_EQ(resolveEdgeSpacing(0.15, 0.6, 0.0, 1.0), (P{0.15, 0.6}));
  EXPECT_EQ(resolveEdgeSpacing(0.0, 0.0, 0.3, 0.5), (P{0.3, 0.5}));   // unset request: the node's
  EXPECT_EQ(resolveEdgeSpacing(0.15, 1.0, 0.3, 0.5), (P{0.15, 0.5}));  // scale 1 (a profile default) is unset
  EXPECT_EQ(resolveEdgeSpacing(0.0, 0.0, 0.0, 1.0), (P{0.0, 1.0}));
  EXPECT_EQ(resolveEdgeSpacing(0.0, 0.0, 0.2, 7.0), (P{0.2, 1.0}));   // clamped
}
