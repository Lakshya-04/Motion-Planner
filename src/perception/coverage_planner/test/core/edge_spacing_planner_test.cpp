#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <vector>

#include "coverage_planner/core/boustrophedon_planner.hpp"
#include "coverage_planner/core/contact_spacing.hpp"
#include "coverage_planner/core/coverage_planner_base.hpp"
#include "coverage_planner/core/sweepline_planner.hpp"

using namespace coverage_planner;  // NOLINT

namespace
{
OccupancyGrid emptyGrid(int w, int h, double res = 0.05)
{
  OccupancyGrid g;
  g.header.frame_id = "wall";
  g.info.resolution = res;
  g.info.width = w;
  g.info.height = h;
  g.data.assign(static_cast<size_t>(w) * h, 0);
  return g;
}

CoveragePlannerParams params(double edge_zone_m, double scale)
{
  CoveragePlannerParams p{};
  p.swath = 0.2;
  p.step = 0.2;
  p.sweep_direction = "vertical";
  p.start_corner = "bottom_left";
  p.obstacle_inflation_radius = 0.0;
  p.allow_unknown = false;
  p.pose_mode = "identity";
  p.map_frame = "wall";
  p.plan_all_components = true;
  p.min_component_cells = 1;
  p.requires_contact = true;
  p.ensure_coverage_within_boundary = true;   // sweepline: last strip flush with the far edge
  p.edge_zone_m = edge_zone_m;
  p.edge_spacing_scale = scale;
  return p;
}

// Sorted x of every strip (vertical sweeps keep x constant along a strip).
std::vector<double> stripXs(const CoveragePlannerResult & r)
{
  std::set<long> keys;
  for (const auto & s : r.segments) {
    if (s.label.rfind("strip", 0) != 0) {continue;}
    keys.insert(std::lround(r.path.poses[s.start_index].pose.position.x * 1000.0));
  }
  std::vector<double> out;
  for (long k : keys) {out.push_back(k / 1000.0);}
  return out;
}

std::vector<double> gaps(const std::vector<double> & xs)
{
  std::vector<double> g;
  for (size_t i = 1; i < xs.size(); ++i) {g.push_back(xs[i] - xs[i - 1]);}
  return g;
}

template<typename Planner>
void expectAdaptive(const std::string & decomposition)
{
  const auto grid = emptyGrid(80, 60);   // 4.0 x 3.0 m
  auto uniform = params(0.0, 1.0);
  uniform.decomposition = decomposition;
  auto adaptive = params(0.3, 0.5);
  adaptive.decomposition = decomposition;

  Planner planner;
  const auto ru = planner.generatePath(grid, uniform);
  const auto ra = planner.generatePath(grid, adaptive);
  ASSERT_TRUE(ru.success) << ru.message;
  ASSERT_TRUE(ra.success) << ra.message;
  const auto xu = stripXs(ru), xa = stripXs(ra);
  ASSERT_GT(xu.size(), 5U);
  EXPECT_GT(xa.size(), xu.size());   // finer near the two ends

  const double lo = xa.front(), hi = xa.back();
  const double fine = 0.1 + 1e-6, coarse = 0.2;
  bool saw_fine_low = false, saw_fine_high = false, saw_normal_middle = false;
  for (size_t i = 1; i < xa.size(); ++i) {
    const double gap = xa[i] - xa[i - 1], x0 = xa[i - 1];
    if (x0 - lo < 0.3 - 1e-6) {
      EXPECT_LE(gap, fine) << "gap after x=" << x0 << " in the low zone";
      saw_fine_low = true;
    } else if (hi - x0 < 0.3 - 1e-6) {
      EXPECT_LE(gap, fine) << "gap after x=" << x0 << " in the high zone";
      saw_fine_high = true;
    } else {
      EXPECT_NEAR(gap, coarse, 1e-6) << "gap after x=" << x0 << " in the middle";
      saw_normal_middle = true;
    }
  }
  EXPECT_TRUE(saw_fine_low);
  EXPECT_TRUE(saw_fine_high);
  EXPECT_TRUE(saw_normal_middle);
}
}  // namespace

TEST(StripCellPositions, UniformIsThePlainLoop)
{
  EXPECT_EQ(stripCellPositions(0, 20, true, 4, 0, 1.0), (std::vector<int>{0, 4, 8, 12, 16, 20}));
  EXPECT_EQ(stripCellPositions(0, 20, true, 4, 6, 1.0), (std::vector<int>{0, 4, 8, 12, 16, 20}));
  EXPECT_EQ(stripCellPositions(0, 19, true, 4, 0, 1.0), (std::vector<int>{0, 4, 8, 12, 16}));   // no forced last
  EXPECT_EQ(stripCellPositions(0, 20, false, 4, 0, 0.5), (std::vector<int>{20, 16, 12, 8, 4, 0}));
  EXPECT_TRUE(stripCellPositions(5, 3, true, 4, 0, 1.0).empty());
}

TEST(StripCellPositions, ZonesAtBothEnds)
{
  EXPECT_EQ(stripCellPositions(0, 20, true, 4, 6, 0.5), (std::vector<int>{0, 2, 4, 6, 10, 14, 18, 20}));
  EXPECT_EQ(stripCellPositions(0, 20, false, 4, 6, 0.5), (std::vector<int>{20, 18, 16, 14, 10, 6, 2, 0}));
  EXPECT_THROW(stripCellPositions(0, 20, true, 0, 6, 0.5), std::invalid_argument);
  EXPECT_THROW(stripCellPositions(0, 20, true, 4, 6, 0.0), std::invalid_argument);
  EXPECT_THROW(stripCellPositions(0, 20, true, 4, 6, 1.5), std::invalid_argument);
}

TEST(EdgeSpacingPlanner, SweeplineStripsAreDenserAtTheEnds)
{
  expectAdaptive<SweeplinePlanner>("columns");
}

TEST(EdgeSpacingPlanner, BoustrophedonColumnsAreDenserAtTheEnds)
{
  expectAdaptive<BoustrophedonPlanner>("columns");
}

TEST(EdgeSpacingPlanner, BoustrophedonCellsAreDenserAtTheEnds)
{
  expectAdaptive<BoustrophedonPlanner>("cells");
}

TEST(EdgeSpacingPlanner, DefaultsLeaveThePlansUnchanged)
{
  const auto grid = emptyGrid(80, 60);
  for (const char * decomposition : {"columns", "cells"}) {
    auto base = params(0.0, 1.0);
    base.decomposition = decomposition;
    auto zone_only = params(0.3, 1.0);   // zone without a scale change is still uniform
    zone_only.decomposition = decomposition;
    BoustrophedonPlanner b;
    EXPECT_EQ(stripXs(b.generatePath(grid, base)), stripXs(b.generatePath(grid, zone_only))) << decomposition;
  }
  SweeplinePlanner s;
  EXPECT_EQ(stripXs(s.generatePath(grid, params(0.0, 1.0))), stripXs(s.generatePath(grid, params(0.3, 1.0))));
}
