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

namespace
{
// A 4 x 3 m wall with a 0.5 m wide obstacle block, x cells 30..39 (1.5 m to 2.0 m), y cells 20..39.
OccupancyGrid gridWithObstacle()
{
  auto g = emptyGrid(80, 60);
  for (int y = 20; y < 40; ++y) {
    for (int x = 30; x < 40; ++x) {g.data[static_cast<size_t>(y) * 80 + x] = 100;}
  }
  return g;
}

template<typename Planner>
void expectObstacleEdgesOpenAZone()
{
  const auto grid = gridWithObstacle();
  const double res = 0.05, zone = 0.3;
  Planner planner;
  const auto ru = planner.generatePath(grid, params(0.0, 1.0));
  const auto ra = planner.generatePath(grid, params(zone, 0.5));
  ASSERT_TRUE(ru.success) << ru.message;
  ASSERT_TRUE(ra.success) << ra.message;
  const auto xu = stripXs(ru), xa = stripXs(ra);
  EXPECT_GT(xa.size(), xu.size());

  const double edges[2] = {30 * res, 40 * res};   // left and right border of the block
  const double lo = xa.front(), hi = xa.back();
  const double margin = 1.5 * res;   // cell centre vs. cell edge, rounding of the zone in cells
  bool saw_fine_at_obstacle = false, saw_uniform_far = false;
  for (size_t i = 1; i < xa.size(); ++i) {
    const double x0 = xa[i - 1], gap = xa[i] - x0;
    const bool near_end = x0 - lo < zone + margin || hi - x0 < zone + margin;
    const bool near_edge = std::abs(x0 - edges[0]) < zone - margin || std::abs(x0 - edges[1]) < zone - margin;
    const bool far = !near_end && std::abs(x0 - edges[0]) > zone + 0.2 + margin &&
      std::abs(x0 - edges[1]) > zone + 0.2 + margin;
    if (near_edge && !near_end) {
      EXPECT_LE(gap, 0.1 + 1e-6) << "gap after x=" << x0 << " beside the obstacle";
      saw_fine_at_obstacle = true;
    } else if (far) {
      EXPECT_NEAR(gap, 0.2, 1e-6) << "gap after x=" << x0 << " far from every edge";
      saw_uniform_far = true;
    }
  }
  EXPECT_TRUE(saw_fine_at_obstacle);
  EXPECT_TRUE(saw_uniform_far);

  // Without any obstacle in the grid the obstacle edge changes nothing in the middle.
  const auto rempty = planner.generatePath(emptyGrid(80, 60), params(zone, 0.5));
  const auto xe = stripXs(rempty);
  EXPECT_LT(xe.size(), xa.size());
}
}  // namespace

TEST(EdgeSpacingPlanner, SweeplineObstacleEdgesOpenAZone)
{
  expectObstacleEdgesOpenAZone<SweeplinePlanner>();
}

TEST(EdgeSpacingPlanner, BoustrophedonColumnsObstacleEdgesOpenAZone)
{
  expectObstacleEdgesOpenAZone<BoustrophedonPlanner>();
}

TEST(EdgeSpacingPlanner, ObstacleEdgesDoNothingWhenTheZoneIsOff)
{
  const auto grid = gridWithObstacle();
  SweeplinePlanner s;
  BoustrophedonPlanner b;
  for (const auto & p : {params(0.0, 0.5), params(0.3, 1.0)}) {
    EXPECT_EQ(stripXs(s.generatePath(grid, p)), stripXs(s.generatePath(grid, params(0.0, 1.0))));
    EXPECT_EQ(stripXs(b.generatePath(grid, p)), stripXs(b.generatePath(grid, params(0.0, 1.0))));
  }
}
