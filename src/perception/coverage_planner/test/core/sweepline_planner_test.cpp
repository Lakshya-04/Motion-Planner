#include <gtest/gtest.h>

#include <cmath>
#include <set>

#include "coverage_planner/core/coverage_planner_base.hpp"
#include "coverage_planner/core/sweepline_planner.hpp"

using namespace coverage_planner;  // NOLINT

namespace
{
// Row-major grid, y up. `blocked(x, y)` marks obstacle cells.
template<typename F>
OccupancyGrid makeGrid(int w, int h, F blocked, double res = 0.05)
{
  OccupancyGrid g;
  g.header.frame_id = "wall";
  g.info.resolution = res;
  g.info.width = w;
  g.info.height = h;
  g.data.assign(static_cast<size_t>(w) * h, 0);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (blocked(x, y)) {g.data[static_cast<size_t>(y) * w + x] = 100;}
    }
  }
  return g;
}

CoveragePlannerParams params(double swath = 0.2, double step = 0.2)
{
  CoveragePlannerParams p{};
  p.swath = swath;
  p.step = step;
  p.sweep_direction = "vertical";
  p.start_corner = "bottom_left";
  p.obstacle_inflation_radius = 0.0;
  p.allow_unknown = false;
  p.pose_mode = "identity";
  p.map_frame = "wall";
  p.plan_all_components = true;
  p.min_component_cells = 1;
  p.requires_contact = true;
  return p;
}

// Cells covered by the strips' swath footprints (a swath-wide band around each strip).
std::set<std::pair<int, int>> covered(const OccupancyGrid & g, const CoveragePlannerResult & r, double swath)
{
  std::set<std::pair<int, int>> out;
  const int half = static_cast<int>(std::lround(0.5 * swath / g.info.resolution));
  for (const auto & s : r.segments) {
    if (s.label.rfind("strip", 0) != 0) {continue;}
    const auto & a = r.path.poses[s.start_index].pose.position;
    const auto & b = r.path.poses[s.end_index].pose.position;
    const int n = 200;
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / n;
      const int cx = static_cast<int>(std::floor((a.x + t * (b.x - a.x)) / g.info.resolution));
      const int cy = static_cast<int>(std::floor((a.y + t * (b.y - a.y)) / g.info.resolution));
      for (int dx = -half; dx <= half; ++dx) {
        for (int dy = -half; dy <= half; ++dy) {out.insert({cx + dx, cy + dy});}
      }
    }
  }
  return out;
}
}  // namespace

TEST(Sweepline, FactoryListsAndCreatesIt)
{
  const auto names = CoveragePlannerFactory::getAvailablePlanners();
  EXPECT_NE(std::find(names.begin(), names.end(), "sweepline"), names.end());
  const auto planner = CoveragePlannerFactory::createPlanner("sweepline");
  ASSERT_NE(planner, nullptr);
  EXPECT_EQ(planner->getName(), "sweepline");
}

TEST(Sweepline, EmptyWallIsOneSerpentineChain)
{
  const auto g = makeGrid(80, 60, [](int, int) {return false;});  // 4.0 x 3.0 m
  SweeplinePlanner planner;
  const auto r = planner.generatePath(g, params());
  ASSERT_TRUE(r.success) << r.message;
  // slices from x=2 (half swath = 2 cells) every 4 cells up to x=77: 19 + final gap logic
  EXPECT_GE(r.num_strips, 19);
  EXPECT_EQ(r.num_obstacle_splits, 0);  // a single chain
  // Serpentine: consecutive strips alternate direction.
  for (size_t i = 0; i + 1 < r.segments.size(); ++i) {
    const auto & a0 = r.path.poses[r.segments[i].start_index].pose.position;
    const auto & a1 = r.path.poses[r.segments[i].end_index].pose.position;
    const auto & b0 = r.path.poses[r.segments[i + 1].start_index].pose.position;
    const auto & b1 = r.path.poses[r.segments[i + 1].end_index].pose.position;
    EXPECT_LT((a1.y - a0.y) * (b1.y - b0.y), 0.0) << "strips " << i << " and " << i + 1;
  }
  // The end of one strip is next to the start of the next (a short turn, no long jump).
  for (size_t i = 0; i + 1 < r.segments.size(); ++i) {
    const auto & e = r.path.poses[r.segments[i].end_index].pose.position;
    const auto & s = r.path.poses[r.segments[i + 1].start_index].pose.position;
    EXPECT_LT(std::hypot(e.x - s.x, e.y - s.y), 0.45);
  }
}

TEST(Sweepline, ObstacleSplitsPassesAndIsNeverCrossed)
{
  // A 0.6 x 0.6 m block in the middle.
  const auto g = makeGrid(80, 60, [](int x, int y) {return x >= 34 && x < 46 && y >= 24 && y < 36;});
  SweeplinePlanner planner;
  const auto r = planner.generatePath(g, params());
  ASSERT_TRUE(r.success) << r.message;
  for (const auto & s : r.segments) {
    const auto & a = r.path.poses[s.start_index].pose.position;
    const auto & b = r.path.poses[s.end_index].pose.position;
    for (int i = 0; i <= 100; ++i) {
      const double t = i / 100.0;
      const int cx = static_cast<int>(std::floor((a.x + t * (b.x - a.x)) / 0.05));
      const int cy = static_cast<int>(std::floor((a.y + t * (b.y - a.y)) / 0.05));
      EXPECT_FALSE(g.data[static_cast<size_t>(cy) * 80 + cx] > 50) << cx << "," << cy;
    }
  }
  // The strips next to the block are split in two (below and above it).
  EXPECT_GT(r.num_strips, 19);
}

TEST(Sweepline, ConcaveObstacleIsHandled)
{
  // A U-shaped (concave) obstacle: the pocket inside must not be crossed or "bridged".
  const auto g = makeGrid(80, 60, [](int x, int y) {
      const bool left = x >= 30 && x < 34 && y >= 15 && y < 45;
      const bool right = x >= 46 && x < 50 && y >= 15 && y < 45;
      const bool bottom = x >= 30 && x < 50 && y >= 15 && y < 19;
      return left || right || bottom;
    });
  SweeplinePlanner planner;
  const auto r = planner.generatePath(g, params());
  ASSERT_TRUE(r.success) << r.message;
  for (const auto & s : r.segments) {
    const auto & a = r.path.poses[s.start_index].pose.position;
    const auto & b = r.path.poses[s.end_index].pose.position;
    for (int i = 0; i <= 100; ++i) {
      const double t = i / 100.0;
      const int cx = static_cast<int>(std::floor((a.x + t * (b.x - a.x)) / 0.05));
      const int cy = static_cast<int>(std::floor((a.y + t * (b.y - a.y)) / 0.05));
      EXPECT_LE(g.data[static_cast<size_t>(cy) * 80 + cx], 50);
    }
  }
  // The pocket interior (x 34..46, y 19..45) is free space reachable from above: it is covered.
  const auto cov = covered(g, r, 0.2);
  EXPECT_TRUE(cov.count({40, 35}) > 0);
}

TEST(Sweepline, CoversNearlyAllFreeCells)
{
  const auto g = makeGrid(80, 60, [](int x, int y) {return x >= 34 && x < 46 && y >= 24 && y < 36;});
  SweeplinePlanner planner;
  auto p = params();
  p.ensure_coverage_within_boundary = true;
  const auto r = planner.generatePath(g, p);
  ASSERT_TRUE(r.success);
  const auto cov = covered(g, r, 0.2);
  int free_cells = 0, hit = 0;
  for (int y = 0; y < 60; ++y) {
    for (int x = 0; x < 80; ++x) {
      if (g.data[static_cast<size_t>(y) * 80 + x] > 50) {continue;}
      ++free_cells;
      if (cov.count({x, y})) {++hit;}
    }
  }
  EXPECT_GT(static_cast<double>(hit) / free_cells, 0.97);
}

TEST(Sweepline, FullHeightBlockMakesTwoChains)
{
  const auto g = makeGrid(80, 60, [](int x, int) {return x >= 38 && x < 42;});
  SweeplinePlanner planner;
  const auto r = planner.generatePath(g, params());
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_EQ(r.num_obstacle_splits, 1);  // two chains
  std::set<std::string> chains;
  for (const auto & s : r.segments) {
    for (const auto & t : s.tags) {if (t.rfind("chain_", 0) == 0 && t != "chain_start") {chains.insert(t);}}
  }
  EXPECT_EQ(chains.size(), 2U);
}

TEST(Sweepline, HorizontalDirectionTransposesThePlan)
{
  const auto g = makeGrid(80, 60, [](int, int) {return false;});
  SweeplinePlanner planner;
  auto p = params();
  p.sweep_direction = "horizontal";
  const auto r = planner.generatePath(g, p);
  ASSERT_TRUE(r.success);
  for (const auto & s : r.segments) {
    const auto & a = r.path.poses[s.start_index].pose.position;
    const auto & b = r.path.poses[s.end_index].pose.position;
    EXPECT_NEAR(a.y, b.y, 1e-9);  // rows run along x
  }
}

TEST(Sweepline, ConnectorsAreFspAndRetractWhenRequested)
{
  const auto g = makeGrid(40, 30, [](int, int) {return false;});
  SweeplinePlanner planner;
  auto p = params();
  p.add_strip_connectors = true;
  const auto r = planner.generatePath(g, p);
  ASSERT_TRUE(r.success);
  int connectors = 0;
  for (const auto & s : r.segments) {
    if (s.label.rfind("connector", 0) == 0) {
      ++connectors;
      EXPECT_EQ(s.mode, "fsp");
      EXPECT_TRUE(s.requires_retraction);
      EXPECT_FALSE(s.requires_contact);
    } else {
      EXPECT_EQ(s.mode, "cartesian");
      EXPECT_TRUE(s.requires_contact);
    }
  }
  EXPECT_GT(connectors, 0);
}

TEST(Sweepline, EmptyOrInvalidMapsFailWithAMessage)
{
  SweeplinePlanner planner;
  OccupancyGrid empty;
  EXPECT_FALSE(planner.generatePath(empty, params()).success);
  const auto all_blocked = makeGrid(20, 20, [](int, int) {return true;});
  const auto r = planner.generatePath(all_blocked, params());
  EXPECT_FALSE(r.success);
  EXPECT_NE(r.message.find("No free space"), std::string::npos);
  auto bad = params();
  bad.step = 0.0;
  EXPECT_FALSE(planner.generatePath(makeGrid(20, 20, [](int, int) {return false;}), bad).success);
}

TEST(Sweepline, StartCornerChoosesWhereThePlanBegins)
{
  const auto g = makeGrid(80, 60, [](int, int) {return false;});
  SweeplinePlanner planner;
  auto p = params();
  p.start_corner = "top_right";
  const auto r = planner.generatePath(g, p);
  ASSERT_TRUE(r.success);
  const auto & first = r.path.poses.front().pose.position;
  EXPECT_GT(first.x, 2.0);  // starts on the right half
  EXPECT_GT(first.y, 1.5);  // and the top half
}
