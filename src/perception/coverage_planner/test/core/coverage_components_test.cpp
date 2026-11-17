// Coverage of walls with obstacles: every free component is planned, free cells beside an obstacle's
// keep-out edge are wiped with short strokes, and the optional cell decomposition.
//
// The rasters stand in for what the wall pipeline hands the planner: 1 cm cells, obstacles already grown by
// half the swath (the keep-out), a 0.14 m swath. Coverage is judged by CoverageVerifier over the free
// (reachable) cells, the same way score_run judges a recording.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <string>

#include "coverage_planner/core/boustrophedon_planner.hpp"
#include "coverage_planner/core/coverage_verification.hpp"

using namespace coverage_planner;

namespace
{

constexpr double kRes = 0.01;
constexpr double kSwath = 0.14;

struct Raster
{
  int w, h;
  std::vector<int8_t> data;
  Raster(int w_, int h_) : w(w_), h(h_), data(static_cast<size_t>(w_) * h_, 0) {}
  void block(int x0, int x1, int y0, int y1)
  {
    for (int y = std::max(0, y0); y <= std::min(h - 1, y1); ++y) {
      for (int x = std::max(0, x0); x <= std::min(w - 1, x1); ++x) {
        data[static_cast<size_t>(y) * w + x] = 100;
      }
    }
  }
  OccupancyGrid grid() const
  {
    OccupancyGrid g;
    g.header.frame_id = "wall";
    g.info.width = static_cast<uint32_t>(w);
    g.info.height = static_cast<uint32_t>(h);
    g.info.resolution = kRes;
    g.info.origin.orientation.w = 1.0;
    g.data = data;
    return g;
  }
};

CoveragePlannerParams wallParams()
{
  CoveragePlannerParams p;
  p.swath = kSwath;
  p.step = kSwath * 0.85;
  p.sweep_direction = "vertical";
  p.start_corner = "bottom_left";
  p.obstacle_inflation_radius = 0.0;
  p.allow_unknown = false;
  p.enable_boundary_pass = false;
  p.boundary_offset = 0.0;
  p.add_strip_connectors = false;
  p.pose_mode = "identity";
  p.map_frame = "wall";
  p.use_region = false;
  p.plan_all_components = true;
  p.min_component_cells = 25;
  p.optimize_strip_order = false;
  p.requires_contact = true;
  return p;
}

// Covered fraction of the free cells when the planned strokes are worked with the swath.
double coveredFraction(const Raster & r, const CoveragePlannerResult & res)
{
  std::vector<bool> target(r.data.size());
  for (size_t i = 0; i < target.size(); ++i) {
    target[i] = r.data[i] == 0;
  }
  CoverageVerifier v(0.0, r.w * kRes - 1e-9, 0.0, r.h * kRes - 1e-9, kRes, target);
  for (const auto & seg : res.segments) {
    if (seg.label.rfind("strip_", 0) != 0) {
      continue;
    }
    ToolStroke stroke;
    stroke.swath_m = kSwath;
    for (uint32_t i = seg.start_index; i <= seg.end_index; ++i) {
      stroke.samples.push_back({res.path.poses[i].pose.position.x, res.path.poses[i].pose.position.y, true});
    }
    v.addStroke(stroke);
  }
  return v.report().covered_fraction;
}

// No stroke may pass through a blocked cell.
void expectStrokesFree(const Raster & r, const CoveragePlannerResult & res)
{
  for (const auto & seg : res.segments) {
    if (seg.label.rfind("strip_", 0) != 0) {
      continue;
    }
    const auto & a = res.path.poses[seg.start_index].pose.position;
    const auto & b = res.path.poses[seg.end_index].pose.position;
    const int n = static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / kRes)) + 1;
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / n;
      const int x = static_cast<int>(std::floor((a.x + t * (b.x - a.x)) / kRes));
      const int y = static_cast<int>(std::floor((a.y + t * (b.y - a.y)) / kRes));
      ASSERT_EQ(r.data[static_cast<size_t>(y) * r.w + x], 0) << seg.label << " crosses a blocked cell";
    }
  }
}

// A 0.9 x 0.3 m wall with a full-height rib: keep-out x 33..57 (rib 0.04 m + 0.07 m each side).
Raster twoPanels()
{
  Raster r(90, 30);
  r.block(33, 57, 0, 29);
  return r;
}

// 0.5 x 0.3 m wall with a 0.10 x 0.08 m window, keep-out 0.24 x 0.22 m (x 21..44, y 4..25).
Raster window()
{
  Raster r(50, 30);
  r.block(21, 44, 4, 25);
  return r;
}

// 0.5 x 0.3 m wall with a 0.07 x 0.07 m protrusion, keep-out 0.21 x 0.21 m.
Raster protrusion()
{
  Raster r(50, 30);
  r.block(22, 42, 5, 25);
  return r;
}

}  // namespace

TEST(CoverageComponentsTest, TwoPanelsBeyondARibAreBothPlanned)
{
  const Raster r = twoPanels();
  BoustrophedonPlanner planner;
  auto p = wallParams();
  const auto res = planner.generatePath(r.grid(), p);
  ASSERT_TRUE(res.success);
  EXPECT_EQ(res.free_component_count, 2);
  double x_max = 0.0;
  for (const auto & pose : res.path.poses) {
    x_max = std::max(x_max, pose.pose.position.x);
  }
  EXPECT_GT(x_max, 0.8) << "the panel beyond the rib has no strokes";
  expectStrokesFree(r, res);
  EXPECT_GT(coveredFraction(r, res), 0.9);
  // The columns leave a few cells at the panels' far edges; the gap fill takes them.
  auto filled = p;
  filled.fill_gaps = true;
  filled.min_stroke_length_m = 0.02;
  EXPECT_GT(coveredFraction(r, planner.generatePath(r.grid(), filled)), 0.95);

  // Only the largest component when asked to: the old behaviour, the right panel is dropped.
  p.plan_all_components = false;
  const auto one = planner.generatePath(r.grid(), p);
  ASSERT_TRUE(one.success);
  EXPECT_LT(coveredFraction(r, one), 0.6);
}

TEST(CoverageComponentsTest, PanelsAreEmittedLeftToRightAndTagged)
{
  // The right panel is the larger one; largest first would put it first.
  Raster r(90, 30);
  r.block(23, 47, 0, 29);
  BoustrophedonPlanner planner;
  auto p = wallParams();
  p.components_in_sweep_order = true;
  const auto res = planner.generatePath(r.grid(), p);
  ASSERT_TRUE(res.success);
  ASSERT_GE(res.free_component_count, 2);
  double prev_x = -1.0;
  bool crossed = false;
  std::set<std::string> component_tags;
  for (const auto & seg : res.segments) {
    const double x = res.path.poses[seg.start_index].pose.position.x;
    if (x > 0.45) {
      crossed = true;
    } else {
      EXPECT_FALSE(crossed) << "left panel stroke after the right panel started";
    }
    prev_x = x;
    for (const auto & t : seg.tags) {
      if (t.rfind("component_", 0) == 0) {
        component_tags.insert(t);
      }
    }
  }
  EXPECT_GT(prev_x, 0.45);
  EXPECT_EQ(component_tags.size(), 2u);

  // The default keeps the largest-first order.
  p.components_in_sweep_order = false;
  const auto def = planner.generatePath(r.grid(), p);
  ASSERT_TRUE(def.success);
  EXPECT_GT(def.path.poses.front().pose.position.x, 0.45);
}

TEST(CoverageComponentsTest, TinyIslandsStayDroppedByMinComponentCells)
{
  Raster r(60, 30);
  r.block(21, 59, 0, 29);        // free: a panel x 0..20 ...
  for (int y = 5; y < 8; ++y) {  // ... and a 3 x 3 pocket
    for (int x = 40; x < 43; ++x) {
      r.data[static_cast<size_t>(y) * r.w + x] = 0;
    }
  }
  BoustrophedonPlanner planner;
  auto p = wallParams();
  p.min_component_cells = 25;
  EXPECT_EQ(planner.generatePath(r.grid(), p).free_component_count, 1);
  p.min_component_cells = 1;
  EXPECT_EQ(planner.generatePath(r.grid(), p).free_component_count, 2);
}

TEST(CoverageComponentsTest, WindowLeavesGapsWithoutFillAndNoneWithIt)
{
  const Raster r = window();
  BoustrophedonPlanner planner;
  auto p = wallParams();
  const auto plain = planner.generatePath(r.grid(), p);
  ASSERT_TRUE(plain.success);
  const double before = coveredFraction(r, plain);
  EXPECT_LT(before, 0.95);

  p.fill_gaps = true;
  p.min_stroke_length_m = 0.02;
  const auto filled = planner.generatePath(r.grid(), p);
  ASSERT_TRUE(filled.success);
  expectStrokesFree(r, filled);
  EXPECT_GT(filled.num_gap_strokes, 0);
  const double after = coveredFraction(r, filled);
  EXPECT_GT(after, 0.95) << "before " << before;
  EXPECT_GT(after, before);
  // The gap strokes are strokes, not points.
  for (const auto & seg : filled.segments) {
    if (std::find(seg.tags.begin(), seg.tags.end(), "gap_fill") == seg.tags.end()) {
      continue;
    }
    const auto & a = filled.path.poses[seg.start_index].pose.position;
    const auto & b = filled.path.poses[seg.end_index].pose.position;
    EXPECT_GE(std::hypot(b.x - a.x, b.y - a.y), 0.02 - 1e-9);
    EXPECT_TRUE(seg.requires_contact);
  }
}

TEST(CoverageComponentsTest, ProtrusionIsWipedUpToTheKeepOutEdge)
{
  const Raster r = protrusion();
  BoustrophedonPlanner planner;
  auto p = wallParams();
  p.fill_gaps = true;
  p.min_stroke_length_m = 0.02;
  const auto res = planner.generatePath(r.grid(), p);
  ASSERT_TRUE(res.success);
  expectStrokesFree(r, res);
  EXPECT_GT(coveredFraction(r, res), 0.95);
}

TEST(CoverageComponentsTest, HorizontalSweepsFillToo)
{
  const Raster r = window();
  BoustrophedonPlanner planner;
  auto p = wallParams();
  p.sweep_direction = "horizontal";
  p.start_corner = "top_right";
  p.fill_gaps = true;
  p.min_stroke_length_m = 0.02;
  const auto res = planner.generatePath(r.grid(), p);
  ASSERT_TRUE(res.success);
  expectStrokesFree(r, res);
  EXPECT_GT(coveredFraction(r, res), 0.95);
}

TEST(CoverageComponentsTest, MinStrokeLengthDecidesWhichSpansAreStroked)
{
  // A free pocket two cells wide and five cells high beside a block: the regular columns miss it.
  Raster r(40, 30);
  r.block(0, 39, 0, 29);
  r.block(0, 0, 0, 0);
  for (int y = 10; y < 15; ++y) {
    for (int x = 10; x < 12; ++x) {
      r.data[static_cast<size_t>(y) * r.w + x] = 0;
    }
  }
  BoustrophedonPlanner planner;
  auto p = wallParams();
  p.min_component_cells = 1;
  p.fill_gaps = true;
  p.min_stroke_length_m = 0.03;
  auto res = planner.generatePath(r.grid(), p);
  ASSERT_TRUE(res.success);
  // The regular column through the pocket plus nothing extra: the pocket is fully covered by it.
  EXPECT_GT(coveredFraction(r, res), 0.99);

  // A one-cell-wide, one-cell-high pocket cannot hold a stroke of 3 cm and is left alone.
  Raster q(40, 30);
  q.block(0, 39, 0, 29);
  q.data[static_cast<size_t>(10) * q.w + 30] = 0;
  const auto none = planner.generatePath(q.grid(), p);
  EXPECT_FALSE(none.success && none.num_gap_strokes > 0);
}

TEST(CoverageComponentsTest, GapFillIsOffByDefault)
{
  const Raster r = window();
  BoustrophedonPlanner planner;
  const auto res = planner.generatePath(r.grid(), wallParams());
  ASSERT_TRUE(res.success);
  EXPECT_EQ(res.num_gap_strokes, 0);
  for (const auto & seg : res.segments) {
    EXPECT_EQ(std::count(seg.tags.begin(), seg.tags.end(), "gap_fill"), 0);
  }
}

TEST(CoverageComponentsTest, CellDecompositionSplitsAroundTheWindowAndCovers)
{
  const Raster r = window();
  BoustrophedonPlanner planner;
  auto p = wallParams();
  p.decomposition = "cells";
  p.fill_gaps = true;
  p.min_stroke_length_m = 0.02;
  const auto res = planner.generatePath(r.grid(), p);
  ASSERT_TRUE(res.success);
  // Left of the window, above it, below it, right of it.
  EXPECT_EQ(res.num_cells, 4);
  expectStrokesFree(r, res);
  EXPECT_GT(coveredFraction(r, res), 0.95);

  // Strokes carry the cell they belong to and a cell is worked in one block.
  std::vector<int> cell_of;
  for (const auto & seg : res.segments) {
    for (const auto & t : seg.tags) {
      if (t.rfind("cell_", 0) == 0) {
        cell_of.push_back(std::stoi(t.substr(5)));
      }
    }
  }
  ASSERT_FALSE(cell_of.empty());
  std::set<int> closed;
  for (size_t i = 0; i < cell_of.size(); ++i) {
    if (i > 0 && cell_of[i] != cell_of[i - 1]) {
      closed.insert(cell_of[i - 1]);
    }
    EXPECT_EQ(closed.count(cell_of[i]), 0u) << "cell " << cell_of[i] << " is revisited";
  }
}

TEST(CoverageComponentsTest, CellDecompositionOfAnOpenWallIsOneCell)
{
  const Raster r(50, 30);
  BoustrophedonPlanner planner;
  auto p = wallParams();
  p.decomposition = "cells";
  const auto res = planner.generatePath(r.grid(), p);
  ASSERT_TRUE(res.success);
  EXPECT_EQ(res.num_cells, 1);
  // One serpentine: the strokes alternate direction.
  double prev_dy = 0.0;
  for (const auto & seg : res.segments) {
    const double dy = res.path.poses[seg.end_index].pose.position.y -
      res.path.poses[seg.start_index].pose.position.y;
    EXPECT_NE(dy, 0.0);
    EXPECT_LT(dy * prev_dy, 1e-12);
    prev_dy = dy;
  }
}

TEST(CoverageComponentsTest, CellDecompositionPlansBothPanels)
{
  const Raster r = twoPanels();
  BoustrophedonPlanner planner;
  auto p = wallParams();
  p.decomposition = "cells";
  p.fill_gaps = true;
  p.min_stroke_length_m = 0.02;
  const auto res = planner.generatePath(r.grid(), p);
  ASSERT_TRUE(res.success);
  EXPECT_EQ(res.num_cells, 2);
  EXPECT_GT(coveredFraction(r, res), 0.95);
}
