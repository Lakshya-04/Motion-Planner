#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <set>

#include "coverage_planner/core/coverage_from_scan.hpp"
#include "coverage_planner/core/coverage_patterns.hpp"
#include "coverage_planner/core/wall_basis.hpp"

using namespace coverage_planner;  // NOLINT

namespace
{
scene_model::PointCloud wallScan(double width = 4.0, double height = 3.0)
{
  scene_model::PointCloud cloud;
  std::mt19937 rng(1);
  std::normal_distribution<double> noise(0.0, 0.002);
  for (double x = 0.0; x <= width; x += 0.02) {
    for (double z = 0.0; z <= height; z += 0.02) {
      cloud.points.emplace_back(x, 2.0 + noise(rng), z);
    }
  }
  return cloud;
}

ToolCoverageProfile tool()
{
  ToolCoverageProfile t;
  t.name = "mop";
  t.task.contact = true;
  t.coverage.swath_m = 0.3;
  t.coverage.overlap_pct = 20.0;
  t.coverage.sweep_direction = "vertical";
  t.coverage.passes = {PassVariant{}};
  return t;
}

ScanCoverageOptions options(WallPattern pattern, const std::string & planner = "sweepline")
{
  ScanCoverageOptions o;
  o.resolution = 0.05;
  o.plane.normal_hint = scene_model::Vec3(0, -1, 0);
  o.planner = planner;
  o.wall_pattern.pattern = pattern;
  o.wall_pattern.panel_width_m = 1.3;
  o.wall_pattern.panel_overlap_m = 0.1;
  o.wall_pattern.panel_tool_turn_deg = 90.0;
  return o;
}

// Strip end points of the final plan, in execution order.
struct Row
{
  double x0, z0, x1, z1;
  std::string panel;
};
std::vector<Row> rows(const ScanCoverageResult & r)
{
  std::vector<Row> out;
  for (const auto & seg : r.final_plan.segments) {
    if (seg.label.rfind("strip", 0) != 0) {continue;}
    const auto & a = r.final_plan.path.poses[seg.start_index].pose.position;
    const auto & b = r.final_plan.path.poses[seg.end_index].pose.position;
    std::string panel;
    for (const auto & t : seg.tags) {if (t.rfind("panel_", 0) == 0) {panel = t;}}
    out.push_back({a.x, a.z, b.x, b.z, panel});
  }
  return out;
}
}  // namespace

TEST(Patterns, ParseAndName)
{
  EXPECT_EQ(parseWallPattern("vertical"), WallPattern::kVertical);
  EXPECT_EQ(parseWallPattern("horizontal"), WallPattern::kHorizontal);
  EXPECT_EQ(parseWallPattern("zigzag"), WallPattern::kZigzag);
  EXPECT_STREQ(toString(WallPattern::kZigzag), "zigzag");
  EXPECT_THROW(parseWallPattern("spiral"), std::invalid_argument);
}

TEST(Patterns, SplitIntoPanelsCoversTheWallWithOverlap)
{
  const auto panels = splitIntoPanels(0.0, 4.0, 1.3, 0.1, true);
  ASSERT_EQ(panels.size(), 4U);
  EXPECT_NEAR(panels.front().u_min, 0.0, 1e-12);
  EXPECT_NEAR(panels.back().u_max, 4.0, 1e-12);
  for (size_t k = 0; k + 1 < panels.size(); ++k) {
    EXPECT_NEAR(panels[k].u_max - panels[k + 1].u_min, 0.1, 1e-9);  // neighbours overlap
    EXPECT_LE(panels[k].u_max - panels[k].u_min, 1.3 + 1e-9);
    EXPECT_EQ(panels[k].index, static_cast<int>(k));
  }
  const auto from_right = splitIntoPanels(0.0, 4.0, 1.3, 0.1, false);
  EXPECT_NEAR(from_right.front().u_max, 4.0, 1e-12);  // first panel at the high end
  EXPECT_EQ(splitIntoPanels(0.0, 1.0, 1.3, 0.1, true).size(), 1U);
  EXPECT_TRUE(splitIntoPanels(1.0, 1.0, 1.3, 0.1, true).empty());
}

class PatternPlanners : public ::testing::TestWithParam<std::string> {};

TEST_P(PatternPlanners, VerticalIsStrokesOverTheWholeWall)
{
  const auto r = planCoverageFromScan(wallScan(), scene_model::SceneModel(), tool(),
    options(WallPattern::kVertical, GetParam()));
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_TRUE(r.pattern.panels.empty());
  for (const auto & row : rows(r)) {
    EXPECT_NEAR(row.x0, row.x1, 0.03) << "vertical strokes run along z";
    EXPECT_GT(std::abs(row.z1 - row.z0), 1.0);
  }
}

TEST_P(PatternPlanners, HorizontalSplitsIntoPanelsWithRowsAlongTheWall)
{
  const auto r = planCoverageFromScan(wallScan(), scene_model::SceneModel(), tool(),
    options(WallPattern::kHorizontal, GetParam()));
  ASSERT_TRUE(r.success) << r.message;
  ASSERT_EQ(r.pattern.panels.size(), 4U);
  const auto all = rows(r);
  ASSERT_GT(all.size(), 20U);
  std::set<std::string> panels;
  for (const auto & row : all) {
    EXPECT_NEAR(row.z0, row.z1, 0.03) << "horizontal rows run along x";
    panels.insert(row.panel);
  }
  EXPECT_EQ(panels.size(), 4U);
  // Rows stay inside their panel's reach (panel width 1.3 m, plus the swath at the ends).
  for (const auto & row : all) {
    EXPECT_LT(std::abs(row.x1 - row.x0), 1.35);
  }
}

TEST_P(PatternPlanners, ZigzagRunsEveryRowTheSameWayWithSwingBacks)
{
  const auto r = planCoverageFromScan(wallScan(), scene_model::SceneModel(), tool(),
    options(WallPattern::kZigzag, GetParam()));
  ASSERT_TRUE(r.success) << r.message;
  const auto all = rows(r);
  ASSERT_GT(all.size(), 20U);
  for (const auto & row : all) {
    EXPECT_GT(row.x1, row.x0) << "left to right on every row";
  }
  int connectors = 0;
  for (const auto & seg : r.final_plan.segments) {
    if (seg.label.rfind("connector", 0) == 0) {
      ++connectors;
      EXPECT_EQ(seg.mode, "fsp");
      EXPECT_TRUE(seg.requires_retraction);
    }
  }
  EXPECT_GT(connectors, 10);
}

INSTANTIATE_TEST_SUITE_P(Planners, PatternPlanners, ::testing::Values("sweepline", "boustrophedon"));

TEST(Patterns, SerpentineAlternatesRowDirectionWhileZigzagDoesNot)
{
  const auto serp = rows(planCoverageFromScan(wallScan(), scene_model::SceneModel(), tool(),
    options(WallPattern::kHorizontal)));
  // Within one panel consecutive rows go opposite ways.
  int opposite = 0, same = 0;
  for (size_t i = 0; i + 1 < serp.size(); ++i) {
    if (serp[i].panel != serp[i + 1].panel) {continue;}
    ((serp[i].x1 - serp[i].x0) * (serp[i + 1].x1 - serp[i + 1].x0) < 0 ? opposite : same)++;
  }
  EXPECT_GT(opposite, 10);
  EXPECT_EQ(same, 0);
}

TEST(Patterns, HorizontalTurnsTheToolByTheConfiguredAngle)
{
  auto o0 = options(WallPattern::kHorizontal);
  o0.wall_pattern.panel_tool_turn_deg = 0.0;
  auto o90 = options(WallPattern::kHorizontal);
  const auto a = planCoverageFromScan(wallScan(), scene_model::SceneModel(), tool(), o0);
  const auto b = planCoverageFromScan(wallScan(), scene_model::SceneModel(), tool(), o90);
  ASSERT_TRUE(a.success && b.success);
  auto xAxis = [](const ScanCoverageResult & r) {
      const auto & q = r.final_plan.path.poses.front().pose.orientation;
      return (Eigen::Quaterniond(q.w, q.x, q.y, q.z) * Eigen::Vector3d::UnitX()).eval();
    };
  EXPECT_NEAR(xAxis(a).dot(xAxis(b)), 0.0, 0.05);  // 90 degrees apart
}

TEST(Patterns, RowSpacingUsesTheToolAsTurnedForTheRows)
{
  // The horizontal-sweep sprayer, tilted 30 degrees about u (toward v), 0.2 m base swath. The rows
  // run with the tool turned by 90 degrees, so the tilt then lies along the rows and the real
  // coverage across them stays 0.2 m; the tool's own swath (tilt across its sweep) is 0.2309 m.
  ToolCoverageProfile spray;
  spray.name = "sprayer";
  spray.task.contact = false;
  spray.task.standoff_m = 0.3;
  spray.task.tool_tilt_deg = 30.0;
  spray.task.tool_tilt_axis = {1, 0, 0};
  spray.coverage.spray_half_angle_deg = std::atan(1.0 / 3.0) * 180.0 / M_PI;
  spray.coverage.effective_width_fraction = 1.0;
  spray.coverage.overlap_pct = 0.0;
  spray.coverage.sweep_direction = "horizontal";
  spray.coverage.passes = {PassVariant{}};
  ASSERT_NEAR(toolSwath(spray), 0.2 / std::cos(30.0 * M_PI / 180.0), 1e-9);

  const auto r = planCoverageFromScan(wallScan(), scene_model::SceneModel(), spray,
    options(WallPattern::kHorizontal));
  ASSERT_TRUE(r.success) << r.message;
  std::vector<double> zs;
  for (const auto & row : rows(r)) {
    if (row.panel == "panel_0") {zs.push_back(row.z0);}
  }
  ASSERT_GT(zs.size(), 5U);
  std::sort(zs.begin(), zs.end());
  const double mean_spacing = (zs.back() - zs.front()) / static_cast<double>(zs.size() - 1);
  EXPECT_NEAR(mean_spacing, 0.2, 0.01);
}

TEST(Patterns, PanelsStayClearOfTheWallEndsAndTheFloor)
{
  auto o = options(WallPattern::kHorizontal);
  o.wall_pattern.panel_end_margin_m = 0.3;
  o.wall_pattern.floor_clearance_m = 0.2;
  const auto r = planCoverageFromScan(wallScan(), scene_model::SceneModel(), tool(), o);
  ASSERT_TRUE(r.success) << r.message;
  for (const auto & row : rows(r)) {
    EXPECT_GT(std::min(row.x0, row.x1), 0.3 - 0.06);
    EXPECT_LT(std::max(row.x0, row.x1), 3.7 + 0.06);
    EXPECT_GT(row.z0, 0.2 - 0.06);
  }
}

TEST(Patterns, FirstPanelFollowsStartSide)
{
  auto left = options(WallPattern::kHorizontal);
  auto right = options(WallPattern::kHorizontal);
  right.wall_pattern.start_left = false;
  const auto a = planCoverageFromScan(wallScan(), scene_model::SceneModel(), tool(), left);
  const auto b = planCoverageFromScan(wallScan(), scene_model::SceneModel(), tool(), right);
  ASSERT_TRUE(a.success && b.success);
  EXPECT_LT(a.final_plan.path.poses.front().pose.position.x, 1.5);
  EXPECT_GT(b.final_plan.path.poses.front().pose.position.x, 2.5);
  EXPECT_LT(a.pattern.panels.front().stand_u, a.pattern.panels.back().stand_u);
  EXPECT_GT(b.pattern.panels.front().stand_u, b.pattern.panels.back().stand_u);
}

TEST(Patterns, VerticalWithBottomPassStartsWithRowsBelowTheBand)
{
  auto o = options(WallPattern::kVertical);
  o.wall_pattern.enable_bottom_pass = true;
  o.wall_pattern.bottom_band_height_m = 0.5;
  const auto r = planCoverageFromScan(wallScan(), scene_model::SceneModel(), tool(), o);
  ASSERT_TRUE(r.success) << r.message;
  ASSERT_GE(r.pattern.parts.size(), 2U);
  EXPECT_EQ(r.pattern.parts.front().name, "bottom_pass");
  EXPECT_EQ(r.pattern.parts.back().name, "strokes");
  // Bottom rows run along x below the band; strokes stay above it.
  const auto all = rows(r);
  bool saw_low_row = false, saw_stroke = false;
  for (const auto & row : all) {
    if (std::abs(row.z1 - row.z0) < 0.03 && row.z0 < 0.55) {saw_low_row = true;}
    if (std::abs(row.x1 - row.x0) < 0.03) {
      saw_stroke = true;
      EXPECT_GT(std::min(row.z0, row.z1), 0.45);
    }
  }
  EXPECT_TRUE(saw_low_row);
  EXPECT_TRUE(saw_stroke);
}

TEST(Patterns, ObstaclesAreAvoidedByEveryPattern)
{
  scene_model::SceneModel scene("map");
  scene_model::SceneObject cabinet;
  cabinet.id = "cabinet";
  cabinet.geometry = scene_model::Box{{0.6, 0.3, 0.6}};
  cabinet.pose.translation() = scene_model::Vec3(2.0, 1.85, 1.5);
  scene.add(cabinet);
  for (auto pattern : {WallPattern::kVertical, WallPattern::kHorizontal, WallPattern::kZigzag}) {
    const auto r = planCoverageFromScan(wallScan(), scene, tool(), options(pattern));
    ASSERT_TRUE(r.success) << toString(pattern) << ": " << r.message;
    for (const auto & seg : r.final_plan.segments) {
      if (seg.label.rfind("strip", 0) != 0) {continue;}
      const auto & a = r.final_plan.path.poses[seg.start_index].pose.position;
      const auto & b = r.final_plan.path.poses[seg.end_index].pose.position;
      for (int i = 0; i <= 50; ++i) {
        const double t = i / 50.0, x = a.x + t * (b.x - a.x), z = a.z + t * (b.z - a.z);
        EXPECT_FALSE(x > 1.7 && x < 2.3 && z > 1.2 && z < 1.8)
          << toString(pattern) << " crosses the cabinet at " << x << ", " << z;
      }
    }
  }
}

TEST(Patterns, UnknownPlannerNameFailsWithAMessage)
{
  auto o = options(WallPattern::kHorizontal, "teleport");
  const auto r = planCoverageFromScan(wallScan(), scene_model::SceneModel(), tool(), o);
  EXPECT_FALSE(r.success);
}
