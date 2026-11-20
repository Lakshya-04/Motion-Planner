// Shipped tool profiles, the loader path, parameter overrides, and the standoff-dependent swath.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

#include "coverage_planner/core/coverage_from_scan.hpp"
#include "coverage_planner/core/tool_profile_loading.hpp"
#include "ros_adapters/tool_profile_yaml_loader.hpp"

using namespace coverage_planner;  // NOLINT

namespace
{
ToolCoverageProfile shipped(const std::string & name, const ToolOverrides & o = {})
{
  ToolCoverageProfile t =
    toCoverageProfile(ros_adapters::loadToolProfile(TOOL_PROFILES_PATH, name));
  applyToolOverrides(t, o);
  return t;
}

scene_model::PointCloud wallScan()
{
  scene_model::PointCloud cloud;
  std::mt19937 rng(1);
  std::normal_distribution<double> noise(0.0, 0.002);
  for (double x = 0.0; x <= 4.0; x += 0.01) {
    for (double z = 0.0; z <= 3.0; z += 0.01) {
      cloud.points.emplace_back(x, 2.0 + noise(rng), z);
    }
  }
  return cloud;
}

// Mean distance between adjacent vertical strokes (x) of the default vertical pattern.
double meanRowSpacing(const ScanCoverageResult & r)
{
  std::vector<double> zs;
  for (const auto & p : r.final_plan.path.poses) {zs.push_back(p.pose.position.x);}
  std::sort(zs.begin(), zs.end());
  std::vector<double> rows;
  double sum = zs.front();
  int n = 1;
  for (size_t i = 1; i < zs.size(); ++i) {
    if (zs[i] - zs[i - 1] > 0.02) {
      rows.push_back(sum / n);
      sum = 0.0;
      n = 0;
    }
    sum += zs[i];
    ++n;
  }
  rows.push_back(sum / n);
  if (rows.size() < 3) {return 0.0;}
  return (rows.back() - rows.front()) / static_cast<double>(rows.size() - 1);
}
}  // namespace

TEST(ShippedToolProfiles, ContactWiperIsAContactTool)
{
  const auto t = shipped("contact_wiper");
  EXPECT_TRUE(t.task.contact);
  EXPECT_DOUBLE_EQ(t.task.standoff_m, 0.0);
  EXPECT_DOUBLE_EQ(t.coverage.swath_m, 0.20);
  EXPECT_DOUBLE_EQ(t.coverage.overlap_pct, 20.0);
  EXPECT_EQ(t.coverage.passes.size(), 2U);
  EXPECT_DOUBLE_EQ(t.coverage.spray_half_angle_deg, 0.0);
  EXPECT_NEAR(toolSwath(t), 0.20, 1e-12);
  EXPECT_NEAR(toolLineSpacing(t), 0.16, 1e-12);
}

TEST(ShippedToolProfiles, SprayGunIsNonContactWithDerivedSwath)
{
  const auto full = ros_adapters::loadToolProfile(TOOL_PROFILES_PATH, "spray_gun");
  EXPECT_FALSE(full.task.contact);
  EXPECT_EQ(full.task.tool_command, "valve");
  EXPECT_GT(full.task.standoff_m, 0.0);
  const auto t = shipped("spray_gun");
  const double expected = 0.8 * 2.0 * 0.30 * std::tan(20.0 * M_PI / 180.0);
  EXPECT_NEAR(toolSwath(t), expected, 1e-9);
  EXPECT_NEAR(toolLineSpacing(t), expected * 0.8, 1e-9);
  // The informational swath_m in the file agrees with the derived value.
  EXPECT_NEAR(t.coverage.swath_m, expected, 1e-3);
}

TEST(ShippedToolProfiles, UnknownToolAndMissingFileThrow)
{
  EXPECT_THROW(ros_adapters::loadToolProfile(TOOL_PROFILES_PATH, "nope"), std::runtime_error);
  EXPECT_ANY_THROW(ros_adapters::loadToolProfile("/no/such/tool_profiles.yaml", "spray_gun"));
}

TEST(ToolOverridesTest, ExplicitValuesWinOverTheProfile)
{
  ToolOverrides o;
  o.swath_m = 0.3;
  o.overlap_pct = 0.0;
  o.sweep_direction = "vertical";
  const auto t = shipped("contact_wiper", o);
  EXPECT_DOUBLE_EQ(t.coverage.swath_m, 0.3);
  EXPECT_DOUBLE_EQ(t.coverage.overlap_pct, 0.0);
  EXPECT_EQ(t.coverage.sweep_direction, "vertical");
  EXPECT_NEAR(toolLineSpacing(t), 0.3, 1e-12);
  EXPECT_EQ(t.coverage.passes.size(), 2U);  // untouched fields keep the profile value
}

TEST(ToolOverridesTest, ExplicitSwathReplacesTheSprayCone)
{
  ToolOverrides o;
  o.swath_m = 0.5;
  const auto t = shipped("spray_gun", o);
  EXPECT_DOUBLE_EQ(toolSwath(t), 0.5);
}

TEST(ToolOverridesTest, StandoffOverrideMovesTheSprayGunSwath)
{
  ToolOverrides o;
  o.standoff_m = 0.6;
  const auto far = shipped("spray_gun", o);
  EXPECT_NEAR(toolSwath(far), 2.0 * toolSwath(shipped("spray_gun")), 1e-9);
}

TEST(ToolOverridesTest, DoublePassForcesTwoPassesAndInflationFallsBackToHalfSwath)
{
  ToolOverrides o;
  o.double_pass = true;
  o.obstacle_inflation_radius_m = 0.0;
  auto t = shipped("spray_gun", o);
  EXPECT_EQ(t.coverage.passes.size(), 2U);
  EXPECT_TRUE(t.coverage.passes[1].reverse_direction);
  EXPECT_NEAR(t.coverage.obstacle_inflation_radius_m, toolSwath(t) / 2.0, 1e-12);
}

TEST(ToolOverridesTest, EdgeSpacingFromTheProfileAndFromOverrides)
{
  const auto base = shipped("contact_wiper");
  EXPECT_DOUBLE_EQ(base.coverage.edge_zone_m, 0.0);          // shipped profiles stay uniform
  EXPECT_DOUBLE_EQ(base.coverage.edge_spacing_scale, 1.0);
  ToolOverrides o;
  o.edge_zone_m = 0.15;
  o.edge_spacing_scale = 0.6;
  const auto t = shipped("contact_wiper", o);
  EXPECT_DOUBLE_EQ(t.coverage.edge_zone_m, 0.15);
  EXPECT_DOUBLE_EQ(t.coverage.edge_spacing_scale, 0.6);
}

TEST(ToolSwathTest, RejectsNonPositiveStandoffForASprayTool)
{
  auto t = shipped("spray_gun");
  t.task.standoff_m = 0.0;
  EXPECT_THROW(toolSwath(t), std::invalid_argument);
  t.task.standoff_m = -0.1;
  EXPECT_THROW(toolSwath(t), std::invalid_argument);
}

TEST(ToolSwathTest, SpacingForZeroAndThirtyPercentOverlap)
{
  auto t = shipped("spray_gun");
  const double w = toolSwath(t);
  t.coverage.overlap_pct = 0.0;
  EXPECT_NEAR(toolLineSpacing(t), w, 1e-12);
  t.coverage.overlap_pct = 30.0;
  EXPECT_NEAR(toolLineSpacing(t), 0.7 * w, 1e-12);
}

TEST(ToolSwathTest, SwathGrowsMonotonicallyWithStandoff)
{
  auto t = shipped("spray_gun");
  double previous = 0.0;
  for (double d : {0.1, 0.2, 0.3, 0.4, 0.5}) {
    t.task.standoff_m = d;
    EXPECT_GT(toolSwath(t), previous);
    previous = toolSwath(t);
  }
}

TEST(SprayCoverage, RowSpacingFollowsTheStandoff)
{
  ScanCoverageOptions options;
  options.resolution = 0.01;
  options.plane.normal_hint = scene_model::Vec3(0, -1, 0);
  const auto scan = wallScan();

  double previous_spacing = 0.0;
  for (double standoff : {0.2, 0.4}) {
    ToolOverrides o;
    o.standoff_m = standoff;
    const auto tool = shipped("spray_gun", o);
    const auto result = planCoverageFromScan(scan, scene_model::SceneModel(), tool, options);
    ASSERT_TRUE(result.success) << "standoff " << standoff << ": " << result.message;
    const double measured = meanRowSpacing(result);
    EXPECT_NEAR(measured, toolLineSpacing(tool), 0.015) << "standoff " << standoff;
    EXPECT_GT(measured, previous_spacing);
    previous_spacing = measured;
  }
}
