#include <gtest/gtest.h>

#include "manipulation_manager/core/mission_verify.hpp"

using namespace manipulation_manager;

namespace
{
MissionVerifyInput clean()
{
  MissionVerifyInput in;
  in.loaded = true;
  in.tracker_success = true;
  in.done = {0, 1};
  in.peak_force_n = 20.0;
  return in;
}

GeometricCoverageResult geo(double percent)
{
  GeometricCoverageResult g;
  g.available = true;
  g.percent = percent;
  g.planned_cells = 1000;
  g.covered_cells = static_cast<size_t>(percent * 10.0);
  g.column_percent = {{0, percent}, {1, percent}};
  g.samples = 500;
  g.working_samples = 480;
  g.cell_m = 0.01;
  return g;
}
}  // namespace

TEST(MissionVerifyGeometry, NotComputedLeavesTheResultAsBefore)
{
  const auto in = clean();
  const auto r = buildMissionVerify(in);
  EXPECT_TRUE(r.passed);
  const auto j = missionVerifyToJson(r, in);
  EXPECT_TRUE(j["geometric_coverage_percent"].is_null());
  EXPECT_FALSE(j["geometric"]["available"].get<bool>());
  EXPECT_EQ(j["coverage_basis"], "columns");
  EXPECT_FALSE(j.contains("path_quality"));
}

TEST(MissionVerifyGeometry, ReportedWithCountsAndPerColumnShares)
{
  auto in = clean();
  in.geometric = geo(87.5);
  in.swath_m = 0.1;
  const auto r = buildMissionVerify(in);
  EXPECT_TRUE(r.passed);   // no minimum set: reported only
  const auto j = missionVerifyToJson(r, in);
  EXPECT_DOUBLE_EQ(j["geometric_coverage_percent"].get<double>(), 87.5);
  EXPECT_EQ(j["geometric"]["planned_cells"].get<size_t>(), 1000u);
  EXPECT_DOUBLE_EQ(j["geometric"]["column_percent"]["1"].get<double>(), 87.5);
  EXPECT_DOUBLE_EQ(j["geometric"]["swath_m"].get<double>(), 0.1);
  EXPECT_DOUBLE_EQ(j["coverage_percent"].get<double>(), 100.0);   // the column share is unchanged
}

TEST(MissionVerifyGeometry, BelowTheMinimumFailsTheRun)
{
  auto in = clean();
  in.geometric = geo(72.0);
  in.min_geometric_percent = 90.0;
  const auto r = buildMissionVerify(in);
  EXPECT_FALSE(r.passed);
  ASSERT_EQ(r.failures.size(), 1u);
  EXPECT_NE(r.failures[0].find("geometric coverage"), std::string::npos);
}

TEST(MissionVerifyGeometry, AtOrAboveTheMinimumPasses)
{
  auto in = clean();
  in.geometric = geo(90.0);
  in.min_geometric_percent = 90.0;
  EXPECT_TRUE(buildMissionVerify(in).passed);
}

TEST(MissionVerifyGeometry, MinimumWithoutAResultFailsHonestly)
{
  auto in = clean();
  in.min_geometric_percent = 80.0;
  in.geometric.note = "no planned stroke";
  const auto r = buildMissionVerify(in);
  EXPECT_FALSE(r.passed);
  EXPECT_NE(r.failures.at(0).find("not available: no planned stroke"), std::string::npos);
}

TEST(MissionVerifyGeometry, PathQualityIsAWarningNeverAFailure)
{
  auto in = clean();
  in.path_quality_requested = true;
  in.path_quality.available = true;
  in.path_quality.near_singular = 1;
  in.path_quality.columns = {
    {0, "ok", 0.4, 0.2, 3},
    {1, "near_singular", 0.005, 0.1, 17}};
  const auto r = buildMissionVerify(in);
  EXPECT_TRUE(r.passed);
  EXPECT_TRUE(r.failures.empty());
  ASSERT_EQ(r.warnings.size(), 1u);
  EXPECT_NE(r.warnings[0].find("segment 1: near_singular"), std::string::npos);
  const auto j = missionVerifyToJson(r, in);
  EXPECT_EQ(j["path_quality"]["near_singular"], 1);
  EXPECT_EQ(j["path_quality"]["columns"].size(), 2u);
  EXPECT_EQ(j["warnings"].size(), 1u);
}

TEST(MissionVerifyGeometry, UnavailablePathQualityIsOneWarning)
{
  auto in = clean();
  in.path_quality_requested = true;
  in.path_quality.note = "no joint samples";
  const auto r = buildMissionVerify(in);
  EXPECT_TRUE(r.passed);
  ASSERT_EQ(r.warnings.size(), 1u);
  EXPECT_NE(r.warnings[0].find("no joint samples"), std::string::npos);
}

TEST(MissionVerifyStands, NoStandsBlockForASingleStandRun)
{
  const auto in = clean();
  EXPECT_FALSE(missionVerifyToJson(buildMissionVerify(in), in).contains("stands"));
}

TEST(MissionVerifyStands, PerStandColumnsAndGeometry)
{
  auto in = clean();
  in.done = {0, 1};
  in.partial = {2};
  in.failed = {3};
  in.stand_columns = {{0, {0, 1}}, {1, {2, 3, 4}}};
  in.geometric = geo(80.0);
  in.geometric.group = {{0, {100, 100, 100.0}}, {1, {200, 100, 50.0}}};
  const auto j = missionVerifyToJson(buildMissionVerify(in), in);
  ASSERT_EQ(j["stands"].size(), 2u);
  const auto & s0 = j["stands"][0];
  EXPECT_EQ(s0["stand"], 0);
  EXPECT_DOUBLE_EQ(s0["column_percent"].get<double>(), 100.0);
  EXPECT_DOUBLE_EQ(s0["geometric_coverage_percent"].get<double>(), 100.0);
  const auto & s1 = j["stands"][1];
  EXPECT_EQ(s1["partial"].size(), 1u);
  EXPECT_EQ(s1["failed"].size(), 1u);
  EXPECT_EQ(s1["not_run"].size(), 1u);   // column 4 has no outcome
  EXPECT_NEAR(s1["column_percent"].get<double>(), 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(s1["geometric_coverage_percent"].get<double>(), 50.0);
  EXPECT_EQ(s1["planned_cells"], 200);
}

TEST(MissionVerifyStands, GeometryNullWhenNotComputed)
{
  auto in = clean();
  in.stand_columns = {{0, {0, 1}}};
  const auto j = missionVerifyToJson(buildMissionVerify(in), in);
  ASSERT_EQ(j["stands"].size(), 1u);
  EXPECT_TRUE(j["stands"][0]["geometric_coverage_percent"].is_null());
  EXPECT_DOUBLE_EQ(j["stands"][0]["column_percent"].get<double>(), 100.0);
}
