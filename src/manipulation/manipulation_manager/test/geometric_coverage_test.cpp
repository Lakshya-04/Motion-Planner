#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "manipulation_manager/core/geometric_coverage.hpp"

using namespace manipulation_manager;

namespace
{
// Wall at x = 0.98 (normal +x): wall coordinates are y (along) and z (up).
GeometricCoverageConfig cfg()
{
  GeometricCoverageConfig c;
  c.swath_m = 0.10;
  c.cell_m = 0.01;
  c.contact_force_n = 1.0;
  c.max_gap_m = 0.05;
  c.max_standoff_m = 0.02;
  c.plane_origin = {0.98, 0.0, 0.0};
  c.wall_normal = {1.0, 0.0, 0.0};
  return c;
}

PlannedStroke column(size_t id, double y, double z0 = 0.5, double z1 = 1.5)
{
  PlannedStroke s;
  s.column = id;
  s.points = {{0.98, y, z0}, {0.98, y, z1}};
  return s;
}

/// Samples every 1 cm along a column, from z0 to z1 (index order), offset in y and x, with a force.
std::vector<ToolSample> trace(
  size_t id, double y, double z0, double z1, double force, double dy = 0.0, double dx = 0.0)
{
  std::vector<ToolSample> v;
  for (double z = z0; z <= z1 + 1e-9; z += 0.01) {
    ToolSample s;
    s.column = id;
    s.p = {0.98 + dx, y + dy, z};
    s.force_n = force;
    v.push_back(s);
  }
  return v;
}
}  // namespace

TEST(GeometricCoverage, FullExecutionCoversThePatch)
{
  const auto r = computeGeometricCoverage({column(0, 0.2)}, trace(0, 0.2, 0.5, 1.5, 15.0), cfg());
  ASSERT_TRUE(r.available);
  EXPECT_GT(r.percent, 99.0);
  EXPECT_EQ(r.off_plane_samples, 0u);
  EXPECT_EQ(r.working_samples, r.samples);
  EXPECT_NEAR(r.column_percent.at(0), r.percent, 1e-9);
}

TEST(GeometricCoverage, ColumnStoppedHalfwayCoversAboutHalf)
{
  const auto r = computeGeometricCoverage({column(0, 0.2)}, trace(0, 0.2, 0.5, 1.0, 15.0), cfg());
  ASSERT_TRUE(r.available);
  // the end cap of the swath (5 cm) past the last sample adds a little over the straight half
  EXPECT_GT(r.percent, 48.0);
  EXPECT_LT(r.percent, 60.0);
}

TEST(GeometricCoverage, LiftOffLeavesAHole)
{
  auto s = trace(0, 0.2, 0.5, 1.5, 15.0);
  for (auto & x : s) {
    if (x.p.z > 0.8 && x.p.z < 1.0) {x.force_n = 0.2;}   // not in contact between 0.8 and 1.0 m
  }
  const auto r = computeGeometricCoverage({column(0, 0.2)}, s, cfg());
  ASSERT_TRUE(r.available);
  // 1.0 m long patch; the gap of 0.2 m minus the two 5 cm caps leaves about 0.1 m uncovered
  EXPECT_GT(r.percent, 84.0);
  EXPECT_LT(r.percent, 96.0);
}

TEST(GeometricCoverage, SamplesAwayFromThePlaneDoNotCount)
{
  const auto r = computeGeometricCoverage({column(0, 0.2)}, trace(0, 0.2, 0.5, 1.5, 15.0, 0.0, 0.05), cfg());
  ASSERT_TRUE(r.available);
  EXPECT_DOUBLE_EQ(r.percent, 0.0);
  EXPECT_EQ(r.off_plane_samples, r.samples);
  EXPECT_EQ(r.working_samples, 0u);
}

TEST(GeometricCoverage, StandoffCheckCanBeSwitchedOff)
{
  auto c = cfg();
  c.max_standoff_m = 0.0;
  const auto r = computeGeometricCoverage({column(0, 0.2)}, trace(0, 0.2, 0.5, 1.5, 15.0, 0.0, 0.05), c);
  ASSERT_TRUE(r.available);
  EXPECT_GT(r.percent, 99.0);
}

TEST(GeometricCoverage, SidewaysDriftBeyondHalfASwathCoversLittle)
{
  const auto small = computeGeometricCoverage({column(0, 0.2)}, trace(0, 0.2, 0.5, 1.5, 15.0, 0.02), cfg());
  const auto big = computeGeometricCoverage({column(0, 0.2)}, trace(0, 0.2, 0.5, 1.5, 15.0, 0.08), cfg());
  ASSERT_TRUE(small.available);
  ASSERT_TRUE(big.available);
  EXPECT_GT(small.percent, 55.0);   // a 2 cm drift still covers most of the band
  // an 8 cm drift: the two 10 cm bands overlap by 2 cm, so about a fifth of the patch
  EXPECT_GT(big.percent, 15.0);
  EXPECT_LT(big.percent, 25.0);
  EXPECT_GT(small.percent, big.percent);
}

TEST(GeometricCoverage, PerColumnShareAndOverall)
{
  const auto r = computeGeometricCoverage(
    {column(0, 0.2), column(1, 0.6)}, trace(1, 0.6, 0.5, 1.5, 15.0), cfg());
  ASSERT_TRUE(r.available);
  EXPECT_NEAR(r.percent, 50.0, 2.0);
  EXPECT_LT(r.column_percent.at(0), 1.0);
  EXPECT_GT(r.column_percent.at(1), 99.0);
}

TEST(GeometricCoverage, NegativeForceThresholdCountsEverySample)
{
  auto c = cfg();
  c.contact_force_n = -1.0;
  auto s = trace(0, 0.2, 0.5, 1.5, -1.0);   // no force topic: force -1 on every sample
  const auto r = computeGeometricCoverage({column(0, 0.2)}, s, c);
  ASSERT_TRUE(r.available);
  EXPECT_GT(r.percent, 99.0);
}

TEST(GeometricCoverage, SamplesFromAnotherColumnStillCoverTheirOwnPlace)
{
  // coverage is by place: column 1's trace over column 0's band covers column 0's cells
  const auto r = computeGeometricCoverage({column(0, 0.2)}, trace(1, 0.2, 0.5, 1.5, 15.0), cfg());
  ASSERT_TRUE(r.available);
  EXPECT_GT(r.percent, 99.0);
}

TEST(GeometricCoverage, UnusableInputsAreReportedNotCrashed)
{
  EXPECT_FALSE(computeGeometricCoverage({}, trace(0, 0.2, 0.5, 1.5, 15.0), cfg()).available);   // no plan
  auto c = cfg();
  c.wall_normal = {0.0, 0.0, 0.0};
  const auto z = computeGeometricCoverage({column(0, 0.2)}, {}, c);
  EXPECT_FALSE(z.available);
  EXPECT_FALSE(z.note.empty());
  c = cfg();
  c.swath_m = 0.0;
  EXPECT_FALSE(computeGeometricCoverage({column(0, 0.2)}, {}, c).available);
  auto nan_plan = column(0, 0.2);
  nan_plan.points[0].y = std::numeric_limits<double>::quiet_NaN();
  nan_plan.points[1].y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(computeGeometricCoverage({nan_plan}, {}, cfg()).available);
}

TEST(GeometricCoverage, NoSamplesIsZeroCoverageOfAKnownPatch)
{
  const auto r = computeGeometricCoverage({column(0, 0.2)}, {}, cfg());
  ASSERT_TRUE(r.available);
  EXPECT_DOUBLE_EQ(r.percent, 0.0);
  EXPECT_GT(r.planned_cells, 0u);
}

TEST(GeometricCoverage, NanSampleBreaksTheLineWithoutPoisoningTheResult)
{
  auto s = trace(0, 0.2, 0.5, 1.5, 15.0);
  s[50].p.y = std::numeric_limits<double>::quiet_NaN();
  const auto r = computeGeometricCoverage({column(0, 0.2)}, s, cfg());
  ASSERT_TRUE(r.available);
  EXPECT_GT(r.percent, 95.0);   // the next samples are 2 cm away, within max_gap after re-start: a hole of one sample
}

TEST(GeometricCoverage, LargePatchCoarsensTheGrid)
{
  auto c = cfg();
  c.max_cells = 2000;
  const auto r = computeGeometricCoverage({column(0, 0.2, 0.0, 3.0)}, trace(0, 0.2, 0.0, 3.0, 15.0), c);
  ASSERT_TRUE(r.available);
  EXPECT_GT(r.cell_m, c.cell_m);
  EXPECT_FALSE(r.note.empty());
  EXPECT_GT(r.percent, 95.0);
}

TEST(GeometricCoverage, CeilingPlaneWorksToo)
{
  GeometricCoverageConfig c = cfg();
  c.plane_origin = {0.0, 0.0, 2.5};
  c.wall_normal = {0.0, 0.0, -1.0};
  PlannedStroke p;
  p.column = 0;
  p.points = {{0.5, 0.0, 2.5}, {1.5, 0.0, 2.5}};
  std::vector<ToolSample> s;
  for (double x = 0.5; x <= 1.5 + 1e-9; x += 0.01) {s.push_back({0, {x, 0.0, 2.5}, 10.0});}
  const auto r = computeGeometricCoverage({p}, s, c);
  ASSERT_TRUE(r.available);
  EXPECT_GT(r.percent, 99.0);
}
