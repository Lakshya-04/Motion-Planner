#include <gtest/gtest.h>

#include <cmath>

#include "coverage_planner/core/coverage_pass_expander.hpp"
#include "coverage_planner/core/plan_redundancy.hpp"
#include "coverage_planner/core/wall_basis.hpp"

using namespace coverage_planner;  // NOLINT

namespace
{

constexpr double kPitch = 0.2;
constexpr double kHeight = 2.0;

WallPlane makeWall()
{
  WallPlane wall;
  wall.pose.pose = encodeWallBasis(
    Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, -1, 0));
  wall.u_min = 0.0; wall.u_max = 4.0; wall.v_min = 0.0; wall.v_max = kHeight;
  return wall;
}

PoseStamped flat(double u, double v)
{
  PoseStamped p;
  p.pose.position.x = u;
  p.pose.position.z = v;   // world frame: the wall plane is y = 0, u = x, v = z
  p.pose.orientation.w = 1.0;
  return p;
}

/// N vertical strips in the wall plane, flat path as the planner makes it: the first strip goes up, the next one
/// starts at its end (serpentine), with a pitch-long connector between them.
void makeFlat(int n, Path & path, std::vector<MotionPlanSegment> & segs)
{
  for (int i = 0; i < n; ++i) {
    const bool up = i % 2 == 0;
    MotionPlanSegment s;
    s.label = "strip_" + std::to_string(i);
    s.mode = "cartesian";
    s.start_index = static_cast<uint32_t>(path.poses.size());
    const double u = 0.1 + i * kPitch;
    path.poses.push_back(flat(u, up ? 0.0 : kHeight));
    path.poses.push_back(flat(u, up ? kHeight : 0.0));
    s.end_index = static_cast<uint32_t>(path.poses.size()) - 1;
    segs.push_back(s);
    if (i + 1 < n) {
      MotionPlanSegment c;
      c.label = "connector_" + std::to_string(i);
      c.mode = "fsp";
      c.start_index = static_cast<uint32_t>(path.poses.size());
      path.poses.push_back(flat(u, up ? kHeight : 0.0));
      path.poses.push_back(flat(u + kPitch, up ? kHeight : 0.0));
      c.end_index = static_cast<uint32_t>(path.poses.size()) - 1;
      segs.push_back(c);
    }
  }
}

ExpandedCoverage expand(int n, bool double_pass, bool connectors = true)
{
  Path path;
  std::vector<MotionPlanSegment> segs;
  makeFlat(n, path, segs);
  if (!connectors) {
    // The planner's default: strips only, no connector waypoints.
    Path p2;
    std::vector<MotionPlanSegment> s2;
    for (const auto & s : segs) {
      if (s.mode != "cartesian") {continue;}
      MotionPlanSegment t = s;
      t.start_index = static_cast<uint32_t>(p2.poses.size());
      p2.poses.push_back(path.poses[s.start_index]);
      p2.poses.push_back(path.poses[s.end_index]);
      t.end_index = t.start_index + 1;
      s2.push_back(t);
    }
    path = p2;
    segs = s2;
  }
  ToolCoverageProfile prof;
  prof.task.contact = true;
  prof.task.standoff_m = 0.0;
  prof.task.tool_normal_policy = "constant";
  prof.coverage.passes = {PassVariant{}};
  if (double_pass) {prof.coverage.passes.push_back(PassVariant{0.0, true});}
  return expandCoveragePasses(path, segs, makeWall(), prof);
}

}  // namespace

class SerpentineN : public ::testing::TestWithParam<int> {};

TEST_P(SerpentineN, SinglePassHasNoJumpsAlternatesAndEndsLowForEvenN)
{
  const int n = GetParam();
  const auto e = expand(n, false);
  const auto r = analyzePlanRedundancy(e.path, e.segments);
  EXPECT_EQ(r.strips, static_cast<size_t>(n));
  EXPECT_EQ(r.jumps, 0u);
  EXPECT_NEAR(r.pitch_m, kPitch, 1e-9);
  EXPECT_NEAR(r.strip_length_m, n * kHeight, 1e-9);
  EXPECT_NEAR(r.connector_length_m, (n - 1) * kPitch, 1e-9);
  EXPECT_TRUE(r.serpentine_expected);
  EXPECT_TRUE(r.alternation_ok);
  EXPECT_EQ(r.cells_swept_more_than_expected, 0u);
  EXPECT_DOUBLE_EQ(r.double_sweep_fraction, 0.0);
  // The first strip goes up: an even count ends at the bottom, an odd count at the top.
  EXPECT_EQ(r.ends_at_bottom, n % 2 == 0);
  EXPECT_EQ(r.ends_at_top, n % 2 == 1);
}

INSTANTIATE_TEST_SUITE_P(Counts, SerpentineN, ::testing::Values(2, 4, 6, 10, 3, 5, 9));

TEST(PlanRedundancy, DoublePassHasNoJumpAbovePitchAndNoCrossStripSweep)
{
  for (int n : {2, 5, 10}) {
   for (bool connectors : {false, true}) {
    const auto e = expand(n, true, connectors);
    const auto r = analyzePlanRedundancy(e.path, e.segments);
    EXPECT_EQ(r.strips, static_cast<size_t>(n));
    EXPECT_EQ(r.strip_passes, static_cast<size_t>(2 * n));
    EXPECT_EQ(r.jumps, 0u) << n;
    EXPECT_LE(r.max_jump_m, kPitch + 1e-9);
    EXPECT_FALSE(r.serpentine_expected);
    EXPECT_TRUE(r.alternation_ok);
    EXPECT_EQ(r.cells_swept_more_than_expected, 0u);
    EXPECT_EQ(r.max_passes_per_cell, 2);
   }
  }
}

TEST(PlanRedundancy, ReportsTheOldFullLengthJumpsAndDoubleSweeps)
{
  // The plan #196 fixed: double-pass strips alternating like a single pass, so every strip joint is a
  // full-length jump.
  Path path;
  std::vector<MotionPlanSegment> segs;
  for (int i = 0; i < 4; ++i) {
    const bool up = i % 2 == 0;
    for (int pass = 0; pass < 2; ++pass) {
      MotionPlanSegment s;
      s.mode = "cartesian";
      s.start_index = static_cast<uint32_t>(path.poses.size());
      const bool dir_up = (pass == 0) ? up : !up;
      path.poses.push_back(flat(0.1 + i * kPitch, dir_up ? 0.0 : kHeight));
      path.poses.push_back(flat(0.1 + i * kPitch, dir_up ? kHeight : 0.0));
      s.end_index = static_cast<uint32_t>(path.poses.size()) - 1;
      if (pass == 1) {s.tags.push_back("second_pass");}
      segs.push_back(s);
    }
  }
  const auto r = analyzePlanRedundancy(path, segs, RedundancyOptions{kPitch});
  EXPECT_EQ(r.jumps, 3u);
  EXPECT_GT(r.max_jump_m, 1.9);
  EXPECT_FALSE(r.alternation_ok);
  EXPECT_GT(r.direction_violations, 0u);

  // The same strip walked twice as separate strips is a double sweep.
  Path p2;
  std::vector<MotionPlanSegment> s2;
  makeFlat(3, p2, s2);
  const auto base = p2.poses.size();
  for (int k = 0; k < 2; ++k) {
    MotionPlanSegment s;
    s.mode = "cartesian";
    s.start_index = static_cast<uint32_t>(p2.poses.size());
    p2.poses.push_back(flat(0.1, 0.0));
    p2.poses.push_back(flat(0.1, kHeight));
    s.end_index = static_cast<uint32_t>(p2.poses.size()) - 1;
    s2.push_back(s);
  }
  (void)base;
  const auto r2 = analyzePlanRedundancy(p2, s2, RedundancyOptions{kPitch});
  EXPECT_GT(r2.double_sweep_fraction, 0.0);
}

TEST(PlanRedundancy, JsonRoundTripOfCoverageOutput)
{
  const auto e = expand(4, false);
  nlohmann::json poses = nlohmann::json::array();
  for (const auto & p : e.path.poses) {
    poses.push_back({{"position", {{"x", p.pose.position.x}, {"y", p.pose.position.y}, {"z", p.pose.position.z}}}});
  }
  nlohmann::json segs = nlohmann::json::array();
  for (const auto & s : e.segments) {
    segs.push_back({{"label", s.label}, {"mode", s.mode}, {"start_index", s.start_index}, {"end_index", s.end_index},
      {"tags", s.tags}});
  }
  const nlohmann::json doc = {{"final_lifted_and_expanded", {{"path", {{"poses", poses}}}, {"segments", segs}}}};
  const auto back = expandedCoverageFromJson(doc);
  const auto r = analyzePlanRedundancy(back.path, back.segments);
  EXPECT_EQ(r.jumps, 0u);
  EXPECT_TRUE(r.ends_at_bottom);
  const auto j = redundancyReportToJson(r);
  EXPECT_EQ(j.at("jumps_over_2x_pitch"), 0);
  EXPECT_FALSE(redundancyReportToTable(r).empty());
}

// ---- stand / strip order -------------------------------------------------------------------------------------------

namespace
{
StandOrderEntry stand(double bx, const std::vector<int> & order, int n)
{
  // n columns at x = bx + 0.2 * i, serpentine direction by column index; `order` is the executed order.
  StandOrderEntry s;
  s.base = {bx, -1.0, 0.0};
  for (int i : order) {
    const bool up = i % 2 == 0;
    const double x = bx + 0.2 * i;
    s.strips.push_back(OrderStrip{{x, 0, up ? 0.0 : 2.0}, {x, 0, up ? 2.0 : 0.0}});
  }
  (void)n;
  return s;
}
}  // namespace

TEST(TravelOrder, SerpentineOrderIsOptimalForOneToThreeStands)
{
  for (int stands = 1; stands <= 3; ++stands) {
    std::vector<StandOrderEntry> v;
    for (int k = 0; k < stands; ++k) {v.push_back(stand(2.0 * k, {0, 1, 2, 3, 4, 5}, 6));}
    const auto r = checkTravelOrder(v);
    EXPECT_NEAR(r.excess_m, 0.0, 1e-9) << stands;
    EXPECT_NEAR(r.strip_travel_executed_m, stands * 5 * 0.2, 1e-9);
    EXPECT_TRUE(r.exact);
  }
}

TEST(TravelOrder, ShuffledStripsAndStandsShowExcess)
{
  std::vector<StandOrderEntry> v = {stand(0.0, {0, 2, 1, 3}, 4), stand(6.0, {0, 1, 2, 3}, 4), stand(2.0, {0, 1}, 2)};
  const auto r = checkTravelOrder(v);
  EXPECT_GT(r.strip_travel_executed_m, r.strip_travel_optimal_m);
  EXPECT_GT(r.stand_travel_executed_m, r.stand_travel_optimal_m);  // 6 + 4 vs 2 + 4
  EXPECT_GT(r.excess_m, 0.0);
  EXPECT_NEAR(r.stand_travel_optimal_m, 6.0, 1e-9);
}

TEST(TravelOrder, NearestNeighbourBeyondEightStripsIsMarkedInexactAndSerpentineStaysZero)
{
  std::vector<int> order;
  for (int i = 0; i < 12; ++i) {order.push_back(i);}
  const auto r = checkTravelOrder({stand(0.0, order, 12)});
  EXPECT_FALSE(r.exact);
  EXPECT_NEAR(r.excess_m, 0.0, 1e-9);
}
