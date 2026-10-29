#include <gtest/gtest.h>

#include "manipulation_manager/core/coverage_feasibility_analyzer.hpp"

namespace manipulation_manager
{

using motion_planner::CollisionRequest;
using motion_planner::CollisionResult;
using motion_planner::IkRequest;
using motion_planner::IkResult;
using motion_planner::Planner;
using motion_planner::PlanningRequest;
using motion_planner::PlanningResult;

namespace
{

/// Scripted Planner double: ik()/collision() return pre-set results by call
/// order, so tests can script exactly which dense points are feasible.
class FakePlanner : public Planner
{
public:
  std::string name() const override {return "fake";}

  PlanningResult plan(const PlanningRequest & /*request*/) override
  {
    return PlanningResult{};
  }

  IkResult ik(const IkRequest & request) override
  {
    seeds_seen.push_back(request.joint_seed);
    const size_t call = ik_call_count++;
    if (call < scripted_ik_success.size() && !scripted_ik_success[call]) {
      IkResult result;
      result.success = false;
      result.failure_reason_code = "ik_failed";
      return result;
    }
    IkResult result;
    result.success = true;
    // Each solution is distinct (keyed by call index) so tests can assert
    // seed continuity: call N's seed should equal call N-1's solution.
    result.joint_solution = {static_cast<double>(call), 0.0};
    return result;
  }

  CollisionResult collision(const CollisionRequest & /*request*/) override
  {
    const size_t call = collision_call_count++;
    CollisionResult result;
    result.success = true;
    result.collision_free = call >= scripted_collision_free.size() ||
      scripted_collision_free[call];
    return result;
  }

  std::vector<bool> scripted_ik_success;
  std::vector<bool> scripted_collision_free;
  std::vector<std::vector<double>> seeds_seen;
  size_t ik_call_count{0};
  size_t collision_call_count{0};
};

std::vector<CoverageSegmentIntent> makeSingleCartesianIntent(size_t pose_count)
{
  CoverageSegmentIntent intent;
  intent.label = "strip_0";
  intent.mode_hint = "cartesian";
  intent.poses.resize(pose_count);
  return {intent};
}

TEST(CoverageFeasibilityAnalyzerTest, AllFeasibleProducesOneSegmentCoveringAllPoints)
{
  FakePlanner planner;
  planner.scripted_ik_success = {true, true, true, true};
  planner.scripted_collision_free = {true, true, true, true};

  CoverageFeasibilityAnalyzer analyzer(planner);
  const auto report = analyzer.analyze(makeSingleCartesianIntent(4), "base_link", "arm_7dof");

  ASSERT_EQ(report.segments.size(), 1u);
  EXPECT_EQ(report.segments[0].label, "strip_0");
  EXPECT_EQ(report.segments[0].mode_hint, "cartesian");
  EXPECT_EQ(report.segments[0].waypoint_indices, (std::vector<size_t>{0, 1, 2, 3}));
  EXPECT_TRUE(report.excluded_points.empty());
  EXPECT_EQ(report.waypoint_poses.size(), 4u);
}

TEST(CoverageFeasibilityAnalyzerTest, MiddleGapSplitsIntoTwoSegmentsWithExcludedPoint)
{
  FakePlanner planner;
  // 5 points: 0,1 feasible, 2 fails IK (the gap), 3,4 feasible again.
  planner.scripted_ik_success = {true, true, false, true, true};
  planner.scripted_collision_free = {true, true, true, true};

  CoverageFeasibilityAnalyzer analyzer(planner);
  const auto report = analyzer.analyze(makeSingleCartesianIntent(5), "base_link", "arm_7dof");

  ASSERT_EQ(report.segments.size(), 2u);
  EXPECT_EQ(report.segments[0].waypoint_indices, (std::vector<size_t>{0, 1}));
  EXPECT_EQ(report.segments[1].waypoint_indices, (std::vector<size_t>{3, 4}));

  ASSERT_EQ(report.excluded_points.size(), 1u);
  EXPECT_EQ(report.excluded_points[0].global_index, 2u);
  EXPECT_EQ(report.excluded_points[0].reason, "ik_failed");
}

TEST(CoverageFeasibilityAnalyzerTest, CollisionDetectedExcludesPointWithCorrectReason)
{
  FakePlanner planner;
  // 5 points: 0,1 feasible, 2's IK succeeds but the solution collides
  // (the gap), 3,4 feasible again -- long enough on each side to form
  // real 2-point segments, isolating the collision as the only excluded point.
  planner.scripted_ik_success = {true, true, true, true, true};
  planner.scripted_collision_free = {true, true, false, true, true};

  CoverageFeasibilityAnalyzer analyzer(planner);
  const auto report = analyzer.analyze(makeSingleCartesianIntent(5), "base_link", "arm_7dof");

  ASSERT_EQ(report.excluded_points.size(), 1u);
  EXPECT_EQ(report.excluded_points[0].global_index, 2u);
  EXPECT_EQ(report.excluded_points[0].reason, "collision_detected");
  ASSERT_EQ(report.segments.size(), 2u);
  EXPECT_EQ(report.segments[0].waypoint_indices, (std::vector<size_t>{0, 1}));
  EXPECT_EQ(report.segments[1].waypoint_indices, (std::vector<size_t>{3, 4}));
}

TEST(CoverageFeasibilityAnalyzerTest, IsolatedSinglePointRunIsExcludedNotSegmented)
{
  FakePlanner planner;
  // 5 points: 0 fails, 1 feasible alone (isolated, run length 1), 2 fails, 3,4 feasible.
  planner.scripted_ik_success = {false, true, false, true, true};
  planner.scripted_collision_free = {true, true, true, true};

  CoverageFeasibilityAnalyzer analyzer(planner);
  const auto report = analyzer.analyze(makeSingleCartesianIntent(5), "base_link", "arm_7dof");

  // Point 1's isolated run (length 1, below kMinFeasibleRunLength=2) becomes
  // an excluded point, not a one-point segment.
  ASSERT_EQ(report.segments.size(), 1u);
  EXPECT_EQ(report.segments[0].waypoint_indices, (std::vector<size_t>{3, 4}));

  bool found_isolated = false;
  for (const auto & excluded : report.excluded_points) {
    if (excluded.global_index == 1u) {
      EXPECT_EQ(excluded.reason, "isolated_point");
      found_isolated = true;
    }
  }
  EXPECT_TRUE(found_isolated);
}

TEST(CoverageFeasibilityAnalyzerTest, SeedContinuityCarriesPreviousSolutionForward)
{
  FakePlanner planner;
  planner.scripted_ik_success = {true, true, true};
  planner.scripted_collision_free = {true, true, true};

  CoverageFeasibilityAnalyzer analyzer(planner);
  analyzer.analyze(makeSingleCartesianIntent(3), "base_link", "arm_7dof");

  ASSERT_EQ(planner.seeds_seen.size(), 3u);
  EXPECT_TRUE(planner.seeds_seen[0].empty());        // first point: no seed yet.
  EXPECT_EQ(planner.seeds_seen[1], (std::vector<double>{0.0, 0.0}));  // call 0's solution.
  EXPECT_EQ(planner.seeds_seen[2], (std::vector<double>{1.0, 0.0}));  // call 1's solution.
}

TEST(CoverageFeasibilityAnalyzerTest, FspHintedIntentPassesThroughWithoutPerPointChecks)
{
  FakePlanner planner;

  CoverageSegmentIntent intent;
  intent.label = "connector_0";
  intent.mode_hint = "fsp";
  intent.poses.resize(2);

  CoverageFeasibilityAnalyzer analyzer(planner);
  const auto report = analyzer.analyze({intent}, "base_link", "arm_7dof");

  ASSERT_EQ(report.segments.size(), 1u);
  EXPECT_EQ(report.segments[0].mode_hint, "fsp");
  EXPECT_EQ(report.segments[0].waypoint_indices, (std::vector<size_t>{0, 1}));
  EXPECT_TRUE(report.excluded_points.empty());
  EXPECT_EQ(planner.ik_call_count, 0u);
  EXPECT_EQ(planner.collision_call_count, 0u);
}

TEST(CoverageFeasibilityAnalyzerTest, TargetEefSpeedCarriesThroughToMotionSegmentSpec)
{
  FakePlanner planner;
  planner.scripted_ik_success = {true, true};
  planner.scripted_collision_free = {true, true};

  CoverageSegmentIntent cartesian_intent;
  cartesian_intent.label = "strip_0";
  cartesian_intent.mode_hint = "cartesian";
  cartesian_intent.poses.resize(2);
  cartesian_intent.target_eef_speed = 0.05;

  CoverageSegmentIntent fsp_intent;
  fsp_intent.label = "connector_0";
  fsp_intent.mode_hint = "fsp";
  fsp_intent.poses.resize(1);
  fsp_intent.target_eef_speed = 0.1;

  CoverageFeasibilityAnalyzer analyzer(planner);
  const auto report = analyzer.analyze({cartesian_intent, fsp_intent}, "base_link", "arm_7dof");

  ASSERT_EQ(report.segments.size(), 2u);
  EXPECT_NEAR(report.segments[0].target_eef_speed, 0.05, 1e-9);
  EXPECT_NEAR(report.segments[1].target_eef_speed, 0.1, 1e-9);
}

TEST(CoverageFeasibilityAnalyzerTest, MultipleIntentsUseGlobalContinuousIndices)
{
  FakePlanner planner;
  planner.scripted_ik_success = {true, true, true, true};
  planner.scripted_collision_free = {true, true, true, true};

  CoverageSegmentIntent strip_a;
  strip_a.label = "strip_a";
  strip_a.mode_hint = "cartesian";
  strip_a.poses.resize(2);

  CoverageSegmentIntent strip_b;
  strip_b.label = "strip_b";
  strip_b.mode_hint = "cartesian";
  strip_b.poses.resize(2);

  CoverageFeasibilityAnalyzer analyzer(planner);
  const auto report = analyzer.analyze({strip_a, strip_b}, "base_link", "arm_7dof");

  ASSERT_EQ(report.segments.size(), 2u);
  EXPECT_EQ(report.segments[0].waypoint_indices, (std::vector<size_t>{0, 1}));
  EXPECT_EQ(report.segments[1].waypoint_indices, (std::vector<size_t>{2, 3}));
  EXPECT_EQ(report.waypoint_poses.size(), 4u);
}

TEST(CoverageFeasibilityAnalyzerTest, SeedContinuityCarriesAcrossIntentBoundaries)
{
  FakePlanner planner;
  planner.scripted_ik_success = {true, true, true, true};
  planner.scripted_collision_free = {true, true, true, true};

  CoverageSegmentIntent strip_a;
  strip_a.label = "strip_a";
  strip_a.mode_hint = "cartesian";
  strip_a.poses.resize(2);

  CoverageSegmentIntent strip_b;
  strip_b.label = "strip_b";
  strip_b.mode_hint = "cartesian";
  strip_b.poses.resize(2);

  CoverageFeasibilityAnalyzer analyzer(planner);
  analyzer.analyze({strip_a, strip_b}, "base_link", "arm_7dof");

  ASSERT_EQ(planner.seeds_seen.size(), 4u);
  EXPECT_TRUE(planner.seeds_seen[0].empty());                         // strip_a point 0: no seed yet.
  EXPECT_EQ(planner.seeds_seen[1], (std::vector<double>{0.0, 0.0}));  // strip_a point 1, seeded by call 0.
  // strip_b point 0: seeded by strip_a's own last solution, carried across
  // the intent boundary -- not reset to empty just because a new intent started.
  EXPECT_EQ(planner.seeds_seen[2], (std::vector<double>{1.0, 0.0}));
  EXPECT_EQ(planner.seeds_seen[3], (std::vector<double>{2.0, 0.0}));  // strip_b point 1, seeded by call 2.
}

}  // namespace
}  // namespace manipulation_manager
