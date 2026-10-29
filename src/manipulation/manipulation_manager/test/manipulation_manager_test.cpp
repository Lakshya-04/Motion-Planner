#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "motion_planner/core/motion_planner.hpp"
#include "motion_planner_moveit/moveit_planning_backend.hpp"
#include "motion_planner_moveit/ompl_planner.hpp"
#include "motion_planner/core/planners/planner.hpp"

#include "manipulation_manager/core/execution/execution_port.hpp"
#include "manipulation_manager/core/motion_segment_spec.hpp"
#include "manipulation_manager/core/manipulation_manager.hpp"

namespace manipulation_manager
{
namespace
{

using motion_planner::CollisionRequest;
using motion_planner::CollisionResult;
using motion_planner::IkRequest;
using motion_planner::IkResult;
using motion_planner::MotionPlanner;
using motion_planner::Planner;
using motion_planner::PlanningRequest;
using motion_planner::PlanningResult;

/// Counts execute() calls instead of talking to live MoveIt -- lets tests
/// verify the manager only executes what a planner actually planned, without
/// needing a live ROS/MoveIt system.
class FakeExecutionPort : public ExecutionPort
{
public:
  ExecutionResult execute(const ExecutionRequest & /*request*/) override
  {
    ++call_count;
    ExecutionResult result;
    result.success = should_succeed;
    result.failure_reason_code = should_succeed ? "none" : "fake_execution_failed";
    return result;
  }

  int call_count{0};
  bool should_succeed{true};
};

/// Scripted Planner double so analyzeFeasibleSegments() tests don't need
/// live MoveIt -- mirrors coverage_feasibility_analyzer_test.cpp's FakePlanner.
class FakeFeasibilityPlanner : public Planner
{
public:
  std::string name() const override {return "fake";}
  PlanningResult plan(const PlanningRequest & /*request*/) override {return PlanningResult{};}

  IkResult ik(const IkRequest & /*request*/) override
  {
    IkResult result;
    result.success = ik_succeeds;
    result.failure_reason_code = ik_succeeds ? "none" : "ik_failed";
    if (ik_succeeds) {
      result.joint_solution = {0.0};
    }
    return result;
  }

  CollisionResult collision(const CollisionRequest & /*request*/) override
  {
    CollisionResult result;
    result.success = true;
    result.collision_free = true;
    return result;
  }

  bool ik_succeeds{true};
};

/// Scripted Planner double whose plan() outcome is fixed at construction --
/// used to test dispatch/routing through the manager's injected MotionPlanner.
class FakePlanRoutingPlanner : public Planner
{
public:
  FakePlanRoutingPlanner(std::string name, bool plan_succeeds)
  : name_(std::move(name)), plan_succeeds_(plan_succeeds)
  {
  }

  std::string name() const override {return name_;}

  PlanningResult plan(const PlanningRequest & request) override
  {
    ++plan_call_count;
    PlanningResult result;
    result.success = plan_succeeds_;
    result.status = plan_succeeds_ ? "planned" : "failed";
    result.failure_reason_code = plan_succeeds_ ? "none" : "fake_plan_failed";
    result.path_fraction = plan_succeeds_ ? 1.0 : 0.0;
    if (plan_succeeds_) {
      // Mirrors the real OmplPlanner's own accounting: a joint-target
      // request has no waypoint_indices to count (there's no pose array it
      // indexes into) -- one unit of work, same as any single-pose segment.
      result.waypoints_completed = !request.target_joint_state.empty() ?
        1 : request.waypoint_indices.size();
    }
    return result;
  }

  IkResult ik(const IkRequest & /*request*/) override {return IkResult{};}
  CollisionResult collision(const CollisionRequest & /*request*/) override
  {
    return CollisionResult{};
  }

  int plan_call_count{0};

private:
  std::string name_;
  bool plan_succeeds_;
};

/// Captures each plan() call's request.start_joint_state via an external
/// pointer, and always succeeds with a scripted single-point trajectory
/// ending at end_positions -- used to verify seed continuity is threaded
/// (now via MotionPlanner) from one segment's planned end to the next
/// segment's start.
class FakeSeedCapturingPlanner : public Planner
{
public:
  FakeSeedCapturingPlanner(
    std::string name, std::vector<double> end_positions,
    std::vector<std::vector<double>> * captured_start_states)
  : name_(std::move(name)), end_positions_(std::move(end_positions)),
    captured_start_states_(captured_start_states)
  {
  }

  std::string name() const override {return name_;}

  PlanningResult plan(const PlanningRequest & request) override
  {
    captured_start_states_->push_back(request.start_joint_state);
    PlanningResult result;
    result.success = true;
    result.status = "planned";
    result.failure_reason_code = "none";
    result.path_fraction = 1.0;
    core_types::TrajectoryPoint point;
    point.positions = end_positions_;
    result.trajectory.points = {point};
    return result;
  }

  IkResult ik(const IkRequest & /*request*/) override {return IkResult{};}
  CollisionResult collision(const CollisionRequest & /*request*/) override
  {
    return CollisionResult{};
  }

private:
  std::string name_;
  std::vector<double> end_positions_;
  std::vector<std::vector<double>> * captured_start_states_;
};

/// Wraps two already-constructed fakes into a real MotionPlanner -- the only
/// way to inject planners into ManipulationManager since the extraction
/// (setMotionPlanner() replaces the old setCartesianPlanner()/
/// setFreeSpacePlanner() pair).
std::unique_ptr<MotionPlanner> makeMotionPlanner(
  std::unique_ptr<Planner> cartesian, std::unique_ptr<Planner> fsp)
{
  return std::make_unique<MotionPlanner>(std::move(cartesian), std::move(fsp));
}

TEST(ManipulationManagerTest, EmptySegmentsReturnEmptyStatus)
{
  ManipulationManager manager;

  const MotionExec exec = manager.executeSegmentedMotionPlan("base_link", {});
  const MotionRunSummary summary = manager.lastRunSummary();

  EXPECT_FALSE(exec.success);
  EXPECT_EQ(exec.status, "empty");
  EXPECT_EQ(summary.total_segments, 0u);
}

TEST(ManipulationManagerTest, InvalidSegmentIndicesFailAtPrecheck)
{
  ManipulationManager manager;
  manager.setMotionPlanner(
    makeMotionPlanner(
      std::make_unique<FakePlanRoutingPlanner>("cartesian", false),
      std::make_unique<FakePlanRoutingPlanner>("fsp", false)));

  const std::vector<std::vector<size_t>> segments = {{2, 1}};
  const std::vector<std::string> labels = {"sweep_strip_0"};

  const MotionExec exec = manager.executeSegmentedMotionPlan("base_link", segments, labels);
  const MotionRunSummary summary = manager.lastRunSummary();
  const auto & metrics = manager.lastSegmentMetrics();

  EXPECT_FALSE(exec.success);
  EXPECT_EQ(exec.status, "partial");
  ASSERT_EQ(metrics.size(), 1u);
  EXPECT_EQ(metrics[0].failure_reason_code, "invalid_segment_indices");
  EXPECT_EQ(metrics[0].failure_stage, "precheck");
  EXPECT_EQ(metrics[0].outcome, SegmentExecutionState::kFailedRuntime);
  EXPECT_EQ(summary.failed_segments, 1u);
  EXPECT_EQ(summary.terminal_failures, 1u);
}

TEST(ManipulationManagerTest, CartesianRoutedSegmentsAreNeverForcedToFspOnDiscontinuousIndices)
{
  // Post-extraction behavior: waypoint-index adjacency to the previous
  // segment is no longer a routing signal at all (see
  // MotionPlanner::planSegment()) -- a "sweep_strip"-labeled (cartesian-
  // routed) segment stays on cartesian regardless of index continuity, since
  // these carry real coverage value and must never be silently rerouted.
  ManipulationManager manager;
  manager.setMotionPlanner(
    makeMotionPlanner(
      std::make_unique<FakePlanRoutingPlanner>("cartesian", false),
      std::make_unique<FakePlanRoutingPlanner>("fsp", false)));

  const std::vector<std::vector<size_t>> segments = {
    {0, 1},
    {5, 6}
  };
  const std::vector<std::string> labels = {
    "sweep_strip_0",
    "sweep_strip_1"
  };

  const MotionExec exec = manager.executeSegmentedMotionPlan("base_link", segments, labels);
  const MotionRunSummary summary = manager.lastRunSummary();
  const auto & metrics = manager.lastSegmentMetrics();

  EXPECT_FALSE(exec.status.empty());
  ASSERT_EQ(metrics.size(), 2u);
  EXPECT_EQ(metrics[0].mode_used, "cartesian");
  EXPECT_EQ(metrics[1].mode_used, "cartesian")
    << "a cartesian-routed segment must stay on cartesian even with a discontinuous index gap";
  EXPECT_FALSE(metrics[1].mode_switch_used);
  EXPECT_EQ(summary.mode_switch_count, 0u);
}

TEST(ManipulationManagerTest, SegmentHintsOverrideDefaultsWhenValid)
{
  ManipulationManager manager;
  // The cartesian-first attempt always runs for a fsp-routed segment (see
  // MotionPlanner::planSegment()), so a cartesian planner must exist even
  // though this test is about the fsp/ompl fallback path -- it just needs to
  // fail so dispatch stays on fsp. free_space_planner_ is the real OmplPlanner
  // so the "missing_target_poses" precheck assertion below reflects genuine
  // OmplPlanner behavior, not a fake's scripted string.
  manager.setMotionPlanner(
    makeMotionPlanner(
      std::make_unique<FakePlanRoutingPlanner>("cartesian", false),
      std::make_unique<motion_planner::OmplPlanner>(
        std::make_shared<motion_planner::MoveItPlanningBackend>())));

  const std::vector<std::vector<size_t>> segments = {{0, 1, 2}};
  const std::vector<std::string> labels = {"sweep_strip_0"};
  const std::vector<std::string> modes = {"fsp"};
  const std::vector<std::string> planners = {"unsupported_planner"};

  const MotionExec exec = manager.executeSegmentedMotionPlan(
    "base_link",
    segments,
    labels,
    modes,
    planners);
  const auto & metrics = manager.lastSegmentMetrics();

  EXPECT_FALSE(exec.success);
  EXPECT_EQ(exec.status, "partial");
  ASSERT_EQ(metrics.size(), 1u);
  EXPECT_EQ(metrics[0].mode_used, "fsp");
  EXPECT_EQ(metrics[0].planner_used, "ompl");
  EXPECT_TRUE(metrics[0].fallback_used);
  EXPECT_EQ(metrics[0].failure_reason_code, "missing_target_poses");
}

TEST(ManipulationManagerTest, ExecutionPortNeverCalledWhenPlanningFails)
{
  ManipulationManager manager;
  auto fake_port = std::make_shared<FakeExecutionPort>();
  manager.setExecutionPort(fake_port);
  manager.setMotionPlanner(
    makeMotionPlanner(
      std::make_unique<FakePlanRoutingPlanner>("cartesian", false),
      std::make_unique<FakePlanRoutingPlanner>("fsp", false)));

  // No waypoint_poses given -> target_poses stays empty -> planning fails,
  // before the manager ever reaches the ExecutionPort. Verifies the
  // plan/execute decoupling's gating (execute() only runs on planning
  // success) without needing a live ROS/MoveIt system.
  std::vector<MotionSegmentSpec> segments(1);
  segments[0].label = "sweep_strip_0";
  segments[0].waypoint_indices = {0, 1, 2};
  segments[0].mode_hint = "cartesian";

  const MotionExec exec = manager.executeMotionSegments("base_link", segments);

  EXPECT_FALSE(exec.success);
  EXPECT_EQ(fake_port->call_count, 0);
}

TEST(ManipulationManagerTest, ExecuteMotionSegmentsEmptySegmentsReturnEmptyStatus)
{
  ManipulationManager manager;

  const MotionExec exec = manager.executeMotionSegments("base_link", {});
  const MotionRunSummary summary = manager.lastRunSummary();

  EXPECT_FALSE(exec.success);
  EXPECT_EQ(exec.status, "empty");
  EXPECT_EQ(summary.total_segments, 0u);
}

TEST(ManipulationManagerTest, ExecuteMotionSegmentsRoutesByExplicitModeHint)
{
  ManipulationManager manager;
  // Neither segment supplies waypoint_poses, so both always fail regardless
  // -- these just need to exist for the explicit-mode-hint routing under
  // test to run without a live default planner.
  manager.setMotionPlanner(
    makeMotionPlanner(
      std::make_unique<FakePlanRoutingPlanner>("cartesian", false),
      std::make_unique<FakePlanRoutingPlanner>("fsp", false)));

  std::vector<MotionSegmentSpec> segments(2);
  segments[0].label = "sweep_strip_0";
  segments[0].waypoint_indices = {0, 1};
  segments[0].mode_hint = "cartesian";
  segments[1].label = "sweep_strip_1";
  segments[1].waypoint_indices = {5, 6};
  segments[1].mode_hint = "fsp";

  const MotionExec exec = manager.executeMotionSegments("base_link", segments);
  const auto & metrics = manager.lastSegmentMetrics();

  EXPECT_FALSE(exec.status.empty());
  ASSERT_EQ(metrics.size(), 2u);
  EXPECT_EQ(metrics[0].mode_used, "cartesian");
  EXPECT_EQ(metrics[1].mode_used, "fsp");
}

TEST(ManipulationManagerTest, CartesianFirstFallbackUsesCartesianWhenItCanSolveExplicitFspSegment)
{
  ManipulationManager manager;
  manager.setMotionPlanner(
    makeMotionPlanner(
      std::make_unique<FakePlanRoutingPlanner>("cartesian", true),
      std::make_unique<FakePlanRoutingPlanner>("fsp", true)));
  manager.setExecutionPort(std::make_shared<FakeExecutionPort>());

  std::vector<MotionSegmentSpec> segments(1);
  segments[0].label = "connector_0";
  segments[0].waypoint_indices = {0, 1};
  segments[0].mode_hint = "fsp";

  manager.executeMotionSegments("base_link", segments);
  const auto & metrics = manager.lastSegmentMetrics();

  ASSERT_EQ(metrics.size(), 1u);
  EXPECT_EQ(metrics[0].mode_used, "cartesian")
    << "an explicitly-fsp-hinted segment that Cartesian can solve should report cartesian, "
    "not the original hint -- RRTConnect's path isn't the most direct one available";
}

TEST(ManipulationManagerTest, CartesianFirstFallbackFallsBackToFspWhenCartesianCannotSolve)
{
  ManipulationManager manager;
  manager.setMotionPlanner(
    makeMotionPlanner(
      std::make_unique<FakePlanRoutingPlanner>("cartesian", false),
      std::make_unique<FakePlanRoutingPlanner>("fsp", true)));
  manager.setExecutionPort(std::make_shared<FakeExecutionPort>());

  std::vector<MotionSegmentSpec> segments(1);
  segments[0].label = "connector_0";
  segments[0].waypoint_indices = {0, 1};
  segments[0].mode_hint = "fsp";

  manager.executeMotionSegments("base_link", segments);
  const auto & metrics = manager.lastSegmentMetrics();

  ASSERT_EQ(metrics.size(), 1u);
  EXPECT_EQ(metrics[0].mode_used, "fsp")
    << "when Cartesian genuinely can't solve it, the explicit fsp hint should still be honored";
}

TEST(ManipulationManagerTest, StartJointStateThreadsFromPreviousSegmentEnd)
{
  ManipulationManager manager;
  std::vector<std::vector<double>> captured_start_states;
  const std::vector<double> segment0_end_positions = {0.1, 0.2, 0.3};

  manager.setMotionPlanner(
    makeMotionPlanner(
      std::make_unique<FakeSeedCapturingPlanner>(
        "cartesian", segment0_end_positions, &captured_start_states),
      std::make_unique<FakePlanRoutingPlanner>("fsp", false)));
  manager.setExecutionPort(std::make_shared<FakeExecutionPort>());

  std::vector<MotionSegmentSpec> segments(2);
  segments[0].label = "sweep_strip_0";
  segments[0].waypoint_indices = {0, 1};
  segments[0].mode_hint = "cartesian";
  segments[1].label = "sweep_strip_1";
  segments[1].waypoint_indices = {2, 3};
  segments[1].mode_hint = "cartesian";

  manager.executeMotionSegments("base_link", segments);

  ASSERT_EQ(captured_start_states.size(), 2u);
  EXPECT_TRUE(captured_start_states[0].empty())
    << "the first segment has no previous segment to seed from";
  EXPECT_EQ(captured_start_states[1], segment0_end_positions)
    << "segment 1's start_joint_state should be seeded from segment 0's own planned trajectory end";
}

TEST(ManipulationManagerTest, SegmentProgressCallbackFiresOncePerSegment)
{
  ManipulationManager manager;
  manager.setMotionPlanner(
    makeMotionPlanner(
      std::make_unique<FakePlanRoutingPlanner>("cartesian", true),
      std::make_unique<FakePlanRoutingPlanner>("fsp", true)));
  manager.setExecutionPort(std::make_shared<FakeExecutionPort>());

  std::vector<MotionSegmentSpec> segments(2);
  segments[0].label = "segment_a";
  segments[0].waypoint_indices = {0, 1};
  segments[0].mode_hint = "cartesian";
  segments[1].label = "segment_b";
  segments[1].waypoint_indices = {2, 3};
  segments[1].mode_hint = "cartesian";

  std::vector<std::string> callback_labels;
  manager.setSegmentProgressCallback(
    [&callback_labels](const MotionSegmentExecMetrics & metrics) {
      callback_labels.push_back(metrics.segment_label);
    });

  manager.executeMotionSegments("base_link", segments);

  ASSERT_EQ(callback_labels.size(), 2u);
  EXPECT_EQ(callback_labels[0], "segment_a");
  EXPECT_EQ(callback_labels[1], "segment_b");
}

TEST(ManipulationManagerTest, CancelRequestedStopsAtNextSegmentBoundary)
{
  ManipulationManager manager;
  manager.setMotionPlanner(
    makeMotionPlanner(
      std::make_unique<FakePlanRoutingPlanner>("cartesian", true),
      std::make_unique<FakePlanRoutingPlanner>("fsp", true)));
  manager.setExecutionPort(std::make_shared<FakeExecutionPort>());

  std::vector<MotionSegmentSpec> segments(2);
  segments[0].label = "segment_a";
  segments[0].waypoint_indices = {0, 1};
  segments[0].mode_hint = "cartesian";
  segments[1].label = "segment_b";
  segments[1].waypoint_indices = {2, 3};
  segments[1].mode_hint = "cartesian";

  int cancel_check_count = 0;
  manager.setCancelRequestedCallback(
    [&cancel_check_count]() {
      ++cancel_check_count;
      return cancel_check_count > 1;  // false before segment_a, true before segment_b
    });

  manager.executeMotionSegments("base_link", segments);
  const auto & metrics = manager.lastSegmentMetrics();

  ASSERT_EQ(metrics.size(), 1u)
    << "only segment_a should have run before cancellation stopped the loop";
  EXPECT_EQ(metrics[0].segment_label, "segment_a");
  EXPECT_EQ(manager.lastRunSummary().completed_segments, 1u);
}

TEST(ManipulationManagerTest, JointTargetSegmentBypassesEmptyWaypointsPrecheck)
{
  ManipulationManager manager;
  // The cartesian-first attempt always runs for a fsp-routed segment (see
  // MotionPlanner::planSegment()) -- it just needs to exist and fail here,
  // so dispatch falls through to the injected fake FSP planner below.
  manager.setMotionPlanner(
    makeMotionPlanner(
      std::make_unique<FakePlanRoutingPlanner>("cartesian", false),
      std::make_unique<FakePlanRoutingPlanner>("fsp", true)));
  manager.setExecutionPort(std::make_shared<FakeExecutionPort>());

  std::vector<MotionSegmentSpec> segments(1);
  segments[0].label = "joint_move";
  segments[0].mode_hint = "fsp";
  segments[0].target_joint_state = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  // waypoint_indices deliberately left empty -- there's no pose array a
  // joint-target segment indexes into.

  manager.executeMotionSegments("base_link", segments);
  const auto & metrics = manager.lastSegmentMetrics();

  ASSERT_EQ(metrics.size(), 1u);
  EXPECT_NE(metrics[0].failure_reason_code, "empty_segment")
    << "a joint-target segment must not be rejected by the empty-waypoints precheck";
  EXPECT_EQ(metrics[0].attempted_waypoints, 1u);
  EXPECT_EQ(metrics[0].outcome, SegmentExecutionState::kCompleted);
}

TEST(ManipulationManagerTest, AnalyzeFeasibleSegmentsUsesInjectedCartesianPlanner)
{
  ManipulationManager manager;
  auto fake_planner = std::make_unique<FakeFeasibilityPlanner>();
  fake_planner->ik_succeeds = true;
  manager.setMotionPlanner(
    makeMotionPlanner(std::move(fake_planner), std::make_unique<FakeFeasibilityPlanner>()));

  CoverageSegmentIntent intent;
  intent.label = "strip_0";
  intent.mode_hint = "cartesian";
  intent.poses.resize(3);

  const FeasibilityReport report = manager.analyzeFeasibleSegments(
    {intent}, "base_link", "arm_7dof");

  ASSERT_EQ(report.segments.size(), 1u);
  EXPECT_EQ(report.segments[0].label, "strip_0");
  EXPECT_EQ(report.segments[0].waypoint_indices, (std::vector<size_t>{0, 1, 2}));
  EXPECT_TRUE(report.excluded_points.empty());
}

TEST(ManipulationManagerTest, AnalyzeFeasibleSegmentsReportsExcludedPointsOnIkFailure)
{
  ManipulationManager manager;
  auto fake_planner = std::make_unique<FakeFeasibilityPlanner>();
  fake_planner->ik_succeeds = false;
  manager.setMotionPlanner(
    makeMotionPlanner(std::move(fake_planner), std::make_unique<FakeFeasibilityPlanner>()));

  CoverageSegmentIntent intent;
  intent.label = "strip_0";
  intent.mode_hint = "cartesian";
  intent.poses.resize(2);

  const FeasibilityReport report = manager.analyzeFeasibleSegments(
    {intent}, "base_link", "arm_7dof");

  EXPECT_TRUE(report.segments.empty());
  EXPECT_EQ(report.excluded_points.size(), 2u);
  for (const auto & excluded : report.excluded_points) {
    EXPECT_EQ(excluded.reason, "ik_failed");
  }
}

TEST(ManipulationManagerTest, SetMotionPlannerIgnoresNullWithoutCrashing)
{
  ManipulationManager manager;
  // Passing null must not null out motion_planner_ itself (which would make
  // any later call through it crash) -- verified by the setter simply not
  // throwing/crashing here.
  EXPECT_NO_THROW(manager.setMotionPlanner(nullptr));
}

/// Same shape as FakePlanRoutingPlanner, but plan_call_count is atomic --
/// used only by the pipelining tests below, where a background thread (the
/// speculative look-ahead) and the main thread (reading the count from
/// inside a synchronous execute() call) both touch it.
class AtomicCountingPlanner : public Planner
{
public:
  explicit AtomicCountingPlanner(std::string name)
  : name_(std::move(name))
  {
  }

  std::string name() const override {return name_;}

  PlanningResult plan(const PlanningRequest & /*request*/) override
  {
    ++plan_call_count;
    PlanningResult result;
    result.success = true;
    result.status = "planned";
    result.failure_reason_code = "none";
    result.path_fraction = 1.0;
    result.waypoints_completed = 2;
    core_types::TrajectoryPoint point;
    point.positions = {0.1, 0.2, 0.3};
    result.trajectory.points = {point};
    return result;
  }

  IkResult ik(const IkRequest & /*request*/) override {return IkResult{};}
  CollisionResult collision(const CollisionRequest & /*request*/) override
  {
    return CollisionResult{};
  }

  std::atomic<int> plan_call_count{0};

private:
  std::string name_;
};

/// Sleeps on its first call (standing in for real, slow trajectory
/// execution) so a test can observe whether a background look-ahead plan
/// completed *during* that sleep -- proving genuine overlap rather than
/// sequential planning-then-execution.
class DelayedExecutionPort : public ExecutionPort
{
public:
  explicit DelayedExecutionPort(std::atomic<int> * next_segment_plan_call_count)
  : next_segment_plan_call_count_(next_segment_plan_call_count)
  {
  }

  ExecutionResult execute(const ExecutionRequest & /*request*/) override
  {
    ++call_count;
    if (call_count == 1) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      plan_call_count_seen_during_first_execute = next_segment_plan_call_count_->load();
    }
    ExecutionResult result;
    result.success = true;
    return result;
  }

  int call_count{0};
  int plan_call_count_seen_during_first_execute{-1};

private:
  std::atomic<int> * next_segment_plan_call_count_;
};

/// Like DelayedExecutionPort, but sleeps briefly before *every* execute()
/// call (not just the first) and records the plan_call_count observed each
/// time -- lets a test see the look-ahead queue's depth-over-time shape
/// across an entire multi-segment mission, not just prove one instance of
/// overlap.
class QueueDepthRecordingExecutionPort : public ExecutionPort
{
public:
  explicit QueueDepthRecordingExecutionPort(std::atomic<int> * plan_call_count)
  : plan_call_count_(plan_call_count)
  {
  }

  ExecutionResult execute(const ExecutionRequest & /*request*/) override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    plan_call_counts_seen.push_back(plan_call_count_->load());
    ExecutionResult result;
    result.success = true;
    return result;
  }

  std::vector<int> plan_call_counts_seen;

private:
  std::atomic<int> * plan_call_count_;
};

/// Fails execution on exactly its Nth call (1-indexed), succeeds on every
/// other -- used to trigger a cascading queue discard at a specific point
/// mid-mission rather than on the very first segment.
class FailAtCallExecutionPort : public ExecutionPort
{
public:
  explicit FailAtCallExecutionPort(int fail_at_call)
  : fail_at_call_(fail_at_call)
  {
  }

  ExecutionResult execute(const ExecutionRequest & /*request*/) override
  {
    ++call_count;
    ExecutionResult result;
    result.success = call_count != fail_at_call_;
    result.failure_reason_code = result.success ? "none" : "fake_execution_failed";
    return result;
  }

  int call_count{0};

private:
  int fail_at_call_;
};

/// Like FakeSeedCapturingPlanner, but each call's own trajectory ends at a
/// distinct, identifiable value (its own call index) instead of a fixed
/// one -- a fixed end-state fake can't tell a genuinely-fresh plan's start
/// state apart from a discarded chained speculative plan's stale
/// assumption, since every speculative seed would coincidentally match.
class IncrementingSeedCapturingPlanner : public Planner
{
public:
  IncrementingSeedCapturingPlanner(
    std::string name, std::vector<std::vector<double>> * captured_start_states)
  : name_(std::move(name)), captured_start_states_(captured_start_states)
  {
  }

  std::string name() const override {return name_;}

  PlanningResult plan(const PlanningRequest & request) override
  {
    ++call_count;
    captured_start_states_->push_back(request.start_joint_state);
    PlanningResult result;
    result.success = true;
    result.status = "planned";
    result.failure_reason_code = "none";
    result.path_fraction = 1.0;
    core_types::TrajectoryPoint point;
    point.positions = {static_cast<double>(call_count)};
    result.trajectory.points = {point};
    return result;
  }

  IkResult ik(const IkRequest & /*request*/) override {return IkResult{};}
  CollisionResult collision(const CollisionRequest & /*request*/) override
  {
    return CollisionResult{};
  }

  int call_count{0};

private:
  std::string name_;
  std::vector<std::vector<double>> * captured_start_states_;
};

TEST(ManipulationManagerTest, PipeliningPlansNextSegmentWhileCurrentSegmentExecutes)
{
  auto cartesian = std::make_unique<AtomicCountingPlanner>("cartesian");
  auto * cartesian_ptr = cartesian.get();

  ManipulationManager manager;
  manager.setMotionPlanner(
    makeMotionPlanner(std::move(cartesian), std::make_unique<FakePlanRoutingPlanner>("fsp", true)));
  manager.setPipeliningEnabled(true);

  auto execution_port = std::make_shared<DelayedExecutionPort>(&cartesian_ptr->plan_call_count);
  manager.setExecutionPort(execution_port);

  std::vector<MotionSegmentSpec> segments(2);
  segments[0].label = "segment_a";
  segments[0].waypoint_indices = {0, 1};
  segments[0].mode_hint = "cartesian";
  segments[1].label = "segment_b";
  segments[1].waypoint_indices = {2, 3};
  segments[1].mode_hint = "cartesian";

  manager.executeMotionSegments("base_link", segments);

  // By the time segment 1's execute() woke up from its own sleep, segment
  // 1's own plan (count=1) plus segment 2's speculative plan (count=2)
  // should both have already happened -- proving the speculative plan
  // genuinely ran on a background thread concurrently with execute(), not
  // sequentially after it.
  EXPECT_GE(execution_port->plan_call_count_seen_during_first_execute, 2)
    << "segment 2's speculative plan should complete on a background thread "
    "while segment 1's execute() is still sleeping";
}

TEST(ManipulationManagerTest, PipeliningDisabledByDefaultDoesNotPlanAhead)
{
  auto cartesian = std::make_unique<AtomicCountingPlanner>("cartesian");
  auto * cartesian_ptr = cartesian.get();

  ManipulationManager manager;  // pipelining left at its default (off)
  manager.setMotionPlanner(
    makeMotionPlanner(std::move(cartesian), std::make_unique<FakePlanRoutingPlanner>("fsp", true)));

  auto execution_port = std::make_shared<DelayedExecutionPort>(&cartesian_ptr->plan_call_count);
  manager.setExecutionPort(execution_port);

  std::vector<MotionSegmentSpec> segments(2);
  segments[0].label = "segment_a";
  segments[0].waypoint_indices = {0, 1};
  segments[0].mode_hint = "cartesian";
  segments[1].label = "segment_b";
  segments[1].waypoint_indices = {2, 3};
  segments[1].mode_hint = "cartesian";

  manager.executeMotionSegments("base_link", segments);

  EXPECT_EQ(execution_port->plan_call_count_seen_during_first_execute, 1)
    << "without pipelining, segment 2 must not be planned until after segment 1's execute() returns";
}

TEST(ManipulationManagerTest, PipeliningDiscardsSpeculativePlanOnExecutionFailure)
{
  std::vector<std::vector<double>> captured_start_states;
  const std::vector<double> segment0_end_positions = {0.1, 0.2, 0.3};

  ManipulationManager manager;
  manager.setMotionPlanner(
    makeMotionPlanner(
      std::make_unique<FakeSeedCapturingPlanner>(
        "cartesian", segment0_end_positions, &captured_start_states),
      std::make_unique<FakePlanRoutingPlanner>("fsp", false)));
  manager.setPipeliningEnabled(true);

  auto execution_port = std::make_shared<FakeExecutionPort>();
  execution_port->should_succeed = false;  // segment 1's execution always fails
  manager.setExecutionPort(execution_port);

  std::vector<MotionSegmentSpec> segments(2);
  segments[0].label = "segment_a";
  segments[0].waypoint_indices = {0, 1};
  segments[0].mode_hint = "cartesian";
  segments[1].label = "segment_b";
  segments[1].waypoint_indices = {2, 3};
  segments[1].mode_hint = "cartesian";

  manager.executeMotionSegments("base_link", segments);

  // Segment 1's own real plan (empty seed) + segment 2's speculative plan
  // (seeded from segment 1's own planned-but-never-confirmed end state,
  // still captured as a side effect even though its result is discarded) +
  // segment 2's own fresh, real plan once its turn actually comes -- and
  // that last one must be seeded from empty (unadvanced) continuity, not
  // reused from the discarded speculative attempt's stale seed.
  ASSERT_GE(captured_start_states.size(), 3u);
  EXPECT_TRUE(captured_start_states.back().empty())
    << "segment 2 should be planned fresh from empty (unadvanced) continuity -- segment 1's "
    "execution never succeeded, so continuity never advanced, and the discarded speculative "
    "plan (seeded from segment 1's own end state) must not be reused";
}

TEST(ManipulationManagerTest, PipeliningQueueFillsToDepthAndRefillsByOneOnConsume)
{
  auto cartesian = std::make_unique<AtomicCountingPlanner>("cartesian");
  auto * cartesian_ptr = cartesian.get();

  ManipulationManager manager;
  manager.setMotionPlanner(
    makeMotionPlanner(std::move(cartesian), std::make_unique<FakePlanRoutingPlanner>("fsp", true)));
  manager.setPipeliningEnabled(true);
  manager.setPipeliningDepth(3);

  auto execution_port =
    std::make_shared<QueueDepthRecordingExecutionPort>(&cartesian_ptr->plan_call_count);
  manager.setExecutionPort(execution_port);

  std::vector<MotionSegmentSpec> segments(5);
  for (size_t i = 0; i < segments.size(); ++i) {
    segments[i].label = "segment_" + std::to_string(i);
    segments[i].waypoint_indices = {2 * i, 2 * i + 1};
    segments[i].mode_hint = "cartesian";
  }

  manager.executeMotionSegments("base_link", segments);

  ASSERT_EQ(execution_port->plan_call_counts_seen.size(), 5u);
  // Segment 0's own real plan plus segments 1, 2, 3 all queued ahead (depth
  // 3) must all have completed on background threads during segment 0's own
  // (sleeping) execute() call -- proving the queue fills past a single slot.
  EXPECT_EQ(execution_port->plan_call_counts_seen[0], 4)
    << "queue should fill to depth 3 (segments 1,2,3) alongside segment 0's own real plan";
  // Consuming segment 1 from the queue immediately refills by one (segment
  // 4) -- so by the time segment 1's own execute() call happens, all 5
  // segments must already be planned.
  EXPECT_EQ(execution_port->plan_call_counts_seen[1], 5)
    << "consuming segment 1 should refill the queue by exactly one (segment 4)";
  // No more segments left to queue -- the count stays flat for the rest of the mission.
  EXPECT_EQ(execution_port->plan_call_counts_seen[2], 5);
  EXPECT_EQ(execution_port->plan_call_counts_seen[3], 5);
  EXPECT_EQ(execution_port->plan_call_counts_seen[4], 5);
}

TEST(ManipulationManagerTest, PipeliningCascadingDiscardDrainsEntireQueueOnExecutionFailure)
{
  std::vector<std::vector<double>> captured_start_states;

  ManipulationManager manager;
  manager.setMotionPlanner(
    makeMotionPlanner(
      std::make_unique<IncrementingSeedCapturingPlanner>("cartesian", &captured_start_states),
      std::make_unique<FakePlanRoutingPlanner>("fsp", true)));
  manager.setPipeliningEnabled(true);
  manager.setPipeliningDepth(3);

  // Segment 1 (the 2nd execute() call)'s own real execution fails --
  // everything queued ahead of it at that point (segments 2, 3, 4's
  // speculative plans) must be discarded as a whole, not just segment 2's.
  auto execution_port = std::make_shared<FailAtCallExecutionPort>(2);
  manager.setExecutionPort(execution_port);

  std::vector<MotionSegmentSpec> segments(5);
  for (size_t i = 0; i < segments.size(); ++i) {
    segments[i].label = "segment_" + std::to_string(i);
    segments[i].waypoint_indices = {2 * i, 2 * i + 1};
    segments[i].mode_hint = "cartesian";
  }

  manager.executeMotionSegments("base_link", segments);

  // Calls, in order: 1) segment 0's own real plan (empty seed); 2) segment
  // 1's speculative plan (seeded from call 1's end); 3) segment 2's
  // speculative plan (seeded from call 2's end); 4) segment 3's speculative
  // plan (seeded from call 3's end) -- queue now at depth 3; 5) segment 4's
  // speculative plan, queued after consuming segment 1 and refilling by
  // one. Segment 1's real *execution* then fails, discarding segments
  // 2/3/4 wholesale; segment 2's real, fresh plan (call 6) must resume from
  // continuity as it stood after segment 0's own success (call 1's end
  // state), never from the discarded chain's own (higher-numbered, never
  // confirmed) assumptions.
  ASSERT_GE(captured_start_states.size(), 6u);
  EXPECT_TRUE(captured_start_states[0].empty()) << "segment 0 starts from empty (fresh mission)";
  EXPECT_EQ(captured_start_states[5], std::vector<double>{1.0})
    << "segment 2's real fresh plan must resume from segment 0's own confirmed end state, "
    "not from any discarded speculative chain entry";
}

TEST(ManipulationManagerTest, SegmentPlannedCallbackFiresWithTrajectoryAndProvenance)
{
  ManipulationManager manager;
  manager.setMotionPlanner(
    makeMotionPlanner(
      std::make_unique<AtomicCountingPlanner>("cartesian"),
      std::make_unique<FakePlanRoutingPlanner>("fsp", true)));

  auto execution_port = std::make_shared<FakeExecutionPort>();
  manager.setExecutionPort(execution_port);

  struct PlannedCall
  {
    size_t segment_id;
    std::string planner_name;
    size_t trajectory_points;
    std::vector<motion_planner::TrajectorySegmentProvenance> trajectory_segments;
  };
  std::vector<PlannedCall> planned_calls;
  manager.setSegmentPlannedCallback(
    [&planned_calls](
      size_t segment_id, const std::string & planner_name,
      const core_types::Trajectory & trajectory,
      const std::vector<motion_planner::TrajectorySegmentProvenance> &
      trajectory_segments) {
      planned_calls.push_back(
        {segment_id, planner_name, trajectory.points.size(), trajectory_segments});
    });

  std::vector<MotionSegmentSpec> segments(2);
  segments[0].label = "segment_a";
  segments[0].waypoint_indices = {0, 1};
  segments[0].mode_hint = "cartesian";
  segments[1].label = "segment_b";
  segments[1].waypoint_indices = {2, 3};
  segments[1].mode_hint = "cartesian";

  manager.executeMotionSegments("base_link", segments);

  // AtomicCountingPlanner's own plan() always succeeds with a single-point
  // trajectory -- one callback firing per segment, each carrying that
  // segment's own trajectory (not the other segment's) and MotionPlanner's
  // own default single-chunk provenance (the whole trajectory attributed to
  // whichever planner produced it, since no bridging occurred here).
  ASSERT_EQ(planned_calls.size(), 2u);
  EXPECT_EQ(planned_calls[0].segment_id, 0u);
  EXPECT_EQ(planned_calls[0].planner_name, "cartesian");
  EXPECT_EQ(planned_calls[0].trajectory_points, 1u);
  ASSERT_EQ(planned_calls[0].trajectory_segments.size(), 1u);
  EXPECT_EQ(planned_calls[0].trajectory_segments[0].planner_name, "cartesian");
  EXPECT_EQ(planned_calls[0].trajectory_segments[0].start_index, 0u);
  EXPECT_EQ(planned_calls[0].trajectory_segments[0].end_index, 1u);

  EXPECT_EQ(planned_calls[1].segment_id, 1u);
  EXPECT_EQ(planned_calls[1].trajectory_points, 1u);

  const auto & metrics = manager.lastSegmentMetrics();
  ASSERT_EQ(metrics.size(), 2u);
  EXPECT_EQ(metrics[0].bridge_count, 0)
    << "AtomicCountingPlanner's plan() never bridges -- bridge_count must stay 0, not garbage";
}

}  // namespace
}  // namespace manipulation_manager
