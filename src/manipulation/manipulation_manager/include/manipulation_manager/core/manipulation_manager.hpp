/**
 * @file manipulation_manager.hpp
 * @brief Orchestrator: sequencing, retry-on-execution-failure, and reporting
 *        for a segmented motion mission.
 * @details The interface exposing manipulation to the rest of the system --
 *          mission-level ordering, cancellation, per-segment metrics/
 *          progress, and driving the ExecutionPort. Per-segment mode/planner
 *          routing (which of Cartesian/FSP actually gets used, and whether a
 *          routing hint should be overridden by what a real attempt
 *          achieves) is a distinct responsibility owned by an injected
 *          motion_planner::MotionPlanner, not this class -- see
 *          motion_planner.hpp. Pure core -- no default concrete
 *          MotionPlanner/ExecutionPort construction lives here (that would
 *          drag in MoveIt/rclcpp); the caller (a ROS-adapter node or action
 *          server, in manipulation_manager's ros/ half) must inject
 *          setMotionPlanner()/setExecutionPort() before calling
 *          execute*()/analyzeFeasibleSegments(). ROS-optional except for the
 *          injected core_types::Logger, which an adapter (e.g.
 *          RclcppLogger) supplies.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core_types/logger.hpp"
#include "core_types/null_logger.hpp"
#include "core_types/pose.hpp"
#include "motion_planner/core/motion_planner.hpp"

#include "manipulation_manager/core/coverage_feasibility_analyzer.hpp"
#include "manipulation_manager/core/coverage_source.hpp"
#include "manipulation_manager/core/execution/execution_port.hpp"
#include "manipulation_manager/core/motion_exec.hpp"
#include "manipulation_manager/core/motion_exec_metrics.hpp"
#include "manipulation_manager/core/motion_exec_reporter.hpp"
#include "manipulation_manager/core/motion_segment_spec.hpp"
#include "manipulation_manager/core/segment_ordering_policy.hpp"

namespace manipulation_manager
{

/// Whole-body planning group of mobile_manipulator_description's SRDF (the profile's
/// `groups.whole_body`); the default when a caller names no group.
inline constexpr char kDefaultPlanningGroup[] = "lift_arm";

/**
 * @class ManipulationManager
 * @brief Sequences segments, drives execution/retry, and reports outcomes --
 *        delegates per-segment mode/planner routing to an injected
 *        MotionPlanner.
 */
class ManipulationManager
{
public:
  ManipulationManager();

  /// Pass-throughs to the injected MotionPlanner -- no-op (with a warning)
  /// if called before setMotionPlanner().
  void setMotionModePolicy(const std::string & policy);
  void setPlanner(const std::string & planner_name);
  void setPlannerFallback(const std::string & fallback_name);
  void setSegmentOrderingPolicy(std::unique_ptr<SegmentOrderingPolicy> policy);

  /**
   * @brief Enable look-ahead pipelined planning (default off).
   * @details When enabled, executeMotionSegments() plans segment N+1 on a
   *          background thread while segment N is still executing (via
   *          MotionPlanner::planSegmentWithStartState(), seeded from segment
   *          N's own already-computed expected end state), so N+1's plan is
   *          ready (or partway there) by the time N's execution finishes. If
   *          N's execution fails, the speculative N+1 plan is discarded and
   *          N+1 is planned fresh next -- this does not change today's
   *          existing behavior of continuing to the next segment regardless
   *          of a failed one. Residual
   *          risk of speculative planning: (a speculative plan reflects the planning scene as it
   *          existed before N's execution completes).
   */
  void setPipeliningEnabled(bool enabled);

  /**
   * @brief Set how many segments ahead look-ahead pipelining plans
   *        speculatively (default 1, matching the original single-slot
   *        behavior exactly). Only has an effect while pipelining is
   *        enabled via setPipeliningEnabled(true).
   * @details Plans are chained: the speculative plan for order_pos+2 is
   *          seeded from order_pos+1's own (speculative) expected end state,
   *          not from the currently-executing segment's -- each entry after
   *          the first therefore waits (on a background thread, never the
   *          caller's) for the previous entry's plan to resolve before it
   *          can itself start planning. A real execution failure discards
   *          the *entire* queue, not just its front, since every queued plan
   *          is transitively seeded from an assumption that failure just
   *          invalidated.
   * @param[in] depth Number of segments to keep speculatively planned ahead
   *                   of the one currently executing.
   */
  void setPipeliningDepth(size_t depth);

  /**
   * @brief Inject a logging backend (defaults to a no-op NullLogger).
   * @param[in] logger Logger implementation, e.g. an RclcppLogger from the
   *                   calling node -- see ros_adapters/rclcpp_logger.hpp.
   */
  void setLogger(std::shared_ptr<core_types::Logger> logger);

  /**
   * @brief Inject an execution backend. Required before calling any
   *        execute*() method that reaches a successful plan -- there is no
   *        default (a default MoveItExecutionAdapter would pull MoveIt/rclcpp
   *        into this pure core). The ROS-adapter side (manipulation_manager_
   *        node.cpp, manipulation_manager_action_server.cpp,
   *        benchmark/motion_validator.cpp) injects a real
   *        MoveItExecutionAdapter immediately after construction; tests
   *        inject a fake.
   */
  void setExecutionPort(std::shared_ptr<ExecutionPort> execution_port);

  /**
   * @brief Inject the MotionPlanner (owns the Cartesian/FSP planner
   *        backends -- see motion_planner.hpp). Required before calling
   *        executeMotionSegments()/analyzeFeasibleSegments() -- there is no
   *        default (a default-constructed one would need concrete
   *        CartesianPlanner/OmplPlanner instances, pulling MoveIt/rclcpp
   *        into this pure core). The ROS-adapter side injects a real one
   *        (wrapping real CartesianPlanner/OmplPlanner) immediately after
   *        construction; tests inject one wrapping fakes.
   */
  void setMotionPlanner(std::unique_ptr<motion_planner::MotionPlanner> motion_planner);

  /// Invoked once per segment, right after its outcome is recorded -- lets a
  /// caller (e.g. a ROS action server) surface real incremental progress
  /// without polling lastSegmentMetrics() while executeMotionSegments() is
  /// still running on the same call stack. No-op (default) if unset.
  using SegmentProgressCallback = std::function<void (const MotionSegmentExecMetrics &)>;
  void setSegmentProgressCallback(SegmentProgressCallback callback);

  /// Optional: where the arm is right now. When set, every executeMotionSegments() call seeds its first segment's
  /// start state from it (needed by planners that take an explicit start, such as cuRobo). Unset (default): the
  /// first segment plans from the planner's own current state, as before.
  using StartStateProvider = std::function<std::vector<double>()>;
  void setStartStateProvider(StartStateProvider provider);

  /**
   * @brief Invoked once per segment right after a successful plan (before
   *        its execute() call), with the raw trajectory and its per-chunk
   *        planner provenance -- e.g. for marker-based visualization that
   *        needs to color a bridged Cartesian pass's alternating
   *        cartesian/fsp legs, which the lightweight, JSON-serialized
   *        MotionSegmentExecMetrics deliberately never carries (see
   *        SegmentProgressCallback above). Additive: unrelated to and does
   *        not replace SegmentProgressCallback, which still fires once per
   *        segment's final recorded outcome. No-op (default) if unset.
   * @param[in] segment_id Index into the executeMotionSegments() call's own
   *                       `segments` argument.
   * @param[in] planner_name mode_used for this plan, e.g. "cartesian"/"fsp".
   * @param[in] trajectory The full stitched trajectory this segment planned.
   * @param[in] trajectory_segments Per-chunk planner attribution -- see
   *                                motion_planner::TrajectorySegmentProvenance.
   */
  using SegmentPlannedCallback = std::function<void (
        size_t segment_id, const std::string & planner_name,
        const core_types::Trajectory & trajectory,
        const std::vector<motion_planner::TrajectorySegmentProvenance> & trajectory_segments)>;
  void setSegmentPlannedCallback(SegmentPlannedCallback callback);

  /// Polled once at the top of each segment's iteration; if it returns true,
  /// the remaining segments are skipped and the run ends with whatever
  /// completed so far. Cancellation is honored at segment boundaries only --
  /// a segment already being planned/executed always runs to completion. No
  /// cancellation (default) if unset.
  using CancelRequestedCallback = std::function<bool ()>;
  void setCancelRequestedCallback(CancelRequestedCallback callback);

  /**
   * @brief Proactively classify a coverage plan's poses as plannable or not,
   *        before any plan()/execute() attempt -- see
   *        coverage_feasibility_analyzer.hpp. Uses this manager's own
   *        cartesian_planner_ (one MoveGroupInterface, no second
   *        planning-scene connection).
   * @param[in] intents The coverage plan, in mission order.
   * @param[in] frame_id Reference frame the intents' poses are expressed in.
   * @param[in] planning_group MoveIt planning group to check feasibility for.
   */
  /// Optional column reachability check (core/column_reach_port.hpp) handed to the analyzer by
  /// analyzeFeasibleSegments(). Non-owning, must outlive the manager's use of it; nullptr (default) = off.
  void setColumnReachability(ColumnReachabilityPort * port);

  FeasibilityReport analyzeFeasibleSegments(
    const std::vector<CoverageSegmentIntent> & intents,
    const std::string & frame_id,
    const std::string & planning_group = kDefaultPlanningGroup);

  std::string selectedMode() const;
  std::string selectedPlanner() const;

  MotionExec executeMotionPlan(
    const std::string & frame_id,
    const std::vector<size_t> & waypoint_indices,
    const std::vector<core_types::Pose> & waypoint_poses = {});

  /**
   * @brief Plan and execute a sequence of motion segments, each with its own
   *        routing hints and planning parameters. Primary API -- the
   *        deployment shape (see motion_segment_spec.hpp). Named distinctly
   *        from executeSegmentedMotionPlan() below (rather than overloaded)
   *        because an empty-braces call like `executeX(frame_id, {})` is
   *        ambiguous between `vector<MotionSegmentSpec>` and
   *        `vector<vector<size_t>>` -- verified by an actual compiler error
   *        against this repo's own existing test call sites.
   */
  MotionExec executeMotionSegments(
    const std::string & frame_id,
    const std::vector<MotionSegmentSpec> & segments,
    const std::vector<core_types::Pose> & waypoint_poses = {},
    const std::string & planning_group = kDefaultPlanningGroup);

  /**
   * @brief Convenience overload for callers without per-segment planning
   *        parameters -- builds a MotionSegmentSpec per segment from the
   *        parallel arrays (params default) and delegates to
   *        executeMotionSegments() above. Unchanged signature -- existing
   *        callers (the BT server, manipulation_manager_node, tests) keep
   *        working as-is.
   */
  MotionExec executeSegmentedMotionPlan(
    const std::string & frame_id,
    const std::vector<std::vector<size_t>> & segment_waypoint_indices,
    const std::vector<std::string> & segment_labels = {},
    const std::vector<std::string> & segment_modes = {},
    const std::vector<std::string> & segment_planners = {},
    const std::vector<core_types::Pose> & waypoint_poses = {},
    const std::string & planning_group = kDefaultPlanningGroup);

  MotionRunSummary lastRunSummary() const;
  const std::vector<MotionSegmentExecMetrics> & lastSegmentMetrics() const;

private:
  /// The routing-resolved inputs one planSegment[WithStartState]() call
  /// needs -- built once by buildSegmentPlanningInputs() and reused both for
  /// the current segment's own real plan and (when pipelining is enabled)
  /// for the next segment's speculative one.
  struct SegmentPlanningInputs
  {
    motion_planner::PlanningRequest request;
    motion_planner::SegmentRoutingSpec routing;
    size_t segment_id{0};
  };

  /// Resolves order[order_pos] into a ready-to-plan request/routing pair, or
  /// std::nullopt if order_pos is out of range or the segment fails the same
  /// preconditions executeMotionSegments()'s own loop already checks
  /// (invalid segment index, empty non-joint-target segment, unsorted
  /// waypoint indices) -- nullopt is a silent "don't speculatively plan
  /// this one," not a reported failure; the real iteration for that segment
  /// (when its turn comes) still runs its own precheck and records its own
  /// outcome unchanged.
  std::optional<SegmentPlanningInputs> buildSegmentPlanningInputs(
    const std::vector<size_t> & order,
    size_t order_pos,
    const std::string & frame_id,
    const std::vector<MotionSegmentSpec> & segments,
    const std::vector<core_types::Pose> & waypoint_poses,
    const std::string & planning_group) const;

  std::unique_ptr<motion_planner::MotionPlanner> motion_planner_;  ///< No default -- inject before use.
  ColumnReachabilityPort * column_reach_{nullptr};  ///< Non-owning; null = no column reach check.
  std::unique_ptr<SegmentOrderingPolicy> segment_ordering_policy_;
  std::shared_ptr<core_types::Logger> logger_{std::make_shared<core_types::NullLogger>()};
  std::shared_ptr<ExecutionPort> execution_port_;  ///< No default -- inject before use.
  SegmentProgressCallback segment_progress_callback_;
  StartStateProvider start_state_provider_;
  SegmentPlannedCallback segment_planned_callback_;
  CancelRequestedCallback cancel_requested_callback_;
  bool pipelining_enabled_{false};
  size_t pipelining_depth_{1};

  MotionExecReporter reporter_;
  MotionRunSummary last_summary_;
};

}  // namespace manipulation_manager
