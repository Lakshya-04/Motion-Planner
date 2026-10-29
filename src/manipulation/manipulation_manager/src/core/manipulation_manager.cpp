#include "manipulation_manager/core/manipulation_manager.hpp"

#include <algorithm>
#include <deque>
#include <sstream>
#include <utility>

namespace manipulation_manager
{

using motion_planner::FailureStage;
using motion_planner::PlanningRequest;
using motion_planner::PlanningResult;
using motion_planner::SegmentRoutingSpec;

namespace
{

constexpr size_t kMaxImmediateRetriesPerSegment = 1;

bool isRetryableFailureCode(const std::string & failure_reason_code)
{
  return failure_reason_code == "none" ||
         failure_reason_code == "timeout" ||
         failure_reason_code == "controller" ||
         failure_reason_code == "constraints" ||
         failure_reason_code == "partial_completion";
}

SegmentExecutionState toSkippedOutcome(const std::string & failure_reason_code)
{
  if (failure_reason_code == "no_ik") {
    return SegmentExecutionState::kSkippedNoIk;
  }
  if (failure_reason_code == "collision") {
    return SegmentExecutionState::kSkippedCollision;
  }
  if (failure_reason_code == "unreachable") {
    return SegmentExecutionState::kSkippedUnreachable;
  }
  return SegmentExecutionState::kFailedRuntime;
}

std::string toFailureStageString(const FailureStage stage)
{
  switch (stage) {
    case FailureStage::kPrecheck:
      return "precheck";
    case FailureStage::kPlanning:
      return "planning";
    case FailureStage::kExecution:
      return "execution";
    case FailureStage::kNone:
    default:
      return "none";
  }
}

}  // namespace

ManipulationManager::ManipulationManager()
{
  segment_ordering_policy_ = std::make_unique<InputOrderSegmentOrderingPolicy>();
}

void ManipulationManager::setMotionModePolicy(const std::string & policy)
{
  if (!motion_planner_) {
    logger_->warn("setMotionModePolicy() called before setMotionPlanner(). Ignoring.");
    return;
  }
  motion_planner_->setMotionModePolicy(policy);
}

void ManipulationManager::setPlanner(const std::string & planner_name)
{
  if (!motion_planner_) {
    logger_->warn("setPlanner() called before setMotionPlanner(). Ignoring.");
    return;
  }
  motion_planner_->setPlannerName(planner_name);
}

void ManipulationManager::setPlannerFallback(const std::string & fallback_name)
{
  if (!motion_planner_) {
    logger_->warn("setPlannerFallback() called before setMotionPlanner(). Ignoring.");
    return;
  }
  motion_planner_->setPlannerFallback(fallback_name);
}

void ManipulationManager::setSegmentOrderingPolicy(std::unique_ptr<SegmentOrderingPolicy> policy)
{
  if (!policy) {
    logger_->warn("Received null segment ordering policy. Keeping existing policy.");
    return;
  }
  segment_ordering_policy_ = std::move(policy);
}

void ManipulationManager::setPipeliningEnabled(bool enabled)
{
  pipelining_enabled_ = enabled;
}

void ManipulationManager::setPipeliningDepth(size_t depth)
{
  pipelining_depth_ = depth;
}

void ManipulationManager::setLogger(std::shared_ptr<core_types::Logger> logger)
{
  if (!logger) {
    return;
  }
  logger_ = std::move(logger);
  if (motion_planner_) {
    motion_planner_->setLogger(logger_);
  }
}

void ManipulationManager::setExecutionPort(std::shared_ptr<ExecutionPort> execution_port)
{
  if (!execution_port) {
    logger_->warn("Received null execution port. Keeping existing port.");
    return;
  }
  execution_port_ = std::move(execution_port);
}

void ManipulationManager::setMotionPlanner(
  std::unique_ptr<motion_planner::MotionPlanner> motion_planner)
{
  if (!motion_planner) {
    logger_->warn("Received null motion planner. Keeping existing motion planner.");
    return;
  }
  motion_planner_ = std::move(motion_planner);
  motion_planner_->setLogger(logger_);
}

void ManipulationManager::setSegmentProgressCallback(SegmentProgressCallback callback)
{
  segment_progress_callback_ = std::move(callback);
}

void ManipulationManager::setSegmentPlannedCallback(SegmentPlannedCallback callback)
{
  segment_planned_callback_ = std::move(callback);
}

void ManipulationManager::setCancelRequestedCallback(CancelRequestedCallback callback)
{
  cancel_requested_callback_ = std::move(callback);
}

FeasibilityReport ManipulationManager::analyzeFeasibleSegments(
  const std::vector<CoverageSegmentIntent> & intents,
  const std::string & frame_id,
  const std::string & planning_group)
{
  CoverageFeasibilityAnalyzer analyzer(motion_planner_->cartesianPlanner());
  return analyzer.analyze(intents, frame_id, planning_group);
}

std::string ManipulationManager::selectedMode() const
{
  return motion_planner_ ? motion_planner_->selectedMode() : "cartesian";
}

std::string ManipulationManager::selectedPlanner() const
{
  return motion_planner_ ? motion_planner_->selectedPlanner() : "ompl";
}

MotionExec ManipulationManager::executeMotionPlan(
  const std::string & frame_id,
  const std::vector<size_t> & waypoint_indices,
  const std::vector<core_types::Pose> & waypoint_poses)
{
  return executeSegmentedMotionPlan(
    frame_id, {waypoint_indices}, {"segment_0"}, {}, {},
    waypoint_poses);
}

MotionExec ManipulationManager::executeSegmentedMotionPlan(
  const std::string & frame_id,
  const std::vector<std::vector<size_t>> & segment_waypoint_indices,
  const std::vector<std::string> & segment_labels,
  const std::vector<std::string> & segment_modes,
  const std::vector<std::string> & segment_planners,
  const std::vector<core_types::Pose> & waypoint_poses,
  const std::string & planning_group)
{
  std::vector<MotionSegmentSpec> segments;
  segments.reserve(segment_waypoint_indices.size());
  for (size_t segment_id = 0; segment_id < segment_waypoint_indices.size(); ++segment_id) {
    MotionSegmentSpec spec;
    spec.waypoint_indices = segment_waypoint_indices[segment_id];
    spec.label = (segment_id < segment_labels.size()) ? segment_labels[segment_id] : "";
    spec.mode_hint = (segment_id < segment_modes.size()) ? segment_modes[segment_id] : "";
    spec.planner_hint =
      (segment_id < segment_planners.size()) ? segment_planners[segment_id] : "";
    segments.push_back(std::move(spec));
  }
  return executeMotionSegments(frame_id, segments, waypoint_poses, planning_group);
}

std::optional<ManipulationManager::SegmentPlanningInputs>
ManipulationManager::buildSegmentPlanningInputs(
  const std::vector<size_t> & order,
  size_t order_pos,
  const std::string & frame_id,
  const std::vector<MotionSegmentSpec> & segments,
  const std::vector<core_types::Pose> & waypoint_poses,
  const std::string & planning_group) const
{
  if (order_pos >= order.size()) {
    return std::nullopt;
  }
  const size_t segment_id = order[order_pos];
  if (segment_id >= segments.size()) {
    return std::nullopt;
  }

  const MotionSegmentSpec & segment_spec = segments[segment_id];
  const auto & waypoints = segment_spec.waypoint_indices;
  const bool is_joint_target_segment = !segment_spec.target_joint_state.empty();
  if (waypoints.empty() && !is_joint_target_segment) {
    return std::nullopt;
  }
  if (!std::is_sorted(waypoints.begin(), waypoints.end())) {
    return std::nullopt;
  }

  PlanningRequest request;
  request.frame_id = frame_id;
  request.planning_group = planning_group;
  request.waypoint_indices = waypoints;
  request.target_poses.reserve(waypoints.size());
  for (const size_t waypoint_index : waypoints) {
    if (waypoint_index < waypoint_poses.size()) {
      request.target_poses.push_back(waypoint_poses[waypoint_index]);
    }
  }
  request.eef_step = segment_spec.eef_step;
  request.revolute_jump_threshold = segment_spec.revolute_jump_threshold;
  request.prismatic_jump_threshold = segment_spec.prismatic_jump_threshold;
  request.min_path_fraction = segment_spec.min_path_fraction;
  request.target_eef_speed = segment_spec.target_eef_speed;
  request.allow_target_contact = segment_spec.allow_target_contact;
  request.target_collision_object_id = segment_spec.target_collision_object_id;
  request.wall_reference_pose = segment_spec.wall_reference_pose;
  request.max_wall_penetration = segment_spec.max_wall_penetration;
  request.target_joint_state = segment_spec.target_joint_state;

  const std::string label =
    !segment_spec.label.empty() ? segment_spec.label : ("segment_" + std::to_string(segment_id));
  SegmentRoutingSpec routing;
  routing.label = label;
  routing.mode_hint = segment_spec.mode_hint;
  routing.planner_hint = segment_spec.planner_hint;

  return SegmentPlanningInputs{std::move(request), std::move(routing), segment_id};
}

MotionExec ManipulationManager::executeMotionSegments(
  const std::string & frame_id,
  const std::vector<MotionSegmentSpec> & segments,
  const std::vector<core_types::Pose> & waypoint_poses,
  const std::string & planning_group)
{
  reporter_.reset();

  if (segments.empty()) {
    last_summary_ = reporter_.buildSummary();

    MotionExec empty_exec;
    empty_exec.success = false;
    empty_exec.status = "empty";
    empty_exec.message = "No segments were provided for execution.";
    empty_exec.waypoints_completed = 0;
    return empty_exec;
  }

  // Continuity (previous segment's own ending joint state seeding the next
  // segment's start) is now tracked inside motion_planner_ itself, not here
  // -- see MotionPlanner::recordExecutionOutcome()/resetContinuity(). Reset
  // once per mission, after the empty-segments early return so a caller
  // that never injected a MotionPlanner (e.g. the empty-mission precheck
  // path) doesn't need one just to get that far.
  motion_planner_->resetContinuity();

  const auto order = segment_ordering_policy_->buildExecutionOrder(segments.size());

  // Every reporter_.recordSegment() call below goes through here too, so a
  // caller (e.g. an action server) gets real incremental feedback for every
  // attempted segment -- skipped/failed ones included -- not just the fully
  // successful ones.
  auto recordSegment = [&](const MotionSegmentExecMetrics & segment_metrics) {
      reporter_.recordSegment(segment_metrics);
      if (segment_progress_callback_) {
        segment_progress_callback_(segment_metrics);
      }
    };

  // Look-ahead pipelining state (see setPipeliningEnabled()/setPipeliningDepth()'s
  // doc comments). Local to this one executeMotionSegments() call -- there's
  // no reason for a speculative plan to outlive the mission that kicked it
  // off. Entries are queued in mission order (front = next segment to
  // execute); shared_future (not future) so a chained refill can peek an
  // in-flight entry's own result to seed the one after it without consuming
  // it -- the real consume at the top of that segment's own turn still
  // works, since shared_future permits get() to be called more than once.
  std::deque<std::pair<size_t,
    std::shared_future<motion_planner::MotionPlanner::SegmentPlanResult>>> lookahead_queue;
  const auto discardLookahead = [&lookahead_queue]() {
      // Cascading: every queued entry after the first is transitively seeded
      // from the one before it, so draining just the front on a real
      // execution failure would leave later entries built on an assumption
      // that failure just invalidated.
      while (!lookahead_queue.empty()) {
        lookahead_queue.front().second.get();  // join -- never leave a background plan dangling
        lookahead_queue.pop_front();
      }
    };

  for (size_t order_pos = 0; order_pos < order.size(); ++order_pos) {
    if (cancel_requested_callback_ && cancel_requested_callback_()) {
      logger_->info("Cancellation requested; stopping before the next segment.");
      discardLookahead();
      break;
    }

    const size_t segment_id = order[order_pos];
    if (segment_id >= segments.size()) {
      std::ostringstream oss;
      oss << "Ordering policy returned invalid segment index=" << segment_id
          << " (segment_count=" << segments.size() << "). Ignoring.";
      logger_->warn(oss.str());
      continue;
    }

    const MotionSegmentSpec & segment_spec = segments[segment_id];
    const auto & waypoints = segment_spec.waypoint_indices;
    const std::string label =
      !segment_spec.label.empty() ? segment_spec.label : ("segment_" + std::to_string(segment_id));

    MotionSegmentExecMetrics metrics;
    metrics.segment_id = segment_id;
    metrics.segment_label = label;
    metrics.frame_id = frame_id;

    const bool is_joint_target_segment = !segment_spec.target_joint_state.empty();
    // A joint-target segment has no waypoint_indices to count (there's no
    // pose array it indexes into) -- treat reaching the target configuration
    // as one unit of work, same as any single-pose segment.
    metrics.attempted_waypoints = is_joint_target_segment ? 1 : waypoints.size();
    metrics.start_waypoint_index = waypoints.empty() ? 0 : waypoints.front();
    metrics.end_waypoint_index = waypoints.empty() ? 0 : waypoints.back();
    metrics.outcome = SegmentExecutionState::kAttempted;

    if (waypoints.empty() && !is_joint_target_segment) {
      metrics.outcome = SegmentExecutionState::kSkippedUnreachable;
      metrics.failure_reason_code = "empty_segment";
      metrics.failure_stage = "precheck";
      recordSegment(metrics);
      reporter_.logSegmentRecord(metrics, *logger_);
      continue;
    }

    if (!std::is_sorted(waypoints.begin(), waypoints.end())) {
      metrics.outcome = SegmentExecutionState::kFailedRuntime;
      metrics.failure_reason_code = "invalid_segment_indices";
      metrics.failure_stage = "precheck";
      recordSegment(metrics);
      reporter_.logSegmentRecord(metrics, *logger_);
      continue;
    }

    // Guaranteed non-nullopt -- the precheck this same iteration already
    // ran above (empty/non-joint-target, unsorted waypoints) is exactly
    // what buildSegmentPlanningInputs() itself checks.
    auto planning_inputs = buildSegmentPlanningInputs(
      order, order_pos, frame_id, segments, waypoint_poses, planning_group);
    PlanningRequest request = std::move(planning_inputs->request);
    SegmentRoutingSpec routing = std::move(planning_inputs->routing);

    PlanningResult planning_result;
    MotionExec exec;
    uint64_t total_planning_time_ms = 0;
    uint64_t total_execution_time_ms = 0;
    std::string mode_used;
    std::string planner_used;
    bool fallback_used = false;
    bool mode_switch_used = false;
    int bridge_count = 0;

    // Mode/planner routing (including the attempt-based Cartesian-vs-FSP
    // decision -- see MotionPlanner::planSegment()'s doc comment) is
    // motion_planner_'s responsibility, not this loop's. This retry loop
    // covers execution failures too (not just planning ones), so each
    // retryable failure re-asks motion_planner_ to plan fresh -- world state
    // may have changed since the last attempt.
    for (size_t attempt = 0; attempt <= kMaxImmediateRetriesPerSegment; ++attempt) {
      motion_planner::MotionPlanner::SegmentPlanResult plan_result;
      if (attempt == 0 && !lookahead_queue.empty() && lookahead_queue.front().first == segment_id) {
        // Consume the plan kicked off speculatively while an earlier
        // segment was still executing -- may block briefly here if this
        // segment's own turn arrived before the background plan finished;
        // that's still never worse than the non-pipelined baseline. Timed
        // so the log can show whether it was already done (real overlap
        // achieved) or this segment had to wait on it.
        const bool was_ready = lookahead_queue.front().second.wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready;
        const auto wait_start = std::chrono::steady_clock::now();
        plan_result = lookahead_queue.front().second.get();
        lookahead_queue.pop_front();
        const auto wait_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - wait_start).count();
        std::ostringstream oss;
        oss << "segment=" << segment_id << " label=" << label
            << ": consumed look-ahead plan (" <<
          (was_ready ? "was already ready -- full overlap" : "still running") <<
          ", blocked " << wait_ms << "ms waiting on it; " << lookahead_queue.size() <<
          " more queued ahead).";
        logger_->info(oss.str());
      } else {
        discardLookahead();  // stale/mismatched -- never reuse a plan seeded for the wrong segment
        plan_result = motion_planner_->planSegment(request, routing);
      }
      planning_result = plan_result.planning_result;
      total_planning_time_ms += planning_result.planning_time_ms;
      mode_used = plan_result.mode_used;
      planner_used = plan_result.planner_used;
      fallback_used = plan_result.fallback_used;
      mode_switch_used = plan_result.mode_switch_used;
      bridge_count = plan_result.bridge_count;

      // Plan/execute decoupling (§6.1/§6.5): the planner only planned. If it
      // succeeded, hand the resulting trajectory to the ExecutionPort to
      // actually move the robot -- an execution failure is folded back into
      // planning_result (stage=kExecution) so the existing outcome
      // classification below keeps working unchanged.
      if (planning_result.success) {
        if (segment_planned_callback_) {
          segment_planned_callback_(
            segment_id, plan_result.mode_used, planning_result.trajectory,
            plan_result.trajectory_segments);
        }

        // Look-ahead pipelining: top the queue up to pipelining_depth_
        // entries on background threads right before this segment's
        // (blocking) execute() call -- each entry's own expected end state
        // is known as soon as its plan is kicked off, before execution even
        // starts. planSegmentWithStartState() (not planSegment()) so
        // background threads never touch motion_planner_'s tracked
        // continuity, which stays main-thread-only via
        // recordExecutionOutcome() below. The first new entry is seeded
        // from this segment's own already-computed trajectory; every entry
        // after that is chained -- seeded from the previous queue entry's
        // own (still possibly in-flight) result, fetched inside that
        // entry's own background task rather than blocking this thread.
        if (pipelining_enabled_) {
          while (lookahead_queue.size() < pipelining_depth_) {
            auto next_inputs = buildSegmentPlanningInputs(
              order, order_pos + 1 + lookahead_queue.size(), frame_id, segments, waypoint_poses,
              planning_group);
            if (!next_inputs) {
              break;  // end of mission -- nothing further to queue
            }
            const size_t next_segment_id = next_inputs->segment_id;
            {
              std::ostringstream oss;
              oss << "segment=" << segment_id << " label=" << label
                  << ": queueing look-ahead plan for segment=" << next_segment_id
                  << " on a background thread (queue depth after this: " <<
                (lookahead_queue.size() + 1) << "/" << pipelining_depth_ << ").";
              logger_->info(oss.str());
            }
            if (lookahead_queue.empty()) {
              const std::vector<double> seed = planning_result.trajectory.points.empty() ?
                std::vector<double>{} : planning_result.trajectory.points.back().positions;
              lookahead_queue.emplace_back(
                next_segment_id,
                std::shared_future<motion_planner::MotionPlanner::SegmentPlanResult>(
                  std::async(
                    std::launch::async,
                    [this, req = std::move(next_inputs->request),
                    rt = std::move(next_inputs->routing), seed]() mutable {
                      return motion_planner_->planSegmentWithStartState(std::move(req), rt, seed);
                    })));
            } else {
              std::shared_future<motion_planner::MotionPlanner::SegmentPlanResult> prev_future =
                lookahead_queue.back().second;
              lookahead_queue.emplace_back(
                next_segment_id,
                std::shared_future<motion_planner::MotionPlanner::SegmentPlanResult>(
                  std::async(
                    std::launch::async,
                    [this, req = std::move(next_inputs->request),
                    rt = std::move(next_inputs->routing), prev_future]() mutable {
                      // Blocks this background task (never the caller) on the
                      // previous queue entry's own plan -- its expected end
                      // state can't be known any sooner than that.
                      const auto prev_result = prev_future.get();
                      const std::vector<double> seed =
                      prev_result.planning_result.trajectory.points.empty() ?
                      std::vector<double>{} :
                      prev_result.planning_result.trajectory.points.back().positions;
                      return motion_planner_->planSegmentWithStartState(std::move(req), rt, seed);
                    })));
            }
          }
        }

        ExecutionRequest execution_request;
        execution_request.planning_group = planning_group;
        execution_request.trajectory = planning_result.trajectory;
        const ExecutionResult execution_result = execution_port_->execute(execution_request);
        total_execution_time_ms += execution_result.execution_time_ms;

        if (!execution_result.success) {
          planning_result.success = false;
          planning_result.status = "failed";
          planning_result.message = execution_result.message;
          planning_result.waypoints_completed = 0;
          planning_result.failure_reason_code = execution_result.failure_reason_code;
          planning_result.failure_stage = FailureStage::kExecution;
          // Segment N's real end state may not match what N+1's speculative
          // plan assumed (it was seeded from N's *planned* end state, not a
          // confirmed one) -- discard rather than risk executing a plan
          // built from a start state the robot never actually reached.
          discardLookahead();
        }
      }

      exec.success = planning_result.success;
      exec.status = planning_result.status;
      exec.message = planning_result.message;
      exec.waypoints_completed = planning_result.waypoints_completed;

      if (exec.success) {
        break;
      }

      const bool can_retry = attempt < kMaxImmediateRetriesPerSegment &&
        isRetryableFailureCode(planning_result.failure_reason_code);
      if (can_retry) {
        std::ostringstream oss;
        oss << "segment=" << segment_id << " label=" << label
            << " failed with reason=" << planning_result.failure_reason_code
            << " stage=" << toFailureStageString(planning_result.failure_stage)
            << ". Retrying (" << (attempt + 1) << "/" << kMaxImmediateRetriesPerSegment << ").";
        logger_->warn(oss.str());
        continue;
      }

      break;
    }

    // Recorded once, after the retry loop settles on a final outcome -- not
    // per-attempt, so a retried segment's own in-progress attempts never
    // corrupt mode_switch_used's comparison basis (which reads
    // motion_planner_'s continuity state) for its own next retry.
    motion_planner_->recordExecutionOutcome(exec.success, mode_used, planning_result.trajectory);

    metrics.mode_used = mode_used;
    metrics.planner_used = planner_used;
    metrics.fallback_used = fallback_used;
    metrics.mode_switch_used = mode_switch_used;
    metrics.bridge_count = bridge_count;
    metrics.planning_time_ms = total_planning_time_ms;
    metrics.execution_time_ms = total_execution_time_ms;
    metrics.completed_waypoints = exec.waypoints_completed;
    metrics.path_fraction = planning_result.path_fraction;
    metrics.completion_ratio = (metrics.attempted_waypoints > 0) ?
      static_cast<double>(metrics.completed_waypoints) /
      static_cast<double>(metrics.attempted_waypoints) :
      0.0;

    if (exec.success && metrics.completed_waypoints >= metrics.attempted_waypoints) {
      metrics.outcome = SegmentExecutionState::kCompleted;
      metrics.failure_reason_code = "none";
      metrics.failure_stage = "none";
    } else if (exec.success) {
      // Covers 0 < completed < attempted *and* the degenerate completed==0
      // case: CartesianPlanner's own success flag can be true even at a low
      // achieved fraction (proven BT-server semantics -- "return true but
      // warn, caller decides whether to proceed"), and a low fraction times
      // a small waypoint count can floor to exactly 0 completed. That's
      // still "planner reported success," not a runtime failure -- routing
      // it into the else branch below previously misclassified it as
      // kFailedRuntime with a fabricated "failed_runtime" reason code.
      metrics.outcome = SegmentExecutionState::kPartiallyCompleted;
      metrics.failure_reason_code = (planning_result.failure_reason_code != "none") ?
        planning_result.failure_reason_code :
        "partial_completion";
      metrics.failure_stage = (planning_result.failure_stage != FailureStage::kNone) ?
        toFailureStageString(planning_result.failure_stage) :
        "execution";
    } else {
      metrics.outcome = toSkippedOutcome(planning_result.failure_reason_code);
      metrics.failure_reason_code = (planning_result.failure_reason_code != "none") ?
        planning_result.failure_reason_code :
        "failed_runtime";
      metrics.failure_stage = (planning_result.failure_stage != FailureStage::kNone) ?
        toFailureStageString(planning_result.failure_stage) :
        "execution";

      if (metrics.outcome == SegmentExecutionState::kFailedRuntime) {
        metrics.failure_reason_code = (planning_result.failure_reason_code != "none") ?
          planning_result.failure_reason_code :
          "failed_runtime";
      }
    }

    recordSegment(metrics);
    reporter_.logSegmentRecord(metrics, *logger_);
  }
  // Defensive -- by construction there's nothing left to discard once the
  // loop exits normally (the last segment's own kickoff attempt for "the
  // one after it" always resolves to nullopt, out of range), but never
  // leave a background plan dangling past this function's return.
  discardLookahead();

  last_summary_ = reporter_.buildSummary();
  reporter_.logRunSummary(last_summary_, *logger_);

  MotionExec aggregate_exec;
  aggregate_exec.success = (last_summary_.total_segments > 0 &&
    last_summary_.completed_segments == last_summary_.total_segments);
  aggregate_exec.status = aggregate_exec.success ? "completed" : "partial";

  std::ostringstream message;
  message << "segments completed=" << last_summary_.completed_segments
          << "/" << last_summary_.total_segments
          << " partial=" << last_summary_.partially_completed_segments
          << " failed=" << last_summary_.failed_segments
          << " ratio=" << last_summary_.global_completion_ratio;
  aggregate_exec.message = message.str();
  aggregate_exec.waypoints_completed = last_summary_.completed_waypoints;

  return aggregate_exec;
}

MotionRunSummary ManipulationManager::lastRunSummary() const
{
  return last_summary_;
}

const std::vector<MotionSegmentExecMetrics> & ManipulationManager::lastSegmentMetrics() const
{
  return reporter_.segmentMetrics();
}

}  // namespace manipulation_manager
