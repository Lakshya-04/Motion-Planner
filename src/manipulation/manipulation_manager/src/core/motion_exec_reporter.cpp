#include "manipulation_manager/core/motion_exec_reporter.hpp"

#include <sstream>

namespace manipulation_manager
{

const char * toString(const SegmentExecutionState outcome)
{
  switch (outcome) {
    case SegmentExecutionState::kPlanned:
      return "planned";
    case SegmentExecutionState::kAttempted:
      return "attempted";
    case SegmentExecutionState::kCompleted:
      return "completed";
    case SegmentExecutionState::kPartiallyCompleted:
      return "partially_completed";
    case SegmentExecutionState::kSkippedUnreachable:
      return "skipped_unreachable";
    case SegmentExecutionState::kSkippedCollision:
      return "skipped_collision";
    case SegmentExecutionState::kSkippedNoIk:
      return "skipped_no_ik";
    case SegmentExecutionState::kFailedRuntime:
      return "failed_runtime";
  }
  return "unknown";
}

namespace
{

bool isRecoverableFailure(const SegmentExecutionState outcome)
{
  return outcome == SegmentExecutionState::kPartiallyCompleted ||
         outcome == SegmentExecutionState::kSkippedUnreachable ||
         outcome == SegmentExecutionState::kSkippedCollision ||
         outcome == SegmentExecutionState::kSkippedNoIk;
}

bool isTerminalFailure(const SegmentExecutionState outcome)
{
  return outcome == SegmentExecutionState::kFailedRuntime;
}

bool isSkipped(const SegmentExecutionState outcome)
{
  return outcome == SegmentExecutionState::kSkippedUnreachable ||
         outcome == SegmentExecutionState::kSkippedCollision ||
         outcome == SegmentExecutionState::kSkippedNoIk;
}

}  // namespace

void MotionExecReporter::reset()
{
  segment_metrics_.clear();
}

void MotionExecReporter::recordSegment(const MotionSegmentExecMetrics & metrics)
{
  segment_metrics_.push_back(metrics);
}

const std::vector<MotionSegmentExecMetrics> & MotionExecReporter::segmentMetrics() const
{
  return segment_metrics_;
}

MotionRunSummary MotionExecReporter::buildSummary() const
{
  MotionRunSummary summary;
  summary.total_segments = segment_metrics_.size();

  for (const auto & segment : segment_metrics_) {
    summary.total_waypoints += segment.attempted_waypoints;
    summary.completed_waypoints += segment.completed_waypoints;
    summary.total_planning_time_ms += segment.planning_time_ms;
    summary.total_execution_time_ms += segment.execution_time_ms;

    if (segment.mode_switch_used) {
      ++summary.mode_switch_count;
    }
    if (segment.fallback_used) {
      ++summary.fallback_count;
    }

    if (segment.outcome != SegmentExecutionState::kPlanned) {
      ++summary.attempted_segments;
    }
    if (segment.outcome == SegmentExecutionState::kCompleted) {
      ++summary.completed_segments;
    }
    if (segment.outcome == SegmentExecutionState::kPartiallyCompleted) {
      ++summary.partially_completed_segments;
    }
    if (isSkipped(segment.outcome)) {
      ++summary.skipped_segments;
    }
    if (isRecoverableFailure(segment.outcome)) {
      ++summary.recoverable_failures;
    }
    if (isTerminalFailure(segment.outcome)) {
      ++summary.terminal_failures;
      ++summary.failed_segments;
    }
  }

  if (summary.total_waypoints > 0) {
    summary.global_completion_ratio =
      static_cast<double>(summary.completed_waypoints) /
      static_cast<double>(summary.total_waypoints);
  }

  return summary;
}

void MotionExecReporter::logSegmentRecord(
  const MotionSegmentExecMetrics & metrics,
  const core_types::Logger & logger) const
{
  std::ostringstream oss;
  oss << "segment_exec id=" << metrics.segment_id
      << " label=" << metrics.segment_label
      << " outcome=" << toString(metrics.outcome)
      << " mode=" << metrics.mode_used
      << " planner=" << metrics.planner_used
      << " fsp_backend=" << (metrics.free_space_backend.empty() ? "-" : metrics.free_space_backend)
      << " frame=" << metrics.frame_id
      << " wp=[" << metrics.start_waypoint_index << "," << metrics.end_waypoint_index << "]"
      << " attempted=" << metrics.attempted_waypoints
      << " completed=" << metrics.completed_waypoints
      << " ratio=" << metrics.completion_ratio
      << " path_fraction=" << metrics.path_fraction
      << " reason=" << metrics.failure_reason_code
      << " stage=" << metrics.failure_stage
      << " planning_ms=" << metrics.planning_time_ms
      << " execution_ms=" << metrics.execution_time_ms
      << " fallback=" << (metrics.fallback_used ? "true" : "false")
      << " mode_switch=" << (metrics.mode_switch_used ? "true" : "false")
      << " traj_pts=" << metrics.planned_points
      << " traj_s=" << metrics.planned_duration_s;
  logger.info(oss.str());
}

void MotionExecReporter::logRunSummary(
  const MotionRunSummary & summary,
  const core_types::Logger & logger) const
{
  std::ostringstream oss;
  oss << "motion_exec_summary segments(total=" << summary.total_segments
      << " attempted=" << summary.attempted_segments
      << " completed=" << summary.completed_segments
      << " partial=" << summary.partially_completed_segments
      << " skipped=" << summary.skipped_segments
      << " failed=" << summary.failed_segments << ")"
      << " waypoints(total=" << summary.total_waypoints
      << " completed=" << summary.completed_waypoints
      << " ratio=" << summary.global_completion_ratio << ")"
      << " timing(planning_ms=" << summary.total_planning_time_ms
      << " execution_ms=" << summary.total_execution_time_ms << ")"
      << " switches(mode=" << summary.mode_switch_count
      << " fallback=" << summary.fallback_count << ")"
      << " failures(recoverable=" << summary.recoverable_failures
      << " terminal=" << summary.terminal_failures << ")";
  logger.info(oss.str());
}

}  // namespace manipulation_manager
