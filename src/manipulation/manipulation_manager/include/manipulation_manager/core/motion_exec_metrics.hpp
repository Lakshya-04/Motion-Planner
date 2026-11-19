#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace manipulation_manager
{

enum class SegmentExecutionState : uint8_t
{
  kPlanned = 0,
  kAttempted,
  kCompleted,
  kPartiallyCompleted,
  kSkippedUnreachable,
  kSkippedCollision,
  kSkippedNoIk,
  kFailedRuntime,
};

/// Shared with any reporter/printer of SegmentExecutionState (motion_exec_reporter.cpp's
/// per-segment log line, MotionValidator's/PlannerBenchmark's summary tables) so a
/// human-readable outcome never regresses back to a raw enum int in one of them
/// while the other stays correct.
const char * toString(SegmentExecutionState outcome);

struct MotionSegmentExecMetrics
{
  size_t segment_id{0};
  std::string segment_label;
  std::string mode_used;
  std::string planner_used;
  std::string free_space_backend;  ///< free-space plugin that planned (part of) the segment; empty if none
  std::string frame_id;

  size_t start_waypoint_index{0};
  size_t end_waypoint_index{0};
  size_t attempted_waypoints{0};
  size_t completed_waypoints{0};
  double completion_ratio{0.0};
  double path_fraction{0.0};  ///< Planner-reported achieved fraction of the requested path (0-1).

  std::string failure_reason_code{"none"};
  std::string failure_stage{"none"};

  uint64_t planning_time_ms{0};
  uint64_t execution_time_ms{0};

  bool fallback_used{false};
  bool mode_switch_used{false};
  /// How many bridge-then-resume cycles survived into this segment's final
  /// plan -- see motion_planner::MotionPlanner::SegmentPlanResult::bridge_count,
  /// copied through unchanged.
  int bridge_count{0};
  SegmentExecutionState outcome{SegmentExecutionState::kPlanned};
};

struct MotionRunSummary
{
  size_t total_segments{0};
  size_t attempted_segments{0};
  size_t completed_segments{0};
  size_t partially_completed_segments{0};
  size_t skipped_segments{0};
  size_t failed_segments{0};

  size_t total_waypoints{0};
  size_t completed_waypoints{0};
  double global_completion_ratio{0.0};

  uint64_t total_planning_time_ms{0};
  uint64_t total_execution_time_ms{0};

  size_t mode_switch_count{0};
  size_t fallback_count{0};

  size_t recoverable_failures{0};
  size_t terminal_failures{0};
};

}  // namespace manipulation_manager
