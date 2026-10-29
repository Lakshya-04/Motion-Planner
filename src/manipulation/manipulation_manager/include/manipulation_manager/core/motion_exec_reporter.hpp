/**
 * @file motion_exec_reporter.hpp
 * @brief Accumulates per-segment execution metrics and logs them via a
 *        core_types::Logger (no direct rclcpp dependency).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <vector>

#include "core_types/logger.hpp"

#include "manipulation_manager/core/motion_exec_metrics.hpp"

namespace manipulation_manager
{

/**
 * @class MotionExecReporter
 * @brief Records segment metrics, builds run summaries, and logs both.
 */
class MotionExecReporter
{
public:
  /// @brief Clear all recorded segment metrics for a new execution run.
  void reset();

  /**
   * @brief Record one segment's execution metrics.
   * @param[in] metrics Metrics for the completed/skipped/failed segment.
   */
  void recordSegment(const MotionSegmentExecMetrics & metrics);

  const std::vector<MotionSegmentExecMetrics> & segmentMetrics() const;

  /// @brief Aggregate all recorded segment metrics into a run summary.
  MotionRunSummary buildSummary() const;

  /**
   * @brief Log a single segment's metrics at info level.
   * @param[in] metrics Metrics to format and log.
   * @param[in] logger Logging backend to write through.
   */
  void logSegmentRecord(
    const MotionSegmentExecMetrics & metrics,
    const core_types::Logger & logger) const;

  /**
   * @brief Log an aggregated run summary at info level.
   * @param[in] summary Summary to format and log.
   * @param[in] logger Logging backend to write through.
   */
  void logRunSummary(const MotionRunSummary & summary, const core_types::Logger & logger) const;

private:
  std::vector<MotionSegmentExecMetrics> segment_metrics_;
};

}  // namespace manipulation_manager
