// MotionExecReporter, toString(SegmentExecutionState) and the input-order policy: the run
// summary every execution prints and the outcome names that appear in logs and reports.
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "manipulation_manager/core/motion_exec_reporter.hpp"
#include "manipulation_manager/core/segment_ordering_policy.hpp"

namespace manipulation_manager
{
namespace
{

class CaptureLogger : public core_types::Logger
{
public:
  void info(const std::string & m) const override { infos.push_back(m); }
  void warn(const std::string & m) const override { warns.push_back(m); }
  mutable std::vector<std::string> infos;
  mutable std::vector<std::string> warns;
};

MotionSegmentExecMetrics seg(
  SegmentExecutionState outcome, size_t attempted, size_t completed, uint64_t plan_ms = 0,
  uint64_t exec_ms = 0)
{
  MotionSegmentExecMetrics m;
  m.outcome = outcome;
  m.attempted_waypoints = attempted;
  m.completed_waypoints = completed;
  m.planning_time_ms = plan_ms;
  m.execution_time_ms = exec_ms;
  return m;
}

}  // namespace

TEST(MotionExecReporterTest, ToStringNamesEveryOutcome)
{
  EXPECT_STREQ(toString(SegmentExecutionState::kPlanned), "planned");
  EXPECT_STREQ(toString(SegmentExecutionState::kAttempted), "attempted");
  EXPECT_STREQ(toString(SegmentExecutionState::kCompleted), "completed");
  EXPECT_STREQ(toString(SegmentExecutionState::kPartiallyCompleted), "partially_completed");
  EXPECT_STREQ(toString(SegmentExecutionState::kSkippedUnreachable), "skipped_unreachable");
  EXPECT_STREQ(toString(SegmentExecutionState::kSkippedCollision), "skipped_collision");
  EXPECT_STREQ(toString(SegmentExecutionState::kSkippedNoIk), "skipped_no_ik");
  EXPECT_STREQ(toString(SegmentExecutionState::kFailedRuntime), "failed_runtime");
  EXPECT_STREQ(toString(static_cast<SegmentExecutionState>(200)), "unknown");
}

TEST(MotionExecReporterTest, EmptyReporterGivesZeroSummary)
{
  MotionExecReporter r;
  const auto s = r.buildSummary();
  EXPECT_EQ(s.total_segments, 0U);
  EXPECT_EQ(s.total_waypoints, 0U);
  EXPECT_DOUBLE_EQ(s.global_completion_ratio, 0.0);  // no division by zero
}

TEST(MotionExecReporterTest, SummaryCountsEachOutcomeClass)
{
  MotionExecReporter r;
  r.recordSegment(seg(SegmentExecutionState::kCompleted, 10, 10, 100, 1000));
  r.recordSegment(seg(SegmentExecutionState::kPartiallyCompleted, 10, 4, 50, 500));
  r.recordSegment(seg(SegmentExecutionState::kSkippedCollision, 6, 0));
  r.recordSegment(seg(SegmentExecutionState::kSkippedNoIk, 2, 0));
  r.recordSegment(seg(SegmentExecutionState::kFailedRuntime, 2, 0, 5, 7));
  r.recordSegment(seg(SegmentExecutionState::kPlanned, 0, 0));

  const auto s = r.buildSummary();
  EXPECT_EQ(s.total_segments, 6U);
  EXPECT_EQ(s.attempted_segments, 5U);  // everything but kPlanned
  EXPECT_EQ(s.completed_segments, 1U);
  EXPECT_EQ(s.partially_completed_segments, 1U);
  EXPECT_EQ(s.skipped_segments, 2U);
  EXPECT_EQ(s.failed_segments, 1U);
  EXPECT_EQ(s.recoverable_failures, 3U);  // partial + 2 skips
  EXPECT_EQ(s.terminal_failures, 1U);
  EXPECT_EQ(s.total_waypoints, 30U);
  EXPECT_EQ(s.completed_waypoints, 14U);
  EXPECT_NEAR(s.global_completion_ratio, 14.0 / 30.0, 1e-12);
  EXPECT_EQ(s.total_planning_time_ms, 155U);
  EXPECT_EQ(s.total_execution_time_ms, 1507U);
}

TEST(MotionExecReporterTest, SummaryCountsFallbacksAndModeSwitches)
{
  MotionExecReporter r;
  auto a = seg(SegmentExecutionState::kCompleted, 1, 1);
  a.fallback_used = true;
  auto b = seg(SegmentExecutionState::kCompleted, 1, 1);
  b.fallback_used = true;
  b.mode_switch_used = true;
  r.recordSegment(a);
  r.recordSegment(b);
  r.recordSegment(seg(SegmentExecutionState::kCompleted, 1, 1));
  const auto s = r.buildSummary();
  EXPECT_EQ(s.fallback_count, 2U);
  EXPECT_EQ(s.mode_switch_count, 1U);
}

TEST(MotionExecReporterTest, ResetClearsRecordedSegments)
{
  MotionExecReporter r;
  r.recordSegment(seg(SegmentExecutionState::kCompleted, 3, 3));
  ASSERT_EQ(r.segmentMetrics().size(), 1U);
  r.reset();
  EXPECT_TRUE(r.segmentMetrics().empty());
  EXPECT_EQ(r.buildSummary().total_segments, 0U);
}

TEST(MotionExecReporterTest, LogLinesCarryOutcomeNameAndCounts)
{
  MotionExecReporter r;
  auto m = seg(SegmentExecutionState::kSkippedNoIk, 8, 0);
  m.segment_id = 3;
  m.segment_label = "stroke";
  m.failure_reason_code = "ik";
  r.recordSegment(m);
  CaptureLogger log;
  r.logSegmentRecord(m, log);
  r.logRunSummary(r.buildSummary(), log);
  ASSERT_EQ(log.infos.size(), 2U);
  EXPECT_NE(log.infos[0].find("id=3"), std::string::npos);
  EXPECT_NE(log.infos[0].find("label=stroke"), std::string::npos);
  EXPECT_NE(log.infos[0].find("outcome=skipped_no_ik"), std::string::npos);
  EXPECT_NE(log.infos[0].find("reason=ik"), std::string::npos);
  EXPECT_NE(log.infos[1].find("skipped=1"), std::string::npos);
  EXPECT_NE(log.infos[1].find("recoverable=1"), std::string::npos);
}

TEST(SegmentOrderingPolicyTest, InputOrderIsIdentity)
{
  InputOrderSegmentOrderingPolicy p;
  EXPECT_TRUE(p.buildExecutionOrder(0).empty());
  EXPECT_EQ(p.buildExecutionOrder(4), (std::vector<size_t>{0, 1, 2, 3}));
}

}  // namespace manipulation_manager
