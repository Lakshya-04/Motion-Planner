#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>

#include "coverage_task_msgs/action/execute_coverage.hpp"
#include "manipulation_manager/core/mission_fsm.hpp"
#include "manipulation_manager/core/mission_verify.hpp"

using namespace manipulation_manager;

namespace
{
MissionVerifyInput clean()
{
  MissionVerifyInput in;
  in.loaded = true;
  in.tracker_success = true;
  in.done = {0, 1, 2};
  in.peak_force_n = 20.0;
  in.column_peaks_n = {{0, 18.0}, {1, 20.0}, {2, 12.0}};
  return in;
}
}  // namespace

TEST(MissionVerify, CleanRunPasses)
{
  const auto r = buildMissionVerify(clean());
  EXPECT_TRUE(r.passed);
  EXPECT_DOUBLE_EQ(r.coverage_percent, 100.0);
  EXPECT_TRUE(r.failed_segment_ids.empty());
  EXPECT_TRUE(r.failures.empty());
}

TEST(MissionVerify, PartialAndFailedColumnsFailTheirSegments)
{
  auto in = clean();
  in.done = {0};
  in.partial = {2};
  in.failed = {1};
  in.tracker_success = false;
  const auto r = buildMissionVerify(in);
  EXPECT_FALSE(r.passed);
  EXPECT_DOUBLE_EQ(r.coverage_percent, 100.0 / 3.0);
  EXPECT_EQ(r.failed_segment_ids, (std::vector<size_t>{1, 2}));
}

TEST(MissionVerify, PeakLimitFailsTheColumnAndTheRun)
{
  auto in = clean();
  in.max_peak_force_n = 19.0;
  const auto r = buildMissionVerify(in);
  EXPECT_FALSE(r.passed);
  EXPECT_EQ(r.failed_segment_ids, (std::vector<size_t>{1}));   // 20 N > 19 N; 18 N is fine
}

TEST(MissionVerify, NoLimitMeansNoPeakCheck)
{
  auto in = clean();
  in.peak_force_n = 500.0;
  EXPECT_TRUE(buildMissionVerify(in).passed);
}

TEST(MissionVerify, UnknownPeakNeverTripsTheLimit)
{
  auto in = clean();
  in.peak_force_n = -1.0;   // no force topic
  in.column_peaks_n.clear();
  in.max_peak_force_n = 10.0;
  EXPECT_TRUE(buildMissionVerify(in).passed);
}

TEST(MissionVerify, CanceledOrNotStartedFails)
{
  auto c = clean();
  c.canceled = true;
  EXPECT_FALSE(buildMissionVerify(c).passed);
  MissionVerifyInput n;
  n.message = "cannot load the coverage plan x";
  const auto r = buildMissionVerify(n);
  EXPECT_FALSE(r.passed);
  EXPECT_DOUBLE_EQ(r.coverage_percent, 0.0);
  ASSERT_FALSE(r.failures.empty());
  EXPECT_NE(r.failures[0].find("cannot load"), std::string::npos);
}

TEST(MissionVerify, UnreachedStandFailsWithAReason)
{
  auto in = clean();
  in.tracker_success = false;   // all columns done, a stand was not reached
  const auto r = buildMissionVerify(in);
  EXPECT_FALSE(r.passed);
  EXPECT_FALSE(r.failures.empty());
}

TEST(MissionVerify, JsonCarriesTheVerifyResultKeys)
{
  auto in = clean();
  in.report_path = "ledger.json";
  const auto j = missionVerifyToJson(buildMissionVerify(in), in);
  for (const char * k : {"passed", "coverage_percent", "failed_segment_ids", "report_path", "source", "failures"}) {
    EXPECT_TRUE(j.contains(k)) << k;
  }
  EXPECT_EQ(j["source"], "manager");
  EXPECT_EQ(j["coverage_basis"], "columns");
  EXPECT_EQ(j["report_path"], "ledger.json");
}

TEST(MissionVerify, WritesAFileAndRefusesAnEmptyOrBadPath)
{
  const std::string path = ::testing::TempDir() + "mission_verify_test.json";
  const auto in = clean();
  ASSERT_TRUE(writeMissionVerify(path, missionVerifyToJson(buildMissionVerify(in), in)));
  std::ifstream f(path);
  const auto j = nlohmann::json::parse(f);
  EXPECT_TRUE(j["passed"].get<bool>());
  std::remove(path.c_str());
  EXPECT_FALSE(writeMissionVerify("", {}));
  EXPECT_FALSE(writeMissionVerify("/nonexistent_dir_xyz/r.json", {}));
}

TEST(ExecuteCoverageFeedback, CarriesTheMissionFsmState)
{
  coverage_task_msgs::action::ExecuteCoverage::Feedback fb;
  EXPECT_TRUE(fb.mission_state.empty());
  MissionFsm fsm;
  fsm.start();
  fsm.requestPause();
  fsm.atBoundary();
  fb.mission_state = toString(fsm.state());
  EXPECT_EQ(fb.mission_state, "PAUSED");
}
