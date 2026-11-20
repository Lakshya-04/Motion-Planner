#include <gtest/gtest.h>

#include <stdexcept>

#include "coverage_planner/core/verify_result.hpp"

using namespace coverage_planner;  // NOLINT

namespace
{
WallPlane wall()
{
  WallPlane w;
  w.id = 1;
  w.pose.pose.orientation.w = 1.0;
  w.u_min = 0.0; w.u_max = 2.0; w.v_min = 0.0; w.v_max = 1.0;
  return w;
}

// Vertical strokes every 0.16 m; column k spans t in [2.2k, 2.2k + 2.1]. `valve` marks tool_active.
std::vector<RunSample> run(double force, bool valve, std::vector<SegmentWindow> * segs)
{
  std::vector<RunSample> out;
  double t = 0.0;
  uint32_t id = 0;
  for (double u = 0.1; u <= 1.9001; u += 0.16) {
    const double t0 = t;
    for (double y = 0.0; y <= 1.0001; y += 0.05) {
      RunSample s;
      s.t_s = t; t += 0.1;
      s.x = u; s.y = y;
      s.force_n = force;
      s.tool_active = valve;
      out.push_back(s);
    }
    segs->push_back({id++, t0, t - 0.1});
    RunSample lift;
    lift.t_s = t; t += 0.1;
    lift.x = u; lift.y = 1.0;
    out.push_back(lift);
  }
  return out;
}

ScorecardOptions contactOptions()
{
  ScorecardOptions o;
  o.contact = true; o.swath_m = 0.2; o.resolution_m = 0.01;
  o.engaged_force_n = 1.0; o.target_force_n = 10.0; o.force_band_n = 1.0;
  return o;
}

ScorecardOptions nonContactOptions()
{
  ScorecardOptions o;
  o.contact = false; o.swath_m = 0.2; o.resolution_m = 0.01;
  return o;
}
}  // namespace

TEST(VerifyResult, ModeLimitsAreTheDocumentedNumbers)
{
  const auto c = limitsForMode(RunMode::Contact);
  EXPECT_DOUBLE_EQ(c.max_force_peak_n, 60.0);
  EXPECT_DOUBLE_EQ(c.min_in_band_fraction, 0.8);
  EXPECT_DOUBLE_EQ(c.min_covered_fraction, 0.95);
  EXPECT_DOUBLE_EQ(limitsForMode(RunMode::NonContact).max_force_peak_n, 5.0);
  EXPECT_DOUBLE_EQ(limitsForMode(RunMode::Spray).max_thickness_cov, 0.5);
  EXPECT_EQ(runModeFromString("non_contact"), RunMode::NonContact);
  EXPECT_THROW(runModeFromString("laser"), std::invalid_argument);
}

TEST(VerifyResult, GoodContactRunPassesWithNoFailedSegments)
{
  std::vector<SegmentWindow> segs;
  const auto samples = run(10.1, false, &segs);
  const auto r = verifyFromTelemetry(wall(), {}, samples, segs, contactOptions(), limitsForMode(RunMode::Contact), "card.json");
  EXPECT_TRUE(r.passed);
  EXPECT_GE(r.coverage_percent, 95.0);
  EXPECT_TRUE(r.failed_segment_ids.empty());
  EXPECT_EQ(r.report_path, "card.json");
  EXPECT_EQ(r.source, "telemetry");
}

TEST(VerifyResult, OneOverpressedColumnFailsTheRunAndIsNamed)
{
  std::vector<SegmentWindow> segs;
  auto samples = run(10.1, false, &segs);
  for (auto & s : samples) {
    if (s.t_s >= segs[3].t_start_s && s.t_s <= segs[3].t_end_s) {s.force_n = 70.0;}
  }
  const auto r = verifyFromTelemetry(wall(), {}, samples, segs, contactOptions(), limitsForMode(RunMode::Contact));
  EXPECT_FALSE(r.passed);
  ASSERT_EQ(r.failed_segment_ids.size(), 1U);
  EXPECT_EQ(r.failed_segment_ids[0], 3U);
}

TEST(VerifyResult, ColumnOutsideTheForceBandFailsOnlyThatSegment)
{
  std::vector<SegmentWindow> segs;
  auto samples = run(10.1, false, &segs);
  for (auto & s : samples) {
    if (s.t_s >= segs[5].t_start_s && s.t_s <= segs[5].t_end_s) {s.force_n = 20.0;}  // under the 60 N peak, off target
  }
  const auto r = verifyFromTelemetry(wall(), {}, samples, segs, contactOptions(), limitsForMode(RunMode::Contact));
  EXPECT_FALSE(r.passed);
  ASSERT_EQ(r.failed_segment_ids.size(), 1U);
  EXPECT_EQ(r.failed_segment_ids[0], 5U);
}

TEST(VerifyResult, SegmentWithNoWorkIsFailed)
{
  std::vector<SegmentWindow> segs;
  const auto samples = run(10.1, false, &segs);
  segs.push_back({99, 1000.0, 1010.0});   // a window the recording never reached
  const auto r = verifyFromTelemetry(wall(), {}, samples, segs, contactOptions(), limitsForMode(RunMode::Contact));
  EXPECT_FALSE(r.passed);
  ASSERT_EQ(r.failed_segment_ids.size(), 1U);
  EXPECT_EQ(r.failed_segment_ids[0], 99U);
}

TEST(VerifyResult, NonContactRunFailsOnAnyContactAbove5N)
{
  std::vector<SegmentWindow> segs;
  auto clean = run(0.0, true, &segs);
  const auto ok = verifyFromTelemetry(wall(), {}, clean, segs, nonContactOptions(), limitsForMode(RunMode::NonContact));
  EXPECT_TRUE(ok.passed) << (ok.failures.empty() ? "" : ok.failures.front());

  clean[40].force_n = 5.5;   // one brush against the wall
  const auto bad = verifyFromTelemetry(wall(), {}, clean, segs, nonContactOptions(), limitsForMode(RunMode::NonContact));
  EXPECT_FALSE(bad.passed);
  EXPECT_FALSE(bad.failures.empty());
  EXPECT_FALSE(bad.failed_segment_ids.empty());

  clean[40].force_n = 4.9;   // below the limit
  EXPECT_TRUE(verifyFromTelemetry(wall(), {}, clean, segs, nonContactOptions(), limitsForMode(RunMode::NonContact)).passed);
}

TEST(VerifyResult, LowCoverageFailsWithoutNamingSegments)
{
  std::vector<SegmentWindow> segs;
  auto samples = run(10.1, false, &segs);
  samples.resize(samples.size() / 2);   // the run stopped half way
  std::vector<SegmentWindow> done(segs.begin(), segs.begin() + 5);
  const auto r = verifyFromTelemetry(wall(), {}, samples, done, contactOptions(), limitsForMode(RunMode::Contact));
  EXPECT_FALSE(r.passed);
  EXPECT_LT(r.coverage_percent, 95.0);
  EXPECT_TRUE(r.failed_segment_ids.empty());
}
