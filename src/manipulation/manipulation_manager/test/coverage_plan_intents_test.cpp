// The ROS CoverageSource's conversion, validation and wait, on hand-built plans (the field set of
// coverage_planner_msgs/CoveragePlan, as a plain struct). The same plan through the JSON source gives the same intents.
#include <cstdio>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "manipulation_manager/core/coverage_plan_intents.hpp"
#include "manipulation_manager/ros/coverage/coverage_output_json_source.hpp"

namespace manipulation_manager
{
namespace
{
core_types::Pose pose(double x, double y, double z)
{
  core_types::Pose p;
  p.x = x; p.y = y; p.z = z; p.qx = 0.5; p.qy = 0.5; p.qz = 0.5; p.qw = 0.5;
  return p;
}

RawCoveragePlan twoColumnPlan()
{
  RawCoveragePlan plan;
  plan.frame_id = "base_link";
  plan.poses = {pose(-0.1, 1.0, 1.2), pose(-0.1, 1.0, 1.4), pose(0.3, 1.0, 1.4), pose(0.3, 1.0, 1.2)};
  plan.segments = {{0, 1}, {2, 3}};
  return plan;
}

CoverageIntentOptions opts()
{
  CoverageIntentOptions o;
  o.z_step_m = 0.1; o.z_min_m = 1.0; o.z_max_m = 2.0;
  return o;
}
}  // namespace

TEST(CoveragePlanIntentsTest, ColumnsAreOrderedAndResampled)
{
  const auto intents = buildCoverageIntents(twoColumnPlan(), opts());
  ASSERT_EQ(intents.size(), 2U);
  EXPECT_NEAR(intents[0].poses.front().x, -0.1, 1e-9);
  EXPECT_NEAR(intents[1].poses.front().x, 0.3, 1e-9);
  ASSERT_EQ(intents[0].poses.size(), 3U);  // 1.2, 1.3, 1.4
  EXPECT_NEAR(intents[0].poses.front().z, 1.2, 1e-9);
  EXPECT_NEAR(intents[0].poses.back().z, 1.4, 1e-9);
  EXPECT_EQ(intents[0].mode_hint, "cartesian");
  EXPECT_TRUE(intents[0].requires_contact);
}

TEST(CoveragePlanIntentsTest, ZOffsetAndFollowPlanDirectionApply)
{
  auto o = opts();
  o.z_offset = 0.31;
  o.follow_plan_direction = true;
  const auto intents = buildCoverageIntents(twoColumnPlan(), o);
  ASSERT_EQ(intents.size(), 2U);
  EXPECT_NEAR(intents[0].poses.front().z, 1.51, 1e-9);  // column 0 goes up
  EXPECT_GT(intents[1].poses.front().z, intents[1].poses.back().z);  // column 1 runs down, as planned
}

TEST(CoveragePlanIntentsTest, SameIntentsAsTheJsonSourceForTheSamePlan)
{
  const std::string path = ::testing::TempDir() + "same_plan.json";
  {
    std::ofstream f(path);
    f << R"({"final_lifted_and_expanded":{"path":{"poses":[)";
    const auto plan = twoColumnPlan();
    for (size_t i = 0; i < plan.poses.size(); ++i) {
      const auto & p = plan.poses[i];
      f << (i ? "," : "") << R"({"position":{"x":)" << p.x << R"(,"y":)" << p.y << R"(,"z":)" << p.z
        << R"(},"orientation":{"x":0.5,"y":0.5,"z":0.5,"w":0.5}})";
    }
    f << R"(]},"segments":[{"start_index":0,"end_index":1},{"start_index":2,"end_index":3}]}})";
  }
  CoverageOutputJsonSource json(path, "base_footprint", 0.0, 0.1, 1.0, 2.0);
  const auto a = json.getCoveragePlan();
  const auto b = buildCoverageIntents(twoColumnPlan(), opts());
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) {
    ASSERT_EQ(a[i].poses.size(), b[i].poses.size());
    EXPECT_EQ(a[i].label, b[i].label);
    for (size_t k = 0; k < a[i].poses.size(); ++k) {
      EXPECT_NEAR(a[i].poses[k].x, b[i].poses[k].x, 1e-9);
      EXPECT_NEAR(a[i].poses[k].z, b[i].poses[k].z, 1e-9);
    }
  }
  std::remove(path.c_str());
}

TEST(CoveragePlanIntentsTest, ValidationRejectsFailureEmptyAndBadIndices)
{
  auto plan = twoColumnPlan();
  EXPECT_NO_THROW(validateCoveragePlan(plan));
  auto failed = plan;
  failed.success = false;
  failed.message = "no free cells";
  try {
    validateCoveragePlan(failed);
    FAIL();
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("no free cells"), std::string::npos);
  }
  RawCoveragePlan empty;
  EXPECT_THROW(validateCoveragePlan(empty), std::runtime_error);
  auto bad = plan;
  bad.segments.push_back({3, 9});
  EXPECT_THROW(validateCoveragePlan(bad), std::runtime_error);
  auto reversed = plan;
  reversed.segments = {{2, 1}};
  EXPECT_THROW(validateCoveragePlan(reversed), std::runtime_error);
}

TEST(CoveragePlanIntentsTest, FingerprintIsStableAndChangesWithThePlan)
{
  auto a = twoColumnPlan();
  auto b = twoColumnPlan();
  EXPECT_EQ(coveragePlanFingerprint(a), coveragePlanFingerprint(b));
  b.poses[0].z += 0.01;
  EXPECT_NE(coveragePlanFingerprint(a), coveragePlanFingerprint(b));
}

TEST(CoveragePlanInboxTest, TimesOutWhenNothingArrives)
{
  CoveragePlanInbox inbox;
  RawCoveragePlan out;
  int pumps = 0;
  EXPECT_FALSE(inbox.waitFor(0.05, [&]() {++pumps;}, out));
  EXPECT_GE(pumps, 1);
}

TEST(CoveragePlanInboxTest, ReturnsAPlanThatArrivesWhileWaitingAndKeepsIt)
{
  CoveragePlanInbox inbox;
  RawCoveragePlan out;
  int pumps = 0;
  // The "spin" delivers the plan on its third call, as a subscription callback would.
  EXPECT_TRUE(inbox.waitFor(2.0, [&]() {if (++pumps == 3) {inbox.put(twoColumnPlan());}}, out));
  EXPECT_EQ(out.poses.size(), 4U);
  RawCoveragePlan again;
  EXPECT_TRUE(inbox.waitFor(0.0, nullptr, again));  // still there, no pump needed
  EXPECT_EQ(again.poses.size(), 4U);
}

TEST(CoveragePlanInboxTest, ALaterPlanReplacesTheEarlierOne)
{
  CoveragePlanInbox inbox;
  inbox.put(twoColumnPlan());
  auto second = twoColumnPlan();
  second.poses.pop_back();
  second.segments = {{0, 1}};
  inbox.put(second);
  RawCoveragePlan out;
  ASSERT_TRUE(inbox.waitFor(0.0, nullptr, out));
  EXPECT_EQ(out.poses.size(), 3U);
}

}  // namespace manipulation_manager
