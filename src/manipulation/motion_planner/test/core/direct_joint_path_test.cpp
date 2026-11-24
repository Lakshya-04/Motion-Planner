#include <gtest/gtest.h>

#include "motion_planner/core/direct_joint_path.hpp"

namespace motion_planner
{
namespace
{

TEST(DirectJointPath, DensifyRespectsPerJointStepAndEndpoints)
{
  DirectJointPathConfig cfg;
  cfg.prismatic = {true, false};
  const auto s = densify({0.0, 0.0}, {0.3, 0.1}, cfg);  // lift 0.3 m / 0.01 = 30 segments
  ASSERT_EQ(s.size(), 31u);
  EXPECT_EQ(s.front(), (std::vector<double>{0.0, 0.0}));
  EXPECT_EQ(s.back(), (std::vector<double>{0.3, 0.1}));
  for (std::size_t i = 1; i < s.size(); ++i) {
    EXPECT_LE(std::fabs(s[i][0] - s[i - 1][0]), 0.01 + 1e-9);
    EXPECT_LE(std::fabs(s[i][1] - s[i - 1][1]), 0.05 + 1e-9);
  }
}

TEST(DirectJointPath, OkWhenAllSamplesValid)
{
  int calls = 0;
  const auto r = buildDirectJointPath(
    {0, 0, 0}, {1.0, -1.0, 0.5}, DirectJointPathConfig{},
    [&](const std::vector<double> &) {++calls; return true;});
  EXPECT_EQ(r.status, DirectJointPathResult::Status::kOk);
  EXPECT_EQ(r.samples.size(), 21u);  // 1.0 / 0.05
  EXPECT_EQ(static_cast<std::size_t>(calls), r.samples.size());
}

TEST(DirectJointPath, StopsAtFirstInvalidSample)
{
  int calls = 0;
  const auto r = buildDirectJointPath(
    {0.0}, {1.0}, DirectJointPathConfig{},
    [&](const std::vector<double> & q) {++calls; return q[0] < 0.5;});
  EXPECT_EQ(r.status, DirectJointPathResult::Status::kInvalidSample);
  EXPECT_EQ(r.invalid_index, 10u);
  EXPECT_EQ(calls, 11);
}

TEST(DirectJointPath, InvalidStartRefuses)
{
  const auto r = buildDirectJointPath(
    {0.0}, {1.0}, DirectJointPathConfig{}, [](const std::vector<double> &) {return false;});
  EXPECT_EQ(r.status, DirectJointPathResult::Status::kInvalidSample);
  EXPECT_EQ(r.invalid_index, 0u);
}

TEST(DirectJointPath, NoMotionAndBadInput)
{
  auto yes = [](const std::vector<double> &) {return true;};
  EXPECT_EQ(
    buildDirectJointPath({1.0}, {1.0}, DirectJointPathConfig{}, yes).status,
    DirectJointPathResult::Status::kNoMotion);
  EXPECT_EQ(
    buildDirectJointPath({1.0}, {1.0, 2.0}, DirectJointPathConfig{}, yes).status,
    DirectJointPathResult::Status::kBadInput);
  EXPECT_EQ(
    buildDirectJointPath({1.0}, {NAN}, DirectJointPathConfig{}, yes).status,
    DirectJointPathResult::Status::kBadInput);
  EXPECT_EQ(
    buildDirectJointPath({1.0}, {2.0}, DirectJointPathConfig{}, nullptr).status,
    DirectJointPathResult::Status::kBadInput);
  DirectJointPathConfig bad;
  bad.revolute_step = 0.0;
  EXPECT_EQ(buildDirectJointPath({1.0}, {2.0}, bad, yes).status, DirectJointPathResult::Status::kBadInput);
}

TEST(DirectJointPath, DurationIsSlowestJoint)
{
  // joint 0: 2 rad at v=1, a=2 -> d_ramp 0.5 <= 2: 2/1 + 1/2 = 2.5 s. joint 1: tiny.
  EXPECT_NEAR(slowestJointDuration({0, 0}, {2.0, 0.01}, {1.0, 1.0}, {2.0, 2.0}), 2.5, 1e-9);
  // Triangle profile: d=0.25, v=10, a=1 -> 2*sqrt(0.25) = 1 s.
  EXPECT_NEAR(restToRestDuration(0.25, 10.0, 1.0), 1.0, 1e-9);
  EXPECT_EQ(restToRestDuration(0.0, 1.0, 1.0), 0.0);
}

}  // namespace
}  // namespace motion_planner
