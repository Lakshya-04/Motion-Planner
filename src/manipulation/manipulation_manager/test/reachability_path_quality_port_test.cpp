#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "manipulation_manager/adapters/reachability_path_quality_port.hpp"
#include "reachability/core/arm_model.hpp"

using namespace manipulation_manager;

namespace
{
// The generic 6-DOF reference arm (UR-like proportions), limits +-pi except where a test narrows them.
std::shared_ptr<reachability::ChainModel> arm(double lim1 = M_PI)
{
  using reachability::ChainJoint;
  using reachability::Vec3;
  auto joint = [](Vec3 xyz, Vec3 axis) {
      ChainJoint j;
      j.origin.translation() = xyz;
      j.axis = axis;
      j.lower = -M_PI;
      j.upper = M_PI;
      return j;
    };
  std::vector<ChainJoint> joints = {
    joint({0, 0, 0}, Vec3::UnitZ()),
    joint({0, 0, 0.18}, Vec3::UnitY()),
    joint({0, -0.12, 0.612}, Vec3::UnitY()),
    joint({0, 0, 0.572}, Vec3::UnitY()),
    joint({0, 0.12, 0}, Vec3::UnitZ()),
    joint({0, 0, 0.12}, Vec3::UnitY()),
  };
  joints[1].lower = -lim1;
  joints[1].upper = lim1;
  reachability::Pose tool = reachability::Pose::Identity();
  tool.translation() = Vec3(0, 0.06, 0);
  tool.linear() = Eigen::AngleAxisd(-M_PI / 2, Vec3::UnitX()).toRotationMatrix();
  reachability::Pose tcp = reachability::Pose::Identity();
  tcp.translation() = Vec3(0, 0, 0.165);
  return std::make_shared<reachability::ChainModel>(joints, tool * tcp);
}

const std::vector<double> kBent{0.3, -1.0, 1.6, -0.6, 0.4, 0.8};
const std::vector<double> kStretched{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

std::vector<double> withLift(double lift, const std::vector<double> & q)
{
  std::vector<double> v{lift};
  v.insert(v.end(), q.begin(), q.end());
  return v;
}
}  // namespace

TEST(ReachabilityPathQualityPort, BentColumnIsOkAndStretchedIsNearSingular)
{
  ReachabilityPathQualityPort port(arm(), reachability::PathQualityConfig{}, 1);
  const auto r = port.audit({
    {0, {withLift(0.4, kBent), withLift(0.4, kBent)}},
    {3, {withLift(0.4, kBent), withLift(0.4, kStretched), withLift(0.4, kBent)}}});
  ASSERT_TRUE(r.available);
  ASSERT_EQ(r.columns.size(), 2u);
  EXPECT_EQ(r.columns[0].column, 0u);
  EXPECT_EQ(r.columns[0].reason, "ok");
  EXPECT_EQ(r.columns[1].column, 3u);
  EXPECT_EQ(r.columns[1].reason, "near_singular");
  EXPECT_EQ(r.columns[1].worst_index, 1);
  EXPECT_EQ(r.near_singular, 1);
  EXPECT_EQ(r.near_limit, 0);
  EXPECT_LT(r.min_sigma, 1e-3);
  EXPECT_DOUBLE_EQ(r.sigma_threshold, 0.02);
}

TEST(ReachabilityPathQualityPort, WaypointNearAJointLimitIsNearLimit)
{
  // joint 2 limited to +-1.05 rad: q = -1.0 sits 0.05 from the limit, 2.4 % of the range
  reachability::PathQualityConfig cfg;
  cfg.min_limit_margin = 0.05;
  ReachabilityPathQualityPort port(arm(1.05), cfg, 1);
  const auto r = port.audit({{1, {withLift(0.0, kBent)}}});
  ASSERT_TRUE(r.available);
  EXPECT_EQ(r.columns[0].reason, "near_limit");
  EXPECT_EQ(r.near_limit, 1);
}

TEST(ReachabilityPathQualityPort, BareArmGroupNeedsNoSkip)
{
  ReachabilityPathQualityPort port(arm(), reachability::PathQualityConfig{}, 0);
  const auto r = port.audit({{0, {kBent}}});
  ASSERT_TRUE(r.available);
  EXPECT_EQ(r.columns[0].reason, "ok");
}

TEST(ReachabilityPathQualityPort, WrongSizedSamplesAreUnavailableNotAFalseOk)
{
  ReachabilityPathQualityPort port(arm(), reachability::PathQualityConfig{}, 0);
  const auto r = port.audit({{0, {withLift(0.4, kBent)}}});   // 7 values for a 6-joint chain, no skip
  EXPECT_FALSE(r.available);
  EXPECT_NE(r.note.find("no joint sample matched"), std::string::npos);
}

TEST(ReachabilityPathQualityPort, NothingRecordedOrNoModelSaysSo)
{
  ReachabilityPathQualityPort port(arm(), reachability::PathQualityConfig{}, 1);
  const auto none = port.audit({});
  EXPECT_FALSE(none.available);
  EXPECT_FALSE(none.note.empty());
  ReachabilityPathQualityPort nomodel(nullptr, reachability::PathQualityConfig{}, 1);
  const auto r = nomodel.audit({{0, {withLift(0.4, kBent)}}});
  EXPECT_FALSE(r.available);
  EXPECT_EQ(r.note, "no arm model");
}
