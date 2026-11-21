// Stand planning on the map of a 7-DOF (redundant) arm: nothing in planStands, planColumnStands or the verifier
// depends on the joint count, so these tests pin that down on a Panda-kinematics chain (see redundancy_test) and
// show what the null-space policy of the map changes. RM4D estimates, not IK: the verifier is a fake.
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <memory>
#include <random>
#include <set>

#include "reachability/core/builder.hpp"
#include "reachability/core/column_stands.hpp"
#include "reachability/core/global_stands.hpp"
#include "reachability/core/io.hpp"
#include "reachability/core/stands.hpp"
#include "support.hpp"

using namespace reachability;  // NOLINT
using namespace reachability_test;  // NOLINT

namespace
{
constexpr double kWallX = 2.0;
constexpr double kMount = 0.2;   // arm base above the lift position (m)

MapParams sevenParams()
{
  MapParams p;
  p.xy_extent = 1.4;
  p.z_min = -1.4;
  p.z_max = 1.6;
  p.resolution = 0.1;
  p.theta_bin = M_PI / 9;
  return p;
}

Map4D build(const RedundancyPolicy & policy)
{
  static const ChainModel arm = sevenDofArm();
  BuildOptions o;
  o.samples = 2'000'000;
  o.threads = 4;
  o.redundancy = policy;
  return buildMap(arm, sevenParams(), o);
}

const Map4D & unionMap()
{
  static const Map4D m = build({});
  return m;
}

// Joint 3 held at 0 (+- 0.05): the restricted policy of docs/reachability.md.
const Map4D & restrictedMap()
{
  static const Map4D m = [] {
      RedundancyPolicy p;
      p.joint = 2;
      p.values = {0.0};
      p.tolerance = 0.05;
      return build(p);
    }();
  return m;
}

Pose towardWall(double y, double z)
{
  Pose T = Pose::Identity();
  T.translation() = Vec3(kWallX, y, z);
  T.linear() = Eigen::AngleAxisd(M_PI / 2, Vec3::UnitY()).toRotationMatrix();
  return T;
}

StationRequest patch(const std::vector<double> & ys, const std::vector<double> & zs)
{
  StationRequest r;
  for (double y : ys) {
    for (double z : zs) {r.targets_world.push_back(towardWall(y, z));}
  }
  r.wall_point = Vec3(kWallX, 0.0, 0.0);
  r.wall_normal = Vec3(-1.0, 0.0, 0.0);
  r.arm_mount_offset_z = kMount;
  r.min_clearance = 0.3;
  r.max_standoff = 1.0;
  r.lift_min = 0.0;
  r.lift_max = 1.2;
  r.lift_step = 0.1;
  r.grid_step = 0.1;
  r.lateral_margin = 0.2;
  return r;
}

// Independent of the planner: is target `t` reachable in the map from arm base (x, y) at lift height h (yaw 0)?
bool reachableFrom(const Map4D & map, const Pose & t, double x, double y, double lift)
{
  Pose in_arm = t;
  in_arm.translation() -= Vec3(x, y, lift + kMount);
  return map.isReachable(in_arm);
}

TargetVerdict ok7()
{
  TargetVerdict v;
  v.reason = VerifyReason::kOk;
  v.joints = {0.1, -0.4, 0.0, -1.5, 0.2, 1.5, 0.3};
  return v;
}
}  // namespace

TEST(StandsRedundant, WallPatchGetsStandsAndEveryAssignmentHoldsInTheUnionMap)
{
  const auto r = patch({-0.2, 0.0, 0.2}, {0.5, 0.8, 1.1});
  const StandPlan plan = planStands(unionMap(), r);
  ASSERT_FALSE(plan.stands.empty());
  EXPECT_TRUE(plan.unreachable.empty());
  EXPECT_TRUE(plan.not_planned.empty());
  std::set<size_t> seen;
  for (const auto & a : plan.stands) {
    for (const auto & b : a.station.plan.bands) {
      for (size_t i : b.pose_indices) {
        EXPECT_TRUE(reachableFrom(unionMap(), r.targets_world[i], a.stand.arm_xy.x(), a.stand.arm_xy.y(), b.lift_height))
          << "target " << i;
        seen.insert(i);
      }
    }
  }
  EXPECT_EQ(seen.size(), r.targets_world.size());
}

TEST(StandsRedundant, SetCoverAndGreedyBothWorkOnASevenDofMap)
{
  const auto r = patch({-0.6, -0.2, 0.2, 0.6}, {0.5, 0.9});
  const StandPlan g = planStands(unionMap(), r, std::nullopt, 32, StandStrategy::Greedy);
  const StandPlan s = planStands(unionMap(), r, std::nullopt, 32, StandStrategy::SetCover);
  EXPECT_TRUE(g.unreachable.empty());
  EXPECT_TRUE(s.unreachable.empty());
  EXPECT_LE(s.stands.size(), g.stands.size());
}

TEST(StandsRedundant, RestrictedMapIsASubsetOfTheUnionAndNeedsAtLeastAsManyStands)
{
  EXPECT_LT(restrictedMap().nonzeroCells(), unionMap().nonzeroCells());
  std::printf("cells: union %zu restricted %zu\n", unionMap().nonzeroCells(), restrictedMap().nonzeroCells());
  // Poses the arm reaches with joint 3 pinned at 0 are in both maps; poses reached only with joint 3 far from
  // 0 (the elbow swung out) are in the union and mostly not in the restricted map.
  const ChainModel arm = sevenDofArm();
  std::mt19937 rng(21);
  std::vector<double> q(7);
  Pose T;
  int union_hits = 0, union_hits_total = 0, in_union_only = 0, pinned_in_both = 0, pinned_total = 0, swung_total = 0;
  for (int i = 0; i < 20000; ++i) {
    for (int j = 0; j < 7; ++j) {q[j] = std::uniform_real_distribution<double>(arm.lower(j), arm.upper(j))(rng);}
    const bool pinned = i % 2 == 0;
    q[2] = pinned ? 0.0 : (i % 4 == 1 ? 2.0 : -2.0);
    if (!arm.evaluate(q, T) || !unionMap().keyOf(T).valid) {continue;}
    ++union_hits_total;
    union_hits += unionMap().isReachable(T) ? 1 : 0;
    if (pinned) {
      ++pinned_total;
      pinned_in_both += restrictedMap().isReachable(T) ? 1 : 0;
    } else {
      ++swung_total;
      in_union_only += restrictedMap().isReachable(T) ? 0 : 1;
    }
  }
  // The union holds almost every fresh sample (a rare one falls in a cell the build did not draw).
  EXPECT_GT(static_cast<double>(union_hits) / union_hits_total, 0.95);
  EXPECT_GT(static_cast<double>(pinned_in_both) / pinned_total, 0.9);
  // The 4D cells (0.1 m, 20 degrees) are coarse and the null space overlaps a lot, so the restriction removes
  // only a small share of the poses an elbow-swung arm reaches: measured about 1 percent here, nonzero.
  const double share = static_cast<double>(in_union_only) / swung_total;
  std::printf("swung-elbow poses missing from the restricted map: %.3f\n", share);
  EXPECT_GT(share, 0.001);

  // A wall patch the union covers is never covered with fewer stands by the restricted map.
  const auto r = patch({-0.6, -0.2, 0.2, 0.6}, {0.4, 0.8, 1.2});
  const StandPlan u = planStands(unionMap(), r);
  const StandPlan n = planStands(restrictedMap(), r);
  EXPECT_TRUE(u.unreachable.empty());
  EXPECT_GE(n.unreachable.size() + n.stands.size(), u.unreachable.size() + u.stands.size());
  EXPECT_GE(n.unreachable.size(), u.unreachable.size());
}

TEST(StandsRedundant, ColumnsStayWholeUnderTheUnionOverTheNullSpace)
{
  std::vector<ColumnPlan> cols;
  for (double y : {-0.2, 0.0, 0.2}) {
    ColumnPlan c;
    c.label = "col" + std::to_string(y);
    for (double z = 0.6; z <= 1.0 + 1e-9; z += 0.1) {c.strokes_world.push_back(towardWall(y, z));}
    cols.push_back(c);
  }
  StationRequest station = patch({0.0}, {0.8});
  station.targets_world.clear();
  ColumnStandOptions o;
  o.sequence.standoff_m = 0.10;
  o.sequence.approach_extra_m = 0.10;
  o.sequence.press_depth_m = 0.02;
  o.margin.min_score = 0.0;
  const auto plan = planColumnStands(unionMap(), station, cols, std::nullopt, o);
  ASSERT_TRUE(plan.success) << plan.message;
  EXPECT_EQ(plan.whole, cols.size());
  EXPECT_EQ(plan.split, 0U);
  for (const auto & c : plan.columns) {EXPECT_EQ(c.strokes_covered, c.strokes_total);}
}

TEST(StandsRedundant, VerifiedPlanKeepsSevenJointSolutionsAndRoutesAroundARefusingStand)
{
  const auto r = patch({-0.2, 0.0, 0.2}, {0.5, 0.8, 1.1});
  // accept everything: same coverage as the plain plan, 7 joints preserved in every report
  VerifiedPlanOptions all;
  all.verifier = std::make_shared<FunctionStandVerifier>([](const Stand &, double, const Pose &) {return ok7();});
  const VerifiedStandPlan ok = planStandsVerified(unionMap(), r, std::nullopt, 32, StandStrategy::Greedy, {}, all);
  ASSERT_FALSE(ok.plan.stands.empty());
  EXPECT_TRUE(ok.plan.unreachable.empty());
  EXPECT_TRUE(ok.rejected_by_verifier.empty());
  ASSERT_EQ(ok.verification.size(), ok.plan.stands.size());
  for (const auto & v : ok.verification) {
    for (const auto & t : v.targets) {
      if (t.confirmed) {EXPECT_EQ(t.joints.size(), 7U);}
    }
  }

  // an IK that fails at the first stand's position: another stand takes over, all targets still confirmed
  const Eigen::Vector2d bad = ok.plan.stands.front().stand.arm_xy;
  VerifiedPlanOptions some;
  some.verifier = std::make_shared<FunctionStandVerifier>([bad](const Stand & s, double, const Pose &) {
      if ((s.arm_xy - bad).norm() < 1e-6) {
        TargetVerdict v;
        v.reason = VerifyReason::kInCollision;
        return v;
      }
      return ok7();
    });
  const VerifiedStandPlan re = planStandsVerified(unionMap(), r, std::nullopt, 32, StandStrategy::Greedy, {}, some);
  EXPECT_TRUE(re.rejected_by_verifier.empty());
  EXPECT_FALSE(re.refused.empty());
  for (const auto & a : re.plan.stands) {EXPECT_GT((a.stand.arm_xy - bad).norm(), 1e-6);}

  // an IK that never solves: nothing is claimed
  VerifiedPlanOptions none;
  none.verifier = std::make_shared<FunctionStandVerifier>([](const Stand &, double, const Pose &) {
      TargetVerdict v;
      v.reason = VerifyReason::kIkFailed;
      return v;
    });
  const VerifiedStandPlan nope = planStandsVerified(unionMap(), r, std::nullopt, 32, StandStrategy::Greedy, {}, none);
  EXPECT_TRUE(nope.plan.stands.empty());
  EXPECT_EQ(nope.rejected_by_verifier.size(), r.targets_world.size());
}

TEST(StandsRedundant, SelectStandsWithAVerifierWorksOnASevenDofMap)
{
  const auto r = patch({-0.2, 0.0, 0.2}, {0.5, 0.8, 1.1});
  StandSelectionOptions opt;
  opt.verifier = std::make_shared<FunctionStandVerifier>([](const Stand &, double, const Pose &) {return ok7();});
  const StandSelection sel = selectStands(unionMap(), r, std::nullopt, opt);
  EXPECT_TRUE(sel.success) << sel.message;
  EXPECT_FALSE(sel.stands.empty());
  EXPECT_TRUE(sel.verification.enabled);
}

TEST(StandsRedundant, MapInfoDescriptionSaysWhichNullSpaceSamplesWereUsed)
{
  MapInfo info;
  EXPECT_NE(describeMapInfo(info).find("not recorded"), std::string::npos);
  info.arm_dof = 7;
  info.redundancy = "joint 3 in {0} +- 0.05 rad";
  EXPECT_EQ(describeMapInfo(info), "7 joints, redundancy: joint 3 in {0} +- 0.05 rad");
  info.redundancy.clear();
  EXPECT_EQ(describeMapInfo(info), "7 joints, redundancy: not recorded");
}
