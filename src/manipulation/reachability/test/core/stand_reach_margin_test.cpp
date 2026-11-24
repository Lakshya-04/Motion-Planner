// Reach margin of stand placement on the tall-wall case of the two-stand MuJoCo run: an analytic sphere-shell
// reach model (every tool pose with the approach axis towards the wall is reachable within `reach` of the arm
// base), the first column group of the wall, a base that starts near the wall's end. Without the options the stand
// stays near the start pose (the drive is the tie-break) and the far columns sit at the edge of the reach; with the
// balance option the stand centres on its columns; the cap rejects poses beyond a fraction of the reach.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "reachability/core/stands.hpp"
#include "synthetic_map.hpp"

using namespace reachability;  // NOLINT
using namespace reachability_test;  // NOLINT

namespace
{
constexpr double kReach = 1.3;      // UR10e
constexpr double kWallX = 0.98;
constexpr double kMountZ = 1.17;    // arm base above the lift's zero

/// Sphere-shell map: every tool pose with a +x approach axis whose position is within `reach` of the arm base.
Map4D shellMap(double reach)
{
  Map4D map;
  const auto & p = map.params();
  const int itheta = map.keyOf(towardPlusX(0.5, 0.0, 0.0)).itheta;
  for (int iz = 0; iz < map.nz(); ++iz) {
    const double z = p.z_min + (iz + 0.5) * p.resolution;
    for (int ix = 0; ix < map.nxy(); ++ix) {
      const double x = -p.xy_extent + (ix + 0.5) * p.resolution;
      for (int iy = 0; iy < map.nxy(); ++iy) {
        const double y = -p.xy_extent + (iy + 0.5) * p.resolution;
        if (std::sqrt(x * x + y * y + z * z) <= reach) {map.raw()[map.index(iz, itheta, ix, iy)] = 5;}
      }
    }
  }
  return map;
}

const Map4D & shared()
{
  static const Map4D m = shellMap(kReach);
  return m;
}

/// First column group of the tall wall: columns at y 0.31 .. 0.81 (6 of the 10, span 0.5 m), z 0.6 .. 2.0.
StationRequest tallWallGroup()
{
  StationRequest r;
  for (int c = 0; c < 6; ++c) {
    for (double z = 0.6; z <= 2.0 + 1e-9; z += 0.1) {r.targets_world.push_back(towardPlusX(kWallX, 0.31 + 0.1 * c, z));}
  }
  r.wall_point = Vec3(kWallX, 0.0, 0.0);
  r.wall_normal = Vec3(-1.0, 0.0, 0.0);
  r.min_clearance = 0.42;
  r.arm_mount_offset_z = kMountZ;
  r.lift_min = 0.0;
  r.lift_max = 1.5;
  r.lift_step = 0.05;
  r.grid_step = 0.1;
  r.lateral_margin = 0.5;
  BaseModel b;
  b.drive = DriveType::kOmni;
  b.can_strafe = true;
  b.footprint = {{-0.4, -0.3}, {0.4, -0.3}, {0.4, 0.3}, {-0.4, 0.3}};
  r.base = b;
  r.from = Stand{Eigen::Vector2d(0.06, 0.01), 1.571};   // where the robot starts
  r.reach.reach_m = kReach;
  return r;
}

double columnMid(const StationRequest & r)
{
  double lo = 1e9, hi = -1e9;
  for (const auto & t : r.targets_world) {
    lo = std::min(lo, t.translation().y());
    hi = std::max(hi, t.translation().y());
  }
  return 0.5 * (lo + hi);
}
}  // namespace

TEST(StandReachMargin, DefaultsKeepTheStandNearTheStartPoseAndTheToolAtTheReachEdge)
{
  StationRequest r = tallWallGroup();
  r.reach.reach_m = 0.0;   // off
  const Station off = chooseStation(shared(), r);
  ASSERT_TRUE(off.complete());
  // the reproduction of the sim run: the stand is 0.3 m or more along the wall from the middle of its columns
  EXPECT_GT(std::abs(off.y - columnMid(r)), 0.3);
  EXPECT_LT(std::abs(off.y - 0.01), 0.25);   // still the stand nearest the start pose (grid step 0.1)
  // and the tool is at 94 % or more of the reach on the far columns
  StationRequest m = r;
  m.reach.reach_m = kReach;
  m.reach.balance = true;
  const auto plan = planLiftBands(shared(), r.targets_world, off.x, off.y, heightRange(0.0, 1.5, 0.05), kMountZ, m.reach);
  EXPECT_GT(plan.max_reach_fraction, 0.9);
  std::printf("off: stand (%.2f, %.2f) standoff %.2f worst reach fraction %.3f\n", off.x, off.y, off.standoff, plan.max_reach_fraction);
}

TEST(StandReachMargin, BalanceCentresTheStandOnItsColumnsAndLowersTheWorstFraction)
{
  StationRequest r = tallWallGroup();
  r.reach.balance = true;
  const Station st = chooseStation(shared(), r);
  ASSERT_TRUE(st.complete());
  EXPECT_NEAR(st.y, columnMid(r), 0.15);
  EXPECT_LT(st.plan.max_reach_fraction, 0.85);

  StationRequest base = r;
  base.reach.balance = false;
  const Station off = chooseStation(shared(), base);
  const auto off_plan = planLiftBands(shared(), r.targets_world, off.x, off.y, heightRange(0.0, 1.5, 0.05), kMountZ, r.reach);
  EXPECT_LT(st.plan.max_reach_fraction, off_plan.max_reach_fraction - 0.1);
  std::printf("balance: stand (%.2f, %.2f) standoff %.2f worst reach fraction %.3f\n", st.x, st.y, st.standoff, st.plan.max_reach_fraction);
  EXPECT_GE(st.standoff, r.min_clearance - 1e-9);   // the clearance floor is honoured
}

TEST(StandReachMargin, StartPoseIsOnlyATieBreak)
{
  // a start pose right at the centre or far away changes nothing about the margin: the same stand up to the
  // equal-margin tolerance, never a worse worst fraction
  StationRequest r = tallWallGroup();
  r.reach.balance = true;
  const Station a = chooseStation(shared(), r);
  r.from = Stand{Eigen::Vector2d(-0.5, 0.01), 1.571};
  const Station b = chooseStation(shared(), r);
  ASSERT_TRUE(a.complete());
  ASSERT_TRUE(b.complete());
  EXPECT_NEAR(a.plan.max_reach_fraction, b.plan.max_reach_fraction, kReachQuantum);
}

TEST(StandReachMargin, CapRejectsPosesBeyondTheFractionOfTheReach)
{
  StationRequest r = tallWallGroup();
  r.reach.max_fraction = 0.85;
  const Station st = chooseStation(shared(), r);
  ASSERT_TRUE(st.found);
  EXPECT_LE(st.plan.max_reach_fraction, 0.85 + 1e-9);
  // a cap nothing can meet leaves targets unreachable rather than quietly placing them
  r.reach.max_fraction = 0.3;
  const Station tight = chooseStation(shared(), r);
  EXPECT_FALSE(tight.found && tight.plan.unreachable.empty());
}

TEST(StandReachMargin, CapAlsoFiltersTheHeightTable)
{
  const StationRequest r = tallWallGroup();
  const auto heights = heightRange(0.0, 1.5, 0.05);
  ReachLimit cap;
  cap.reach_m = kReach;
  cap.max_fraction = 0.6;
  const auto open = reachableByHeight(shared(), r.targets_world, 0.06, 0.0, heights, kMountZ);
  const auto capped = reachableByHeight(shared(), r.targets_world, 0.06, 0.0, heights, kMountZ, cap);
  size_t n_open = 0, n_cap = 0;
  for (size_t i = 0; i < open.size(); ++i) {
    for (size_t h = 0; h < heights.size(); ++h) {
      n_open += open[i][h];
      n_cap += capped[i][h];
      if (capped[i][h]) {EXPECT_TRUE(open[i][h]);}
    }
  }
  EXPECT_LT(n_cap, n_open);
}

TEST(StandReachMargin, OffMeansUnchanged)
{
  // reach known but neither the cap nor balance requested: identical to a request that never heard of the option
  StationRequest r = tallWallGroup();
  StationRequest plain = r;
  plain.reach = ReachLimit{};
  const Station a = chooseStation(shared(), r);
  const Station b = chooseStation(shared(), plain);
  EXPECT_EQ(a.x, b.x);
  EXPECT_EQ(a.y, b.y);
  EXPECT_EQ(a.standoff, b.standoff);
  EXPECT_EQ(a.plan.bands.size(), b.plan.bands.size());
  EXPECT_EQ(a.plan.max_reach_fraction, 0.0);
}

TEST(StandReachMargin, EstimateReachOfTheShellMap)
{
  EXPECT_NEAR(shared().estimateReach(), kReach, 0.05);
  EXPECT_EQ(Map4D().estimateReach(), 0.0);
}
