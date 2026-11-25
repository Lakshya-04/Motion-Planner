// Property / fuzz tests of the base station planning on an ANALYTIC reach map (test/core/synthetic_map.hpp: a pose is
// reachable exactly inside a known box, so every verdict can be checked against geometry): seeded random walls,
// targets and requests, with NaN / inf / zero / negative request fields. Invariants: the station is on the free side
// of the wall inside [min_clearance, max_standoff]; every target is either in one lift band or unreachable, with the
// band heights inside the lift range; targets the plan calls reachable are analytically inside the reach box (with a
// quantisation margin) and unreachable ones are not clearly inside it; the chosen station is the best of the grid
// (fewest unreachable, then fewest bands); a request designed to be feasible is found complete; an invalid request is
// refused with kInvalidRequest and never hangs.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

#include "fuzz_util.hpp"
#include "reachability/core/lift_bands.hpp"
#include "reachability/core/station.hpp"
#include "synthetic_map.hpp"

using namespace reachability;  // NOLINT
using fuzz::Rng;

namespace
{
const Map4D & analyticMap()
{
  static const Map4D map = reachability_test::syntheticMap();
  return map;
}

// analytic reachability of a +x-pointing tool at (xs, ys, zs) relative to the arm base, with a margin m: +m = clearly
// inside the box, -m = clearly outside
bool insideBox(double xs, double ys, double zs, double m)
{
  const reachability_test::SyntheticReach r;
  return xs >= r.x_lo + m && xs < r.x_hi - r.edge_band - m && std::fabs(ys) <= r.y_half - m && zs >= r.z_lo + m &&
         zs <= r.z_hi - m;
}
bool outsideBox(double xs, double ys, double zs, double m)
{
  const reachability_test::SyntheticReach r;
  return xs < r.x_lo - m || xs >= r.x_hi + m || std::fabs(ys) > r.y_half + m || zs < r.z_lo - m || zs > r.z_hi + m;
}

Pose target(double x, double y, double z) {return reachability_test::towardPlusX(x, y, z);}
}  // namespace

TEST(StationFuzz, StationsAreOnTheFreeSideAndPlansMatchTheAnalyticMap)
{
  Rng rng(0x7EA0001);
  const int cases = 2500;
  int found_n = 0, complete_n = 0, invalid_n = 0;
  for (int c = 0; c < cases; ++c) {
    SCOPED_TRACE("case " + std::to_string(c));
    StationRequest q;
    const double W = rng.uni(1.0, 3.0);
    q.wall_point = Vec3(W, rng.uni(-0.5, 0.5), 0.0);
    q.wall_normal = Vec3(-1.0, 0.0, 0.0);
    const int nt = rng.integer(0, 5);
    for (int i = 0; i < nt; ++i) {
      q.targets_world.push_back(target(W, rng.wild(-0.8, 0.8, 0.01) + q.wall_point.y(), rng.wild(0.0, 1.8, 0.01)));
    }
    q.min_clearance = rng.wild(0.1, 0.8, 0.03);
    q.max_standoff = rng.wild(0.8, 1.8, 0.03);
    q.grid_step = rng.wild(0.1, 0.3, 0.04);
    q.lateral_margin = rng.wild(0.0, 0.6, 0.03);
    q.lift_min = rng.uni(0.0, 0.3);
    q.lift_max = q.lift_min + rng.uni(0.0, 1.4);
    q.lift_step = rng.chance(0.03) ? rng.wild(0.0, 0.0, 1.0) : rng.uni(0.05, 0.2);
    q.arm_mount_offset_z = rng.uni(0.0, 0.4);
    if (rng.chance(0.02)) {q.wall_normal = Vec3(rng.wild(0, 0, 1.0), 0.0, 0.0);}
    if (rng.chance(0.15)) {
      Eigen::Vector2d a(W - rng.uni(0.4, 1.0), q.wall_point.y());
      q.avoid_stations.push_back(a);
      q.avoid_radius = rng.uni(0.0, 0.3);
    }
    const Station s = chooseStation(analyticMap(), q);
    ASSERT_EQ(s.found, s.code == StationCode::kOk);
    if (!s.found) {
      if (s.code == StationCode::kInvalidRequest) {++invalid_n;}
      ASSERT_TRUE(s.plan.bands.empty());
      continue;
    }
    ++found_n;
    ASSERT_FALSE(q.targets_world.empty());
    ASSERT_TRUE(std::isfinite(s.x) && std::isfinite(s.y));
    // free side: signed distance from the wall plane along the normal
    const double d = (Vec3(s.x, s.y, 0.0) - q.wall_point).dot(q.wall_normal.normalized());
    ASSERT_NEAR(d, s.standoff, 1e-9);
    ASSERT_GE(d, q.min_clearance - 1e-9);
    ASSERT_LE(d, q.max_standoff + 1e-9);
    ASSERT_GT(d, 0.0);
    ASSERT_LT(s.x, W);
    for (const auto & a : q.avoid_stations) {ASSERT_GT((a - Eigen::Vector2d(s.x, s.y)).norm(), q.avoid_radius);}
    // the plan partitions the targets
    std::set<size_t> seen;
    for (const auto & b : s.plan.bands) {
      ASSERT_GE(b.lift_height, q.lift_min - 1e-9);
      ASSERT_LE(b.lift_height, q.lift_max + 1e-9);
      ASSERT_FALSE(b.pose_indices.empty());
      for (const size_t i : b.pose_indices) {
        ASSERT_LT(i, q.targets_world.size());
        ASSERT_TRUE(seen.insert(i).second) << "target " << i << " in two bands";
        // the analytic map: the target is not clearly outside the box at this band's lift
        const Vec3 t = q.targets_world[i].translation();
        ASSERT_FALSE(outsideBox(t.x() - s.x, t.y() - s.y, t.z() - (b.lift_height + q.arm_mount_offset_z), 0.08))
          << "band target " << i << " is outside the reach box";
      }
    }
    for (const size_t i : s.plan.unreachable) {
      ASSERT_LT(i, q.targets_world.size());
      ASSERT_TRUE(seen.insert(i).second);
      // unreachable: not clearly inside the box at any height of the lift range
      const Vec3 t = q.targets_world[i].translation();
      for (double h = q.lift_min; h <= q.lift_max + 1e-9; h += 0.01) {
        ASSERT_FALSE(insideBox(t.x() - s.x, t.y() - s.y, t.z() - (h + q.arm_mount_offset_z), 0.08))
          << "target " << i << " called unreachable but clearly inside the box at lift " << h;
      }
    }
    ASSERT_EQ(seen.size(), q.targets_world.size());
    ASSERT_EQ(s.plan.reachable_count + s.plan.unreachable.size(), q.targets_world.size());
    ASSERT_GT(s.plan.reachable_count, 0u);
    ASSERT_EQ(s.complete(), s.plan.unreachable.empty());
    complete_n += s.complete() ? 1 : 0;
    // the chosen station is the best of the grid: no other grid point has fewer unreachable targets, or the same
    // number and fewer bands
    const auto heights = heightRange(q.lift_min, q.lift_max, q.lift_step);
    for (const auto & g : stationGrid(q)) {
      const auto p = planLiftBands(analyticMap(), q.targets_world, g.arm_xy.x(), g.arm_xy.y(), heights, q.arm_mount_offset_z);
      if (p.reachable_count == 0) {continue;}
      const auto k0 = std::make_pair(s.plan.unreachable.size(), s.plan.bands.size());
      const auto k1 = std::make_pair(p.unreachable.size(), p.bands.size());
      ASSERT_FALSE(k1 < k0) << "a better station exists at " << g.arm_xy.x() << ", " << g.arm_xy.y();
    }
  }
  RecordProperty("found", found_n);
  EXPECT_GT(found_n, 1000);
  EXPECT_GT(complete_n, 300);
  EXPECT_GT(invalid_n, 20);
}

TEST(StationFuzz, AFeasibleRequestIsFoundCompleteAndOnTheFreeSide)
{
  Rng rng(0x7EA0002);
  for (int c = 0; c < 1500; ++c) {
    SCOPED_TRACE("case " + std::to_string(c));
    StationRequest q;
    const double W = rng.uni(1.0, 3.0);
    const double cy = rng.uni(-1.0, 1.0);
    q.wall_point = Vec3(W, cy, 0.0);
    q.wall_normal = Vec3(-1.0, 0.0, 0.0);
    // targets within 0.5 m along the wall and 0.4 m in height: one stand at standoff 0.45..0.75 and some lift reaches all
    const double z0 = rng.uni(0.3, 1.2);
    for (int i = 0; i < rng.integer(1, 5); ++i) {
      q.targets_world.push_back(target(W, cy + rng.uni(-0.25, 0.25), z0 + rng.uni(0.0, 0.4)));
    }
    q.min_clearance = rng.uni(0.2, 0.45);
    q.max_standoff = rng.uni(0.8, 1.6);
    q.grid_step = rng.uni(0.1, 0.2);
    q.lateral_margin = rng.uni(0.1, 0.5);
    q.lift_min = 0.0;
    q.lift_max = 1.5;
    q.lift_step = 0.05;
    q.arm_mount_offset_z = rng.uni(0.0, 0.3);
    const Station s = chooseStation(analyticMap(), q);
    ASSERT_TRUE(s.found);
    ASSERT_TRUE(s.complete()) << s.plan.unreachable.size() << " targets unreachable";
    ASSERT_LT(s.x, W);
    ASSERT_GE(s.standoff, q.min_clearance - 1e-9);
    ASSERT_LE(s.standoff, q.max_standoff + 1e-9);
    // analytically reachable at the standoff
    ASSERT_TRUE(s.standoff >= 0.30 - 0.06 && s.standoff < 0.90) << s.standoff;
  }
}

TEST(StationFuzz, InvalidRequestsAreRefusedQuicklyAndNeverFound)
{
  Rng rng(0x7EA0003);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (int c = 0; c < 3000; ++c) {
    SCOPED_TRACE("case " + std::to_string(c));
    StationRequest q;
    q.wall_point = Vec3(2.0, 0.0, 0.0);
    q.wall_normal = Vec3(-1.0, 0.0, 0.0);
    q.targets_world.push_back(target(2.0, 0.0, 0.8));
    q.arm_mount_offset_z = 0.2;
    q.grid_step = 0.2;
    switch (c % 12) {
      case 0: q.grid_step = nan; break;
      case 1: q.grid_step = 0.0; break;
      case 2: q.grid_step = -0.1; break;
      case 3: q.grid_step = 1e-9; break;      // a grid of 1e9 points
      case 4: q.max_standoff = inf; break;
      case 5: q.min_clearance = nan; break;
      case 6: q.lateral_margin = inf; break;
      case 7: q.wall_normal = Vec3(0, 0, 0); break;
      case 8: q.targets_world.clear(); break;
      case 9: q.targets_world[0].translation() = Vec3(nan, 0.0, 0.8); break;
      case 10: q.wall_normal = Vec3(nan, 0.0, 0.0); break;
      default: q.max_standoff = 1e18; break;
    }
    const Station s = chooseStation(analyticMap(), q);
    ASSERT_FALSE(s.found);
    ASSERT_EQ(s.code, StationCode::kInvalidRequest) << "case kind " << c % 12;
  }
}
