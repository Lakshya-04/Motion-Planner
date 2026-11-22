#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "standoff_advisor/standoff_sweep.hpp"

using namespace standoff_advisor;  // NOLINT
using reachability::Map4D;

namespace
{
// A hand-made map: for a tool whose approach axis is +x (toward a wall on the +x side) every cell with
// x* in [x_lo, x_hi), |y*| <= y_half and tool height z in [z_lo, z_hi] above the arm base holds `count`,
// the outer `edge_band` of x* holds `edge_count`. Nothing else is reachable.
struct Reach
{
  double x_lo{0.50}, x_hi{1.10};
  double y_half{0.80};
  double z_lo{-0.50}, z_hi{0.70};
  int count{20};
  double edge_band{0.10};
  int edge_count{1};
};

Map4D fixture(const Reach & r = {})
{
  Map4D map;
  const auto & p = map.params();
  reachability::Pose T = reachability::Pose::Identity();
  T.translation() = Vec3(0.5, 0.0, 0.0);
  T.linear() = Eigen::AngleAxisd(M_PI / 2, Vec3::UnitY()).toRotationMatrix();
  const int itheta = map.keyOf(T).itheta;
  for (int iz = 0; iz < map.nz(); ++iz) {
    const double z = p.z_min + (iz + 0.5) * p.resolution;
    if (z < r.z_lo || z > r.z_hi) {continue;}
    for (int ix = 0; ix < map.nxy(); ++ix) {
      const double x = -p.xy_extent + (ix + 0.5) * p.resolution;
      if (x < r.x_lo || x >= r.x_hi) {continue;}
      for (int iy = 0; iy < map.nxy(); ++iy) {
        const double y = -p.xy_extent + (iy + 0.5) * p.resolution;
        if (std::fabs(y) > r.y_half) {continue;}
        map.raw()[map.index(iz, itheta, ix, iy)] =
          static_cast<uint8_t>(x >= r.x_hi - r.edge_band || x < r.x_lo + r.edge_band ? r.edge_count : r.count);
      }
    }
  }
  return map;
}

// Wall at x = 3 facing the robot (-x); targets 0.3 m wide, 0.4 m high, tool z in 0.0..0.4 (arm base at z 0).
StandoffSweepRequest request()
{
  StandoffSweepRequest q;
  q.wall_point = Vec3(3.0, 0.0, 0.0);
  q.wall_normal = Vec3(-1.0, 0.0, 0.0);
  q.patch = WallPatch{Vec3(3.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0), Vec3(0.0, 0.0, 1.0), -0.15, 0.15, 0.0, 0.4, 0.1};
  q.lift_min = q.lift_max = 0.0;
  q.lift_step = 0.05;
  q.arm_mount_offset_z = 0.0;
  q.standoff_min = 0.30;
  q.standoff_max = 1.60;
  q.standoff_step = 0.02;
  q.min_clearance = 0.30;
  q.nav_tolerance_m = 0.05;
  return q;
}
}  // namespace

TEST(Standoff, PatchTargetsPointIntoTheWall)
{
  const auto q = request();
  const auto t = patchTargets(*q.patch, q.wall_normal);
  ASSERT_EQ(t.size(), 20u);   // 4 x 5
  EXPECT_NEAR((t[0].linear().col(2) - Vec3(1, 0, 0)).norm(), 0.0, 1e-9);   // tool z = -normal = +x
  EXPECT_NEAR(t[0].translation().x(), 3.0, 1e-9);
  EXPECT_TRUE(patchTargets(WallPatch{}, q.wall_normal).size() == 1u);   // a zero-size patch is one point
  WallPatch bad = *q.patch;
  bad.resolution = 0.0;
  EXPECT_TRUE(patchTargets(bad, q.wall_normal).empty());
}

TEST(Standoff, PicksTheMiddleOfThePlateauNotTheBestSingleDistance)
{
  const auto res = sweepStandoff(fixture(), request());
  ASSERT_EQ(res.status, SweepStatus::kOk) << res.recommended.reason;
  const auto & rec = res.recommended;
  ASSERT_TRUE(rec.found);
  // Reachable (x*, the distance) 0.5..1.1; the first and last 0.1 m have count 1 (quality 0.125 < 0.5).
  EXPECT_GE(rec.min_distance, 0.60 - 1e-9);
  EXPECT_LE(rec.max_distance, 1.0 + 1e-9);
  EXPECT_NEAR(rec.standoff_m, 0.80, 0.03);
  EXPECT_NEAR(res.max_fraction, 1.0, 1e-9);
  EXPECT_NEAR(rec.reachable_fraction, 1.0, 1e-9);
  EXPECT_TRUE(rec.robust);
  EXPECT_TRUE(rec.quality_ok);
  EXPECT_FALSE(rec.touches_range);
  EXPECT_GT(rec.mean_quality, 0.7);   // count score 1, boundary score 6/8: the fixture has one theta bin
  EXPECT_EQ(rec.lift_low, 0.0);
  // Plateau rows are flagged.
  size_t flagged = 0;
  for (const auto & r : res.rows) {flagged += r.in_plateau ? 1u : 0u;}
  EXPECT_GE(flagged, 15u);
}

TEST(Standoff, NavToleranceShrinksTheRobustBand)
{
  auto q = request();
  q.nav_tolerance_m = 0.02;
  const auto small = sweepStandoff(fixture(), q);
  q.nav_tolerance_m = 0.12;
  const auto big = sweepStandoff(fixture(), q);
  ASSERT_TRUE(small.recommended.robust);
  ASSERT_TRUE(big.recommended.robust);
  EXPECT_NEAR(small.recommended.min_distance, big.recommended.min_distance, 1e-9);   // the raw plateau is the same
  EXPECT_LT(big.recommended.robust_max - big.recommended.robust_min,
    small.recommended.robust_max - small.recommended.robust_min - 0.15);
  EXPECT_NEAR(big.recommended.robust_min, big.recommended.min_distance + 0.12, 0.011);
  // A tolerance wider than the plateau is not robust, and says so.
  q.nav_tolerance_m = 0.30;
  const auto wide = sweepStandoff(fixture(), q);
  EXPECT_TRUE(wide.recommended.found);
  EXPECT_FALSE(wide.recommended.robust);
  EXPECT_NE(wide.recommended.reason.find("not robust"), std::string::npos);
}

TEST(Standoff, RobustFractionSeesTheCliffNextToADistance)
{
  const auto res = sweepStandoff(fixture(), request());
  // Just inside the far edge (x_hi = 1.10): full at 1.08, but 1.12 is out so the robust value is lower.
  for (const auto & r : res.rows) {
    if (std::fabs(r.distance - 1.08) < 1e-9) {
      EXPECT_NEAR(r.reachable_fraction, 1.0, 1e-9);
      EXPECT_LT(r.robust_fraction, 1.0);
    }
    if (std::fabs(r.distance - 0.80) < 1e-9) {EXPECT_NEAR(r.robust_fraction, 1.0, 1e-9);}
  }
}

TEST(Standoff, FootprintClearanceMovesThePlateau)
{
  auto q = request();
  reachability::BaseModel b;
  b.footprint = {{-0.9, -0.3}, {0.9, -0.3}, {0.9, 0.3}, {-0.9, 0.3}};   // 1.8 m long, 0.6 m wide
  q.base = b;
  q.min_clearance = 0.50;
  // Best yaw turns the base sideways: clearance is distance - 0.3, so it fits from 0.80 m.
  const auto res = sweepStandoff(fixture(), q);
  ASSERT_EQ(res.status, SweepStatus::kOk) << res.recommended.reason;
  for (const auto & r : res.rows) {
    EXPECT_EQ(r.footprint_fits, r.distance >= 0.80 - 1e-9) << r.distance;
  }
  EXPECT_GE(res.recommended.min_distance, 0.80 - 1e-9);
  EXPECT_GT(res.recommended.standoff_m, 0.80);
  // Base origin = arm base here (mount 0, 0), so equal distances.
  EXPECT_NEAR(res.recommended.base_origin_distance, res.recommended.standoff_m, 1e-6);
}

TEST(Standoff, ArmMountOffsetShiftsBaseOriginDistance)
{
  auto q = request();
  reachability::BaseModel b;
  b.footprint = {{-0.3, -0.3}, {0.3, -0.3}, {0.3, 0.3}, {-0.3, 0.3}};
  b.arm_mount = Eigen::Vector2d(0.2, 0.0);   // the arm stands 0.2 m ahead of the base origin
  q.base = b;
  const auto res = sweepStandoff(fixture(), q);
  ASSERT_TRUE(res.recommended.found);
  // Facing the wall: the origin is 0.2 m further from the wall than the arm base.
  EXPECT_NEAR(res.recommended.base_origin_distance - res.recommended.standoff_m, 0.2, 0.02);
}

TEST(Standoff, FootprintTooBigIsReportedNotRecommended)
{
  auto q = request();
  reachability::BaseModel b;
  b.footprint = {{-5, -5}, {5, -5}, {5, 5}, {-5, 5}};
  q.base = b;
  const auto res = sweepStandoff(fixture(), q);
  EXPECT_EQ(res.status, SweepStatus::kNoFootprintFit);
  EXPECT_FALSE(res.recommended.found);
  EXPECT_NE(res.recommended.reason.find("footprint"), std::string::npos);
}

TEST(Standoff, NothingReachable)
{
  const auto res = sweepStandoff(Map4D(), request());   // empty map
  EXPECT_EQ(res.status, SweepStatus::kNothingReachable);
  EXPECT_FALSE(res.recommended.found);
  EXPECT_FALSE(res.recommended.reason.empty());
  // Reachable only above the lift range: raising the lift cannot help when lift is fixed to 0.
  Reach far;
  far.z_lo = 1.0;
  far.z_hi = 1.5;
  EXPECT_EQ(sweepStandoff(fixture(far), request()).status, SweepStatus::kNothingReachable);
}

TEST(Standoff, LiftRangeIsReportedAndAMovableLiftRecoversReach)
{
  Reach high;   // the tool must be 0.8..1.2 above the arm base
  high.z_lo = 0.8;
  high.z_hi = 1.2;
  auto q = request();
  q.lift_min = 0.0;
  q.lift_max = 0.0;
  EXPECT_EQ(sweepStandoff(fixture(high), q).status, SweepStatus::kNothingReachable);
  // Targets at z 0..0.4: the arm base must be 0.4..0.8 m below... i.e. negative lift is impossible, so lift the
  // targets instead: a wall patch higher on the wall with a lift range reaches.
  q.patch->v0 = 1.0;
  q.patch->v1 = 1.4;
  q.lift_max = 1.0;
  const auto res = sweepStandoff(fixture(high), q);
  ASSERT_EQ(res.status, SweepStatus::kOk) << res.recommended.reason;
  EXPECT_GT(res.recommended.lift_high, 0.0);
  EXPECT_LE(res.recommended.lift_high, 1.0);
  EXPECT_GE(res.recommended.lift_low, 0.0);
}

TEST(Standoff, SinglePointPlateauIsNotRobust)
{
  Reach thin;
  thin.x_lo = 0.80;
  thin.x_hi = 0.87;
  thin.edge_band = 0.0;
  auto q = request();
  q.quality_min = 0.3;      // one lone cell scores 0.5 at best
  q.standoff_min = 0.325;   // mid-cell distances: no rounding at a cell edge
  q.standoff_step = 0.05;
  const auto res = sweepStandoff(fixture(thin), q);
  ASSERT_EQ(res.status, SweepStatus::kOk) << res.recommended.reason;
  EXPECT_NEAR(res.recommended.min_distance, res.recommended.max_distance, 1e-9);
  EXPECT_FALSE(res.recommended.robust);
  EXPECT_NE(res.recommended.reason.find("single-point"), std::string::npos);
}

TEST(Standoff, PartialReachPrefersTheHighestFractionAndUsesTheLateralSearch)
{
  Reach narrow;
  narrow.y_half = 0.30;   // the arm covers 0.6 m of a 1.5 m wide wall from one stand
  auto q = request();
  q.patch = WallPatch{Vec3(3.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0), Vec3(0.0, 0.0, 1.0), -0.75, 0.75, 0.0, 0.4, 0.1};
  const auto res = sweepStandoff(fixture(narrow), q);
  ASSERT_EQ(res.status, SweepStatus::kOk);
  EXPECT_GT(res.max_fraction, 0.3);
  EXPECT_LT(res.max_fraction, 0.6);
  // Fixed middle position only: a stand centred on the wall reaches the same share here, but a fixed offset loses.
  q.lateral_step = 0.0;
  const auto fixed = sweepStandoff(fixture(narrow), q);
  EXPECT_NEAR(fixed.max_fraction, res.max_fraction, 0.1);
}

TEST(Standoff, QualityGateDroppedWithAWarning)
{
  auto q = request();
  q.quality_min = 0.99;   // the fixture's count score is 1.0 only inside, boundary edges lower; make it unreachable
  Reach weak;
  weak.count = 2;         // score 0.25 everywhere
  const auto res = sweepStandoff(fixture(weak), q);
  ASSERT_TRUE(res.recommended.found);
  EXPECT_FALSE(res.recommended.quality_ok);
  EXPECT_NE(res.recommended.reason.find("WARNING"), std::string::npos);
}

TEST(Standoff, PlateauAtTheRangeEndIsFlagged)
{
  auto q = request();
  q.standoff_max = 0.80;   // the good band continues beyond the swept range
  const auto res = sweepStandoff(fixture(), q);
  ASSERT_TRUE(res.recommended.found);
  EXPECT_TRUE(res.recommended.touches_range);
  EXPECT_NE(res.recommended.reason.find("extend"), std::string::npos);
}

TEST(Standoff, RefusesNonFiniteAndBadInput)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  auto q = request();
  q.standoff_step = nan;
  EXPECT_EQ(sweepStandoff(fixture(), q).status, SweepStatus::kInvalidRequest);
  q = request();
  q.wall_normal = Vec3(nan, 0, 0);
  EXPECT_EQ(sweepStandoff(fixture(), q).status, SweepStatus::kInvalidRequest);
  q = request();
  q.standoff_step = 0.0;
  EXPECT_FALSE(sweepProblem(fixture(), q).empty());
  q = request();
  q.standoff_step = 1e-6;   // 1.3 million steps
  EXPECT_FALSE(sweepProblem(fixture(), q).empty());
  q = request();
  q.wall_normal = Vec3(0, 0, 1);   // a ceiling
  EXPECT_FALSE(sweepProblem(fixture(), q).empty());
  q = request();
  q.patch.reset();
  EXPECT_FALSE(sweepProblem(fixture(), q).empty());   // no targets
  q = request();
  q.nav_tolerance_m = -0.1;
  EXPECT_FALSE(sweepProblem(fixture(), q).empty());
  q = request();
  q.patch->v0 = nan;
  EXPECT_FALSE(sweepProblem(fixture(), q).empty());
  EXPECT_TRUE(sweepProblem(fixture(), request()).empty());
}

TEST(Standoff, EvaluateStandoffMatchesTheSweepRow)
{
  const auto q = request();
  const auto res = sweepStandoff(fixture(), q);
  const auto row = evaluateStandoff(fixture(), q, 0.80);
  for (const auto & r : res.rows) {
    if (std::fabs(r.distance - 0.80) < 1e-9) {
      EXPECT_DOUBLE_EQ(r.reachable_fraction, row.reachable_fraction);
      EXPECT_DOUBLE_EQ(r.mean_quality, row.mean_quality);
    }
  }
  EXPECT_EQ(evaluateStandoff(fixture(), q, 0.10).reachable_count, 0u);
}
