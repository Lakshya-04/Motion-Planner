// ReachabilityColumnPort: the RM4D-backed ColumnReachabilityPort, against a hand-made map (no arm behind it).
// The scenarios mirror reachability's column_sequence_test.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "manipulation_manager/adapters/reachability_column_port.hpp"

using manipulation_manager::ColumnReachInput;
using manipulation_manager::ColumnReachStatus;
using manipulation_manager::liftHeights;
using manipulation_manager::ReachabilityColumnConfig;
using manipulation_manager::ReachabilityColumnPort;

namespace
{

// Tool pose whose approach (z) axis is +x: the tool points at a wall on the +x side (rotation of 90 deg about y).
core_types::Pose towardPlusX(double x, double y, double z)
{
  core_types::Pose p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.qx = 0.0;
  p.qy = std::sqrt(0.5);
  p.qz = 0.0;
  p.qw = std::sqrt(0.5);
  return p;
}

reachability::Pose towardPlusXEigen(double x, double y, double z)
{
  reachability::Pose T = reachability::Pose::Identity();
  T.translation() = reachability::Vec3(x, y, z);
  T.linear() = Eigen::AngleAxisd(M_PI / 2, reachability::Vec3::UnitY()).toRotationMatrix();
  return T;
}

// For approach axis +x: every cell with x* in [0.30, 0.90), |y*| <= 0.8, tool height above the arm base in
// [-0.5, 0.7] holds 20; the outer 5 cm of x* hold 1 (reachable but weak). Everything else is empty.
std::shared_ptr<const reachability::Map4D> syntheticMap()
{
  auto map = std::make_shared<reachability::Map4D>();   // default params: extent 1.8, z -1..1.8, 0.05 m cells
  const auto & p = map->params();
  const int itheta = map->keyOf(towardPlusXEigen(0.5, 0.0, 0.0)).itheta;
  for (int iz = 0; iz < map->nz(); ++iz) {
    const double z = p.z_min + (iz + 0.5) * p.resolution;
    if (z < -0.5 || z > 0.7) {continue;}
    for (int ix = 0; ix < map->nxy(); ++ix) {
      const double x = -p.xy_extent + (ix + 0.5) * p.resolution;
      if (x < 0.30 || x >= 0.90) {continue;}
      for (int iy = 0; iy < map->nxy(); ++iy) {
        const double y = -p.xy_extent + (iy + 0.5) * p.resolution;
        if (std::fabs(y) > 0.80) {continue;}
        map->raw()[map->index(iz, itheta, ix, iy)] = static_cast<uint8_t>(x >= 0.85 ? 1 : 20);
      }
    }
  }
  return map;
}

// A column on the wall x = 2, strokes every 0.1 m from z0 to z1.
ColumnReachInput column(const std::string & label, double y, double z0, double z1)
{
  ColumnReachInput c;
  c.label = label;
  for (double z = z0; z <= z1 + 1e-9; z += 0.1) {c.strokes.push_back(towardPlusX(2.0, y, z));}
  return c;
}

ReachabilityColumnConfig config(double standoff, double extra, double press, double retract = -1.0)
{
  ReachabilityColumnConfig c;
  c.sequence.wall_normal = reachability::Vec3(-1, 0, 0);
  c.sequence.standoff_m = standoff;
  c.sequence.approach_extra_m = extra;
  c.sequence.press_depth_m = press;
  c.sequence.retract_m = retract;
  c.lift_heights = {0.0};
  return c;
}

ReachabilityColumnPort::StandProvider standAt(double x, double y = 0.0)
{
  return [x, y]() -> std::optional<Eigen::Vector2d> {return Eigen::Vector2d(x, y);};
}

}  // namespace

TEST(ReachabilityColumnPort, WholeColumnWhenEveryPoseOfTheSequenceIsInReach)
{
  // Strokes at x* = 0.60, approach at 0.60 - 0.22 = 0.38, contact at 0.63.
  ReachabilityColumnPort port(syntheticMap(), config(0.12, 0.10, 0.03), standAt(2.0 - 0.60));
  const auto v = port.check({column("a", 0.0, 0.2, 0.6)});
  ASSERT_EQ(v.size(), 1u);
  EXPECT_EQ(v[0].status, ColumnReachStatus::kWhole);
  EXPECT_TRUE(v[0].limiting_pose.empty());
  ASSERT_EQ(v[0].stroke_usable.size(), 5u);
  for (size_t i = 0; i < 5; ++i) {EXPECT_TRUE(v[0].strokeUsable(i));}
  EXPECT_GT(v[0].min_score, 0.15);
}

TEST(ReachabilityColumnPort, UnreachableApproachNamesTheApproachThoughTheStrokesAreReachable)
{
  // Strokes at x* = 0.45 are fine; the approach at 0.45 - 0.22 = 0.23 is outside the map.
  ReachabilityColumnPort port(syntheticMap(), config(0.12, 0.10, 0.0), standAt(2.0 - 0.45));
  const auto v = port.check({column("a", 0.0, 0.2, 0.6)});
  ASSERT_EQ(v.size(), 1u);
  EXPECT_EQ(v[0].status, ColumnReachStatus::kUnreachable);
  EXPECT_EQ(v[0].limiting_pose, "approach");
  EXPECT_EQ(v[0].limiting_reason, "out_of_reach");
  for (size_t i = 0; i < 5; ++i) {EXPECT_TRUE(v[0].strokeUsable(i)) << "stroke " << i;}
}

TEST(ReachabilityColumnPort, RetractIsReportedAsTheLimitingPose)
{
  ReachabilityColumnPort port(syntheticMap(), config(0.0, 0.0, 0.0, 0.5), standAt(2.0 - 0.60));
  const auto v = port.check({column("a", 0.0, 0.2, 0.6)});
  ASSERT_EQ(v.size(), 1u);
  EXPECT_EQ(v[0].status, ColumnReachStatus::kUnreachable);
  EXPECT_EQ(v[0].limiting_pose, "retract");
}

TEST(ReachabilityColumnPort, LowMarginIsAReasonOfItsOwn)
{
  // Strokes at x* = 0.87: the cell holds 1 sample, score 1/8 < 0.15.
  ReachabilityColumnPort port(syntheticMap(), config(0.0, 0.0, 0.0), standAt(2.0 - 0.87));
  const auto v = port.check({column("a", 0.0, 0.2, 0.6)});
  ASSERT_EQ(v.size(), 1u);
  EXPECT_EQ(v[0].status, ColumnReachStatus::kUnreachable);
  EXPECT_EQ(v[0].limiting_reason, "low_margin");

  auto lax = config(0.0, 0.0, 0.0);
  lax.margin.min_score = 0.0;
  ReachabilityColumnPort port2(syntheticMap(), lax, standAt(2.0 - 0.87));
  EXPECT_EQ(port2.check({column("a", 0.0, 0.2, 0.6)})[0].status, ColumnReachStatus::kWhole);
}

TEST(ReachabilityColumnPort, SplitColumnFlagsTheUnusableStrokes)
{
  // z in [-0.5, 0.7] above the arm base: strokes at 0.2 .. 1.0 with the lift at 0 lose the top ones. No retract
  // pose (it would itself be out of the window).
  ReachabilityColumnPort port(syntheticMap(), config(0.12, 0.0, 0.0, 0.0), standAt(2.0 - 0.60));
  const auto v = port.check({column("a", 0.0, 0.2, 1.0)});
  ASSERT_EQ(v.size(), 1u);
  EXPECT_EQ(v[0].status, ColumnReachStatus::kSplit);
  ASSERT_EQ(v[0].stroke_usable.size(), 9u);
  EXPECT_TRUE(v[0].strokeUsable(0));
  EXPECT_FALSE(v[0].strokeUsable(8));
  EXPECT_EQ(v[0].limiting_pose, "stroke");
  EXPECT_NE(v[0].limiting_stroke, manipulation_manager::kNoLimitingStroke);
  EXPECT_FALSE(v[0].strokeUsable(v[0].limiting_stroke));
}

TEST(ReachabilityColumnPort, OfferedLiftHeightsMakeTheSameColumnWhole)
{
  auto cfg = config(0.12, 0.0, 0.0, 0.0);
  cfg.lift_heights = liftHeights(0.0, 0.4, 0.2);
  ReachabilityColumnPort port(syntheticMap(), cfg, standAt(2.0 - 0.60));
  const auto v = port.check({column("a", 0.0, 0.2, 1.0)});
  ASSERT_EQ(v.size(), 1u);
  EXPECT_EQ(v[0].status, ColumnReachStatus::kWhole);
}

TEST(ReachabilityColumnPort, ArmMountOffsetShiftsTheWindow)
{
  // Strokes at z 0.2 .. 0.6 sit inside [-0.5, 0.7] above an arm base at 0; with the base 1.0 m up they are 0.8 m
  // below it; the low strokes fall out of the map, so the column is no longer whole (the top strokes, at -0.5 and
  // above relative to the base, still are inside, so it comes back split rather than unreachable).
  auto cfg = config(0.0, 0.0, 0.0);
  cfg.arm_mount_offset_z = 1.0;
  ReachabilityColumnPort port(syntheticMap(), cfg, standAt(2.0 - 0.60));
  const auto v = port.check({column("a", 0.0, 0.2, 0.6)});
  ASSERT_EQ(v.size(), 1u);
  EXPECT_NE(v[0].status, ColumnReachStatus::kWhole);
  ASSERT_FALSE(v[0].stroke_usable.empty());
  EXPECT_FALSE(v[0].stroke_usable.front());
}

TEST(ReachabilityColumnPort, UnknownStandOrMissingMapAnswersNothing)
{
  ReachabilityColumnPort no_stand(
    syntheticMap(), config(0.0, 0.0, 0.0), []() -> std::optional<Eigen::Vector2d> {return std::nullopt;});
  EXPECT_TRUE(no_stand.check({column("a", 0.0, 0.2, 0.6)}).empty());

  ReachabilityColumnPort no_map(nullptr, config(0.0, 0.0, 0.0), standAt(1.4));
  EXPECT_TRUE(no_map.check({column("a", 0.0, 0.2, 0.6)}).empty());
}

TEST(ReachabilityColumnPort, OneVerdictPerColumnInOrderAndEmptyColumnIsUnreachable)
{
  ReachabilityColumnPort port(syntheticMap(), config(0.0, 0.0, 0.0), standAt(2.0 - 0.60));
  ColumnReachInput empty;
  empty.label = "empty";
  const auto v = port.check({column("a", 0.0, 0.2, 0.6), empty, column("b", 0.5, 0.2, 0.6)});
  ASSERT_EQ(v.size(), 3u);
  EXPECT_EQ(v[0].status, ColumnReachStatus::kWhole);
  EXPECT_EQ(v[1].status, ColumnReachStatus::kUnreachable);
  EXPECT_EQ(v[2].status, ColumnReachStatus::kWhole);
}

TEST(ReachabilityColumnPort, NonUnitQuaternionIsNormalised)
{
  auto col = column("a", 0.0, 0.2, 0.6);
  for (auto & p : col.strokes) {p.qy *= 3.0; p.qw *= 3.0;}
  ReachabilityColumnPort port(syntheticMap(), config(0.0, 0.0, 0.0), standAt(2.0 - 0.60));
  EXPECT_EQ(port.check({col})[0].status, ColumnReachStatus::kWhole);
}

TEST(LiftHeights, SpansTheRangeIncludingBothEnds)
{
  EXPECT_EQ(liftHeights(0.0, 0.0, 0.1), std::vector<double>({0.0}));
  EXPECT_EQ(liftHeights(0.2, 0.1, 0.1), std::vector<double>({0.2}));
  EXPECT_EQ(liftHeights(0.0, 0.5, 0.0), std::vector<double>({0.0}));
  const auto h = liftHeights(0.0, 0.5, 0.2);   // 0, 0.2, 0.4, 0.5
  ASSERT_EQ(h.size(), 4u);
  EXPECT_DOUBLE_EQ(h.front(), 0.0);
  EXPECT_DOUBLE_EQ(h.back(), 0.5);
  const auto g = liftHeights(0.0, 0.4, 0.2);   // exact multiple: no duplicate end
  ASSERT_EQ(g.size(), 3u);
  EXPECT_DOUBLE_EQ(g.back(), 0.4);
}
