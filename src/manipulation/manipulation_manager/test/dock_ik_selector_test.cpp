#include <cmath>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "manipulation_manager/core/dock_ik_selector.hpp"

namespace manipulation_manager
{
namespace
{
using V = std::vector<double>;

// Fake IK: returns the table entry whose index is the position of the seed in `seeds`.
DockIkFn tableIk(const std::vector<V> & seeds, const std::vector<std::optional<V>> & out)
{
  return [seeds, out](const V & seed) -> std::optional<V> {
           for (size_t i = 0; i < seeds.size(); ++i) {
             if (seeds[i] == seed) {
               return out[i];
             }
           }
           return std::nullopt;
         };
}
}  // namespace

TEST(DockIkSelectorTest, PicksTheClosestToTheReference)
{
  const V ref{0.5, 1.0, -0.6, 0.0};
  const std::vector<V> seeds{{1, 0, 0, 0}, {2, 0, 0, 0}};
  const auto ik = tableIk(seeds, {V{0.5, 1.0, -0.2, 0.9}, V{0.5, 1.0, -0.6, 0.1}});
  DockIkSelectorParams p;
  p.branch_joints = {2};
  const auto c = selectDockIk(ik, seeds, ref, p);
  ASSERT_TRUE(c.ok);
  EXPECT_EQ(c.seed_index, 1u);
  EXPECT_NEAR(c.cost, 0.1, 1e-12);
  EXPECT_EQ(c.candidates, 2u);
}

TEST(DockIkSelectorTest, RejectsTheFoldedBackBranch)
{
  const V ref{0.5, 1.0, -0.62, 0.0};
  const std::vector<V> seeds{{1, 0, 0, 0}};
  const auto ik = tableIk(seeds, {V{0.5, 1.0, 2.49, 0.0}});
  DockIkSelectorParams p;
  p.branch_joints = {2};
  const auto c = selectDockIk(ik, seeds, ref, p);
  EXPECT_FALSE(c.ok);
  EXPECT_EQ(c.rejected_branch, 1u);
}

TEST(DockIkSelectorTest, RejectsWristWinding)
{
  const V ref{0.5, 1.0, -0.6, -2.0};
  const std::vector<V> seeds{{1, 0, 0, 0}, {2, 0, 0, 0}};
  const auto ik = tableIk(seeds, {V{0.5, 1.0, -0.6, -4.6}, V{0.5, 1.0, -0.6, -2.5}});
  DockIkSelectorParams p;
  p.wrist_joint = 3;
  const auto c = selectDockIk(ik, seeds, ref, p);
  ASSERT_TRUE(c.ok);
  EXPECT_EQ(c.seed_index, 1u);
  EXPECT_EQ(c.rejected_wrist, 1u);
}

TEST(DockIkSelectorTest, ContinuousJointsAreWrappedLinearOnesAreNot)
{
  DockIkSelectorParams p;   // joint 0 linear
  EXPECT_NEAR(dockIkCost({0.0, 3.0}, {0.0, -3.0}, p), 2.0 * M_PI - 6.0, 1e-12);
  EXPECT_NEAR(dockIkCost({2.0, 0.0}, {-1.0, 0.0}, p), 3.0, 1e-12);
}

TEST(DockIkSelectorTest, RejectsALargeJumpAndFallsBackWhenNothingIsLeft)
{
  const V ref{0.0, 0.0};
  const std::vector<V> seeds{{1, 0}};
  DockIkSelectorParams p;
  p.max_dq_rad = 1.0;
  EXPECT_FALSE(selectDockIk(tableIk(seeds, {V{0.0, 2.0}}), seeds, ref, p).ok);
  EXPECT_FALSE(selectDockIk(tableIk(seeds, {std::nullopt}), seeds, ref, p).ok);
  EXPECT_FALSE(selectDockIk(tableIk(seeds, {V{0.0, 0.5}}), {}, ref, p).ok);
  EXPECT_FALSE(selectDockIk(tableIk(seeds, {V{0.0, 0.5}}), seeds, {}, p).ok);
}

TEST(DockIkSelectorTest, InsideTheDeadbandIsNoBranchChange)
{
  DockIkSelectorParams p;
  p.branch_joints = {0};
  EXPECT_FALSE(dockBranchChanged({0.01}, {-0.5}, p));
  EXPECT_TRUE(dockBranchChanged({0.3}, {-0.5}, p));
}

// ---- reach preference ----

// Fake IK that keeps the lift entry of its seed (the lift is free for it) and returns fixed arm joints.
DockIkFn liftKeepingIk()
{
  return [](const V & seed) -> std::optional<V> {
           return V{seed[0], 0.5, -0.6, 0.0};
         };
}

DockIkSelectorParams reachParams()
{
  DockIkSelectorParams p;
  p.reach_target_fraction = 0.7;
  p.reach_m = 1.0;
  p.lift_joint = 0;
  p.lift_min_m = 0.0;
  p.lift_max_m = 1.5;
  return p;
}

TEST(DockIkSelectorTest, ReachFractionAndIdealLift)
{
  const auto p = reachParams();
  const DockReachContext c{0.6, 1.0};   // tool 0.6 m out, 1.0 m above the arm base at zero lift
  EXPECT_NEAR(dockReachFraction(0.0, c, p), std::sqrt(0.36 + 1.0), 1e-12);
  EXPECT_NEAR(dockReachFraction(1.0, c, p), 0.6, 1e-12);
  // dz allowed = sqrt(0.49 - 0.36) = 0.3606: the lift nearest 0 is 1.0 - 0.3606
  EXPECT_NEAR(idealLift(0.0, c, p), 1.0 - std::sqrt(0.13), 1e-9);
  EXPECT_NEAR(idealLift(1.0, c, p), 1.0, 1e-12);       // already inside the band
  EXPECT_NEAR(dockReachFraction(idealLift(0.0, c, p), c, p), 0.7, 1e-9);
}

TEST(DockIkSelectorTest, IdealLiftClampsAndHandlesAnUnreachableFraction)
{
  auto p = reachParams();
  const DockReachContext far{0.9, 1.0};   // 0.9 m out alone is over 0.7: no lift helps, level with the base is best
  EXPECT_NEAR(idealLift(0.0, far, p), 1.0, 1e-12);
  const DockReachContext high{0.3, 3.0};  // wanted lift above the stroke: clamped to the top
  EXPECT_NEAR(idealLift(0.0, high, p), 1.5, 1e-12);
}

TEST(DockIkSelectorTest, ReachPreferenceMovesTheLiftWhenTheSeedsLeaveItLow)
{
  const auto p = reachParams();
  const DockReachContext c{0.6, 1.0};
  const V ref{0.0, 0.5, -0.6, 0.0};
  const auto ch = selectDockIk(liftKeepingIk(), {V{0.0, 0.5, -0.6, 0.0}}, ref, p, &c);
  ASSERT_TRUE(ch.ok);
  EXPECT_EQ(ch.seed_index, 1u);                 // the lift-adjusted copy of the first seed
  EXPECT_GT(ch.q[0], 0.5);
  EXPECT_LE(ch.reach_fraction, 0.7 + 1e-9);
  EXPECT_FALSE(ch.reach_over_target);
}

TEST(DockIkSelectorTest, OffMeansTheCheapestAsBefore)
{
  auto p = reachParams();
  const DockReachContext c{0.6, 1.0};
  const V ref{0.0, 0.5, -0.6, 0.0};
  p.reach_target_fraction = 0.0;                // off
  auto ch = selectDockIk(liftKeepingIk(), {ref}, ref, p, &c);
  ASSERT_TRUE(ch.ok);
  EXPECT_EQ(ch.seed_index, 0u);
  EXPECT_NEAR(ch.q[0], 0.0, 1e-12);
  EXPECT_LT(ch.reach_fraction, 0.0);
  p = reachParams();                            // on but no context
  ch = selectDockIk(liftKeepingIk(), {ref}, ref, p, nullptr);
  EXPECT_EQ(ch.seed_index, 0u);
  EXPECT_NEAR(ch.q[0], 0.0, 1e-12);
}

TEST(DockIkSelectorTest, NoCandidateUnderTheTargetFallsBackToTheCheapest)
{
  const auto p = reachParams();
  const DockReachContext c{0.9, 1.0};           // never under 0.7
  const V ref{0.2, 0.5, -0.6, 0.0};
  const auto ch = selectDockIk(liftKeepingIk(), {ref}, ref, p, &c);
  ASSERT_TRUE(ch.ok);
  EXPECT_TRUE(ch.reach_over_target);
  EXPECT_EQ(ch.seed_index, 0u);                 // cost 0: the seed itself
}

TEST(DockIkSelectorTest, AnOverTargetCandidateLosesEvenWhenCheaper)
{
  const auto p = reachParams();
  const DockReachContext c{0.6, 1.0};
  const V a{0.0, 0.5, -0.6, 0.0};
  const V b{1.0, 0.5, -0.6, 0.0};               // under the target, farther from the reference
  const auto ik = [&](const V & seed) -> std::optional<V> {return seed == a ? a : b;};
  const auto ch = selectDockIk(ik, {a, b}, a, p, &c);
  ASSERT_TRUE(ch.ok);
  EXPECT_NEAR(ch.q[0], 1.0, 1e-12);
  EXPECT_LE(ch.reach_fraction, 0.7 + 1e-9);
}

TEST(DockIkRebaseTest, MovesTwoPiEquivalentsNextToTheReference)
{
  const V ref{0.0, 0.0, 0.0, 0.0};
  DockIkSelectorParams p;
  const V q = rebaseDockIk({1.5, 5.35, -5.90, 1.13}, ref, p);
  EXPECT_DOUBLE_EQ(q[0], 1.5);
  EXPECT_NEAR(q[1], 5.35 - 2.0 * M_PI, 1e-12);
  EXPECT_NEAR(q[1], -0.933, 1e-3);
  EXPECT_NEAR(q[2], -5.90 + 2.0 * M_PI, 1e-12);
  EXPECT_NEAR(q[2], 0.383, 1e-3);
  EXPECT_DOUBLE_EQ(q[3], 1.13);
}

TEST(DockIkRebaseTest, LinearJointsUntouchedAndAlreadyNearestUnchanged)
{
  DockIkSelectorParams p;
  const V ref{0.2, 0.1, -0.3};
  const V near{7.0, 0.4, -0.5};  // joint 0 is linear: 7.0 stays
  EXPECT_EQ(rebaseDockIk(near, ref, p), near);
  EXPECT_EQ(rebaseDockIk({0.0}, {0.0, 1.0}, p), V{0.0});  // size mismatch
}

TEST(DockIkRebaseTest, RespectsLimits)
{
  DockIkSelectorParams p;
  p.joint_lower = {0.0, -6.0, -1.0};
  p.joint_upper = {1.5, 6.0, 6.0};
  const V ref{0.0, 0.0, 0.0};
  // joint 1: nearest is -0.93, inside; joint 2: ref-nearest of 5.0 is -1.28, below -1.0 so 5.0 stays
  const V q = rebaseDockIk({0.5, 5.35, 5.0}, ref, p);
  EXPECT_NEAR(q[1], 5.35 - 2.0 * M_PI, 1e-12);
  EXPECT_DOUBLE_EQ(q[2], 5.0);
  // no limits and a value beyond 2*pi is pulled back inside
  DockIkSelectorParams n;
  const V r = rebaseDockIk({0.0, 7.0}, {0.0, 6.5}, n);
  EXPECT_LE(std::fabs(r[1]), 2.0 * M_PI + 1e-9);
}

TEST(DockIkRebaseTest, SelectDockIkReturnsTheRebasedSolution)
{
  const V ref{0.0, 0.0, 0.0, 0.0};
  const std::vector<V> seeds{ref};
  DockIkSelectorParams p;
  const auto c = selectDockIk(tableIk(seeds, {V{1.5, 5.35, -5.90, 1.13}}), seeds, ref, p);
  ASSERT_TRUE(c.ok);
  EXPECT_NEAR(c.q[1], -0.933, 1e-3);
  EXPECT_NEAR(c.q[2], 0.383, 1e-3);
}

}  // namespace manipulation_manager
