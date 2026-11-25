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
  EXPECT_NEAR(dockIkCost({2.0, 0.0}, {-1.0, 0.0}, p, false), 3.0, 1e-12);   // raw: the lift is not wrapped
}

TEST(DockIkSelectorTest, LiftDeltaIsWeightedInTheCost)
{
  DockIkSelectorParams p;   // joint 0 linear, default lift_cost_weight 2
  EXPECT_NEAR(dockIkCost({1.0, 0.0}, {0.0, 0.0}, p), 2.0, 1e-12);
  EXPECT_NEAR(dockIkCost({1.0, 0.0}, {0.0, 0.0}, p, false), 1.0, 1e-12);
  p.lift_cost_weight = 1.0;   // old behaviour exactly
  EXPECT_NEAR(dockIkCost({1.0, 0.5}, {0.0, 0.0}, p), 1.0, 1e-12);
}

TEST(DockIkSelectorTest, PrefersArmMotionOverLiftMotionAndWeightOneRestoresTheOldChoice)
{
  // seed 0: lift moves 0.8 m, arm 0.3 rad. seed 1: lift stays, arm moves 1.2 rad.
  // weight 1: max |dq| 0.8 vs 1.2 -> seed 0. weight 2: 1.6 vs 1.2 -> seed 1 (arm motion preferred).
  const V ref{0.0, 0.0};
  const std::vector<V> seeds{V{0.8, 0.3}, V{0.0, 1.2}};
  const DockIkFn ik = [](const V & sd) -> std::optional<V> {return sd;};
  DockIkSelectorParams p;
  p.branch_joints = {};
  p.lift_cost_weight = 1.0;
  EXPECT_EQ(selectDockIk(ik, seeds, ref, p).seed_index, 0u);
  p.lift_cost_weight = 2.0;
  const auto c = selectDockIk(ik, seeds, ref, p);
  ASSERT_TRUE(c.ok);
  EXPECT_EQ(c.seed_index, 1u);
  EXPECT_NEAR(c.cost, 1.2, 1e-12);
}

TEST(DockIkSelectorTest, MaxDqLimitStaysRawForTheLift)
{
  DockIkSelectorParams p;
  p.branch_joints = {};
  p.max_dq_rad = 1.5;   // lift 1.0 m weighted 2.0 > 1.5, but the raw limit is what rejects
  const DockIkFn ik = [](const V & sd) -> std::optional<V> {return sd;};
  EXPECT_TRUE(selectDockIk(ik, {V{1.0, 0.0}}, V{0.0, 0.0}, p).ok);
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

TEST(DockIkSelectorTest, FirstDockIsMeasuredAgainstTheDockSeedNotTheStow)
{
  DockIkSelectorParams p;
  p.branch_joints = {1, 3};
  const V stow{0.4, 0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
  const V seed{0.1, 2.79, -2.80, -1.30, -2.20, -0.36, -1.5708};
  const V good{0.05, 2.80, -2.82, -1.27, -2.20, -0.36, -1.5708};   // the seed's branch
  const V bad{0.04, -0.94, 0.60, 0.92, 1.70, -2.16, 1.57};         // the stow's branch (elbow up, pan flipped)
  const auto ik = [&](const V & sd) -> std::optional<V> {return sd == seed ? good : bad;};
  const auto & ref = dockBranchReference({}, seed, stow);
  EXPECT_EQ(ref, seed);
  const auto ch = selectDockIk(ik, {V{}, stow, seed}, ref, p);
  ASSERT_TRUE(ch.ok);
  EXPECT_NEAR(ch.q[1], 2.80, 1e-12);
  EXPECT_EQ(ch.rejected_branch, 1u);
  // against the stow the seed's branch is the one dropped (the failure this guards)
  const auto old = selectDockIk(ik, {V{}, stow, seed}, stow, p);
  ASSERT_TRUE(old.ok);
  EXPECT_NEAR(old.q[1], -0.94, 1e-12);
}

TEST(DockIkSelectorTest, BranchReferencePrefersPreviousDockThenSeedThenLive)
{
  const V prev{1, 1, 1}, seed{2, 2, 2}, live{3, 3, 3}, short_seed{2, 2};
  EXPECT_EQ(dockBranchReference(prev, seed, live), prev);
  EXPECT_EQ(dockBranchReference({}, seed, live), seed);
  EXPECT_EQ(dockBranchReference({}, {}, live), live);
  EXPECT_EQ(dockBranchReference({}, short_seed, live), live);
}

}  // namespace manipulation_manager
