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

}  // namespace manipulation_manager
