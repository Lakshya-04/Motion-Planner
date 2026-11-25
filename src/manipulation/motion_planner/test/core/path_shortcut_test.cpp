#include <gtest/gtest.h>

#include "motion_planner/core/path_shortcut.hpp"

using namespace motion_planner;

namespace
{
// 2-D joint space, forbidden box x in [0.4, 0.6], y in [-0.5, 0.5].
bool isFree(const std::vector<double> & q)
{
  return !(q[0] > 0.4 && q[0] < 0.6 && q[1] > -0.5 && q[1] < 0.5);
}

PathShortcutConfig config()
{
  PathShortcutConfig c;
  c.sampling.revolute_step = 0.02;
  return c;
}
}  // namespace

TEST(PathShortcut, FreeDetourBecomesStraight)
{
  const std::vector<std::vector<double>> detour{{0, 0}, {0.5, 1.0}, {1, 0}, {1.5, 1.0}, {2, 0}};
  const auto out = shortcutPath(detour, config(), [](const std::vector<double> &) {return true;});
  EXPECT_EQ(out.size(), 2u);
  EXPECT_EQ(out.front(), detour.front());
  EXPECT_EQ(out.back(), detour.back());
}

TEST(PathShortcut, NeverCutsThroughObstacleAndNeverLonger)
{
  const std::vector<std::vector<double>> path{{0, 0}, {0.3, 0.8}, {0.5, 1.0}, {0.7, 0.8}, {1, 0}};
  ASSERT_TRUE(segmentValid(path[0], path[1], config().sampling, isFree));
  const auto out = improvePath(path, config(), PathQualityConfig{}, isFree);
  for (std::size_t i = 1; i < out.size(); ++i) {
    EXPECT_TRUE(segmentValid(out[i - 1], out[i], config().sampling, isFree));
  }
  EXPECT_LE(weightedPathLength(out, PathQualityConfig{}), weightedPathLength(path, PathQualityConfig{}) + 1e-12);
  EXPECT_EQ(out.front(), path.front());
  EXPECT_EQ(out.back(), path.back());
}

TEST(PathShortcut, SmoothingShortensCornerAndKeepsEnds)
{
  const std::vector<std::vector<double>> corner{{0, 0}, {1, 0}, {1, 1}};
  auto c = config();
  const auto out = smoothPath(corner, c, [](const std::vector<double> &) {return true;});
  EXPECT_LT(weightedPathLength(out, PathQualityConfig{}), 2.0);
  EXPECT_EQ(out.front(), corner.front());
  EXPECT_EQ(out.back(), corner.back());
}

TEST(PathShortcut, DeterministicForSameSeed)
{
  const std::vector<std::vector<double>> path{{0, 0}, {0.3, 0.8}, {0.5, 1.0}, {0.7, 0.8}, {1, 0}, {1.2, 0.5}};
  EXPECT_EQ(shortcutPath(path, config(), isFree), shortcutPath(path, config(), isFree));
}

TEST(PathShortcutLiftWeight, ImprovePathNeverIncreasesTheLiftWeightedLength)
{
  PathQualityConfig q;
  q.weights = applyLiftCostWeight({1.0, 1.0}, {true, false}, 2.0);
  const std::vector<std::vector<double>> path{{0.0, 0.0}, {0.5, 0.0}, {0.5, 0.5}, {0.0, 0.5}, {0.0, 1.0}};
  const auto out = improvePath(path, config(), q, isFree);
  EXPECT_LE(weightedPathLength(out, q), weightedPathLength(path, q) + 1e-12);
  EXPECT_LT(weightedPathLength(out, q), weightedPathLength(path, q));
}
