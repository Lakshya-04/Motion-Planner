#include <gtest/gtest.h>

#include <cmath>

#include "scene_model/core/voxel.hpp"

using namespace scene_model;  // NOLINT

TEST(Voxel, VoxelizeCoversTheCloudAndMarksPoints)
{
  PointCloud c;
  c.points = {Vec3(0, 0, 0), Vec3(0.5, 0, 0), Vec3(0.5, 0.5, 0.5)};
  const VoxelGrid g = voxelize(c, 0.1);
  EXPECT_EQ(g.nx, 6);
  EXPECT_EQ(g.ny, 6);
  EXPECT_EQ(g.nz, 6);
  EXPECT_EQ(g.occupied[g.index(0, 0, 0)], 1);
  EXPECT_EQ(g.occupied[g.index(5, 0, 0)], 1);
  EXPECT_EQ(g.occupied[g.index(5, 5, 5)], 1);
  EXPECT_EQ(g.occupied[g.index(2, 2, 2)], 0);
}

TEST(Voxel, MinPointsFiltersIsolatedNoise)
{
  PointCloud c;
  for (int i = 0; i < 10; ++i) {c.points.emplace_back(0.01 * i, 0, 0);}  // dense strip
  c.points.emplace_back(1.0, 0, 0);                                     // lone point
  const VoxelGrid g = voxelize(c, 0.1, /*min_points=*/3);
  EXPECT_EQ(g.occupied[g.index(0, 0, 0)], 1);
  EXPECT_EQ(g.occupied[g.index(g.nx - 1, 0, 0)], 0);
}

TEST(Voxel, RejectsHugeGridAndBadSize)
{
  PointCloud c;
  c.points = {Vec3(0, 0, 0), Vec3(1000, 1000, 1000)};
  EXPECT_THROW(voxelize(c, 0.01), std::runtime_error);
  EXPECT_THROW(voxelize(c, 0.0), std::invalid_argument);
}

TEST(Voxel, OccupiedCentresRoundTrip)
{
  PointCloud c;
  c.points = {Vec3(0, 0, 0), Vec3(0.3, 0.2, 0.1)};
  const VoxelGrid g = voxelize(c, 0.1);
  const PointCloud back = occupiedCentres(g);
  ASSERT_EQ(back.points.size(), 2U);
  EXPECT_NEAR((back.points[1] - c.points[1]).norm(), 0.0, 1e-9);
}

TEST(Esdf, MatchesBruteForceOnRandomGrid)
{
  VoxelGrid g;
  g.voxel_size = 0.05;
  g.nx = 9;
  g.ny = 7;
  g.nz = 6;
  g.occupied.assign(static_cast<size_t>(g.nx) * g.ny * g.nz, 0);
  unsigned seed = 12345;
  for (auto & v : g.occupied) {
    seed = seed * 1664525U + 1013904223U;
    v = (seed >> 24) % 7 == 0 ? 1 : 0;
  }
  g.occupied[g.index(4, 3, 2)] = 1;
  g.occupied[g.index(0, 0, 0)] = 0;
  const EsdfGrid e = computeEsdf(g);

  auto brute = [&](int i, int j, int k, bool target_occupied) {
      double best = 1e18;
      for (int a = 0; a < g.nx; ++a) {
        for (int b = 0; b < g.ny; ++b) {
          for (int c = 0; c < g.nz; ++c) {
            if ((g.occupied[g.index(a, b, c)] != 0) == target_occupied) {
              const double d2 = (a - i) * (a - i) + (b - j) * (b - j) + (c - k) * (c - k);
              best = std::min(best, d2);
            }
          }
        }
      }
      return g.voxel_size * std::sqrt(best);
    };
  for (int k = 0; k < g.nz; ++k) {
    for (int j = 0; j < g.ny; ++j) {
      for (int i = 0; i < g.nx; ++i) {
        const bool occ = g.occupied[g.index(i, j, k)] != 0;
        const double expected = occ ? -brute(i, j, k, false) : brute(i, j, k, true);
        ASSERT_NEAR(e.distance[e.index(i, j, k)], expected, 1e-5) << i << "," << j << "," << k;
      }
    }
  }
}

TEST(Esdf, SignsAndSingleVoxelDistances)
{
  VoxelGrid g;
  g.voxel_size = 0.1;
  g.nx = g.ny = g.nz = 5;
  g.occupied.assign(125, 0);
  g.occupied[g.index(2, 2, 2)] = 1;
  const EsdfGrid e = computeEsdf(g);
  EXPECT_LT(e.distance[e.index(2, 2, 2)], 0.0F);
  EXPECT_NEAR(e.distance[e.index(4, 2, 2)], 0.2, 1e-6);
  EXPECT_NEAR(e.distance[e.index(3, 3, 2)], 0.1 * std::sqrt(2.0), 1e-6);
}

TEST(Esdf, EmptyAndFullGridsDoNotProduceInfinity)
{
  VoxelGrid empty;
  empty.voxel_size = 0.1;
  empty.nx = empty.ny = empty.nz = 3;
  empty.occupied.assign(27, 0);
  for (float d : computeEsdf(empty).distance) {EXPECT_TRUE(std::isfinite(d)); EXPECT_GT(d, 0.0F);}
  VoxelGrid full = empty;
  full.occupied.assign(27, 1);
  for (float d : computeEsdf(full).distance) {EXPECT_TRUE(std::isfinite(d)); EXPECT_LT(d, 0.0F);}
}

namespace
{
// 7-voxel line along x: [occupied, free, unknown, free, free, free, free]
VoxelGrid lineGrid()
{
  VoxelGrid g;
  g.voxel_size = 1.0;
  g.nx = 7;
  g.ny = g.nz = 1;
  g.occupied = {kVoxelOccupied, kVoxelFree, kVoxelUnknown, kVoxelFree, kVoxelFree, kVoxelFree, kVoxelFree};
  return g;
}
}  // namespace

TEST(EsdfUnobserved, FreePolicyLetsDistancesPropagateThroughUnknown)
{
  const EsdfGrid e = computeEsdf(lineGrid(), UnobservedPolicy::kFree);
  EXPECT_NEAR(e.distance[2], 2.0, 1e-6);  // unknown voxel counted as free: 2 from the obstacle
  EXPECT_NEAR(e.distance[6], 6.0, 1e-6);
}

TEST(EsdfUnobserved, OccupiedPolicyMakesUnknownAnObstacle)
{
  const EsdfGrid e = computeEsdf(lineGrid(), UnobservedPolicy::kOccupied);
  EXPECT_LT(e.distance[2], 0.0F);          // the unknown voxel itself is "inside"
  EXPECT_NEAR(e.distance[3], 1.0, 1e-6);   // free space is now close to the unknown obstacle
  EXPECT_NEAR(e.distance[6], 4.0, 1e-6);
}

TEST(EsdfUnobserved, MarkPolicyKeepsDistancesButFlagsUnknown)
{
  const EsdfGrid e = computeEsdf(lineGrid(), UnobservedPolicy::kMarkUnobserved);
  EXPECT_TRUE(isUnobserved(e.distance[2]));
  EXPECT_FALSE(isUnobserved(e.distance[3]));
  EXPECT_NEAR(e.distance[3], 3.0, 1e-6);   // propagated as if free
}

TEST(EsdfUnobserved, OnlyOccupiedVoxelsBecomePoints)
{
  const PointCloud pts = occupiedCentres(lineGrid());
  ASSERT_EQ(pts.points.size(), 1U);        // the unknown voxel is not an obstacle point
  EXPECT_NEAR(pts.points[0].x(), 0.0, 1e-12);
}
