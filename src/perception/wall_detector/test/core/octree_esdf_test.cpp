#include <gtest/gtest.h>

#include "wall_detector/core/octree_esdf.hpp"

using wall_detector::EsdfWindow;

// A slab at x in [0.5, 0.56) in a 3 cm octree: signed distances in front of, inside and behind it.
TEST(OctreeEsdf, SlabDistances)
{
  octomap::OcTree tree(0.03);
  for (double y = -0.5; y < 0.5; y += 0.015) {
    for (double z = -0.5; z < 0.5; z += 0.015) {
      for (double x = 0.5; x < 0.56; x += 0.015) {
        tree.updateNode(octomap::point3d(x, y, z), true);
      }
    }
  }
  EsdfWindow w;
  w.center = Eigen::Vector3d(0.5, 0, 0);
  w.dims = Eigen::Vector3d(1.0, 0.6, 0.6);
  w.voxel_size = 0.03;
  const auto esdf = wall_detector::octreeToEsdf(tree, w);
  ASSERT_EQ(esdf.distance.size(), static_cast<size_t>(esdf.nx) * esdf.ny * esdf.nz);
  auto at = [&](double x, double y, double z) {
      const int i = static_cast<int>((x - esdf.origin.x()) / esdf.voxel_size);
      const int j = static_cast<int>((y - esdf.origin.y()) / esdf.voxel_size);
      const int k = static_cast<int>((z - esdf.origin.z()) / esdf.voxel_size);
      return esdf.distance[esdf.index(i, j, k)];
    };
  EXPECT_LT(at(0.53, 0, 0), 0.0f);
  EXPECT_NEAR(at(0.30, 0, 0), 0.21, 0.05);
  EXPECT_NEAR(at(0.90, 0, 0), 0.36, 0.05);
  EXPECT_GT(at(0.30, 0, 0), 0.0f);
}
