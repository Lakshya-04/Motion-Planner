#include <gtest/gtest.h>

#include <random>

#include "scene_model/core/plane_fit.hpp"

using namespace scene_model;  // NOLINT

namespace
{
// A wall in the plane y = 2 spanning x in [0, 4], z in [0, 3], with Gaussian thickness noise.
std::vector<Vec3> noisyWall(int n, double sigma, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> ux(0, 4), uz(0, 3);
  std::normal_distribution<double> noise(0, sigma);
  std::vector<Vec3> pts;
  for (int i = 0; i < n; ++i) {pts.emplace_back(ux(rng), 2.0 + noise(rng), uz(rng));}
  return pts;
}
}  // namespace

TEST(PlaneFit, RecoversWallPlaneFrameAndExtents)
{
  PlaneFitParams p;
  p.normal_hint = Vec3(0, -1, 0);  // facing the robot at y < 2
  const auto fit = fitPlane(noisyWall(4000, 0.003, 1), p);
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(fit->normal.dot(Vec3(0, -1, 0)), 1.0, 1e-3);
  EXPECT_NEAR(std::abs(fit->u.x()), 1.0, 1e-3);            // u runs along the wall, horizontal
  EXPECT_NEAR(std::abs(fit->v.z()), 1.0, 1e-3);            // v runs up the wall
  EXPECT_NEAR(fit->u.cross(fit->v).dot(fit->normal), 1.0, 1e-9);  // right-handed
  EXPECT_NEAR(fit->u_max - fit->u_min, 4.0, 0.05);
  EXPECT_NEAR(fit->v_max - fit->v_min, 3.0, 0.05);
  EXPECT_NEAR(fit->rms, 0.003, 0.001);
  EXPECT_LT(fit->spread_p95, 0.01);
  EXPECT_NEAR(fit->origin.y(), 2.0, 0.01);
}

TEST(PlaneFit, RobustToOutliersAndClutter)
{
  auto pts = noisyWall(3000, 0.002, 2);
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> any(-1, 5);
  for (int i = 0; i < 1500; ++i) {pts.emplace_back(any(rng), any(rng), any(rng));}  // 1/3 clutter
  const auto fit = fitPlane(pts);
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(std::abs(fit->normal.y()), 1.0, 5e-3);
  EXPECT_GT(fit->inliers.size(), 2800U);
}

TEST(PlaneFit, FitsALabelAndMapsInliersToFullCloud)
{
  PointCloud c;
  const auto wall = noisyWall(1000, 0.002, 3);
  for (const auto & p : wall) {c.points.push_back(p); c.labels.push_back(5);}
  for (int i = 0; i < 500; ++i) {  // a second wall at x = 8, other label
    c.points.emplace_back(8.0, 0.01 * (i % 50), 0.01 * (i / 50));
    c.labels.push_back(6);
  }
  const auto fit = fitPlaneForLabel(c, 5);
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(std::abs(fit->normal.y()), 1.0, 5e-3);
  for (size_t i : fit->inliers) {EXPECT_EQ(c.labels[i], 5U);}
  const auto other = fitPlaneForLabel(c, 6);
  ASSERT_TRUE(other.has_value());
  EXPECT_NEAR(std::abs(other->normal.x()), 1.0, 5e-3);
}

TEST(PlaneFit, TooFewPointsOrNoPlaneReturnsNothing)
{
  EXPECT_FALSE(fitPlane({Vec3(0, 0, 0), Vec3(1, 0, 0)}).has_value());
  std::mt19937 rng(4);
  std::uniform_real_distribution<double> any(0, 10);
  std::vector<Vec3> cloud;
  for (int i = 0; i < 200; ++i) {cloud.emplace_back(any(rng), any(rng), any(rng));}
  PlaneFitParams p;
  p.inlier_threshold = 0.005;
  p.min_inliers = 150;
  EXPECT_FALSE(fitPlane(cloud, p).has_value());
}

TEST(PlaneFit, HorizontalFloorStillGetsAValidFrame)
{
  std::vector<Vec3> floor;
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) {floor.emplace_back(0.1 * i, 0.1 * j, 0.0);}
  }
  const auto fit = fitPlane(floor);
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(fit->normal.z(), 1.0, 1e-6);  // points up
  EXPECT_NEAR(fit->u.cross(fit->v).dot(fit->normal), 1.0, 1e-9);
  EXPECT_NEAR(fit->u.norm(), 1.0, 1e-9);
}

TEST(PlaneFit, PoseMapsPlaneFrameToCloudFrame)
{
  PlaneFitParams p;
  p.normal_hint = Vec3(0, -1, 0);
  const auto fit = fitPlane(noisyWall(2000, 0.001, 5), p);
  ASSERT_TRUE(fit.has_value());
  const Transform T = fit->pose();
  EXPECT_NEAR((T * Vec3::UnitZ() - T.translation() - fit->normal).norm(), 0.0, 1e-9);
}
