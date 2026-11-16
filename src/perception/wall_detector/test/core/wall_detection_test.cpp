#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>

#include <nlohmann/json.hpp>

#include "scene_model/core/io.hpp"
#include "wall_detector/core/wall_detection.hpp"

using scene_model::Vec3;
using wall_detector::detectWall;

namespace
{
// Wall x = 0.98 (normal -x toward the robot), y in [-0.4, 0.6], z in [0.8, 1.5], range noise sigma
// along x, as a sensor at `eye` would see it.
std::vector<Vec3> wallPoints(int n, double sigma, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> uy(-0.4, 0.6), uz(0.8, 1.5);
  std::normal_distribution<double> noise(0.0, sigma);
  std::vector<Vec3> pts;
  for (int i = 0; i < n; ++i) {
    pts.emplace_back(0.98 + noise(rng), uy(rng), uz(rng));
  }
  return pts;
}
const Vec3 kEye(0.1, 0.0, 1.15);
}  // namespace

TEST(WallDetection, RecoversThePlaneWithinAMillimetreAndAFractionOfADegree)
{
  const auto det = detectWall(wallPoints(5000, 0.004, 3), kEye);
  ASSERT_TRUE(det.has_value());
  EXPECT_GT(det->fit.normal.dot(Vec3(-1, 0, 0)), std::cos(0.2 * M_PI / 180.0));
  EXPECT_NEAR(det->fit.origin.x(), 0.98, 0.001);
  EXPECT_NEAR(-det->fit.normal.dot(det->fit.origin), 0.98, 0.001);  // plane_abcd d for n = -x
  EXPECT_NEAR(det->fit.u_max - det->fit.u_min, 1.0, 0.03);
  EXPECT_NEAR(det->fit.v_max - det->fit.v_min, 0.7, 0.03);
  EXPECT_EQ(det->inlier_points.size(), det->fit.inliers.size());
}

TEST(WallDetection, NormalFacesTheViewpointFromEitherSide)
{
  auto pts = wallPoints(2000, 0.002, 4);
  EXPECT_GT(detectWall(pts, Vec3(2.0, 0, 1.15))->fit.normal.x(), 0.99);
  EXPECT_LT(detectWall(pts, kEye)->fit.normal.x(), -0.99);
}

TEST(WallDetection, IgnoresClutterAndPointsBeyondMaxRange)
{
  auto pts = wallPoints(4000, 0.003, 5);
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> ux(0.2, 0.9), uy(-0.3, 0.5), uz(0.8, 1.5);
  for (int i = 0; i < 800; ++i) {  // an arm and a box in front of the wall
    pts.emplace_back(ux(rng), uy(rng), uz(rng));
  }
  pts.emplace_back(std::nan(""), 0.0, 0.0);
  pts.emplace_back(30.0, 0.0, 1.0);
  wall_detector::WallDetectionParams params;
  params.max_range_m = 5.0;
  const auto det = detectWall(pts, kEye, params);
  ASSERT_TRUE(det.has_value());
  EXPECT_NEAR(det->fit.origin.x(), 0.98, 0.002);
  EXPECT_LT(det->num_points, pts.size());  // NaN and the far point were dropped
}

TEST(WallDetection, NoPlaneInScatteredPoints)
{
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> u(0.0, 2.0);
  std::vector<Vec3> pts;
  for (int i = 0; i < 2000; ++i) {
    pts.emplace_back(u(rng), u(rng), u(rng));
  }
  EXPECT_FALSE(detectWall(pts, kEye).has_value());
  EXPECT_FALSE(detectWall({}, kEye).has_value());
}

TEST(WallDetection, TransformPointsAppliesThePose)
{
  scene_model::Transform t = scene_model::Transform::Identity();
  t.rotate(Eigen::AngleAxisd(M_PI / 2, Vec3::UnitZ()));
  t.pretranslate(Vec3(1, 0, 0));
  const auto out = wall_detector::transformPoints({Vec3(1, 0, 0)}, t);
  EXPECT_NEAR((out[0] - Vec3(1, 1, 0)).norm(), 0.0, 1e-12);
}

TEST(WallScene, JsonHasTheCoveragePlannerSchemaAndTheRightPlane)
{
  const auto det = detectWall(wallPoints(5000, 0.003, 6), kEye);
  ASSERT_TRUE(det.has_value());
  const auto scene = wall_detector::wallSceneJson(det->fit, 0, "base_footprint", "test");
  EXPECT_EQ(scene["frame_id"], "base_footprint");
  ASSERT_EQ(scene["walls"].size(), 1u);
  EXPECT_TRUE(scene["obstacles"].empty());
  const auto & w = scene["walls"][0];
  for (const char * key : {"id", "plane_abcd", "origin_xyz", "basis_u", "basis_v", "normal_outward",
      "u_range_m", "v_range_m", "plane_rms_mm", "spread_p95_mm", "spread_p99_mm"})
  {
    EXPECT_TRUE(w.contains(key)) << key;
  }
  // plane_abcd . (x, 1) = 0 on the wall (same convention as test/data/wall_scene.json: [-1 0 0 0.98])
  EXPECT_NEAR(w["plane_abcd"][0].get<double>(), -1.0, 0.003);
  EXPECT_NEAR(w["plane_abcd"][3].get<double>(), 0.98, 0.001);
  const Vec3 u(w["basis_u"][0], w["basis_u"][1], w["basis_u"][2]);
  const Vec3 v(w["basis_v"][0], w["basis_v"][1], w["basis_v"][2]);
  const Vec3 n(w["normal_outward"][0], w["normal_outward"][1], w["normal_outward"][2]);
  EXPECT_NEAR(u.cross(v).dot(n), 1.0, 1e-9);  // right handed, as the loader's basis encoding needs
  EXPECT_NEAR(std::abs(u.y()), 1.0, 0.01);    // u runs along the wall horizontally
  EXPECT_NEAR(std::abs(v.z()), 1.0, 0.01);
  EXPECT_LT(w["plane_rms_mm"].get<double>(), 15.0);  // within the loader's flatness gate
}

TEST(WallScene, WriteRoundTripsThroughTheSceneModelLoaders)
{
  const auto det = detectWall(wallPoints(3000, 0.003, 7), kEye);
  ASSERT_TRUE(det.has_value());
  const std::string dir = ::testing::TempDir() + "/wall_detector_out";
  const auto path = wall_detector::writeWallScene(dir, det->fit, det->inlier_points, 3, "base_footprint", "t");
  EXPECT_FALSE(std::filesystem::exists(dir + "/walls.pcd.tmp"));
  EXPECT_FALSE(std::filesystem::exists(dir + "/wall_scene.json.tmp"));
  std::ifstream f(path);
  const auto json = nlohmann::json::parse(f);
  EXPECT_EQ(json["walls"][0]["id"], 3);
  const auto cloud = scene_model::loadPcd(dir + "/walls.pcd");
  ASSERT_EQ(cloud.points.size(), det->inlier_points.size());
  ASSERT_EQ(cloud.labels.size(), cloud.points.size());
  EXPECT_EQ(cloud.labels[0], 3u);
  // writing again replaces the files
  EXPECT_NO_THROW(wall_detector::writeWallScene(dir, det->fit, det->inlier_points, 3, "base_footprint", "t"));
}
