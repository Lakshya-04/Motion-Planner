#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "coverage_planner/core/coverage_from_scan.hpp"
#include "coverage_planner/core/wall_basis.hpp"

using namespace coverage_planner;  // NOLINT

namespace
{
// A wall in the plane y = 2 spanning x in [0, 4], z in [0, 3], densely sampled with noise.
scene_model::PointCloud wallScan(unsigned seed = 1)
{
  scene_model::PointCloud cloud;
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, 0.002);
  for (double x = 0.0; x <= 4.0; x += 0.02) {
    for (double z = 0.0; z <= 3.0; z += 0.02) {
      cloud.points.emplace_back(x, 2.0 + noise(rng), z);
    }
  }
  return cloud;
}

ToolCoverageProfile contactTool()
{
  ToolCoverageProfile t;
  t.name = "wipe";
  t.task.contact = true;
  t.task.standoff_m = 0.0;
  t.coverage.swath_m = 0.3;
  t.coverage.overlap_pct = 20.0;
  t.coverage.sweep_direction = "vertical";
  t.coverage.obstacle_inflation_radius_m = 0.1;
  t.coverage.passes = {PassVariant{}};
  return t;
}

ScanCoverageOptions facingRobot()
{
  ScanCoverageOptions o;
  o.resolution = 0.05;
  o.plane.normal_hint = scene_model::Vec3(0, -1, 0);  // wall faces the robot at y < 2
  return o;
}

scene_model::SceneObject boxObstacle(const std::string & id, scene_model::Vec3 size, scene_model::Vec3 at)
{
  scene_model::SceneObject o;
  o.id = id;
  o.geometry = scene_model::Box{size};
  o.pose.translation() = at;
  return o;
}
}  // namespace

TEST(CoverageFromScan, WallPlaneFromFitMatchesTheScan)
{
  const auto fit = scene_model::fitPlane(wallScan().points, facingRobot().plane);
  ASSERT_TRUE(fit.has_value());
  const WallPlane wall = wallFromPlaneFit(*fit, 3, "map");
  EXPECT_EQ(wall.id, 3U);
  EXPECT_NEAR(wall.u_max - wall.u_min, 4.0, 0.05);
  EXPECT_NEAR(wall.v_max - wall.v_min, 3.0, 0.05);
  const WallBasis b = decomposeWallBasis(wall);
  EXPECT_NEAR(b.normal_outward.dot(Eigen::Vector3d(0, -1, 0)), 1.0, 1e-3);
  EXPECT_LT(wall.plane_rms_mm, 5.0);
}

TEST(CoverageFromScan, PlansAContactPassOverTheWholeWall)
{
  const auto result = planCoverageFromScan(wallScan(), scene_model::SceneModel(), contactTool(), facingRobot());
  ASSERT_TRUE(result.success) << result.message;
  ASSERT_GT(result.final_plan.path.poses.size(), 20U);
  ASSERT_FALSE(result.final_plan.segments.empty());

  // Contact tool: every pose sits on the wall plane (y = 2), tool z pointing into the wall.
  for (const auto & p : result.final_plan.path.poses) {
    EXPECT_NEAR(p.pose.position.y, 2.0, 0.02);
    EXPECT_GE(p.pose.position.x, -0.05);
    EXPECT_LE(p.pose.position.x, 4.05);
    EXPECT_GE(p.pose.position.z, -0.05);
    EXPECT_LE(p.pose.position.z, 3.05);
    const Eigen::Quaterniond q(
      p.pose.orientation.w, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z);
    EXPECT_GT((q * Eigen::Vector3d::UnitZ()).dot(Eigen::Vector3d(0, 1, 0)), 0.99);
  }
  // Strips are spaced by step = 0.3 * 0.8 = 0.24 m along x: expect about 4.0 / 0.24 strips.
  double min_x = 1e9, max_x = -1e9;
  for (const auto & p : result.final_plan.path.poses) {
    min_x = std::min(min_x, p.pose.position.x);
    max_x = std::max(max_x, p.pose.position.x);
  }
  EXPECT_LT(min_x, 0.3);
  EXPECT_GT(max_x, 3.7);
}

TEST(CoverageFromScan, StandoffToolHoversInFrontOfTheWall)
{
  ToolCoverageProfile spray = contactTool();
  spray.task.contact = false;
  spray.task.standoff_m = 0.3;
  const auto result = planCoverageFromScan(wallScan(), scene_model::SceneModel(), spray, facingRobot());
  ASSERT_TRUE(result.success) << result.message;
  for (const auto & p : result.final_plan.path.poses) {
    EXPECT_NEAR(p.pose.position.y, 1.7, 0.02);  // 0.3 m in front, toward the robot
  }
}

TEST(CoverageFromScan, PathAvoidsAnObstacleInFrontOfTheWall)
{
  scene_model::SceneModel scene("map");
  scene.add(boxObstacle("cabinet", {0.5, 0.3, 0.6}, {2.0, 1.85, 1.5}));  // touches the wall zone
  const auto with = planCoverageFromScan(wallScan(), scene, contactTool(), facingRobot());
  const auto without = planCoverageFromScan(wallScan(), scene_model::SceneModel(), contactTool(), facingRobot());
  ASSERT_TRUE(with.success) << with.message;
  ASSERT_TRUE(without.success) << without.message;

  // Footprint on the wall: x in [1.75, 2.25], z in [1.2, 1.8]. No waypoint may land inside it.
  for (const auto & seg : with.final_plan.segments) {
    const auto & a = with.final_plan.path.poses[seg.start_index].pose.position;
    const auto & b = with.final_plan.path.poses[seg.end_index].pose.position;
    for (int i = 0; i <= 50; ++i) {  // strips only store their end points: sample along them
      const double t = i / 50.0;
      const double x = a.x + t * (b.x - a.x), z = a.z + t * (b.z - a.z);
      EXPECT_FALSE(x > 1.75 && x < 2.25 && z > 1.2 && z < 1.8) << x << ", " << z;
    }
  }
  // And the obstacle really changed the plan (the same strips pass through it without it).
  bool free_plan_crosses = false;
  for (const auto & seg : without.final_plan.segments) {
    const auto & a = without.final_plan.path.poses[seg.start_index].pose.position;
    const auto & b = without.final_plan.path.poses[seg.end_index].pose.position;
    for (int i = 0; i <= 50; ++i) {  // strips only store their end points: sample along them
      const double t = i / 50.0;
      const double x = a.x + t * (b.x - a.x), z = a.z + t * (b.z - a.z);
      if (x > 1.75 && x < 2.25 && z > 1.2 && z < 1.8) {free_plan_crosses = true;}
    }
  }
  EXPECT_TRUE(free_plan_crosses);
  EXPECT_FALSE(with.grid.data.empty());
}

TEST(CoverageFromScan, ObstacleRegionsIgnoreFarAndBehindObjects)
{
  const auto fit = scene_model::fitPlane(wallScan().points, facingRobot().plane);
  const WallPlane wall = wallFromPlaneFit(*fit, 0, "map");
  scene_model::SceneModel scene;
  scene.add(boxObstacle("near", {0.4, 0.2, 0.4}, {1.0, 1.8, 1.0}));      // 0.1 in front: kept
  scene.add(boxObstacle("far", {0.4, 0.2, 0.4}, {1.0, 0.5, 1.0}));       // 1.4 in front: dropped
  scene.add(boxObstacle("behind", {0.4, 0.2, 0.4}, {1.0, 3.0, 1.0}));    // behind the wall
  const auto regions = obstacleRegionsFromScene(scene, wall);
  ASSERT_EQ(regions.size(), 1U);
  EXPECT_LT(regions[0].u_min, regions[0].u_max);
}

TEST(CoverageFromScan, ReportsAReasonWhenThereIsNoWall)
{
  scene_model::PointCloud sparse;
  sparse.points = {scene_model::Vec3(0, 0, 0), scene_model::Vec3(1, 1, 1)};
  const auto result = planCoverageFromScan(sparse, scene_model::SceneModel(), contactTool(), facingRobot());
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("no wall"), std::string::npos);
}

TEST(CoverageFromScan, LabelSelectsOneWallOfASegmentedScan)
{
  scene_model::PointCloud scan = wallScan();
  scan.labels.assign(scan.points.size(), 4U);
  for (int i = 0; i < 400; ++i) {  // a second wall elsewhere, different label
    scan.points.emplace_back(9.0, 0.02 * (i % 20), 0.02 * (i / 20));
    scan.labels.push_back(5U);
  }
  ScanCoverageOptions o = facingRobot();
  o.label = 4U;
  const auto result = planCoverageFromScan(scan, scene_model::SceneModel(), contactTool(), o);
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_NEAR(result.wall.u_max - result.wall.u_min, 4.0, 0.06);
}
