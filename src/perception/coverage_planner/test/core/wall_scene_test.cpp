#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "coverage_planner/core/wall_basis.hpp"
#include "coverage_planner/core/wall_rasterizer.hpp"
#include "coverage_planner/core/wall_scene_loader.hpp"

using namespace coverage_planner;

namespace
{

#ifndef WALL_SCENE_SAMPLE_PATH
#error "WALL_SCENE_SAMPLE_PATH must be provided by CMake (-D)"
#endif

// Real perception export, checked into the repo -- exercises the full parser against actual
// field values rather than a hand-authored fixture.
const std::string kSampleScenePath = WALL_SCENE_SAMPLE_PATH;

WallPlane makeSyntheticWall()
{
  WallPlane wall;
  wall.id = 0;
  wall.pose.header.frame_id = "map";
  wall.pose.pose = coverage_planner::encodeWallBasis(
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0),
    Eigen::Vector3d(0.0, -1.0, 0.0));
  wall.u_min = 0.0;
  wall.u_max = 2.0;
  wall.v_min = 0.0;
  wall.v_max = 1.0;
  wall.plane_rms_mm = 1.0;
  wall.spread_p95_mm = 2.0;
  wall.spread_p99_mm = 3.0;
  return wall;
}

ObstacleRegion makeCenteredObstacle()
{
  ObstacleRegion obstacle;
  obstacle.id = 0;
  obstacle.parent_wall = 0;
  obstacle.u_min = 0.9;
  obstacle.u_max = 1.1;
  obstacle.v_min = 0.4;
  obstacle.v_max = 0.6;
  return obstacle;
}

}  // namespace

TEST(WallSceneLoader, LoadsRealSampleWallZero)
{
  const auto query = coverage_planner::loadWallScene(kSampleScenePath, 0);

  EXPECT_EQ(query.wall.id, 0u);
  EXPECT_EQ(query.wall.pose.header.frame_id, "link_base");
  EXPECT_NEAR(query.wall.u_min, -5.555562, 1e-5);
  EXPECT_NEAR(query.wall.u_max, 5.159225, 1e-5);
  EXPECT_NEAR(query.wall.v_min, -2.293865, 1e-5);
  EXPECT_NEAR(query.wall.v_max, 1.586123, 1e-5);
  EXPECT_TRUE(query.flatness_warning.empty());

  ASSERT_FALSE(query.obstacles.empty());
  bool found_obstacle_zero = false;
  for (const auto & obstacle : query.obstacles) {
    EXPECT_EQ(obstacle.parent_wall, 0u);
    if (obstacle.id == 0) {
      found_obstacle_zero = true;
      EXPECT_NEAR(obstacle.u_min, 0.770234, 1e-5);
      EXPECT_NEAR(obstacle.u_max, 0.978103, 1e-5);
      EXPECT_NEAR(obstacle.v_min, -1.732319, 1e-5);
      EXPECT_NEAR(obstacle.v_max, -1.504945, 1e-5);
    }
  }
  EXPECT_TRUE(found_obstacle_zero);
}

TEST(WallSceneLoader, ThrowsForUnknownWallId)
{
  EXPECT_THROW(coverage_planner::loadWallScene(kSampleScenePath, 999), std::runtime_error);
}

TEST(WallSceneLoader, ThrowsWhenFlatnessThresholdExceeded)
{
  coverage_planner::WallFlatnessThresholds strict;
  strict.max_spread_p99_mm = 10.0;  // wall 0's real spread_p99_mm is 66.55 -- must reject
  EXPECT_THROW(
    coverage_planner::loadWallScene(kSampleScenePath, 0, strict), std::runtime_error);
}

TEST(WallSceneLoader, AllowOverrideReturnsWarningInsteadOfThrowing)
{
  coverage_planner::WallFlatnessThresholds lenient;
  lenient.max_spread_p99_mm = 10.0;
  lenient.allow_override = true;
  const auto query = coverage_planner::loadWallScene(kSampleScenePath, 0, lenient);
  EXPECT_FALSE(query.flatness_warning.empty());
}

TEST(WallRasterizer, GridDimensionsAndOriginMatchWallBasis)
{
  const auto wall = makeSyntheticWall();
  coverage_planner::ToolCoverage coverage;
  coverage.obstacle_inflation_radius_m = 0.0;

  const auto grid = coverage_planner::rasterizeWallCoverageGrid(wall, {}, coverage, 0.1, "map");

  EXPECT_EQ(grid.info.width, 20u);
  EXPECT_EQ(grid.info.height, 10u);
  EXPECT_NEAR(grid.info.resolution, 0.1, 1e-6);
  // origin = wall.origin + basis_u * u_min + basis_v * v_min = (0,0,0) here (u_min = v_min = 0).
  EXPECT_NEAR(grid.info.origin.position.x, 0.0, 1e-6);
  EXPECT_NEAR(grid.info.origin.position.y, 0.0, 1e-6);
  EXPECT_NEAR(grid.info.origin.position.z, 0.0, 1e-6);
  // All cells free (no obstacles).
  for (auto cell : grid.data) {
    EXPECT_LT(cell, 50);
  }
}

TEST(WallRasterizer, UniformInflationExpandsBothAxes)
{
  const auto wall = makeSyntheticWall();
  coverage_planner::ToolCoverage coverage;
  coverage.obstacle_inflation = "uniform";
  coverage.obstacle_inflation_radius_m = 0.15;
  coverage.sweep_direction = "vertical";

  const auto grid = coverage_planner::rasterizeWallCoverageGrid(
    wall, {makeCenteredObstacle()}, coverage, 0.1, "map");

  // Obstacle occupies u in [0.9, 1.1], v in [0.4, 0.6]; uniform inflation by 0.15 expands both
  // axes to u in [0.75, 1.25], v in [0.25, 0.75]. A cell just outside the ORIGINAL box but inside
  // the inflated one (e.g. u=0.8, v=0.5) must be occupied; one clearly outside both must stay free.
  const int width = static_cast<int>(grid.info.width);
  auto cellAt = [&](double u, double v) {
    const int x = static_cast<int>(std::floor((u - wall.u_min) / grid.info.resolution));
    const int y = static_cast<int>(std::floor((v - wall.v_min) / grid.info.resolution));
    return grid.data[static_cast<size_t>(y) * width + x];
  };

  EXPECT_GT(cellAt(0.8, 0.5), 50);  // inside inflated box, outside original -- must be occupied
  EXPECT_LT(cellAt(0.1, 0.1), 50);  // far away -- must stay free
}

TEST(WallRasterizer, PrimaryAxisInflationOnlyExpandsSweepAxis)
{
  const auto wall = makeSyntheticWall();
  coverage_planner::ToolCoverage coverage;
  coverage.obstacle_inflation = "primary_axis";
  coverage.obstacle_inflation_radius_m = 0.15;
  coverage.sweep_direction = "vertical";  // primary axis = v -- only v should expand

  const auto grid = coverage_planner::rasterizeWallCoverageGrid(
    wall, {makeCenteredObstacle()}, coverage, 0.1, "map");

  const int width = static_cast<int>(grid.info.width);
  auto cellAt = [&](double u, double v) {
    const int x = static_cast<int>(std::floor((u - wall.u_min) / grid.info.resolution));
    const int y = static_cast<int>(std::floor((v - wall.v_min) / grid.info.resolution));
    return grid.data[static_cast<size_t>(y) * width + x];
  };

  // v axis (sweep axis) inflated: v=0.3 is outside the original [0.4,0.6] box but inside the
  // inflated [0.25, 0.75] range -- must be occupied.
  EXPECT_GT(cellAt(1.0, 0.3), 50);
  // u axis NOT inflated: u=0.8 is outside the original [0.9,1.1] box and primary_axis mode must
  // not expand u -- must stay free.
  EXPECT_LT(cellAt(0.8, 0.5), 50);
}

TEST(WallBasis, EncodeAcceptsAnOrthonormalRightHandedBasis)
{
  EXPECT_NO_THROW(coverage_planner::encodeWallBasis(
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 1.0)));
}

TEST(WallBasis, EncodeRejectsALeftHandedBasis)
{
  // Same three axes as the accepted case above, but normal_outward flipped -- exactly the
  // "perception export flips the normal" failure mode this guard exists to catch.
  EXPECT_THROW(
    coverage_planner::encodeWallBasis(
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(0.0, 0.0, -1.0)),
    std::runtime_error);
}

TEST(WallBasis, EncodeRejectsNonOrthogonalAxes)
{
  EXPECT_THROW(
    coverage_planner::encodeWallBasis(
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.7, 0.7, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0)),
    std::runtime_error);
}

TEST(WallBasis, EncodeRejectsNonUnitAxes)
{
  EXPECT_THROW(
    coverage_planner::encodeWallBasis(
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0)),
    std::runtime_error);
}
