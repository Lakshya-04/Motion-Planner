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

TEST(WallRasterizer, KeepOutIsNoWiderThanTheCellsTheInflatedBoxOverlaps)
{
  const auto wall = makeSyntheticWall();
  coverage_planner::ToolCoverage coverage;
  coverage.obstacle_inflation = "uniform";
  coverage.obstacle_inflation_radius_m = 0.15;
  coverage.sweep_direction = "vertical";

  // Inflated box u in [0.75, 1.25], v in [0.25, 0.75] on a 0.1 m grid: cells 7.5..12.5 and 2.5..7.5
  // overlap it, i.e. indices 7..12 and 2..7 (cell 13 and cell 8 in v start beyond the box).
  const auto grid = coverage_planner::rasterizeWallCoverageGrid(
    wall, {makeCenteredObstacle()}, coverage, 0.1, "map");
  const int width = static_cast<int>(grid.info.width);
  int blocked_x_min = width, blocked_x_max = -1, blocked_y_min = 1000, blocked_y_max = -1;
  for (size_t i = 0; i < grid.data.size(); ++i) {
    if (grid.data[i] > 50) {
      const int x = static_cast<int>(i) % width;
      const int y = static_cast<int>(i) / width;
      blocked_x_min = std::min(blocked_x_min, x);
      blocked_x_max = std::max(blocked_x_max, x);
      blocked_y_min = std::min(blocked_y_min, y);
      blocked_y_max = std::max(blocked_y_max, y);
    }
  }
  EXPECT_EQ(blocked_x_min, 7);
  EXPECT_EQ(blocked_x_max, 12);
  EXPECT_EQ(blocked_y_min, 2);
  EXPECT_EQ(blocked_y_max, 7);
}

namespace
{
// Blocked cell index ranges {x_min, x_max, y_min, y_max}; x_min > x_max when nothing is blocked.
std::array<int, 4> blockedRange(const coverage_planner::OccupancyGrid & grid)
{
  const int width = static_cast<int>(grid.info.width);
  std::array<int, 4> r = {width, -1, 1000, -1};
  for (size_t i = 0; i < grid.data.size(); ++i) {
    if (grid.data[i] > 50) {
      const int x = static_cast<int>(i) % width;
      const int y = static_cast<int>(i) / width;
      r[0] = std::min(r[0], x);
      r[1] = std::max(r[1], x);
      r[2] = std::min(r[2], y);
      r[3] = std::max(r[3], y);
    }
  }
  return r;
}

coverage_planner::ToolCoverage noInflation()
{
  coverage_planner::ToolCoverage c;
  c.obstacle_inflation = "uniform";
  c.obstacle_inflation_radius_m = 0.0;
  c.sweep_direction = "vertical";
  return c;
}
}  // namespace

TEST(WallRasterizer, EdgesExactlyOnGridLinesBlockOnlyTheCellsInside)
{
  // u 0.5..0.6 and v 0.3..0.4 on a 0.1 m grid with no inflation: exactly cell 5 in u and cell 3 in v.
  auto obstacle = makeCenteredObstacle();
  obstacle.u_min = 0.5;
  obstacle.u_max = 0.6;
  obstacle.v_min = 0.3;
  obstacle.v_max = 0.4;
  const auto grid = coverage_planner::rasterizeWallCoverageGrid(
    makeSyntheticWall(), {obstacle}, noInflation(), 0.1, "map");
  const auto r = blockedRange(grid);
  EXPECT_EQ(r[0], 5);
  EXPECT_EQ(r[1], 5);
  EXPECT_EQ(r[2], 3);
  EXPECT_EQ(r[3], 3);
}

TEST(WallRasterizer, ObstacleOutsideTheWallBlocksNothing)
{
  auto obstacle = makeCenteredObstacle();
  obstacle.u_min = 2.5;   // wall u runs 0..2
  obstacle.u_max = 3.0;
  const auto grid = coverage_planner::rasterizeWallCoverageGrid(
    makeSyntheticWall(), {obstacle}, noInflation(), 0.1, "map");
  EXPECT_GT(blockedRange(grid)[0], blockedRange(grid)[1]);   // empty range
  auto left = makeCenteredObstacle();
  left.u_min = -1.0;
  left.u_max = -0.5;
  const auto grid2 = coverage_planner::rasterizeWallCoverageGrid(
    makeSyntheticWall(), {left}, noInflation(), 0.1, "map");
  EXPECT_GT(blockedRange(grid2)[0], blockedRange(grid2)[1]);
}

TEST(WallRasterizer, ObstacleOverTheBorderIsClippedToTheGrid)
{
  auto obstacle = makeCenteredObstacle();
  obstacle.u_min = 1.75;   // wall u_max is 2.0: cells 17..19 only
  obstacle.u_max = 2.5;
  const auto grid = coverage_planner::rasterizeWallCoverageGrid(
    makeSyntheticWall(), {obstacle}, noInflation(), 0.1, "map");
  const auto r = blockedRange(grid);
  EXPECT_EQ(r[0], 17);
  EXPECT_EQ(r[1], 19);
}

TEST(WallRasterizer, NonFiniteObstacleBoundsAreRejectedNotIgnored)
{
  auto obstacle = makeCenteredObstacle();
  obstacle.u_max = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    coverage_planner::rasterizeWallCoverageGrid(makeSyntheticWall(), {obstacle}, noInflation(), 0.1, "map"),
    std::runtime_error);
  obstacle = makeCenteredObstacle();
  obstacle.v_min = -std::numeric_limits<double>::infinity();
  EXPECT_THROW(
    coverage_planner::rasterizeWallCoverageGrid(makeSyntheticWall(), {obstacle}, noInflation(), 0.1, "map"),
    std::runtime_error);
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
