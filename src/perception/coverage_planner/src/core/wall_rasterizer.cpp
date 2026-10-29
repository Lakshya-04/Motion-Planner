#include "coverage_planner/wall_rasterizer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#include "coverage_planner/wall_basis.hpp"

namespace coverage_planner
{

// The interfaces live in coverage_planner_msgs; alias them into this package's own
// namespace so `msg::`/`srv::` still mean what they always did here.
namespace msg = coverage_planner_msgs::msg;

namespace
{

struct InflatedObstacle
{
  double u_min;
  double u_max;
  double v_min;
  double v_max;
};

InflatedObstacle inflate(
  const msg::ObstacleRegion & obstacle, const coverage_planner::ToolCoverage & tool_coverage)
{
  InflatedObstacle inflated{obstacle.u_min, obstacle.u_max, obstacle.v_min, obstacle.v_max};
  const double margin = tool_coverage.obstacle_inflation_radius_m;
  if (margin <= 0.0) {
    return inflated;
  }

  if (tool_coverage.obstacle_inflation == "uniform") {
    inflated.u_min -= margin;
    inflated.u_max += margin;
    inflated.v_min -= margin;
    inflated.v_max += margin;
  } else if (tool_coverage.obstacle_inflation == "primary_axis") {
    // The fan is long perpendicular to travel: only the axis a strip actually travels along
    // needs clearance added, not the axis strips are stacked along.
    if (tool_coverage.sweep_direction == "horizontal") {
      inflated.u_min -= margin;
      inflated.u_max += margin;
    } else {
      inflated.v_min -= margin;
      inflated.v_max += margin;
    }
  }
  return inflated;
}

}  // namespace

geometry_msgs::msg::Pose buildWallGridOrigin(const msg::WallPlane & wall)
{
  const WallBasis basis = decomposeWallBasis(wall);
  const Eigen::Vector3d grid_origin =
    basis.origin + basis.basis_u * wall.u_min + basis.basis_v * wall.v_min;
  return encodeWallBasis(grid_origin, basis.basis_u, basis.basis_v, basis.normal_outward);
}

nav_msgs::msg::OccupancyGrid rasterizeWallCoverageGrid(
  const msg::WallPlane & wall, const std::vector<msg::ObstacleRegion> & obstacles,
  const coverage_planner::ToolCoverage & tool_coverage, double resolution,
  const std::string & frame_id, const std::vector<bool> & observed_mask,
  const std::vector<bool> & reachable_mask)
{
  if (resolution <= 0.0) {
    throw std::runtime_error("rasterizeWallCoverageGrid: resolution must be > 0");
  }
  const double u_extent = wall.u_max - wall.u_min;
  const double v_extent = wall.v_max - wall.v_min;
  if (u_extent <= 0.0 || v_extent <= 0.0) {
    throw std::runtime_error("rasterizeWallCoverageGrid: wall has non-positive u/v extent");
  }

  const int width = std::max(1, static_cast<int>(std::ceil(u_extent / resolution)));
  const int height = std::max(1, static_cast<int>(std::ceil(v_extent / resolution)));
  const size_t num_cells = static_cast<size_t>(width) * static_cast<size_t>(height);

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = frame_id;
  grid.info.resolution = static_cast<float>(resolution);
  grid.info.width = static_cast<uint32_t>(width);
  grid.info.height = static_cast<uint32_t>(height);
  grid.info.origin = buildWallGridOrigin(wall);

  if (!observed_mask.empty() && observed_mask.size() != num_cells) {
    throw std::runtime_error(
      "rasterizeWallCoverageGrid: observed_mask size does not match width * height");
  }
  if (!reachable_mask.empty() && reachable_mask.size() != num_cells) {
    throw std::runtime_error(
      "rasterizeWallCoverageGrid: reachable_mask size does not match width * height");
  }

  grid.data.assign(num_cells, 0);
  if (!observed_mask.empty()) {
    for (size_t i = 0; i < num_cells; ++i) {
      if (!observed_mask[i]) {
        grid.data[i] = -1;
      }
    }
  }
  if (!reachable_mask.empty()) {
    // 100 (obstacle), not -1 (unknown): CoveragePlannerBase::isCellFree treats -1 as free
    // whenever allow_unknown is set, which exists specifically to bridge small unscanned gaps --
    // an IK-unreachable cell is a proven negative, not an unscanned one, and must stay
    // non-traversable regardless of that flag.
    for (size_t i = 0; i < num_cells; ++i) {
      if (!reachable_mask[i]) {
        grid.data[i] = 100;
      }
    }
  }

  for (const auto & obstacle : obstacles) {
    const InflatedObstacle inflated = inflate(obstacle, tool_coverage);

    const int x_min =
      std::max(0, static_cast<int>(std::floor((inflated.u_min - wall.u_min) / resolution)));
    const int x_max =
      std::min(width - 1, static_cast<int>(std::ceil((inflated.u_max - wall.u_min) / resolution)));
    const int y_min =
      std::max(0, static_cast<int>(std::floor((inflated.v_min - wall.v_min) / resolution)));
    const int y_max =
      std::min(height - 1, static_cast<int>(std::ceil((inflated.v_max - wall.v_min) / resolution)));

    for (int y = y_min; y <= y_max; ++y) {
      for (int x = x_min; x <= x_max; ++x) {
        grid.data[static_cast<size_t>(y) * width + x] = 100;
      }
    }
  }

  return grid;
}

std::vector<bool> loadWallObservedMask(
  const std::string & pcd_path, uint32_t wall_id, const msg::WallPlane & wall, int width,
  int height, double resolution)
{
  pcl::PointCloud<pcl::PointXYZL> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZL>(pcd_path, cloud) < 0) {
    throw std::runtime_error("loadWallObservedMask: failed to load point cloud from " + pcd_path);
  }

  const WallBasis basis = decomposeWallBasis(wall);

  std::vector<bool> mask(static_cast<size_t>(width) * static_cast<size_t>(height), false);
  for (const auto & point : cloud.points) {
    if (point.label != wall_id) {
      continue;
    }
    const Eigen::Vector3d p(point.x, point.y, point.z);
    const double u = (p - basis.origin).dot(basis.basis_u);
    const double v = (p - basis.origin).dot(basis.basis_v);
    const int x = static_cast<int>(std::floor((u - wall.u_min) / resolution));
    const int y = static_cast<int>(std::floor((v - wall.v_min) / resolution));
    if (x >= 0 && x < width && y >= 0 && y < height) {
      mask[static_cast<size_t>(y) * width + x] = true;
    }
  }
  return mask;
}

}  // namespace coverage_planner
