#include "coverage_planner/core/wall_rasterizer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

#include "coverage_planner/core/wall_basis.hpp"

namespace coverage_planner
{


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
  const ObstacleRegion & obstacle, const coverage_planner::ToolCoverage & tool_coverage)
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

Pose buildWallGridOrigin(const WallPlane & wall)
{
  const WallBasis basis = decomposeWallBasis(wall);
  const Eigen::Vector3d grid_origin =
    basis.origin + basis.basis_u * wall.u_min + basis.basis_v * wall.v_min;
  return encodeWallBasis(grid_origin, basis.basis_u, basis.basis_v, basis.normal_outward);
}

OccupancyGrid rasterizeWallCoverageGrid(
  const WallPlane & wall, const std::vector<ObstacleRegion> & obstacles,
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

  OccupancyGrid grid;
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
    // A NaN or infinite bound would cast to an arbitrary int and silently block nothing (it fails open).
    if (!std::isfinite(inflated.u_min) || !std::isfinite(inflated.u_max) ||
      !std::isfinite(inflated.v_min) || !std::isfinite(inflated.v_max))
    {
      throw std::runtime_error(
        "rasterizeWallCoverageGrid: obstacle " + std::to_string(obstacle.id) + " has a non-finite bound");
    }

    // Every cell the inflated rectangle overlaps, and no more: [floor(min), ceil(max) - 1] in cell
    // indices (the cell past ceil(max) starts beyond the rectangle; it used to be blocked too, which
    // made the keep-out up to one cell wider than the clearance asked for on each side).
    constexpr double kEps = 1e-9;
    const int x_min = std::max(
      0, static_cast<int>(std::floor((inflated.u_min - wall.u_min) / resolution + kEps)));
    const int x_max = std::min(
      width - 1, static_cast<int>(std::ceil((inflated.u_max - wall.u_min) / resolution - kEps)) - 1);
    const int y_min = std::max(
      0, static_cast<int>(std::floor((inflated.v_min - wall.v_min) / resolution + kEps)));
    const int y_max = std::min(
      height - 1, static_cast<int>(std::ceil((inflated.v_max - wall.v_min) / resolution - kEps)) - 1);

    for (int y = y_min; y <= y_max; ++y) {
      for (int x = x_min; x <= x_max; ++x) {
        grid.data[static_cast<size_t>(y) * width + x] = 100;
      }
    }
  }

  return grid;
}

}  // namespace coverage_planner
