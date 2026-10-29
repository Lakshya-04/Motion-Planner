#ifndef COVERAGE_PLANNER__WALL_RASTERIZER_HPP_
#define COVERAGE_PLANNER__WALL_RASTERIZER_HPP_

#include "coverage_planner/core/types.hpp"
#include <cstdint>
#include <string>
#include <vector>


#include "coverage_planner/core/tool_coverage_profile.hpp"

namespace coverage_planner
{


/**
 * @brief Builds an OccupancyGrid origin (position + orientation) from a wall's own basis, so that
 *        CoveragePlannerBase::gridToWorld()'s existing rotation-matrix transform lifts grid
 *        (u, v) cells straight into the wall's actual pose in `wall.frame_id`.
 */
Pose buildWallGridOrigin(const WallPlane & wall);

/**
 * @brief Rasterize one wall's (u, v) extent into a nav_msgs::OccupancyGrid, with obstacles
 *        inflated per the active tool's coverage-facet policy, ready for the boustrophedon
 *        planner. Operates entirely in 2D -- never touches orientation or standoff.
 * @param wall The wall to rasterize (its own u_min/u_max/v_min/v_max define the grid extent).
 * @param obstacles Obstacles already filtered to this wall's `parent_wall`.
 * @param tool_coverage The active tool's coverage facet -- `obstacle_inflation` selects uniform
 *        (contact tools, expand both axes) vs. `primary_axis` (non-contact, expand only along
 *        the sweep direction).
 * @param resolution Grid cell size in meters.
 * @param frame_id Frame to stamp the resulting grid's header with.
 * @param observed_mask Optional, `width * height` row-major mask (see loadWallObservedMask())
 *        marking which cells actually have measured wall material. Empty (default) treats every
 *        cell in the u/v rectangle as free. Non-empty cells outside the mask start as unknown
 *        (-1) rather than free, so a partially-scanned wall isn't swept as if fully solid.
 * @param reachable_mask Optional, same shape as `observed_mask`, marking which cells are
 *        IK-reachable from the robot's current base pose (a cell needs both to stay free).
 * @throws std::runtime_error if `observed_mask` or `reachable_mask` is non-empty but its size
 *         doesn't match `width * height`.
 */
OccupancyGrid rasterizeWallCoverageGrid(
  const WallPlane & wall, const std::vector<ObstacleRegion> & obstacles,
  const coverage_planner::ToolCoverage & tool_coverage, double resolution,
  const std::string & frame_id, const std::vector<bool> & observed_mask = {},
  const std::vector<bool> & reachable_mask = {});

/**
 * @brief Build a `width * height` row-major "was this cell actually scanned" mask for one wall,
 *        straight from its own point cloud (`label` field matches `WallPlane::id`).
 * @details A real, partially-observed wall's u/v bounding rectangle can be mostly empty --
 *          feeding the whole rectangle to the boustrophedon planner as "free" sweeps space with
 *          no wall behind it. `width`/`height`/`resolution` must match the grid this mask will
 *          be passed into.
 * @throws std::runtime_error if `pcd_path` cannot be read as a PCL point cloud, or has no `label`
 *         field.
 */
std::vector<bool> loadWallObservedMask(
  const std::string & pcd_path, uint32_t wall_id, const WallPlane & wall, int width,
  int height, double resolution);

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__WALL_RASTERIZER_HPP_
