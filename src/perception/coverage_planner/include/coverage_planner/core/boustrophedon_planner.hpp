#ifndef COVERAGE_PLANNER__BOUSTROPHEDON_PLANNER_HPP_
#define COVERAGE_PLANNER__BOUSTROPHEDON_PLANNER_HPP_

#include "coverage_planner/core/types.hpp"
#include "coverage_planner/core/coverage_planner_base.hpp"

namespace coverage_planner
{


/**
 * @brief Boustrophedon (lawn-mower) coverage path planner
 *
 * Generates parallel sweeps across the free space with alternating directions.
 *
 * Key Concepts:
 * - SWATH: The effective coverage width of the robot's tool/sensor
 * - STEP: The spacing between parallel sweep lines (typically <= swath for overlap)
 * - Pattern: Alternating back-and-forth sweeps to minimize turns
 *
 * Example with swath=1.0m, step=0.8m (20% overlap):
 *
 *   ┌─────────────────┐
 *   │→→→→→→→→→→→→→→→→ │  Line 1 (y=0)
 *   │                 │
 *   │←←←←←←←←←←←←←←←← │  Line 2 (y=0.8m)
 *   │                 │
 *   │→→→→→→→→→→→→→→→→ │  Line 3 (y=1.6m)
 *   └─────────────────┘
 */
class BoustrophedonPlanner : public CoveragePlannerBase
{
public:
  BoustrophedonPlanner() = default;
  virtual ~BoustrophedonPlanner() = default;

  CoveragePlannerResult generatePath(
    const OccupancyGrid & map, const CoveragePlannerParams & params) override;

  std::string getName() const override { return "boustrophedon"; }

private:
  /**
   * @brief Extract free space bounds from the map
   */
  bool extractFreeSpaceBounds(
    const OccupancyGrid & map, const std::vector<int8_t> & inflated_grid,
    bool allow_unknown, int & min_x, int & max_x, int & min_y, int & max_y);

  /**
   * @brief Generate horizontal sweeps
   */
  void generateHorizontalSweeps(
    const OccupancyGrid & map, const std::vector<int8_t> & inflated_grid,
    const CoveragePlannerParams & params, int min_x, int max_x, int min_y, int max_y,
    Path & path, std::vector<MotionPlanSegment> & segments);

  /**
   * @brief Generate vertical sweeps
   */
  void generateVerticalSweeps(
    const OccupancyGrid & map, const std::vector<int8_t> & inflated_grid,
    const CoveragePlannerParams & params, int min_x, int max_x, int min_y, int max_y,
    Path & path, std::vector<MotionPlanSegment> & segments);

  /**
   * @brief Check if a line segment is mostly free
   */
  bool isLineFree(
    const std::vector<int8_t> & grid, int width, int x1, int y1, int x2, int y2,
    bool allow_unknown) const;

  /**
   * @brief Generate boundary perimeter pass for edge coverage
   */
  void generateBoundaryPass(
    const OccupancyGrid & map, const std::vector<int8_t> & inflated_grid,
    const CoveragePlannerParams & params, int min_x, int max_x, int min_y, int max_y,
    Path & path, std::vector<MotionPlanSegment> & segments);

  /**
   * @brief Trim sweep line to free space boundaries
   */
  void trimSweepLine(
    const std::vector<int8_t> & grid, int width, int & start, int & end, int fixed_coord,
    bool is_horizontal, bool allow_unknown) const;

  /**
   * @brief Add interpolated waypoints between strips in grid space
   */
  bool addStripConnectorWaypoints(
    Path & path, const OccupancyGrid & map,
    const std::vector<int8_t> & inflated_grid, const CoveragePlannerParams & params,
    int current_end_x, int current_end_y, int next_start_x, int next_start_y,
    const PoseStamped * reference_pose,
    std::vector<MotionPlanSegment> & segments);

  /**
   * @brief BFS shortest path (4-connected, free cells only) from (start_x, start_y) to
   *        (end_x, end_y) -- routes around obstacles, unlike addStripConnectorWaypoints()'s own
   *        straight-line check, which rejects the connector outright if anything blocks it.
   * @return The path including both endpoints, in grid cells; empty if unreachable.
   */
  std::vector<std::pair<int, int>> findObstacleAwareConnectorPath(
    const OccupancyGrid & map, const std::vector<int8_t> & inflated_grid,
    const CoveragePlannerParams & params, int start_x, int start_y, int end_x, int end_y) const;
};

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__BOUSTROPHEDON_PLANNER_HPP_
