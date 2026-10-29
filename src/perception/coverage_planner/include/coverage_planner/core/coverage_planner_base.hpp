#ifndef COVERAGE_PLANNER__COVERAGE_PLANNER_BASE_HPP_
#define COVERAGE_PLANNER__COVERAGE_PLANNER_BASE_HPP_

#include "core_types/logger.hpp"
#include "coverage_planner/core/types.hpp"
#include <memory>
#include <cstdarg>
#include <string>
#include <vector>

namespace coverage_planner
{


/**
 * @brief Parameters for coverage path planning
 */
struct CoveragePlannerParams
{
  // Coverage parameters
  double swath;                 // Effective coverage width (m)
  double step;                  // Distance between sweep lines (m)
  std::string sweep_direction;  // "horizontal", "vertical", "auto"
  std::string start_corner;     // "top_left", "top_right", "bottom_left", "bottom_right"

  // Obstacle handling
  double obstacle_inflation_radius;  // Inflation radius for obstacles (m)
  bool allow_unknown;                // Treat unknown cells as free?

  // Boundary coverage enhancement
  bool enable_boundary_pass;  // Add perimeter pass for edges (default: false)
  double boundary_offset;     // Offset from boundary for perimeter pass (m)

  // Strip connector waypoints (for boustrophedon)
  bool
    add_strip_connectors;  // Interpolate linear connectors between strip endpoints (default: false)

  // Pose orientation mode
  std::string pose_mode;  // "navigation", "arm", or "identity"

  // Map frame
  std::string map_frame;

  // Optional planning region
  bool use_region;
  PolygonStamped region;

  // Connected component handling
  bool plan_all_components;  // If false, only largest connected free-space component is planned
  int min_component_cells;   // Ignore tiny components below this size

  // Optional path-level optimization
  bool optimize_strip_order;  // Reorder component subpaths to reduce deadhead travel

  // Tool-driven segment metadata (populated by the caller from the active ToolProfile.task).
  // See MotionPlanSegment emission mapping in generatePath(): strip/boundary segments carry this
  // through as requires_contact; connector segments always emit requires_contact=false.
  bool requires_contact{false};
};

/**
 * @brief Result of coverage path planning
 */
struct CoveragePlannerResult
{
  bool success;
  std::string message;
  Path path;

  // Segment metadata aligned with path.poses (start_index/end_index reference the final,
  // post-reordering result.path, not any intermediate per-component path).
  std::vector<MotionPlanSegment> segments;

  // Metadata
  double total_distance;
  double coverage_area;
  int num_waypoints;
  double planning_time;

  // Extended diagnostics (kept internal/topic-published for now)
  int num_strips;
  int num_obstacle_splits;
  double effective_step_used;
  int free_component_count;
};

/**
 * @brief Abstract base class for coverage path planners
 *
 * This interface defines the contract for all coverage planning strategies.
 * New patterns can be added by inheriting from this class and implementing
 * the generatePath() method.
 */
class CoveragePlannerBase
{
public:
  virtual ~CoveragePlannerBase() = default;

  /**
   * @brief Generate a coverage path for the given occupancy grid
   *
   * @param map Input occupancy grid
   * @param params Planning parameters
   * @return CoveragePlannerResult Path and metadata
   */
  virtual CoveragePlannerResult generatePath(
    const OccupancyGrid & map, const CoveragePlannerParams & params) = 0;

  /**
   * @brief Get the name of this planner strategy
   */
  virtual std::string getName() const = 0;

  /// Inject the logging sink. Null (the default) means silent. Core never owns ROS logging.
  void setLogger(std::shared_ptr<const core_types::Logger> logger) { logger_ = std::move(logger); }

protected:
  void logDebug(const char * fmt, ...) const __attribute__((format(printf, 2, 3)));
  void logInfo(const char * fmt, ...) const __attribute__((format(printf, 2, 3)));
  void logWarn(const char * fmt, ...) const __attribute__((format(printf, 2, 3)));

  std::shared_ptr<const core_types::Logger> logger_;

  /**
   * @brief Inflate obstacles in the occupancy grid
   */
  void inflateObstacles(
    std::vector<int8_t> & grid_data, int width, int height, double resolution,
    double inflation_radius, bool allow_unknown);

  /**
   * @brief Check if a cell is free (accounting for unknown cells)
   */
  bool isCellFree(int8_t value, bool allow_unknown) const;

  /**
   * @brief Convert grid coordinates to world coordinates
   * @param x Grid x coordinate
   * @param y Grid y coordinate
   * @param map Occupancy grid
   * @param prev_pose Previous pose (for calculating movement direction)
   * @param pose_mode "navigation", "arm", or "identity"
   */
  PoseStamped gridToWorld(
    int x, int y, const OccupancyGrid & map,
    const PoseStamped * prev_pose = nullptr,
    const std::string & pose_mode = "identity") const;

  /**
   * @brief Calculate path statistics
   */
  void calculatePathStats(
    const Path & path, double & total_distance, int & num_waypoints) const;

  /**
   * @brief Determine sweep direction in world frame considering map rotation
   * @param sweep_direction_param Requested direction ("horizontal", "vertical", "auto")
   * @param map Occupancy grid with rotation information
   * @return true if should sweep along grid Y-axis (vertical in grid), false for X-axis (horizontal in grid)
   */
  bool determineWorldAwareSweepDirection(
    const std::string & sweep_direction_param, const OccupancyGrid & map) const;
};

/**
 * @brief Factory for creating coverage planner instances
 */
class CoveragePlannerFactory
{
public:
  /**
   * @brief Create a planner instance by name
   *
   * @param pattern_name Name of the pattern (e.g., "boustrophedon")
   * @return Unique pointer to planner instance, or nullptr if not found
   */
  static std::unique_ptr<CoveragePlannerBase> createPlanner(const std::string & pattern_name);

  /**
   * @brief Get list of available planner names
   */
  static std::vector<std::string> getAvailablePlanners();
};

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__COVERAGE_PLANNER_BASE_HPP_
