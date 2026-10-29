#include <Eigen/Dense>
#include <cmath>
#include <queue>
#include "coverage_planner/core/boustrophedon_planner.hpp"
#include "coverage_planner/core/sweepline_planner.hpp"
#include <cstdarg>
#include <cstdio>

#include "coverage_planner/core/coverage_planner_base.hpp"

namespace coverage_planner
{
namespace
{
std::string vformat(const char * fmt, va_list args)
{
  va_list copy;
  va_copy(copy, args);
  const int n = std::vsnprintf(nullptr, 0, fmt, copy);
  va_end(copy);
  if (n <= 0) {
    return {};
  }
  std::string out(static_cast<size_t>(n), '\0');
  std::vsnprintf(out.data(), out.size() + 1, fmt, args);
  return out;
}
}  // namespace

void CoveragePlannerBase::logDebug(const char * fmt, ...) const
{
  if (!logger_) {
    return;
  }
  va_list args;
  va_start(args, fmt);
  logger_->debug(vformat(fmt, args));
  va_end(args);
}

void CoveragePlannerBase::logInfo(const char * fmt, ...) const
{
  if (!logger_) {
    return;
  }
  va_list args;
  va_start(args, fmt);
  logger_->info(vformat(fmt, args));
  va_end(args);
}

void CoveragePlannerBase::logWarn(const char * fmt, ...) const
{
  if (!logger_) {
    return;
  }
  va_list args;
  va_start(args, fmt);
  logger_->warn(vformat(fmt, args));
  va_end(args);
}


void CoveragePlannerBase::inflateObstacles(
  std::vector<int8_t> & grid_data, int width, int height, double resolution,
  double inflation_radius, bool allow_unknown)
{
  if (inflation_radius <= 0.0) return;

  int inflation_cells = static_cast<int>(std::ceil(inflation_radius / resolution));
  std::vector<int8_t> inflated_grid = grid_data;

  // Find all obstacle cells
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int index = y * width + x;

      // If cell is occupied
      // Respect allow_unknown semantics consistently:
      // - unknown cells are treated as obstacles only when unknown is not allowed.
      if (grid_data[index] > 50 || (grid_data[index] == -1 && !allow_unknown)) {
        // Inflate in a square around the obstacle
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            int new_x = x + dx;
            int new_y = y + dy;

            if (new_x >= 0 && new_x < width && new_y >= 0 && new_y < height) {
              double dist = std::sqrt(dx * dx + dy * dy) * resolution;
              if (dist <= inflation_radius) {
                int new_index = new_y * width + new_x;
                // Mark as obstacle if not already
                if (inflated_grid[new_index] < 50) {
                  inflated_grid[new_index] = 100;  // Mark as inflated obstacle
                }
              }
            }
          }
        }
      }
    }
  }

  grid_data = inflated_grid;
}

bool CoveragePlannerBase::isCellFree(int8_t value, bool allow_unknown) const
{
  if (value == -1)  // Unknown
    return allow_unknown;
  return value < 50;  // Free space threshold
}

PoseStamped CoveragePlannerBase::gridToWorld(
  int x, int y, const OccupancyGrid & map,
  const PoseStamped * prev_pose, const std::string & pose_mode) const
{
  (void)prev_pose;  // Unused - orientation computed by application layer
  (void)pose_mode;  // Unused - orientation computed by application layer

  PoseStamped pose;
  pose.header = map.header;

  // Convert grid coordinates to world coordinates
  // Grid coordinates are in the occupancy grid's local frame (X, Y, Z=0)
  // We need to transform them using the grid's origin pose (position + rotation)

  // Step 1: Get local position in grid frame (centered in cell)
  double local_x = (x + 0.5) * map.info.resolution;
  double local_y = (y + 0.5) * map.info.resolution;
  double local_z = 0.0;

  // Step 2: Convert origin quaternion to rotation matrix
  const auto & origin = map.info.origin;
  Eigen::Quaterniond q(
    origin.orientation.w, origin.orientation.x, origin.orientation.y, origin.orientation.z);
  Eigen::Matrix3d rot_matrix = q.toRotationMatrix();

  // Step 3: Transform local coordinates to world frame
  Eigen::Vector3d local_point(local_x, local_y, local_z);
  Eigen::Vector3d world_point = rot_matrix * local_point;

  // Step 4: Add origin position to get final world coordinates
  pose.pose.position.x = origin.position.x + world_point.x();
  pose.pose.position.y = origin.position.y + world_point.y();
  pose.pose.position.z = origin.position.z + world_point.z();

  // NOTE: Orientation is set to identity here
  // The coverage planner only provides 2D path positions (transformed to 3D world frame)
  // Orientation computation is delegated to the application layer (e.g., wall_cleaning_action_server)
  // which has access to wall normal and can compute proper tool orientation for Cartesian motion
  //
  // This separation of concerns allows:
  // - Coverage planner focuses on geometric path planning in 3D space
  // - Application computes task-specific orientations (e.g., normal to wall surface)
  //
  // Previously attempted to compute orientations here but was commented out since:
  // - Planner doesn't have wall normal information
  // - Movement direction alone doesn't determine correct tool orientation for wall cleaning

  pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;

  return pose;
}

void CoveragePlannerBase::calculatePathStats(
  const Path & path, double & total_distance, int & num_waypoints) const
{
  total_distance = 0.0;
  num_waypoints = static_cast<int>(path.poses.size());

  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & p1 = path.poses[i - 1].pose.position;
    const auto & p2 = path.poses[i].pose.position;

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;

    total_distance += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
}

bool CoveragePlannerBase::determineWorldAwareSweepDirection(
  const std::string & sweep_direction_param, const OccupancyGrid & map) const
{
  // The wall occupancy grid is already properly oriented:
  // - Grid X-axis = u_axis (horizontal along wall, typically left-right)
  // - Grid Y-axis = v_axis (vertical along wall, typically up-down)
  // - Grid Z-axis = normal (perpendicular to wall surface)
  //
  // This orientation is set by the wall detection/grid generation code which
  // analyzes the wall's principal directions and aligns:
  //   u_axis = horizontal direction along wall (world X or Y depending on wall orientation)
  //   v_axis = vertical direction (typically aligned with world Z for vertical walls)
  //
  // Therefore, we can directly map sweep_direction to grid axes:
  //   "vertical" -> sweep along grid Y-axis (v_axis = vertical up/down)
  //   "horizontal" -> sweep along grid X-axis (u_axis = horizontal left/right)

  (void)map;  // Map rotation already handled in grid generation

  if (sweep_direction_param == "vertical") {
    // Vertical sweep = along grid Y-axis = generateVerticalSweeps()
    return true;
  } else if (sweep_direction_param == "horizontal") {
    // Horizontal sweep = along grid X-axis = generateHorizontalSweeps()
    return false;
  } else  // "auto"
  {
    // Auto mode: choose direction with fewer sweep lines
    // Favor vertical sweeps if height >= width (typical for walls)
    return map.info.height >= map.info.width;
  }
}

// Factory Implementation
std::unique_ptr<CoveragePlannerBase> CoveragePlannerFactory::createPlanner(
  const std::string & pattern_name)
{
  if (pattern_name == "boustrophedon") {
    return std::make_unique<BoustrophedonPlanner>();
  }
  if (pattern_name == "sweepline") {
    return std::make_unique<SweeplinePlanner>();
  }
  // Add more planners here as they are implemented:
  // else if (pattern_name == "spiral")
  // {
  //   return std::make_unique<SpiralPlanner>();
  // }
  // else if (pattern_name == "zigzag")
  // {
  //   return std::make_unique<ZigzagPlanner>();
  // }

  return nullptr;
}

std::vector<std::string> CoveragePlannerFactory::getAvailablePlanners()
{
  return {
    "boustrophedon",
    "sweepline"
    // Add more patterns here as implemented
    // "spiral",
    // "zigzag"
  };
}

}  // namespace coverage_planner
