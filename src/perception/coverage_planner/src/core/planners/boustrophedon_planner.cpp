#include "coverage_planner/boustrophedon_planner.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <coverage_planner_msgs/msg/motion_plan_segment.hpp>
#include <numeric>
#include <queue>
#include <rclcpp/rclcpp.hpp>

namespace coverage_planner
{

// The interfaces live in coverage_planner_msgs; alias them into this package's own
// namespace so `msg::`/`srv::` still mean what they always did here.
namespace msg = coverage_planner_msgs::msg;
namespace
{

Eigen::Vector2d gridCellCenterToWorld2D(int x, int y, const nav_msgs::msg::OccupancyGrid & map)
{
  const double local_x = (x + 0.5) * map.info.resolution;
  const double local_y = (y + 0.5) * map.info.resolution;
  const auto & origin = map.info.origin;

  Eigen::Quaterniond q(
    origin.orientation.w, origin.orientation.x, origin.orientation.y, origin.orientation.z);
  Eigen::Matrix3d rot = q.toRotationMatrix();
  Eigen::Vector3d world = rot * Eigen::Vector3d(local_x, local_y, 0.0);
  world.x() += origin.position.x;
  world.y() += origin.position.y;
  return Eigen::Vector2d(world.x(), world.y());
}

bool pointInPolygon2D(
  const Eigen::Vector2d & point, const std::vector<geometry_msgs::msg::Point32> & polygon)
{
  if (polygon.size() < 3) {
    return false;
  }

  bool inside = false;
  size_t j = polygon.size() - 1;
  for (size_t i = 0; i < polygon.size(); j = i++) {
    const double xi = polygon[i].x;
    const double yi = polygon[i].y;
    const double xj = polygon[j].x;
    const double yj = polygon[j].y;
    const bool intersect = ((yi > point.y()) != (yj > point.y())) &&
                           (point.x() < ((xj - xi) * (point.y() - yi) / ((yj - yi) + 1e-9) + xi));
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}

std::vector<std::vector<int>> extractConnectedComponents(
  const std::vector<uint8_t> & free_mask, int width, int height, int min_component_cells)
{
  std::vector<std::vector<int>> components;
  std::vector<uint8_t> visited(free_mask.size(), 0);
  const int min_cells = std::max(1, min_component_cells);

  const int dx[4] = {1, -1, 0, 0};
  const int dy[4] = {0, 0, 1, -1};

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int idx = y * width + x;
      if (!free_mask[idx] || visited[idx]) {
        continue;
      }

      std::queue<int> q;
      std::vector<int> component;
      q.push(idx);
      visited[idx] = 1;

      while (!q.empty()) {
        const int current = q.front();
        q.pop();
        component.push_back(current);

        const int cx = current % width;
        const int cy = current / width;
        for (int k = 0; k < 4; ++k) {
          const int nx = cx + dx[k];
          const int ny = cy + dy[k];
          if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
            continue;
          }
          const int nidx = ny * width + nx;
          if (!free_mask[nidx] || visited[nidx]) {
            continue;
          }
          visited[nidx] = 1;
          q.push(nidx);
        }
      }

      if (static_cast<int>(component.size()) >= min_cells) {
        components.push_back(std::move(component));
      }
    }
  }

  std::sort(components.begin(), components.end(), [](const auto & a, const auto & b) {
    return a.size() > b.size();
  });

  return components;
}

double poseDistance(
  const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b)
{
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  const double dz = a.pose.position.z - b.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Segment-emission mapping decided in implementation_plan_manipulation_architecture.md's
// coverage-planner redesign notes (reference_docs/13_cleaning_modes_spec.md SS2.2's landing map):
// strips and the boundary pass are cartesian/contact-driven; connectors are fsp/ompl and always
// force a retraction (never contact-adjacent).
msg::MotionPlanSegment makeSegment(
  const std::string & label, uint32_t start_index, uint32_t end_index, const std::string & frame_id,
  const std::string & mode, const std::string & planner, bool requires_contact,
  bool requires_retraction, const std::vector<std::string> & tags)
{
  msg::MotionPlanSegment segment;
  segment.label = label;
  segment.start_index = start_index;
  segment.end_index = end_index;
  segment.render_start_index = start_index;
  segment.render_end_index = end_index;
  segment.frame_id = frame_id;
  segment.mode = mode;
  segment.planner = planner;
  segment.requires_contact = requires_contact;
  segment.requires_retraction = requires_retraction;
  segment.priority = 0.0f;
  segment.tags = tags;
  return segment;
}

msg::MotionPlanSegment makeStripSegment(
  const std::string & label, uint32_t start_index, uint32_t end_index, const std::string & frame_id,
  const CoveragePlannerParams & params, const std::vector<std::string> & tags)
{
  return makeSegment(
    label, start_index, end_index, frame_id, "cartesian", "cartesian", params.requires_contact,
    false, tags);
}

msg::MotionPlanSegment makeConnectorSegment(
  const std::string & label, uint32_t start_index, uint32_t end_index, const std::string & frame_id)
{
  return makeSegment(
    label, start_index, end_index, frame_id, "fsp", "ompl", false, true, {"connector"});
}

}  // namespace

std::vector<std::pair<int, int>> BoustrophedonPlanner::findObstacleAwareConnectorPath(
  const nav_msgs::msg::OccupancyGrid & map, const std::vector<int8_t> & inflated_grid,
  const CoveragePlannerParams & params, int start_x, int start_y, int end_x, int end_y) const
{
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);

  const auto is_free = [&](int x, int y) {
    if (x < 0 || x >= width || y < 0 || y >= height) {
      return false;
    }
    const int index = y * width + x;
    return index >= 0 && index < static_cast<int>(inflated_grid.size()) &&
           isCellFree(inflated_grid[index], params.allow_unknown);
  };

  if (!is_free(start_x, start_y) || !is_free(end_x, end_y)) {
    return {};
  }
  if (start_x == end_x && start_y == end_y) {
    return {{start_x, start_y}};
  }

  const size_t num_cells = static_cast<size_t>(width) * static_cast<size_t>(height);
  const auto to_index = [width](int x, int y) { return y * width + x; };
  std::vector<int> parent(num_cells, -1);
  std::vector<bool> visited(num_cells, false);
  std::queue<std::pair<int, int>> frontier;

  visited[static_cast<size_t>(to_index(start_x, start_y))] = true;
  frontier.push({start_x, start_y});

  static constexpr int kDx[4] = {1, -1, 0, 0};
  static constexpr int kDy[4] = {0, 0, 1, -1};

  bool found = (start_x == end_x && start_y == end_y);
  while (!frontier.empty() && !found) {
    const auto [current_x, current_y] = frontier.front();
    frontier.pop();
    for (int dir = 0; dir < 4; ++dir) {
      const int next_x = current_x + kDx[dir];
      const int next_y = current_y + kDy[dir];
      if (!is_free(next_x, next_y)) {
        continue;
      }
      const size_t next_index = static_cast<size_t>(to_index(next_x, next_y));
      if (visited[next_index]) {
        continue;
      }
      visited[next_index] = true;
      parent[next_index] = to_index(current_x, current_y);
      if (next_x == end_x && next_y == end_y) {
        found = true;
        break;
      }
      frontier.push({next_x, next_y});
    }
  }

  if (!visited[static_cast<size_t>(to_index(end_x, end_y))]) {
    return {};
  }

  std::vector<std::pair<int, int>> path;
  int current = to_index(end_x, end_y);
  const int start_index = to_index(start_x, start_y);
  while (current != start_index) {
    path.emplace_back(current % width, current / width);
    current = parent[static_cast<size_t>(current)];
  }
  path.emplace_back(start_x, start_y);
  std::reverse(path.begin(), path.end());
  return path;
}

bool BoustrophedonPlanner::addStripConnectorWaypoints(
  nav_msgs::msg::Path & path, const nav_msgs::msg::OccupancyGrid & map,
  const std::vector<int8_t> & inflated_grid, const CoveragePlannerParams & params,
  int current_end_x, int current_end_y, int next_start_x, int next_start_y,
  const geometry_msgs::msg::PoseStamped * reference_pose,
  std::vector<msg::MotionPlanSegment> & segments)
{
  // Fast path: the direct line, exactly as before -- cheap, and correct whenever nothing is
  // actually in the way.
  std::vector<std::pair<int, int>> connector_cells;
  bool direct_line_blocked = false;
  {
    const int dx = std::abs(next_start_x - current_end_x);
    const int sx = (current_end_x < next_start_x) ? 1 : -1;
    const int dy = -std::abs(next_start_y - current_end_y);
    const int sy = (current_end_y < next_start_y) ? 1 : -1;

    int x = current_end_x;
    int y = current_end_y;
    int err = dx + dy;
    connector_cells.reserve(static_cast<size_t>(std::max(dx, -dy)));

    while (!(x == next_start_x && y == next_start_y)) {
      const int e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y += sy;
      }

      if (x == next_start_x && y == next_start_y) {
        break;
      }

      const int grid_x = x;
      const int grid_y = y;
      if (
        grid_x < 0 || grid_x >= static_cast<int>(map.info.width) || grid_y < 0 ||
        grid_y >= static_cast<int>(map.info.height)) {
        direct_line_blocked = true;
        break;
      }

      const int index = grid_y * map.info.width + grid_x;
      if (!isCellFree(inflated_grid[index], params.allow_unknown)) {
        direct_line_blocked = true;
        break;
      }
      connector_cells.emplace_back(grid_x, grid_y);
    }
  }

  // Fallback: the direct line crosses whatever originally split this strip (or the next one's
  // start) -- by definition, exactly the case a straight-line check alone can never satisfy.
  // Route around it through the surrounding free space instead of giving up on the connector.
  if (direct_line_blocked) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("BoustrophedonPlanner"),
      "Connector [%d,%d]->[%d,%d]: direct line blocked, searching for a free-space route around "
      "it",
      current_end_x, current_end_y, next_start_x, next_start_y);
    auto routed = findObstacleAwareConnectorPath(
      map, inflated_grid, params, current_end_x, current_end_y, next_start_x, next_start_y);
    if (routed.empty()) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("BoustrophedonPlanner"),
        "Connector [%d,%d]->[%d,%d]: no free-space route exists either -- rejecting", current_end_x,
        current_end_y, next_start_x, next_start_y);
      return false;
    }
    // Strip both endpoints -- current_end_x/y and next_start_x/y are already each segment's own
    // endpoint pose, same convention the direct-line path above already follows.
    connector_cells.clear();
    if (routed.size() > 2) {
      connector_cells.assign(routed.begin() + 1, routed.end() - 1);
    }
  }

  if (connector_cells.empty()) {
    return false;
  }

  // connector_cells excludes both true endpoints (they already exist as poses belonging to the
  // neighboring strip segments) to avoid duplicating poses. start_index/end_index below cover only
  // this segment's own interior poses, so a copy/extract over them never duplicates a neighbor's
  // pose. render_start_index/render_end_index separately widen by one index on each side (the
  // strip's own end pose, already path.poses.back(); and the index the caller's own next-strip-
  // start pose will land at once it pushes it immediately after this call returns) purely so
  // per-segment line rendering also draws the two joint edges.
  const uint32_t interior_start_index = static_cast<uint32_t>(path.poses.size());
  for (const auto & cell : connector_cells) {
    const auto * prev_pose = !path.poses.empty() ? &path.poses.back() : reference_pose;
    auto connector_pose = gridToWorld(cell.first, cell.second, map, prev_pose, params.pose_mode);
    path.poses.push_back(connector_pose);
  }
  const uint32_t interior_end_index = static_cast<uint32_t>(path.poses.size()) - 1;
  const uint32_t render_start_index =
    interior_start_index > 0 ? interior_start_index - 1 : interior_start_index;
  const uint32_t render_end_index = interior_end_index + 1;
  auto connector_segment = makeConnectorSegment(
    "connector_" + std::to_string(segments.size()), interior_start_index, interior_end_index,
    map.header.frame_id);
  connector_segment.render_start_index = render_start_index;
  connector_segment.render_end_index = render_end_index;
  segments.push_back(connector_segment);
  return true;
}

CoveragePlannerResult BoustrophedonPlanner::generatePath(
  const nav_msgs::msg::OccupancyGrid & map, const CoveragePlannerParams & params)
{
  CoveragePlannerResult result;
  result.success = false;
  result.total_distance = 0.0;
  result.coverage_area = 0.0;
  result.num_waypoints = 0;
  result.planning_time = 0.0;
  result.num_strips = 0;
  result.num_obstacle_splits = 0;
  result.effective_step_used = params.step;
  result.free_component_count = 0;

  auto start_time = std::chrono::steady_clock::now();

  // Validate map
  if (map.data.empty() || map.info.width == 0 || map.info.height == 0) {
    result.message = "Invalid or empty map";
    return result;
  }

  // Create inflated grid
  std::vector<int8_t> inflated_grid = map.data;

  // Count obstacles before inflation
  int obstacle_count_before = 0;
  for (const auto & cell : inflated_grid) {
    if (cell > 50) obstacle_count_before++;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("BoustrophedonPlanner"), "Map size: %dx%d, Obstacles before inflation: %d",
    map.info.width, map.info.height, obstacle_count_before);

  if (params.obstacle_inflation_radius > 0.0) {
    inflateObstacles(
      inflated_grid, map.info.width, map.info.height, map.info.resolution,
      params.obstacle_inflation_radius, params.allow_unknown);

    // Count obstacles after inflation
    int obstacle_count_after = 0;
    for (const auto & cell : inflated_grid) {
      if (cell > 50) obstacle_count_after++;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("BoustrophedonPlanner"), "Obstacles after inflation (radius=%.2fm): %d",
      params.obstacle_inflation_radius, obstacle_count_after);
  }

  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  std::vector<uint8_t> free_mask(inflated_grid.size(), 0);
  for (int idx = 0; idx < static_cast<int>(inflated_grid.size()); ++idx) {
    free_mask[idx] = isCellFree(inflated_grid[idx], params.allow_unknown) ? 1 : 0;
  }

  // Optional polygon clipping (region request)
  if (params.use_region && params.region.polygon.points.size() >= 3) {
    const std::string region_frame = params.region.header.frame_id;
    if (!region_frame.empty() && region_frame != map.header.frame_id) {
      RCLCPP_WARN(
        rclcpp::get_logger("BoustrophedonPlanner"),
        "Region frame '%s' != map frame '%s'; ignoring region clipping", region_frame.c_str(),
        map.header.frame_id.c_str());
    } else {
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          const int idx = y * width + x;
          if (!free_mask[idx]) {
            continue;
          }
          const auto world_xy = gridCellCenterToWorld2D(x, y, map);
          if (!pointInPolygon2D(world_xy, params.region.polygon.points)) {
            free_mask[idx] = 0;
          }
        }
      }
    }
  }

  auto components =
    extractConnectedComponents(free_mask, width, height, params.min_component_cells);
  if (components.empty()) {
    result.message = "No free space found after clipping/filtering";
    return result;
  }
  result.free_component_count = static_cast<int>(components.size());
  {
    std::string sizes;
    for (size_t i = 0; i < components.size() && i < 10; ++i) {
      sizes += std::to_string(components[i].size());
      if (i + 1 < components.size() && i + 1 < 10) sizes += ", ";
    }
    RCLCPP_INFO(
      rclcpp::get_logger("BoustrophedonPlanner"),
      "Free-space components: %zu total (min_component_cells=%d, allow_unknown=%s), "
      "largest 10 sizes: [%s]",
      components.size(), params.min_component_cells, params.allow_unknown ? "true" : "false",
      sizes.c_str());
  }

  // Initialize path
  result.path.header = map.header;  // Use the map's frame_id directly
  // Stamp is left as map.header's own -- this planner has no node handle to source a clock from,
  // and rclcpp::Clock() here would be a detached system clock that ignores use_sim_time. The
  // caller (coverage_planner_node) re-stamps the response path with its own now() instead.

  // Determine sweep direction in world frame (considering map rotation)
  bool use_vertical_sweeps = determineWorldAwareSweepDirection(params.sweep_direction, map);

  RCLCPP_INFO(
    rclcpp::get_logger("BoustrophedonPlanner"),
    "Sweep direction '%s' -> using %s sweeps (map rotation aware)", params.sweep_direction.c_str(),
    use_vertical_sweeps ? "vertical (grid Y-axis)" : "horizontal (grid X-axis)");

  std::vector<size_t> selected_components;
  if (params.plan_all_components) {
    selected_components.resize(components.size());
    std::iota(selected_components.begin(), selected_components.end(), 0);
  } else {
    selected_components.push_back(0);  // Largest component only.
  }

  std::vector<nav_msgs::msg::Path> component_paths;
  component_paths.reserve(selected_components.size());
  std::vector<std::vector<msg::MotionPlanSegment>> component_segments_list;
  component_segments_list.reserve(selected_components.size());

  for (size_t component_id : selected_components) {
    std::vector<int8_t> component_grid(inflated_grid.size(), 100);
    for (int idx : components[component_id]) {
      component_grid[idx] = inflated_grid[idx];
    }

    int min_x, max_x, min_y, max_y;
    if (!extractFreeSpaceBounds(
          map, component_grid, params.allow_unknown, min_x, max_x, min_y, max_y)) {
      continue;
    }

    nav_msgs::msg::Path component_path;
    component_path.header = map.header;
    std::vector<msg::MotionPlanSegment> component_segments;

    if (params.enable_boundary_pass) {
      RCLCPP_INFO(
        rclcpp::get_logger("BoustrophedonPlanner"),
        "Generating boundary pass for component %zu (offset=%.3fm)", component_id,
        params.boundary_offset);
      generateBoundaryPass(
        map, component_grid, params, min_x, max_x, min_y, max_y, component_path,
        component_segments);
    }

    if (use_vertical_sweeps) {
      generateVerticalSweeps(
        map, component_grid, params, min_x, max_x, min_y, max_y, component_path,
        component_segments);
    } else {
      generateHorizontalSweeps(
        map, component_grid, params, min_x, max_x, min_y, max_y, component_path,
        component_segments);
    }

    if (!component_path.poses.empty()) {
      result.num_strips += static_cast<int>(component_path.poses.size() / 2);
      component_paths.push_back(std::move(component_path));
      component_segments_list.push_back(std::move(component_segments));
    }
  }

  if (component_paths.empty()) {
    result.message = "No valid coverage path found in selected free-space components";
    return result;
  }

  // Appends one component's segments into result.segments, remapping start_index/end_index from
  // local (0-based) numbering to their final position once component paths are concatenated
  // (and, under optimize_strip_order, sometimes reversed).
  auto appendComponentSegments = [&result](
                                   const std::vector<msg::MotionPlanSegment> & component_segments,
                                   size_t component_size, size_t offset, bool reversed) {
    for (auto segment : component_segments) {
      if (reversed) {
        const uint32_t new_start = static_cast<uint32_t>(component_size - 1 - segment.end_index);
        const uint32_t new_end = static_cast<uint32_t>(component_size - 1 - segment.start_index);
        segment.start_index = static_cast<uint32_t>(offset) + new_start;
        segment.end_index = static_cast<uint32_t>(offset) + new_end;
      } else {
        segment.start_index += static_cast<uint32_t>(offset);
        segment.end_index += static_cast<uint32_t>(offset);
      }
      result.segments.push_back(std::move(segment));
    }
  };

  // Optional component-subpath ordering optimization.
  if (params.optimize_strip_order && component_paths.size() > 1) {
    std::vector<uint8_t> used(component_paths.size(), 0);
    size_t current_idx = 0;
    used[current_idx] = 1;
    appendComponentSegments(
      component_segments_list[current_idx], component_paths[current_idx].poses.size(), 0, false);
    result.path.poses.insert(
      result.path.poses.end(), component_paths[current_idx].poses.begin(),
      component_paths[current_idx].poses.end());

    while (true) {
      bool found_next = false;
      size_t best_idx = 0;
      bool reverse_candidate = false;
      double best_dist = std::numeric_limits<double>::max();
      const auto & current_end = result.path.poses.back();

      for (size_t i = 0; i < component_paths.size(); ++i) {
        if (used[i] || component_paths[i].poses.empty()) {
          continue;
        }

        const auto & start_pose = component_paths[i].poses.front();
        const auto & end_pose = component_paths[i].poses.back();
        const double dist_to_start = poseDistance(current_end, start_pose);
        const double dist_to_end = poseDistance(current_end, end_pose);
        const bool should_reverse = dist_to_end < dist_to_start;
        const double dist = should_reverse ? dist_to_end : dist_to_start;
        if (dist < best_dist) {
          best_dist = dist;
          best_idx = i;
          reverse_candidate = should_reverse;
          found_next = true;
        }
      }

      if (!found_next) {
        break;
      }

      used[best_idx] = 1;
      appendComponentSegments(
        component_segments_list[best_idx], component_paths[best_idx].poses.size(),
        result.path.poses.size(), reverse_candidate);
      if (reverse_candidate) {
        for (auto it = component_paths[best_idx].poses.rbegin();
             it != component_paths[best_idx].poses.rend(); ++it) {
          result.path.poses.push_back(*it);
        }
      } else {
        result.path.poses.insert(
          result.path.poses.end(), component_paths[best_idx].poses.begin(),
          component_paths[best_idx].poses.end());
      }
    }
  } else {
    for (size_t i = 0; i < component_paths.size(); ++i) {
      appendComponentSegments(
        component_segments_list[i], component_paths[i].poses.size(), result.path.poses.size(),
        false);
      result.path.poses.insert(
        result.path.poses.end(), component_paths[i].poses.begin(), component_paths[i].poses.end());
    }
  }

  // Calculate statistics
  auto end_time = std::chrono::steady_clock::now();
  result.planning_time = std::chrono::duration<double>(end_time - start_time).count();

  calculatePathStats(result.path, result.total_distance, result.num_waypoints);

  // Estimate coverage area (swath * total_distance)
  result.coverage_area = params.swath * result.total_distance;

  result.success = !result.path.poses.empty();
  result.message =
    result.success ? "Coverage path generated successfully" : "Failed to generate coverage path";

  return result;
}

bool BoustrophedonPlanner::extractFreeSpaceBounds(
  const nav_msgs::msg::OccupancyGrid & map, const std::vector<int8_t> & inflated_grid,
  bool allow_unknown, int & min_x, int & max_x, int & min_y, int & max_y)
{
  min_x = map.info.width;
  max_x = 0;
  min_y = map.info.height;
  max_y = 0;

  bool found_free = false;

  for (unsigned int y = 0; y < map.info.height; ++y) {
    for (unsigned int x = 0; x < map.info.width; ++x) {
      int index = y * map.info.width + x;
      if (isCellFree(inflated_grid[index], allow_unknown)) {
        found_free = true;
        min_x = std::min(min_x, static_cast<int>(x));
        max_x = std::max(max_x, static_cast<int>(x));
        min_y = std::min(min_y, static_cast<int>(y));
        max_y = std::max(max_y, static_cast<int>(y));
      }
    }
  }

  return found_free;
}

void BoustrophedonPlanner::generateHorizontalSweeps(
  const nav_msgs::msg::OccupancyGrid & map, const std::vector<int8_t> & inflated_grid,
  const CoveragePlannerParams & params, int min_x, int max_x, int min_y, int max_y,
  nav_msgs::msg::Path & path, std::vector<msg::MotionPlanSegment> & segments)
{
  int step_cells = std::max(1, static_cast<int>(params.step / map.info.resolution));
  bool left_to_right = (params.start_corner == "top_left" || params.start_corner == "bottom_left");
  bool start_from_top = (params.start_corner == "top_left" || params.start_corner == "top_right");
  int y_start = start_from_top ? min_y : max_y;
  int y_end = start_from_top ? max_y : min_y;
  int y_direction = start_from_top ? 1 : -1;

  int sweep_count = 0;
  int segment_count = 0;
  int connectors_attempted = 0;
  int connectors_accepted = 0;
  int connectors_rejected = 0;

  for (int y = y_start; (start_from_top && y <= y_end) || (!start_from_top && y >= y_end);
       y += y_direction * step_cells) {
    sweep_count++;
    int sweep_start = left_to_right ? min_x : max_x;
    int sweep_end = left_to_right ? max_x : min_x;
    int direction = (sweep_end > sweep_start) ? 1 : -1;
    int x = sweep_start;
    bool in_free = false;
    int segment_start = x;
    int obstacles_hit = 0;
    int strip_actual_end_x = sweep_start;  // Track where strip actually ends.
    bool sweep_has_segment = false;

    while ((direction > 0 && x <= sweep_end) || (direction < 0 && x >= sweep_end)) {
      int index = y * map.info.width + x;
      bool current_free =
        (index >= 0 && index < static_cast<int>(inflated_grid.size()) &&
         isCellFree(inflated_grid[index], params.allow_unknown));

      if (current_free) {
        const int next_x = x + direction;
        const bool next_in_sweep = (direction > 0) ? (next_x <= sweep_end) : (next_x >= sweep_end);
        const bool next_in_bounds = next_x >= 0 && next_x < static_cast<int>(map.info.width);
        bool next_free = false;
        if (next_in_sweep && next_in_bounds) {
          const int next_index = y * static_cast<int>(map.info.width) + next_x;
          next_free = next_index >= 0 && next_index < static_cast<int>(inflated_grid.size()) &&
                      isCellFree(inflated_grid[next_index], params.allow_unknown);
        }

        if (!in_free) {
          segment_start = x;
          in_free = true;
        }

        // If approaching obstacle, terminate segment BEFORE the last free cell
        if (!next_free) {
          if (in_free) {
            // End at current free cell.
            int end_x = x;
            if (
              (direction > 0 && end_x >= segment_start) ||
              (direction < 0 && end_x <= segment_start)) {
              segment_count++;
              auto start_pose = gridToWorld(
                segment_start, y, map, path.poses.empty() ? nullptr : &path.poses.back(),
                params.pose_mode);
              auto end_pose = gridToWorld(end_x, y, map, &start_pose, params.pose_mode);
              const uint32_t seg_start_index = static_cast<uint32_t>(path.poses.size());
              path.poses.push_back(start_pose);
              path.poses.push_back(end_pose);
              segments.push_back(makeStripSegment(
                "strip_h_" + std::to_string(segments.size()), seg_start_index, seg_start_index + 1,
                map.header.frame_id, params, {"strip", "obstacle_split"}));
              obstacles_hit++;
              sweep_has_segment = true;
              strip_actual_end_x = end_x;  // Track strip end
            }
            in_free = false;
          }
        }
      } else {
        if (in_free) {
          // Hit obstacle, end previous segment
          int end_x = x - direction;
          if (
            (direction > 0 && end_x >= segment_start) ||
            (direction < 0 && end_x <= segment_start)) {
            segment_count++;
            auto start_pose = gridToWorld(
              segment_start, y, map, path.poses.empty() ? nullptr : &path.poses.back(),
              params.pose_mode);
            auto end_pose = gridToWorld(end_x, y, map, &start_pose, params.pose_mode);
            const uint32_t seg_start_index = static_cast<uint32_t>(path.poses.size());
            path.poses.push_back(start_pose);
            path.poses.push_back(end_pose);
            segments.push_back(makeStripSegment(
              "strip_h_" + std::to_string(segments.size()), seg_start_index, seg_start_index + 1,
              map.header.frame_id, params, {"strip", "obstacle_split"}));
            obstacles_hit++;
            sweep_has_segment = true;
            strip_actual_end_x = end_x;  // Track strip end
          }
          in_free = false;
        }
      }
      x += direction;
    }
    // If ended in free space, close segment
    if (in_free) {
      segment_count++;
      const int clamped_end_x = std::clamp(sweep_end, min_x, max_x);
      auto start_pose = gridToWorld(
        segment_start, y, map, path.poses.empty() ? nullptr : &path.poses.back(), params.pose_mode);
      auto end_pose = gridToWorld(clamped_end_x, y, map, &start_pose, params.pose_mode);
      const uint32_t seg_start_index = static_cast<uint32_t>(path.poses.size());
      path.poses.push_back(start_pose);
      path.poses.push_back(end_pose);
      segments.push_back(makeStripSegment(
        "strip_h_" + std::to_string(segments.size()), seg_start_index, seg_start_index + 1,
        map.header.frame_id, params, {"strip"}));
      sweep_has_segment = true;
      strip_actual_end_x = clamped_end_x;  // Track strip end
    }

    if (obstacles_hit > 0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("BoustrophedonPlanner"),
        "Sweep %d at y=%d: hit %d obstacles, generated %d segment(s)", sweep_count, y,
        obstacles_hit, (obstacles_hit + (in_free ? 1 : 0)));
    }

    // Add connector waypoints to next strip if enabled and not last sweep
    const int next_y = y + y_direction * step_cells;
    const bool has_next_sweep =
      (start_from_top && next_y <= y_end) || (!start_from_top && next_y >= y_end);

    if (params.add_strip_connectors && has_next_sweep && !path.poses.empty() && sweep_has_segment) {
      ++connectors_attempted;

      const bool next_left_to_right = !left_to_right;
      const int scan_start_x = next_left_to_right ? min_x : max_x;
      const int scan_end_x = next_left_to_right ? max_x : min_x;
      const int scan_step = next_left_to_right ? 1 : -1;
      int next_start_x = scan_start_x;
      bool found_next_start = false;
      for (int nx = scan_start_x;
           (scan_step > 0 && nx <= scan_end_x) || (scan_step < 0 && nx >= scan_end_x);
           nx += scan_step) {
        const int next_idx = next_y * static_cast<int>(map.info.width) + nx;
        if (
          next_idx >= 0 && next_idx < static_cast<int>(inflated_grid.size()) &&
          isCellFree(inflated_grid[next_idx], params.allow_unknown)) {
          next_start_x = nx;
          found_next_start = true;
          break;
        }
      }

      if (!found_next_start) {
        ++connectors_rejected;
        RCLCPP_DEBUG(
          rclcpp::get_logger("BoustrophedonPlanner"),
          "Skipping connector at y=%d: next sweep y=%d has no reachable free start", y, next_y);
      } else {
        const auto * ref_pose = &path.poses.back();
        if (addStripConnectorWaypoints(
              path, map, inflated_grid, params, strip_actual_end_x, y, next_start_x, next_y,
              ref_pose, segments)) {
          ++connectors_accepted;
        } else {
          ++connectors_rejected;
        }
      }
    }

    left_to_right = !left_to_right;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("BoustrophedonPlanner"), "Generated %d sweeps with %d total segments%s",
    sweep_count, segment_count, params.add_strip_connectors ? " (with strip connectors)" : "");
  if (params.add_strip_connectors) {
    RCLCPP_INFO(
      rclcpp::get_logger("BoustrophedonPlanner"),
      "Connector summary (horizontal): attempted=%d accepted=%d rejected=%d", connectors_attempted,
      connectors_accepted, connectors_rejected);
  }
}

void BoustrophedonPlanner::generateVerticalSweeps(
  const nav_msgs::msg::OccupancyGrid & map, const std::vector<int8_t> & inflated_grid,
  const CoveragePlannerParams & params, int min_x, int max_x, int min_y, int max_y,
  nav_msgs::msg::Path & path, std::vector<msg::MotionPlanSegment> & segments)
{
  int step_cells = std::max(1, static_cast<int>(params.step / map.info.resolution));
  bool bottom_to_top =
    (params.start_corner == "bottom_left" || params.start_corner == "bottom_right");
  bool start_from_left =
    (params.start_corner == "top_left" || params.start_corner == "bottom_left");
  int x_start = start_from_left ? min_x : max_x;
  int x_end = start_from_left ? max_x : min_x;
  int x_direction = start_from_left ? 1 : -1;
  int connectors_attempted = 0;
  int connectors_accepted = 0;
  int connectors_rejected = 0;

  for (int x = x_start; (start_from_left && x <= x_end) || (!start_from_left && x >= x_end);
       x += x_direction * step_cells) {
    int sweep_start = bottom_to_top ? max_y : min_y;
    int sweep_end = bottom_to_top ? min_y : max_y;
    int direction = (sweep_end > sweep_start) ? 1 : -1;
    int y = sweep_start;
    bool in_free = false;
    int segment_start = y;
    int strip_actual_end_y = sweep_start;  // Track where strip actually ends
    bool sweep_has_segment = false;
    while ((direction > 0 && y <= sweep_end) || (direction < 0 && y >= sweep_end)) {
      int index = y * map.info.width + x;
      bool current_free =
        (index >= 0 && index < static_cast<int>(inflated_grid.size()) &&
         isCellFree(inflated_grid[index], params.allow_unknown));

      if (current_free) {
        const int next_y = y + direction;
        const bool next_in_sweep = (direction > 0) ? (next_y <= sweep_end) : (next_y >= sweep_end);
        const bool next_in_bounds = next_y >= 0 && next_y < static_cast<int>(map.info.height);
        bool next_free = false;
        if (next_in_sweep && next_in_bounds) {
          const int next_index = next_y * static_cast<int>(map.info.width) + x;
          next_free = next_index >= 0 && next_index < static_cast<int>(inflated_grid.size()) &&
                      isCellFree(inflated_grid[next_index], params.allow_unknown);
        }

        if (!in_free) {
          segment_start = y;
          in_free = true;
        }

        // If approaching obstacle, terminate segment BEFORE the last free cell
        if (!next_free) {
          if (in_free) {
            // End at current free cell.
            int end_y = y;
            if (
              (direction > 0 && end_y >= segment_start) ||
              (direction < 0 && end_y <= segment_start)) {
              auto start_pose = gridToWorld(
                x, segment_start, map, path.poses.empty() ? nullptr : &path.poses.back(),
                params.pose_mode);
              auto end_pose = gridToWorld(x, end_y, map, &start_pose, params.pose_mode);
              const uint32_t seg_start_index = static_cast<uint32_t>(path.poses.size());
              path.poses.push_back(start_pose);
              path.poses.push_back(end_pose);
              segments.push_back(makeStripSegment(
                "strip_v_" + std::to_string(segments.size()), seg_start_index, seg_start_index + 1,
                map.header.frame_id, params, {"strip", "obstacle_split"}));
              sweep_has_segment = true;
              strip_actual_end_y = end_y;  // Track strip end
            }
            in_free = false;
          }
        }
      } else {
        if (in_free) {
          // Hit obstacle, end previous segment
          int end_y = y - direction;
          if (
            (direction > 0 && end_y >= segment_start) ||
            (direction < 0 && end_y <= segment_start)) {
            auto start_pose = gridToWorld(
              x, segment_start, map, path.poses.empty() ? nullptr : &path.poses.back(),
              params.pose_mode);
            auto end_pose = gridToWorld(x, end_y, map, &start_pose, params.pose_mode);
            const uint32_t seg_start_index = static_cast<uint32_t>(path.poses.size());
            path.poses.push_back(start_pose);
            path.poses.push_back(end_pose);
            segments.push_back(makeStripSegment(
              "strip_v_" + std::to_string(segments.size()), seg_start_index, seg_start_index + 1,
              map.header.frame_id, params, {"strip", "obstacle_split"}));
            sweep_has_segment = true;
            strip_actual_end_y = end_y;  // Track strip end
          }
          in_free = false;
        }
      }
      y += direction;
    }
    if (in_free) {
      auto start_pose = gridToWorld(
        x, segment_start, map, path.poses.empty() ? nullptr : &path.poses.back(), params.pose_mode);
      auto end_pose = gridToWorld(x, sweep_end, map, &start_pose, params.pose_mode);
      const uint32_t seg_start_index = static_cast<uint32_t>(path.poses.size());
      path.poses.push_back(start_pose);
      path.poses.push_back(end_pose);
      segments.push_back(makeStripSegment(
        "strip_v_" + std::to_string(segments.size()), seg_start_index, seg_start_index + 1,
        map.header.frame_id, params, {"strip"}));
      sweep_has_segment = true;
      strip_actual_end_y = sweep_end;  // Track strip end
    }

    // Add connector waypoints to next strip if enabled and not last sweep
    const int next_x = x + x_direction * step_cells;
    const bool has_next_sweep =
      (start_from_left && next_x <= x_end) || (!start_from_left && next_x >= x_end);

    if (params.add_strip_connectors && has_next_sweep && !path.poses.empty() && sweep_has_segment) {
      ++connectors_attempted;
      const bool next_bottom_to_top = !bottom_to_top;
      const int scan_start_y = next_bottom_to_top ? max_y : min_y;
      const int scan_end_y = next_bottom_to_top ? min_y : max_y;
      const int scan_step = next_bottom_to_top ? -1 : 1;
      int next_start_y = scan_start_y;
      bool found_next_start = false;
      for (int ny = scan_start_y;
           (scan_step > 0 && ny <= scan_end_y) || (scan_step < 0 && ny >= scan_end_y);
           ny += scan_step) {
        const int next_idx = ny * static_cast<int>(map.info.width) + next_x;
        if (
          next_idx >= 0 && next_idx < static_cast<int>(inflated_grid.size()) &&
          isCellFree(inflated_grid[next_idx], params.allow_unknown)) {
          next_start_y = ny;
          found_next_start = true;
          break;
        }
      }
      if (!found_next_start) {
        ++connectors_rejected;
        RCLCPP_DEBUG(
          rclcpp::get_logger("BoustrophedonPlanner"),
          "Skipping connector at x=%d: next sweep x=%d has no reachable free start", x, next_x);
      } else {
        const auto * ref_pose = &path.poses.back();
        if (addStripConnectorWaypoints(
              path, map, inflated_grid, params, x, strip_actual_end_y, next_x, next_start_y,
              ref_pose, segments)) {
          ++connectors_accepted;
        } else {
          ++connectors_rejected;
        }
      }
    }

    bottom_to_top = !bottom_to_top;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("BoustrophedonPlanner"), "Generated vertical sweeps%s",
    params.add_strip_connectors ? " (with strip connectors)" : "");
  if (params.add_strip_connectors) {
    RCLCPP_INFO(
      rclcpp::get_logger("BoustrophedonPlanner"),
      "Connector summary (vertical): attempted=%d accepted=%d rejected=%d", connectors_attempted,
      connectors_accepted, connectors_rejected);
  }
}

void BoustrophedonPlanner::trimSweepLine(
  const std::vector<int8_t> & grid, int width, int & start, int & end, int fixed_coord,
  bool is_horizontal, bool allow_unknown) const
{
  int direction = (end > start) ? 1 : -1;

  // Trim start
  while ((direction > 0 && start < end) || (direction < 0 && start > end)) {
    int x = is_horizontal ? start : fixed_coord;
    int y = is_horizontal ? fixed_coord : start;
    int index = y * width + x;

    if (
      index >= 0 && index < static_cast<int>(grid.size()) &&
      isCellFree(grid[index], allow_unknown)) {
      break;
    }
    start += direction;
  }

  // Trim end
  while ((direction > 0 && end > start) || (direction < 0 && end < start)) {
    int x = is_horizontal ? end : fixed_coord;
    int y = is_horizontal ? fixed_coord : end;
    int index = y * width + x;

    if (
      index >= 0 && index < static_cast<int>(grid.size()) &&
      isCellFree(grid[index], allow_unknown)) {
      break;
    }
    end -= direction;
  }
}

bool BoustrophedonPlanner::isLineFree(
  const std::vector<int8_t> & grid, int width, int x1, int y1, int x2, int y2,
  bool allow_unknown) const
{
  // Bresenham's line algorithm
  int dx = std::abs(x2 - x1);
  int dy = std::abs(y2 - y1);
  int sx = (x1 < x2) ? 1 : -1;
  int sy = (y1 < y2) ? 1 : -1;
  int err = dx - dy;

  int x = x1;
  int y = y1;

  while (true) {
    int index = y * width + x;
    if (index < 0 || index >= static_cast<int>(grid.size())) return false;

    if (!isCellFree(grid[index], allow_unknown)) return false;

    if (x == x2 && y == y2) break;

    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }

  return true;
}

void BoustrophedonPlanner::generateBoundaryPass(
  const nav_msgs::msg::OccupancyGrid & map, const std::vector<int8_t> & inflated_grid,
  const CoveragePlannerParams & params, int min_x, int max_x, int min_y, int max_y,
  nav_msgs::msg::Path & path, std::vector<msg::MotionPlanSegment> & segments)
{
  // Calculate offset in grid cells
  int offset_cells = std::max(1, static_cast<int>(params.boundary_offset / map.info.resolution));

  // Apply offset to bounds (inward)
  int offset_min_x = std::min(min_x + offset_cells, max_x);
  int offset_max_x = std::max(max_x - offset_cells, min_x);
  int offset_min_y = std::min(min_y + offset_cells, max_y);
  int offset_max_y = std::max(max_y - offset_cells, min_y);

  // Validate that we still have space
  if (offset_min_x >= offset_max_x || offset_min_y >= offset_max_y) {
    RCLCPP_WARN(
      rclcpp::get_logger("BoustrophedonPlanner"),
      "Boundary offset too large for map area, skipping boundary pass");
    return;
  }

  // Generate rectangular perimeter: bottom -> right -> top -> left
  std::vector<std::pair<int, int>> perimeter_points;

  // Bottom edge (left to right)
  for (int x = offset_min_x; x <= offset_max_x; x++) {
    int index = offset_min_y * map.info.width + x;
    if (
      index >= 0 && index < static_cast<int>(inflated_grid.size()) &&
      isCellFree(inflated_grid[index], params.allow_unknown)) {
      perimeter_points.push_back({x, offset_min_y});
    }
  }

  // Right edge (bottom to top)
  for (int y = offset_min_y + 1; y <= offset_max_y; y++) {
    int index = y * map.info.width + offset_max_x;
    if (
      index >= 0 && index < static_cast<int>(inflated_grid.size()) &&
      isCellFree(inflated_grid[index], params.allow_unknown)) {
      perimeter_points.push_back({offset_max_x, y});
    }
  }

  // Top edge (right to left)
  for (int x = offset_max_x - 1; x >= offset_min_x; x--) {
    int index = offset_max_y * map.info.width + x;
    if (
      index >= 0 && index < static_cast<int>(inflated_grid.size()) &&
      isCellFree(inflated_grid[index], params.allow_unknown)) {
      perimeter_points.push_back({x, offset_max_y});
    }
  }

  // Left edge (top to bottom)
  for (int y = offset_max_y - 1; y > offset_min_y; y--) {
    int index = y * map.info.width + offset_min_x;
    if (
      index >= 0 && index < static_cast<int>(inflated_grid.size()) &&
      isCellFree(inflated_grid[index], params.allow_unknown)) {
      perimeter_points.push_back({offset_min_x, y});
    }
  }

  // Convert perimeter points to poses, splitting into one segment per contiguous
  // run so a gap left by a skipped (blocked) cell never gets bridged with a
  // straight contact line -- matches how generateHorizontalSweeps handles
  // obstacle-split strips.
  if (!perimeter_points.empty()) {
    RCLCPP_INFO(
      rclcpp::get_logger("BoustrophedonPlanner"), "Adding boundary perimeter with %zu points",
      perimeter_points.size());

    const auto is_adjacent = [](const std::pair<int, int> & a, const std::pair<int, int> & b) {
      return std::abs(a.first - b.first) <= 1 && std::abs(a.second - b.second) <= 1;
    };

    size_t run_start = 0;
    int run_count = 0;
    for (size_t i = 1; i <= perimeter_points.size(); ++i) {
      const bool run_ends = (i == perimeter_points.size()) ||
                            !is_adjacent(perimeter_points[i - 1], perimeter_points[i]);
      if (!run_ends) continue;

      const uint32_t seg_start_index = static_cast<uint32_t>(path.poses.size());
      for (size_t j = run_start; j < i; ++j) {
        auto pose = gridToWorld(
          perimeter_points[j].first, perimeter_points[j].second, map,
          path.poses.empty() ? nullptr : &path.poses.back(), params.pose_mode);
        path.poses.push_back(pose);
      }

      // Close the loop only when the whole perimeter is one unbroken run --
      // otherwise the closing edge would itself bridge a gap across an obstacle.
      if (
        run_start == 0 && i == perimeter_points.size() && perimeter_points.size() > 1 &&
        is_adjacent(perimeter_points.back(), perimeter_points.front())) {
        auto start_pose = gridToWorld(
          perimeter_points.front().first, perimeter_points.front().second, map, &path.poses.back(),
          params.pose_mode);
        path.poses.push_back(start_pose);
      }

      if (path.poses.size() > seg_start_index) {
        segments.push_back(makeStripSegment(
          "boundary_pass_" + std::to_string(run_count++), seg_start_index,
          static_cast<uint32_t>(path.poses.size()) - 1, map.header.frame_id, params, {"boundary"}));
      }
      run_start = i;
    }
  }
}

}  // namespace coverage_planner
