/**
 * @file wall_coverage_demo_node.cpp
 * @brief PCD-in/waypoints-out demo: loads a wall's plane fit + obstacles (wall_scene.json) and
 *        point cloud (walls.pcd), rasterizes it, calls coverage_planner_node's
 *        GenerateCoveragePath service, lifts + pass-expands the result, and publishes both the
 *        flat and final paths plus ground-truth markers for RViz.
 * @details No dependency on MoveIt or any other * package -- reachability/execution is out
 *          of scope here. The tool comes from tool_profiles.yaml (`tool_profiles_file`,
 *          `tool_name`); explicit params override single fields.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "coverage_planner/core/coverage_pass_expander.hpp"
#include "coverage_planner/core/tool_coverage_profile.hpp"
#include "coverage_planner/ros/tool_profile_params.hpp"
#include "coverage_planner/core/wall_basis.hpp"
#include "coverage_planner/core/wall_rasterizer.hpp"
#include "coverage_planner/ros/conversions.hpp"
#include "coverage_planner/core/wall_scene_loader.hpp"
#include "coverage_planner_msgs/srv/generate_coverage_path.hpp"

namespace
{

/// Message-typed view of the core's ExpandedCoverage, for the ROS publishing/JSON helpers below.
struct ExpandedRos
{
  nav_msgs::msg::Path path;
  std::vector<coverage_planner_msgs::msg::MotionPlanSegment> segments;
};


visualization_msgs::msg::MarkerArray buildPoseArrowMarkers(
  const nav_msgs::msg::Path & path, const std::string & ns, float r, float g, float b)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker delete_all;
  delete_all.header = path.header;
  delete_all.ns = ns;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_all);

  for (size_t i = 0; i < path.poses.size(); ++i) {
    visualization_msgs::msg::Marker arrow;
    arrow.header = path.header;
    arrow.ns = ns;
    arrow.id = static_cast<int>(i);
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose = path.poses[i].pose;
    arrow.scale.x = 0.08;
    arrow.scale.y = 0.015;
    arrow.scale.z = 0.015;
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 0.9f;
    arrow.lifetime = rclcpp::Duration::from_seconds(0);
    markers.markers.push_back(arrow);
  }
  return markers;
}

/**
 * @brief Draws each segment's own waypoint-to-waypoint connections as a LINE_LIST, WITHOUT ever
 *        connecting one segment's last point to the next segment's first point.
 * @details nav_msgs::msg::Path's own RViz display draws one continuous line through every pose
 *          regardless of segment boundaries, so an obstacle_split strip (two separate
 *          MotionPlanSegments) renders as a phantom line straight through the obstacle. This
 *          draws the same segments respecting that boundary -- gaps stay gaps.
 */
visualization_msgs::msg::Marker buildSegmentLineMarker(
  const nav_msgs::msg::Path & path,
  const std::vector<coverage_planner_msgs::msg::MotionPlanSegment> & segments,
  const std::string & ns, const std::string & frame_id, float r, float g, float b, double width)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = width;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0f;
  marker.lifetime = rclcpp::Duration::from_seconds(0);

  for (const auto & segment : segments) {
    for (uint32_t i = segment.render_start_index;
         i < segment.render_end_index && i + 1 < path.poses.size(); ++i) {
      marker.points.push_back(path.poses[i].pose.position);
      marker.points.push_back(path.poses[i + 1].pose.position);
    }
  }
  return marker;
}

/**
 * @brief Loads a wall's own raw scan points (ground truth) as a PointCloud2, straight from the
 *        same PCD loadWallObservedMask() reads -- filtered to this wall's own `label` (a real PCD
 *        holds every scanned wall, not just this one).
 */
sensor_msgs::msg::PointCloud2 loadWallPointCloud(
  const std::string & pcd_path, uint32_t wall_id, const std::string & frame_id)
{
  pcl::PointCloud<pcl::PointXYZL> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZL>(pcd_path, cloud) < 0) {
    throw std::runtime_error("loadWallPointCloud: failed to load point cloud from " + pcd_path);
  }

  pcl::PointCloud<pcl::PointXYZ> wall_points;
  wall_points.reserve(cloud.size());
  for (const auto & point : cloud.points) {
    if (point.label == wall_id) {
      wall_points.emplace_back(point.x, point.y, point.z);
    }
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(wall_points, msg);
  msg.header.frame_id = frame_id;
  return msg;
}

/**
 * @brief Draws each obstacle region's (u, v) boundary rectangle -- flat, no depth -- lifted into
 *        3D via the wall's own basis. This is what rasterizeWallCoverageGrid() actually inflates;
 *        compare against buildObstacleBoxMarkers()'s real 3D boxes to sanity-check it.
 */
visualization_msgs::msg::MarkerArray buildObstacleMarkers(
  const std::vector<coverage_planner::ObstacleRegion> & obstacles,
  const coverage_planner::WallPlane & wall, const std::string & frame_id)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker delete_all;
  delete_all.header.frame_id = frame_id;
  delete_all.ns = "obstacles";
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_all);

  const coverage_planner::WallBasis basis = coverage_planner::decomposeWallBasis(wall);
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const auto & obstacle = obstacles[i];

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.ns = "obstacles";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.r = 1.0f;
    marker.color.g = 0.6f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    // Closed loop: 4 corners + repeat the first to close it.
    const std::array<std::pair<double, double>, 5> corners_uv{
      {{obstacle.u_min, obstacle.v_min},
       {obstacle.u_max, obstacle.v_min},
       {obstacle.u_max, obstacle.v_max},
       {obstacle.u_min, obstacle.v_max},
       {obstacle.u_min, obstacle.v_min}}};
    for (const auto & [u, v] : corners_uv) {
      const Eigen::Vector3d corner = basis.origin + basis.basis_u * u + basis.basis_v * v;
      geometry_msgs::msg::Point point;
      point.x = corner.x();
      point.y = corner.y();
      point.z = corner.z();
      marker.points.push_back(point);
    }
    markers.markers.push_back(marker);
  }
  return markers;
}

/// One obstacle's real, oriented 3D bounding box -- visualization-only ground truth, not carried
/// on ObstacleRegion.msg (which is the 2D-footprint-only type the planner itself reads).
struct ObstacleBox3D
{
  uint32_t id;
  Eigen::Vector3d center;   ///< Box center, in the wall's own raw capture frame.
  Eigen::Vector3d extents;  ///< Full (not half) size along the box's own local (u, v, n) axes.
  Eigen::Quaterniond orientation;  ///< Local (u, v, n) axes -> the capture frame.
};

std::vector<ObstacleBox3D> loadWallObstacleBoxes(const std::string & json_path, uint32_t wall_id)
{
  std::ifstream file(json_path);
  if (!file.is_open()) {
    throw std::runtime_error("loadWallObstacleBoxes: could not open " + json_path);
  }
  nlohmann::json scene;
  file >> scene;

  std::vector<ObstacleBox3D> boxes;
  if (!scene.contains("obstacles")) {
    return boxes;
  }
  for (const auto & obstacle_node : scene.at("obstacles")) {
    if (obstacle_node.at("parent_wall").get<uint32_t>() != wall_id) {
      continue;
    }
    ObstacleBox3D box;
    box.id = obstacle_node.at("id").get<uint32_t>();
    const auto & center = obstacle_node.at("box_center_xyz");
    box.center = Eigen::Vector3d(
      center.at(0).get<double>(), center.at(1).get<double>(), center.at(2).get<double>());
    const auto & extents = obstacle_node.at("box_extents_uvn");
    box.extents = Eigen::Vector3d(
      extents.at(0).get<double>(), extents.at(1).get<double>(), extents.at(2).get<double>());
    const auto & orientation = obstacle_node.at("box_orientation_xyzw");
    box.orientation = Eigen::Quaterniond(
      orientation.at(3).get<double>(), orientation.at(0).get<double>(),
      orientation.at(1).get<double>(), orientation.at(2).get<double>());
    boxes.push_back(box);
  }
  return boxes;
}

/// Semi-transparent CUBE per real 3D obstacle box -- the actual object's real size/orientation,
/// vs. buildObstacleMarkers()'s flat 2D footprint outline.
visualization_msgs::msg::MarkerArray buildObstacleBoxMarkers(
  const std::vector<ObstacleBox3D> & boxes, const std::string & frame_id)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker delete_all;
  delete_all.header.frame_id = frame_id;
  delete_all.ns = "obstacle_boxes";
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_all);

  for (const auto & box : boxes) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.ns = "obstacle_boxes";
    marker.id = static_cast<int>(box.id);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = box.center.x();
    marker.pose.position.y = box.center.y();
    marker.pose.position.z = box.center.z();
    marker.pose.orientation.x = box.orientation.x();
    marker.pose.orientation.y = box.orientation.y();
    marker.pose.orientation.z = box.orientation.z();
    marker.pose.orientation.w = box.orientation.w();
    marker.scale.x = box.extents.x();
    marker.scale.y = box.extents.y();
    marker.scale.z = box.extents.z();
    marker.color.r = 1.0f;
    marker.color.g = 0.2f;
    marker.color.b = 0.0f;
    marker.color.a = 0.35f;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    markers.markers.push_back(marker);
  }
  return markers;
}

/// Quaternion -> roll/pitch/yaw (radians, same convention as tf2::Matrix3x3::getRPY()), spelled
/// out directly rather than pulling in tf2 as a dependency for one conversion.
Eigen::Vector3d quaternionToRPY(const geometry_msgs::msg::Quaternion & q)
{
  const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  const double pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}

enum class OrientationFormat { kQuaternion, kRpy, kBoth };

/**
 * @brief Parses the `orientation_format` param ("quaternion" | "rpy" | "both") -- throws on any
 *        other value so a typo is caught at startup instead of silently defaulting.
 */
OrientationFormat parseOrientationFormat(const std::string & value)
{
  if (value == "quaternion") return OrientationFormat::kQuaternion;
  if (value == "rpy") return OrientationFormat::kRpy;
  if (value == "both") return OrientationFormat::kBoth;
  throw std::runtime_error(
    "orientation_format must be 'quaternion', 'rpy', or 'both' -- got '" + value + "'");
}

nlohmann::json poseStampedToJson(
  const geometry_msgs::msg::PoseStamped & pose, OrientationFormat format)
{
  nlohmann::json orientation;
  if (format == OrientationFormat::kQuaternion || format == OrientationFormat::kBoth) {
    orientation["x"] = pose.pose.orientation.x;
    orientation["y"] = pose.pose.orientation.y;
    orientation["z"] = pose.pose.orientation.z;
    orientation["w"] = pose.pose.orientation.w;
  }
  if (format == OrientationFormat::kRpy || format == OrientationFormat::kBoth) {
    const auto rpy = quaternionToRPY(pose.pose.orientation);
    orientation["roll"] = rpy.x();
    orientation["pitch"] = rpy.y();
    orientation["yaw"] = rpy.z();
  }
  return {
    {"position",
     {{"x", pose.pose.position.x}, {"y", pose.pose.position.y}, {"z", pose.pose.position.z}}},
    {"orientation", orientation}};
}

nlohmann::json pathToJson(const nav_msgs::msg::Path & path, OrientationFormat format)
{
  nlohmann::json poses = nlohmann::json::array();
  for (const auto & pose : path.poses) {
    poses.push_back(poseStampedToJson(pose, format));
  }
  return {{"frame_id", path.header.frame_id}, {"poses", poses}};
}

nlohmann::json segmentsToJson(
  const std::vector<coverage_planner_msgs::msg::MotionPlanSegment> & segments)
{
  nlohmann::json out = nlohmann::json::array();
  for (const auto & segment : segments) {
    out.push_back(
      {{"label", segment.label},
       {"start_index", segment.start_index},
       {"end_index", segment.end_index},
       {"render_start_index", segment.render_start_index},
       {"render_end_index", segment.render_end_index},
       {"mode", segment.mode},
       {"planner", segment.planner},
       {"requires_contact", segment.requires_contact},
       {"requires_retraction", segment.requires_retraction},
       {"priority", segment.priority},
       {"tags", segment.tags}});
  }
  return out;
}

/**
 * @brief Saves this run's coverage-planner output -- the GenerateCoveragePath response as
 *        actually received (flat path + segments), and the final lifted/pass-expanded result --
 *        to a JSON file, so one run can be reviewed/diffed later without re-running the pipeline.
 */
void writeCoverageOutputJson(
  const std::string & output_path, const std::string & wall_scene_json, uint32_t wall_id,
  double resolution, const coverage_planner::ToolCoverageProfile & tool_profile,
  const coverage_planner_msgs::srv::GenerateCoveragePath::Response & response,
  const ExpandedRos & expanded, OrientationFormat orientation_format)
{
  const nlohmann::json output = {
    {"wall_scene_json", wall_scene_json},
    {"wall_id", wall_id},
    {"resolution_m", resolution},
    {"tool_profile",
     {{"name", tool_profile.name},
      {"contact", tool_profile.task.contact},
      {"standoff_m", tool_profile.task.standoff_m},
      {"swath_m", tool_profile.coverage.swath_m},
      {"overlap_pct", tool_profile.coverage.overlap_pct},
      {"sweep_direction", tool_profile.coverage.sweep_direction},
      {"passes", tool_profile.coverage.passes.size()}}},
    {"received_from_coverage_planner_node",
     {{"success", response.success},
      {"message", response.message},
      {"num_waypoints", response.num_waypoints},
      {"total_distance_m", response.total_distance},
      {"coverage_area_m2", response.coverage_area},
      {"planning_time_s", response.planning_time},
      {"pattern_used", response.pattern_used},
      {"path", pathToJson(response.path, orientation_format)},
      {"segments", segmentsToJson(response.segments)}}},
    {"final_lifted_and_expanded",
     {{"path", pathToJson(expanded.path, orientation_format)},
      {"segments", segmentsToJson(expanded.segments)}}}};

  std::ofstream file(output_path);
  if (!file.is_open()) {
    throw std::runtime_error("writeCoverageOutputJson: could not open " + output_path);
  }
  file << output.dump(2);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("wall_coverage_demo");

  const std::string wall_scene_json = node->declare_parameter<std::string>("wall_scene_json", "");
  const int64_t wall_id_param = node->declare_parameter<int64_t>("wall_id", 0);
  const double resolution = node->declare_parameter<double>("resolution", 0.02);
  const std::string coverage_service_name = node->declare_parameter<std::string>(
    "coverage_service_name", "/coverage_planner_node/generate_coverage_path");
  const bool allow_flatness_override =
    node->declare_parameter<bool>("allow_flatness_override", false);
  // Empty (the default) skips saving.
  const std::string output_json = node->declare_parameter<std::string>("output_json", "");
  // Default false: without it, each strip only gets its own two endpoints, no waypoints for
  // getting from one strip's end to the next strip's start.
  const bool add_strip_connectors = node->declare_parameter<bool>("add_strip_connectors", false);
  // Default false: refuse to plan through cells the scan never actually covered rather than
  // assume they're clear. Set true once unscanned space is known to be safe to sweep through.
  const bool allow_unknown = node->declare_parameter<bool>("allow_unknown", false);
  // How output_json writes each waypoint's orientation: "quaternion", "rpy", or "both".
  const auto orientation_format =
    parseOrientationFormat(node->declare_parameter<std::string>("orientation_format", "both"));

  if (wall_scene_json.empty()) {
    RCLCPP_ERROR(
      node->get_logger(), "wall_coverage_demo: 'wall_scene_json' parameter is required.");
    rclcpp::shutdown();
    return 1;
  }

  coverage_planner::ToolCoverageProfile tool_profile;
  try {
    tool_profile = coverage_planner::loadToolCoverageProfileFromParams(*node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "wall_coverage_demo: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(
    node->get_logger(), "Tool coverage profile '%s' (contact=%s, swath_m=%.3f, passes=%zu)",
    tool_profile.name.c_str(), tool_profile.task.contact ? "true" : "false",
    coverage_planner::toolSwath(tool_profile), tool_profile.coverage.passes.size());

  // double_pass's last pass is reverse_direction=true, so a strip finishes at its own start pose
  // instead of its end pose. Every connector is generated once, against the flat (single-pass)
  // plan, and always starts at a strip's end pose -- so with both on, the expanded plan asks the
  // executor to jump the strip's full length at every strip/connector joint. Reconciling that
  // would mean re-deriving each connector's route against the actual post-expansion pose, which
  // is out of scope here; reject the combination instead of silently emitting an unexecutable plan.
  if (coverage_planner::stripConnectorsConflictWithPasses(tool_profile, add_strip_connectors)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "wall_coverage_demo: 'double_pass' and 'add_strip_connectors' cannot both be true -- "
      "the expanded plan would jump the full strip length at every strip/connector joint.");
    rclcpp::shutdown();
    return 1;
  }

  coverage_planner::WallFlatnessThresholds thresholds;
  thresholds.allow_override = allow_flatness_override;
  coverage_planner::WallSceneQuery scene;
  try {
    scene = coverage_planner::loadWallScene(
      wall_scene_json, static_cast<uint32_t>(wall_id_param), thresholds);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "wall_coverage_demo: failed to load wall scene: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  if (!scene.flatness_warning.empty()) {
    RCLCPP_WARN(node->get_logger(), "%s", scene.flatness_warning.c_str());
  }
  RCLCPP_INFO(
    node->get_logger(), "Loaded wall %u (frame=%s, u=[%.3f, %.3f], v=[%.3f, %.3f], obstacles=%zu)",
    scene.wall.id, scene.wall.pose.header.frame_id.c_str(), scene.wall.u_min, scene.wall.u_max,
    scene.wall.v_min, scene.wall.v_max, scene.obstacles.size());

  // Mask against the actual point cloud so coverage only plans where the wall was really
  // measured, not wherever its bounding rectangle happens to extend.
  const std::string walls_pcd_path =
    (std::filesystem::path(wall_scene_json).parent_path() / "walls.pcd").string();

  const double u_extent = scene.wall.u_max - scene.wall.u_min;
  const double v_extent = scene.wall.v_max - scene.wall.v_min;
  const int grid_width = std::max(1, static_cast<int>(std::ceil(u_extent / resolution)));
  const int grid_height = std::max(1, static_cast<int>(std::ceil(v_extent / resolution)));

  std::vector<bool> observed_mask;
  try {
    observed_mask = coverage_planner::loadWallObservedMask(
      walls_pcd_path, static_cast<uint32_t>(wall_id_param), scene.wall, grid_width, grid_height,
      resolution);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(), "wall_coverage_demo: failed to load wall point cloud '%s': %s",
      walls_pcd_path.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }
  const size_t observed_cells = std::count(observed_mask.begin(), observed_mask.end(), true);
  RCLCPP_INFO(
    node->get_logger(), "Loaded observed-cell mask from '%s': %zu/%zu cells scanned (%.1f%%)",
    walls_pcd_path.c_str(), observed_cells, observed_mask.size(),
    100.0 * static_cast<double>(observed_cells) / static_cast<double>(observed_mask.size()));

  const auto grid = coverage_planner::rasterizeWallCoverageGrid(
    scene.wall, scene.obstacles, tool_profile.coverage, resolution, scene.wall.pose.header.frame_id,
    observed_mask);
  RCLCPP_INFO(
    node->get_logger(), "Rasterized coverage grid: %ux%u @ %.3fm resolution", grid.info.width,
    grid.info.height, resolution);

  // Ground truth for comparison against the generated plan below.
  sensor_msgs::msg::PointCloud2 wall_point_cloud;
  try {
    wall_point_cloud = loadWallPointCloud(
      walls_pcd_path, static_cast<uint32_t>(wall_id_param), scene.wall.pose.header.frame_id);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(),
      "wall_coverage_demo: failed to load wall point cloud for "
      "visualization '%s': %s",
      walls_pcd_path.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }
  const auto obstacle_markers =
    buildObstacleMarkers(scene.obstacles, scene.wall, scene.wall.pose.header.frame_id);
  std::vector<ObstacleBox3D> obstacle_boxes;
  try {
    obstacle_boxes = loadWallObstacleBoxes(wall_scene_json, static_cast<uint32_t>(wall_id_param));
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      node->get_logger(),
      "wall_coverage_demo: failed to load 3D obstacle boxes for "
      "visualization: %s -- continuing without them.",
      e.what());
  }
  const auto obstacle_box_markers =
    buildObstacleBoxMarkers(obstacle_boxes, scene.wall.pose.header.frame_id);

  auto client = node->create_client<coverage_planner_msgs::srv::GenerateCoveragePath>(
    coverage_service_name);
  RCLCPP_INFO(
    node->get_logger(), "Waiting for coverage service '%s'...", coverage_service_name.c_str());
  while (rclcpp::ok() && !client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(node->get_logger(), "  ...still waiting");
  }
  if (!rclcpp::ok()) {
    rclcpp::shutdown();
    return 1;
  }

  auto request = std::make_shared<coverage_planner_msgs::srv::GenerateCoveragePath::Request>();
  request->map = coverage_planner::ros_conv::toRos(grid);
  request->pattern = "boustrophedon";
  request->swath = static_cast<float>(coverage_planner::toolSwath(tool_profile));
  request->step = static_cast<float>(coverage_planner::toolLineSpacing(tool_profile));
  request->sweep_direction = tool_profile.coverage.sweep_direction;
  request->start_corner = "bottom_left";
  request->obstacle_inflation_radius = 0.0f;  // already rasterized into the grid above.
  request->enable_boundary_pass = tool_profile.coverage.enable_boundary_pass;
  request->boundary_offset = static_cast<float>(tool_profile.coverage.boundary_offset_m);
  request->requires_contact = tool_profile.task.contact;
  request->edge_zone_m = static_cast<float>(tool_profile.coverage.edge_zone_m);
  request->edge_spacing_scale = static_cast<float>(tool_profile.coverage.edge_spacing_scale);
  request->add_strip_connectors = add_strip_connectors;
  request->allow_unknown = allow_unknown;

  RCLCPP_INFO(node->get_logger(), "Calling GenerateCoveragePath...");
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "wall_coverage_demo: service call failed.");
    rclcpp::shutdown();
    return 1;
  }
  auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(
      node->get_logger(), "wall_coverage_demo: coverage generation failed: %s",
      response->message.c_str());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(
    node->get_logger(), "Flat coverage path: %d waypoints, %zu segments", response->num_waypoints,
    response->segments.size());

  std::vector<coverage_planner::MotionPlanSegment> flat_segments;
  flat_segments.reserve(response->segments.size());
  for (const auto & segment : response->segments) {
    flat_segments.push_back(coverage_planner::ros_conv::toCore(segment));
  }
  const auto expanded_core = coverage_planner::expandCoveragePasses(
    coverage_planner::ros_conv::toCore(response->path), flat_segments, scene.wall, tool_profile);
  ExpandedRos expanded;
  expanded.path = coverage_planner::ros_conv::toRos(expanded_core.path);
  expanded.path.header.stamp = response->path.header.stamp;
  expanded.segments = coverage_planner::ros_conv::toRos(expanded_core.segments);
  RCLCPP_INFO(
    node->get_logger(), "Final lifted+expanded path: %zu waypoints, %zu segments",
    expanded.path.poses.size(), expanded.segments.size());

  if (!output_json.empty()) {
    try {
      writeCoverageOutputJson(
        output_json, wall_scene_json, static_cast<uint32_t>(wall_id_param), resolution,
        tool_profile, *response, expanded, orientation_format);
      RCLCPP_INFO(node->get_logger(), "Saved coverage-planner output to '%s'", output_json.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node->get_logger(), "wall_coverage_demo: failed to save output_json '%s': %s",
        output_json.c_str(), e.what());
    }
  }

  auto flat_path_pub = node->create_publisher<nav_msgs::msg::Path>("~/flat_coverage_path", 1);
  auto final_path_pub = node->create_publisher<nav_msgs::msg::Path>("~/final_coverage_path", 1);
  auto final_markers_pub =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("~/final_coverage_markers", 1);
  auto wall_point_cloud_pub =
    node->create_publisher<sensor_msgs::msg::PointCloud2>("~/wall_point_cloud", 1);
  auto obstacle_markers_pub =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacle_markers", 1);
  auto obstacle_box_markers_pub =
    node->create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacle_box_markers", 1);
  // The actual grid the planner ran against -- ground truth for what it actually saw, as
  // opposed to buildObstacleMarkers()'s/buildObstacleBoxMarkers()'s own re-derivation.
  auto coverage_grid_pub =
    node->create_publisher<nav_msgs::msg::OccupancyGrid>("~/coverage_grid", 1);
  // Segment-boundary-respecting line rendering -- see buildSegmentLineMarker()'s doc comment.
  auto flat_lines_pub =
    node->create_publisher<visualization_msgs::msg::Marker>("~/flat_coverage_lines", 1);
  auto final_lines_pub =
    node->create_publisher<visualization_msgs::msg::Marker>("~/final_coverage_lines", 1);

  auto flat_path = response->path;
  const auto final_markers =
    buildPoseArrowMarkers(expanded.path, "final_coverage", 0.1f, 0.9f, 0.1f);
  const auto flat_lines = buildSegmentLineMarker(
    flat_path, response->segments, "flat_coverage_lines", scene.wall.pose.header.frame_id, 0.9f,
    0.1f, 0.1f, 0.01);
  const auto final_lines = buildSegmentLineMarker(
    expanded.path, expanded.segments, "final_coverage_lines", scene.wall.pose.header.frame_id, 0.1f,
    0.9f, 0.1f, 0.02);

  // Re-stamps and republishes the already-computed flat/final path + markers, called once below
  // and then repeatedly in this node's own tail loop, so a late-starting RViz still picks them up.
  auto publish_coverage_visualization = [&]() {
    // stamp=0 ("latest available"), not now() -- avoids RViz's TF filter alternately
    // accept/rejecting a stamp that's always trailing a live TF chain, which flickers.
    const rclcpp::Time latest_available(0, 0, node->get_clock()->get_clock_type());
    auto stamped_flat_path = flat_path;
    stamped_flat_path.header.stamp = latest_available;
    auto stamped_final_path = expanded.path;
    stamped_final_path.header.stamp = latest_available;
    auto stamped_markers = final_markers;
    for (auto & marker : stamped_markers.markers) {
      marker.header.stamp = latest_available;
    }
    auto stamped_cloud = wall_point_cloud;
    stamped_cloud.header.stamp = latest_available;
    auto stamped_obstacle_markers = obstacle_markers;
    for (auto & marker : stamped_obstacle_markers.markers) {
      marker.header.stamp = latest_available;
    }
    auto stamped_obstacle_box_markers = obstacle_box_markers;
    for (auto & marker : stamped_obstacle_box_markers.markers) {
      marker.header.stamp = latest_available;
    }
    auto stamped_grid = coverage_planner::ros_conv::toRos(grid);
    stamped_grid.header.stamp = latest_available;
    auto stamped_flat_lines = flat_lines;
    stamped_flat_lines.header.stamp = latest_available;
    auto stamped_final_lines = final_lines;
    stamped_final_lines.header.stamp = latest_available;
    flat_path_pub->publish(stamped_flat_path);
    final_path_pub->publish(stamped_final_path);
    final_markers_pub->publish(stamped_markers);
    wall_point_cloud_pub->publish(stamped_cloud);
    obstacle_markers_pub->publish(stamped_obstacle_markers);
    obstacle_box_markers_pub->publish(stamped_obstacle_box_markers);
    coverage_grid_pub->publish(stamped_grid);
    flat_lines_pub->publish(stamped_flat_lines);
    final_lines_pub->publish(stamped_final_lines);
    rclcpp::spin_some(node);
  };
  RCLCPP_INFO(
    node->get_logger(),
    "Publishing on ~/flat_coverage_path, ~/final_coverage_path, ~/final_coverage_markers, "
    "~/wall_point_cloud, ~/obstacle_markers, ~/obstacle_box_markers, ~/coverage_grid, "
    "~/flat_coverage_lines, ~/final_coverage_lines (frame='%s') -- visible in RViz now.",
    scene.wall.pose.header.frame_id.c_str());
  publish_coverage_visualization();

  RCLCPP_INFO(
    node->get_logger(),
    "Done -- leave this node running for RViz to keep picking up "
    "~/flat_coverage_path, ~/final_coverage_path, ~/final_coverage_markers, ~/wall_point_cloud, "
    "~/obstacle_markers (frame='%s').",
    scene.wall.pose.header.frame_id.c_str());

  rclcpp::Rate rate(1.0);
  while (rclcpp::ok()) {
    publish_coverage_visualization();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
