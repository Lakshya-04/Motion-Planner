#include "coverage_planner/ros/conversions.hpp"
#include "coverage_planner/ros/coverage_planner_node.hpp"
#include <chrono>
#include <sstream>
#include "coverage_planner/core/coverage_planner_base.hpp"
#include "ros_adapters/rclcpp_logger.hpp"

namespace coverage_planner
{

// The interfaces live in coverage_planner_msgs; alias them into this package's own
// namespace so `msg::`/`srv::` still mean what they always did here.
namespace srv = coverage_planner_msgs::srv;

CoveragePlannerNode::CoveragePlannerNode(const rclcpp::NodeOptions & options)
: Node("coverage_planner_node", options)
{
  // Declare parameters
  declareParameters();

  // Load parameters
  coverage_pattern_ = this->get_parameter("coverage_pattern").as_string();
  swath_ = this->get_parameter("swath").as_double();
  step_ = this->get_parameter("step").as_double();
  sweep_direction_ = this->get_parameter("sweep_direction").as_string();
  start_corner_ = this->get_parameter("start_corner").as_string();
  add_strip_connectors_ = this->get_parameter("add_strip_connectors").as_bool();
  obstacle_inflation_radius_ = this->get_parameter("obstacle_inflation_radius").as_double();
  allow_unknown_ = this->get_parameter("allow_unknown").as_bool();
  plan_all_components_ = this->get_parameter("plan_all_components").as_bool();
  min_component_cells_ = this->get_parameter("min_component_cells").as_int();
  optimize_strip_order_ = this->get_parameter("optimize_strip_order").as_bool();
  components_in_sweep_order_ = this->get_parameter("components_in_sweep_order").as_bool();
  fill_gaps_ = this->get_parameter("fill_gaps").as_bool();
  min_stroke_length_m_ = this->get_parameter("min_stroke_length_m").as_double();
  edge_zone_m_ = this->get_parameter("edge_zone_m").as_double();
  edge_spacing_scale_ = this->get_parameter("edge_spacing_scale").as_double();
  decomposition_ = this->get_parameter("decomposition").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  map_topic_ = this->get_parameter("map_topic").as_string();

  RCLCPP_INFO(this->get_logger(), "Coverage Planner Node Initialized");
  RCLCPP_INFO(this->get_logger(), "  Pattern: %s", coverage_pattern_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Swath: %.2f m", swath_);
  RCLCPP_INFO(this->get_logger(), "  Step: %.2f m", step_);
  RCLCPP_INFO(this->get_logger(), "  Sweep Direction: %s", sweep_direction_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Start Corner: %s", start_corner_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Obstacle Inflation: %.2f m", obstacle_inflation_radius_);
  RCLCPP_INFO(this->get_logger(), "  Allow Unknown: %s", allow_unknown_ ? "true" : "false");
  RCLCPP_INFO(
    this->get_logger(), "  Plan All Components: %s", plan_all_components_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  Min Component Cells: %d", min_component_cells_);
  RCLCPP_INFO(
    this->get_logger(), "  Optimize Strip Order: %s", optimize_strip_order_ ? "true" : "false");

  // Subscribe to map
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)),
    std::bind(&CoveragePlannerNode::mapCallback, this, std::placeholders::_1));

  // Create service
  coverage_service_ = this->create_service<srv::GenerateCoveragePath>(
    "~/generate_coverage_path", std::bind(
                                  &CoveragePlannerNode::generateCoveragePathCallback, this,
                                  std::placeholders::_1, std::placeholders::_2));

  // Publisher for visualization
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/coverage_path", 10);

  markers_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("~/coverage_markers", 10);

  diagnostics_pub_ = this->create_publisher<std_msgs::msg::String>("~/coverage_diagnostics", 10);

  RCLCPP_INFO(this->get_logger(), "Waiting for map on topic: %s", map_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Service ready: ~/generate_coverage_path");
}

void CoveragePlannerNode::declareParameters()
{
  // Coverage pattern
  this->declare_parameter("coverage_pattern", "boustrophedon");

  // Coverage dimensions
  this->declare_parameter("swath", 0.5);  // 0.5m effective coverage width
  this->declare_parameter("step", 0.4);   // 0.4m step (20% overlap)

  // Sweep parameters
  this->declare_parameter("sweep_direction", "vertical");
  this->declare_parameter("start_corner", "bottom_left");

  // Strip connectors (for boustrophedon)
  this->declare_parameter("add_strip_connectors", false);  // Default: no diagonal connectors

  // Obstacle handling
  this->declare_parameter("obstacle_inflation_radius", 0.3);  // 30cm safety margin
  this->declare_parameter("allow_unknown", false);
  // Every free component is planned (a wall cut by a rib is two panels); false = largest only.
  this->declare_parameter("plan_all_components", true);
  this->declare_parameter("min_component_cells", 25);
  this->declare_parameter("optimize_strip_order", false);
  this->declare_parameter("components_in_sweep_order", true);  // panels left to right, not largest first
  this->declare_parameter("fill_gaps", true);                  // short strokes beside obstacle keep-outs
  this->declare_parameter("min_stroke_length_m", 0.02);        // shortest gap-fill stroke
  this->declare_parameter("edge_zone_m", 0.0);                 // adaptive stepover zone at the stepover-axis ends (0 = off)
  this->declare_parameter("edge_spacing_scale", 1.0);          // stepover factor inside the zone, (0, 1]
  this->declare_parameter("decomposition", "columns");         // "columns" | "cells"

  // Frame and topic
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("map_topic", "/map");
}

void CoveragePlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(map_mutex_);
  latest_map_ = msg;

  RCLCPP_INFO_ONCE(
    this->get_logger(), "Received map: %dx%d cells, resolution: %.3f m", msg->info.width,
    msg->info.height, msg->info.resolution);
}

CoveragePlannerParams CoveragePlannerNode::buildPlannerParams(
  const std::shared_ptr<srv::GenerateCoveragePath::Request> & request)
{
  CoveragePlannerParams params;

  // Use request values if provided, otherwise use node defaults
  params.swath = (request->swath > 0.0) ? request->swath : swath_;
  params.step = (request->step > 0.0) ? request->step : step_;

  // Sentinel is -1.0 ("use node default"), not 0.0 -- a default-constructed request field is
  // already 0.0, and >= 0.0 would silently take that as "explicitly zero inflation".
  params.obstacle_inflation_radius = (request->obstacle_inflation_radius > 0.0)
                                       ? request->obstacle_inflation_radius
                                       : obstacle_inflation_radius_;

  params.allow_unknown = request->allow_unknown;  // Use request value

  params.sweep_direction =
    !request->sweep_direction.empty() ? request->sweep_direction : sweep_direction_;

  params.start_corner = !request->start_corner.empty() ? request->start_corner : start_corner_;

  params.pose_mode = !request->pose_mode.empty() ? request->pose_mode
                                                 : "identity";  // Default to identity (no rotation)

  params.map_frame = map_frame_;

  // Boundary coverage parameters
  params.enable_boundary_pass = request->enable_boundary_pass;
  params.boundary_offset =
    (request->boundary_offset > 0.0) ? request->boundary_offset : 0.02;  // Default 2cm offset

  // Strip connector interpolation (for boustrophedon)
  params.add_strip_connectors = request->add_strip_connectors || add_strip_connectors_;

  // Tool contact facet, populated by the caller from the active ToolProfile.task.contact --
  // drives requires_contact on every emitted strip/boundary-pass MotionPlanSegment.
  params.requires_contact = request->requires_contact;

  // Handle optional region
  params.use_region = !request->region.polygon.points.empty();
  params.region = ros_conv::toCore(request->region);
  params.plan_all_components = plan_all_components_;
  params.min_component_cells = min_component_cells_;
  params.optimize_strip_order = optimize_strip_order_;
  params.components_in_sweep_order = components_in_sweep_order_;
  params.fill_gaps = fill_gaps_;
  params.min_stroke_length_m = min_stroke_length_m_;
  params.edge_zone_m = edge_zone_m_;
  params.edge_spacing_scale = edge_spacing_scale_;
  params.decomposition = decomposition_;

  return params;
}

void CoveragePlannerNode::publishWaypointMarkers(const nav_msgs::msg::Path & path)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Delete previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  // Create sphere marker for each waypoint
  for (size_t i = 0; i < path.poses.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = "waypoints";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = path.poses[i].pose;

    // Small sphere size
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Green color with transparency
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8f;

    marker.lifetime = rclcpp::Duration::from_seconds(0);  // Persistent

    marker_array.markers.push_back(marker);
  }

  markers_pub_->publish(marker_array);
  RCLCPP_INFO(this->get_logger(), "Published %zu waypoint markers", path.poses.size());
}

void CoveragePlannerNode::publishDiagnostics(
  const CoveragePlannerResult & result, const std::string & pattern,
  const nav_msgs::msg::OccupancyGrid & map)
{
  std_msgs::msg::String msg;
  std::ostringstream ss;
  ss << "pattern=" << pattern << ", map=" << map.info.width << "x" << map.info.height
     << ", waypoints=" << result.num_waypoints << ", strips=" << result.num_strips
     << ", components=" << result.free_component_count << ", step=" << result.effective_step_used
     << ", distance=" << result.total_distance << ", planning_time=" << result.planning_time;
  msg.data = ss.str();
  diagnostics_pub_->publish(msg);
}

void CoveragePlannerNode::generateCoveragePathCallback(
  const std::shared_ptr<srv::GenerateCoveragePath::Request> request,
  std::shared_ptr<srv::GenerateCoveragePath::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received coverage path generation request");

  response->segments.clear();
  response->segment_labels.clear();

  // Check if we have a map (either from request or subscription)
  // Priority: request map > subscribed map
  bool has_map_in_request = !request->map.data.empty();
  bool has_subscribed_map = false;

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    has_subscribed_map = (latest_map_ != nullptr);
  }

  if (!has_map_in_request && !has_subscribed_map) {
    response->success = false;
    response->message = "No map received yet. Waiting for map on topic: " + map_topic_;
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  // Determine which planner to use
  std::string pattern_to_use = !request->pattern.empty() ? request->pattern : coverage_pattern_;

  // Create planner instance
  auto planner = CoveragePlannerFactory::createPlanner(pattern_to_use);
  if (!planner) {
    response->success = false;
    response->message = "Unknown coverage pattern: " + pattern_to_use;

    auto available = CoveragePlannerFactory::getAvailablePlanners();
    response->message += "\nAvailable patterns: ";
    for (const auto & name : available) {
      response->message += name + " ";
    }

    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  planner->setLogger(std::make_shared<ros_adapters::RclcppLogger>(this->get_logger()));

  // Build parameters
  CoveragePlannerParams params = buildPlannerParams(request);

  RCLCPP_INFO(
    this->get_logger(), "Generating %s coverage path (swath: %.2fm, step: %.2fm, direction: %s)",
    pattern_to_use.c_str(), params.swath, params.step, params.sweep_direction.c_str());

  // Generate path
  nav_msgs::msg::OccupancyGrid map_to_use;

  // Check if map is provided in request
  if (!request->map.data.empty()) {
    // Use map from request
    map_to_use = request->map;
    RCLCPP_INFO(
      this->get_logger(), "Using map from service request: %dx%d cells, resolution: %.3fm",
      map_to_use.info.width, map_to_use.info.height, map_to_use.info.resolution);
  } else {
    // Fall back to subscribed map
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!latest_map_) {
      response->success = false;
      response->message = "No map provided in request and no map received from topic yet";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    map_to_use = *latest_map_;
    RCLCPP_INFO(
      this->get_logger(), "Using map from topic subscription: %dx%d cells", map_to_use.info.width,
      map_to_use.info.height);
  }

  CoveragePlannerResult result = planner->generatePath(ros_conv::toCore(map_to_use), params);
  // The core carries no time stamps; stamp the path with this node's own clock so use_sim_time
  // is respected, before response/publish/markers below so all three see the same stamp.
  nav_msgs::msg::Path path_msg = ros_conv::toRos(result.path);
  path_msg.header.stamp = this->now();

  // Fill response
  response->success = result.success;
  response->message = result.message;
  response->path = path_msg;
  response->segments = ros_conv::toRos(result.segments);
  response->segment_labels.reserve(result.segments.size());
  for (const auto & segment : result.segments) {
    response->segment_labels.push_back(segment.label);
  }
  response->total_distance = result.total_distance;
  response->coverage_area = result.coverage_area;
  response->num_waypoints = result.num_waypoints;
  response->planning_time = result.planning_time;
  response->pattern_used = pattern_to_use;

  if (result.success) {
    RCLCPP_INFO(this->get_logger(), "Coverage path generated successfully:");
    RCLCPP_INFO(this->get_logger(), "  Waypoints: %d", result.num_waypoints);
    RCLCPP_INFO(this->get_logger(), "  Total Distance: %.2f m", result.total_distance);
    RCLCPP_INFO(this->get_logger(), "  Coverage Area: %.2f m²", result.coverage_area);
    RCLCPP_INFO(this->get_logger(), "  Planning Time: %.3f s", result.planning_time);

    // Publish path for visualization
    path_pub_->publish(path_msg);

    // Publish waypoint markers for visualization
    publishWaypointMarkers(path_msg);

    // Publish diagnostics summary for monitoring/troubleshooting.
    publishDiagnostics(result, pattern_to_use, map_to_use);
  } else {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to generate coverage path: %s", result.message.c_str());
  }
}

}  // namespace coverage_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<coverage_planner::CoveragePlannerNode>();

  RCLCPP_INFO(node->get_logger(), "Coverage Planner Node spinning...");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
