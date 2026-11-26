// Copyright 2026 Lakshya Agarwal
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "coverage_planner/ros/wall_coverage_service_node.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>

#include "coverage_planner/core/coverage_pass_expander.hpp"
#include "coverage_planner/core/wall_rasterizer.hpp"
#include "coverage_planner/core/wall_scene_loader.hpp"
#include "coverage_planner/ros/conversions.hpp"
#include "coverage_planner/ros/tool_profile_params.hpp"

namespace coverage_planner
{

// The interfaces live in coverage_planner_msgs; alias them into this package's own
// namespace so `msg::`/`srv::` still mean what they always did here.
namespace msg = coverage_planner_msgs::msg;

WallCoverageServiceNode::WallCoverageServiceNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("wall_coverage_service_node", options)
{
  // wall_scene_json/wall_id/etc. are node parameters, not request fields: ~/start is a
  // std_srvs/Trigger, which carries none. Set these (ros2 param set, or a params file) before
  // calling ~/start.
  this->declare_parameter<std::string>("wall_scene_json", "");
  this->declare_parameter<int64_t>("wall_id", 0);
  this->declare_parameter<double>("resolution", 0.02);
  this->declare_parameter<std::string>(
    "coverage_service_name", "/coverage_planner_node/generate_coverage_path");
  this->declare_parameter<bool>("allow_flatness_override", false);
  this->declare_parameter<bool>("add_strip_connectors", false);
  this->declare_parameter<bool>("allow_unknown", false);

  // Tool profile: tool_profiles_file + tool_name select it, optional params override fields.
  tool_profile_ = loadToolCoverageProfileFromParams(*this);

  // Latched (transient_local), so a consumer that starts after the plan was published (manipulation_manager with
  // coverage_source:=ros) still gets it.
  plan_pub_ = this->create_publisher<msg::CoveragePlan>(
    "~/coverage_plan", rclcpp::QoS(1).reliable().transient_local());

  start_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/start",
    std::bind(
      &WallCoverageServiceNode::startCallback, this, std::placeholders::_1, std::placeholders::_2));
  stop_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/stop",
    std::bind(
      &WallCoverageServiceNode::stopCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Services ready: ~/start, ~/stop. Publishing on ~/coverage_plan");
}

void WallCoverageServiceNode::startCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (busy_.exchange(true)) {
    response->success = false;
    response->message = "a plan is already in progress";
    return;
  }
  stop_requested_ = false;

  try {
    requestCoveragePath();
  } catch (const std::exception & e) {
    busy_ = false;
    response->success = false;
    response->message = e.what();
    return;
  }

  // Acknowledges that a request is under way, not that the plan is ready -- the plan itself
  // arrives later, on ~/coverage_plan (see this class's own doc comment for why: the
  // GenerateCoveragePath call is async on purpose).
  response->success = true;
  response->message = "coverage path requested; result will be published on ~/coverage_plan";
}

void WallCoverageServiceNode::stopCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  stop_requested_ = true;
  response->success = true;
  response->message = busy_ ? "will suppress the in-flight plan's publish" : "nothing was running";
}

void WallCoverageServiceNode::requestCoveragePath()
{
  const auto wall_scene_json = this->get_parameter("wall_scene_json").as_string();
  if (wall_scene_json.empty()) {
    throw std::runtime_error("'wall_scene_json' parameter is required");
  }
  const auto wall_id = static_cast<uint32_t>(this->get_parameter("wall_id").as_int());
  const auto resolution = this->get_parameter("resolution").as_double();

  WallFlatnessThresholds thresholds;
  thresholds.allow_override = this->get_parameter("allow_flatness_override").as_bool();
  const auto scene = loadWallScene(wall_scene_json, wall_id, thresholds);
  if (!scene.flatness_warning.empty()) {
    RCLCPP_WARN(this->get_logger(), "%s", scene.flatness_warning.c_str());
  }
  wall_ = scene.wall;

  const auto & tool_profile = tool_profile_;

  // Mask against the actual point cloud so coverage only plans where the wall was really
  // measured -- see wall_coverage_demo_node's own use of this same pair of calls.
  const std::string walls_pcd_path =
    (std::filesystem::path(wall_scene_json).parent_path() / "walls.pcd").string();
  const double u_extent = wall_.u_max - wall_.u_min;
  const double v_extent = wall_.v_max - wall_.v_min;
  const int grid_width = std::max(1, static_cast<int>(std::ceil(u_extent / resolution)));
  const int grid_height = std::max(1, static_cast<int>(std::ceil(v_extent / resolution)));
  const auto observed_mask =
    loadWallObservedMask(walls_pcd_path, wall_id, wall_, grid_width, grid_height, resolution);

  const auto grid = rasterizeWallCoverageGrid(
    wall_, scene.obstacles, tool_profile.coverage, resolution, wall_.pose.header.frame_id,
    observed_mask);

  auto request = std::make_shared<GenerateCoveragePath::Request>();
  request->map = ros_conv::toRos(grid);
  request->pattern = "boustrophedon";
  request->swath = static_cast<float>(toolSwath(tool_profile));
  request->step = static_cast<float>(toolLineSpacing(tool_profile));
  request->sweep_direction = tool_profile.coverage.sweep_direction;
  request->start_corner = "bottom_left";
  request->obstacle_inflation_radius = 0.0f;  // already rasterized into the grid above.
  request->enable_boundary_pass = tool_profile.coverage.enable_boundary_pass;
  request->boundary_offset = static_cast<float>(tool_profile.coverage.boundary_offset_m);
  request->requires_contact = tool_profile.task.contact;
  request->edge_zone_m = static_cast<float>(tool_profile.coverage.edge_zone_m);
  request->edge_spacing_scale = static_cast<float>(tool_profile.coverage.edge_spacing_scale);
  request->add_strip_connectors = this->get_parameter("add_strip_connectors").as_bool();
  request->allow_unknown = this->get_parameter("allow_unknown").as_bool();

  if (!coverage_client_) {
    coverage_client_ = this->create_client<GenerateCoveragePath>(
      this->get_parameter("coverage_service_name").as_string());
  }
  if (!coverage_client_->wait_for_service(std::chrono::seconds(5))) {
    throw std::runtime_error(
      "no '" + this->get_parameter("coverage_service_name").as_string() + "' service");
  }

  // Async on purpose -- see this class's own doc comment.
  coverage_client_->async_send_request(
    request,
    std::bind(&WallCoverageServiceNode::coveragePathResponseCallback, this, std::placeholders::_1));
}

void WallCoverageServiceNode::coveragePathResponseCallback(
  rclcpp::Client<GenerateCoveragePath>::SharedFuture future)
{
  const bool suppressed = stop_requested_.exchange(false);
  busy_ = false;

  const auto response = future.get();
  if (suppressed) {
    RCLCPP_INFO(this->get_logger(), "plan finished but ~/stop was called first; not publishing");
    return;
  }

  msg::CoveragePlan plan;
  plan.success = response->success;
  plan.message = response->message;
  plan.total_distance = response->total_distance;
  plan.coverage_area = response->coverage_area;
  plan.num_waypoints = response->num_waypoints;
  plan.planning_time = response->planning_time;
  plan.pattern_used = response->pattern_used;

  if (!response->success) {
    RCLCPP_ERROR(this->get_logger(), "coverage generation failed: %s", response->message.c_str());
    plan_pub_->publish(plan);
    return;
  }

  const auto & tool_profile = tool_profile_;
  std::vector<MotionPlanSegment> flat_segments;
  flat_segments.reserve(response->segments.size());
  for (const auto & segment : response->segments) {
    flat_segments.push_back(ros_conv::toCore(segment));
  }
  const auto expanded = expandCoveragePasses(
    ros_conv::toCore(response->path), flat_segments, wall_, tool_profile);
  plan.path = ros_conv::toRos(expanded.path);
  plan.path.header.stamp = response->path.header.stamp;
  plan.segments = ros_conv::toRos(expanded.segments);

  RCLCPP_INFO(
    this->get_logger(), "coverage path: %zu waypoints, %zu segments", expanded.path.poses.size(),
    expanded.segments.size());
  plan_pub_->publish(plan);
}

}  // namespace coverage_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<coverage_planner::WallCoverageServiceNode>());
  rclcpp::shutdown();
  return 0;
}
