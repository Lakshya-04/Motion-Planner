/**
 * @file wall_octomap_probe.cpp
 * @brief See wall_octomap_probe.hpp's own doc comment.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include "compliant_trajectory_controller/wall_octomap_probe.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <thread>
#include <utility>

#include <octomap_msgs/conversions.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "std_msgs/msg/string.hpp"

namespace compliant_trajectory_controller
{

WallOctomapProbe::WallOctomapProbe(rclcpp::Node::SharedPtr node)
: node_(std::move(node)) {}

bool WallOctomapProbe::initialize(
  const std::string & kinematics_base, const std::string & kinematics_tip)
{
  std::string robot_description;
  {
    auto rd_node = std::make_shared<rclcpp::Node>("wall_octomap_probe_rd_fetcher");
    std::promise<std::string> rd_promise;
    auto rd_future = rd_promise.get_future();
    auto sub = rd_node->create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).transient_local(),
      [&rd_promise](const std_msgs::msg::String::SharedPtr msg) {
        rd_promise.set_value(msg->data);
      });
    rclcpp::executors::SingleThreadedExecutor rd_executor;
    rd_executor.add_node(rd_node);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (rd_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready &&
      std::chrono::steady_clock::now() < deadline)
    {
      rd_executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (rd_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(), "WallOctomapProbe: /robot_description not received.");
      return false;
    }
    robot_description = rd_future.get();
  }

  // declare_parameter() directly, NOT NodeOptions::parameter_overrides() -- an override alone
  // sits in a pending pool and is invisible to get_parameter() until something actually declares
  // that name; KinematicsInterfaceKDL::initialize() calls get_parameter() itself with no prior
  // declare_parameter() of its own.
  kinematics_node_ = std::make_shared<rclcpp::Node>("wall_octomap_probe_kdl");
  kinematics_node_->declare_parameter("robot_description", robot_description);
  kinematics_node_->declare_parameter(
    "kinematics.plugin_name", std::string("kinematics_interface_kdl/KinematicsInterfaceKDL"));
  kinematics_node_->declare_parameter(
    "kinematics.plugin_package", std::string("kinematics_interface"));
  kinematics_node_->declare_parameter("kinematics.base", kinematics_base);
  kinematics_node_->declare_parameter("kinematics.tip", kinematics_tip);
  kinematics_node_->declare_parameter("kinematics.alpha", 0.0005);
  kinematics_loader_ = std::make_shared<
    pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
    "kinematics_interface", "kinematics_interface::KinematicsInterface");
  kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
    kinematics_loader_->createUnmanagedInstance("kinematics_interface_kdl/KinematicsInterfaceKDL"));
  // "kinematics": kinematics_node_ already declares "kinematics.tip" = kinematics_tip above.
  if (!kinematics_->initialize(
      robot_description, kinematics_node_->get_node_parameters_interface(), "kinematics"))
  {
    RCLCPP_ERROR(node_->get_logger(), "WallOctomapProbe: kinematics initialize() failed.");
    return false;
  }
  kinematics_tip_ = kinematics_tip;

  auto client = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  if (!client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(), "WallOctomapProbe: /get_planning_scene not available.");
    return false;
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  // WORLD_OBJECT_GEOMETRY (16) | OCTOMAP (32) -- see moveit_msgs::msg::PlanningSceneComponents.
  request->components.components = 16 | 32;
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(10)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "WallOctomapProbe: GetPlanningScene call failed/timed out.");
    return false;
  }
  const auto response = future.get();
  const auto & octomap_with_pose = response->scene.world.octomap;
  if (octomap_with_pose.octomap.data.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "WallOctomapProbe: planning scene's octomap is empty -- was push_world_geometry actually "
      "run (wall_mount_publisher_node/motion_validator_node)?");
    return false;
  }
  octomap_frame_id_ = octomap_with_pose.header.frame_id;
  tf2::fromMsg(octomap_with_pose.origin, octomap_origin_);
  std::unique_ptr<octomap::AbstractOcTree> abstract_tree(
    octomap_msgs::msgToMap(octomap_with_pose.octomap));
  octree_ = std::unique_ptr<octomap::OcTree>(
    dynamic_cast<octomap::OcTree *>(abstract_tree.release()));
  if (!octree_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "WallOctomapProbe: octomap_msgs::msgToMap() failed or returned an unexpected tree type.");
    return false;
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "WallOctomapProbe: loaded a real octree (%zu leaf nodes, resolution=%.3fm, frame='%s') from "
    "the live planning scene.",
    octree_->getNumLeafNodes(), octree_->getResolution(), octomap_frame_id_.c_str());
  return true;
}

bool WallOctomapProbe::calculateTipTransform(
  const std::vector<double> & joint_positions, Eigen::Isometry3d & tip_in_kinematics_base) const
{
  const Eigen::VectorXd q =
    Eigen::VectorXd::Map(joint_positions.data(), static_cast<long>(joint_positions.size()));
  return kinematics_->calculate_link_transform(q, kinematics_tip_, tip_in_kinematics_base);
}

WallOctomapProbe::ContactCheckResult WallOctomapProbe::checkContact(
  const std::vector<double> & joint_positions) const
{
  ContactCheckResult result;
  Eigen::Isometry3d tip_in_kinematics_base;
  if (!calculateTipTransform(joint_positions, tip_in_kinematics_base)) {
    return result;
  }
  const Eigen::Vector3d point_in_octree_frame =
    octomap_origin_.inverse() * tip_in_kinematics_base.translation();
  const double margin = octree_->getResolution();
  // dx=0 listed first in each axis so the (0,0,0) exact point is always tried before any
  // neighbor -- reported precisely, not just "a match was found somewhere in the neighborhood."
  for (double dx : {0.0, -margin, margin}) {
    for (double dy : {0.0, -margin, margin}) {
      for (double dz : {0.0, -margin, margin}) {
        const auto * node_ptr = octree_->search(
          point_in_octree_frame.x() + dx, point_in_octree_frame.y() + dy,
          point_in_octree_frame.z() + dz);
        if (node_ptr != nullptr && octree_->isNodeOccupied(node_ptr)) {
          result.contact = true;
          result.margin_offset_m = std::sqrt(dx * dx + dy * dy + dz * dz);
          result.exact_point_occupied = (result.margin_offset_m == 0.0);
          return result;
        }
      }
    }
  }
  return result;
}

double WallOctomapProbe::computeSignedPenetrationM(
  const std::vector<double> & joint_positions, const Eigen::Vector3d & direction_in_kinematics_base,
  double contact_surface_offset_m, double step_m, double max_search_m) const
{
  Eigen::Isometry3d tip_in_kinematics_base;
  if (!calculateTipTransform(joint_positions, tip_in_kinematics_base)) {
    return 0.0;
  }
  const Eigen::Vector3d direction = direction_in_kinematics_base.normalized();
  const Eigen::Vector3d tip_point =
    tip_in_kinematics_base.translation() + contact_surface_offset_m * direction;

  const auto occupied_at = [this, &direction](
    const Eigen::Vector3d & base_point, double distance_along_direction) {
      const Eigen::Vector3d point_world = base_point + distance_along_direction * direction;
      const Eigen::Vector3d point_in_octree_frame = octomap_origin_.inverse() * point_world;
      const auto * node_ptr = octree_->search(
        point_in_octree_frame.x(), point_in_octree_frame.y(), point_in_octree_frame.z());
      return node_ptr != nullptr && octree_->isNodeOccupied(node_ptr);
    };

  if (!occupied_at(tip_point, 0.0)) {
    return 0.0;  // Still in free air -- no contact force, not a small negative one.
  }
  for (double d = step_m; d <= max_search_m; d += step_m) {
    if (!occupied_at(tip_point, -d)) {
      return d;  // Stepped backward (away from the wall) by d to reach free space again.
    }
  }
  RCLCPP_WARN(
    node_->get_logger(),
    "WallOctomapProbe::computeSignedPenetrationM(): tip is occupied but no free sample found "
    "within %.3fm stepping backward -- returning 0.0 rather than a made-up depth.",
    max_search_m);
  return 0.0;
}

}  // namespace compliant_trajectory_controller
