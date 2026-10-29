/**
 * @file wall_surface_probe.cpp
 * @brief Reusable diagnostic: ray-marches a straight line through the LIVE pushed wall octomap and
 *        reports the first genuinely occupied point along it.
 * @details Built to settle, with pure geometry against the SAME octomap MoveIt itself collision-
 *          checks against, a question the IK/motion-planning tools in this package could not
 *          answer cleanly: exactly how far past an analytically fitted wall plane does the real
 *          scanned surface begin? Two attempts at deriving that Cartesian depth through a robot
 *          motion (moveit_pose_finder's collision_aware:=false setFromIK(), and its
 *          computeCartesianPath()+ACM-relaxation mode) each turned out to be confounded by
 *          something UNRELATED to the wall itself -- unconstrained IK overshooting clean through a
 *          thin voxel shell with nothing to stop it, and a 7-DOF redundant chain's Jacobian-based
 *          Cartesian interpolation drifting through large, uncontrolled null-space motion between
 *          waypoints. Ray-marching the octomap directly has neither confound: no IK, no
 *          redundancy, no robot at all -- just "is this exact 3D point occupied," walked along one
 *          direction from a known-free start.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <chrono>
#include <cmath>
#include <cstdio>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("wall_surface_probe");

  node->declare_parameter("start_xyz", std::vector<double>{0.0, 0.0, 0.0});
  node->declare_parameter("direction_xyz", std::vector<double>{0.0, -1.0, 0.0});
  node->declare_parameter("max_distance_m", 0.30);
  node->declare_parameter("step_m", 0.005);
  // false (default): stop and report at the first occupied point. true: walk the WHOLE ray and
  // log every free<->occupied transition -- lets a caller see whether a real scanned wall is a
  // thin single-voxel shell or has real depth/thickness, instead of just its near face.
  node->declare_parameter("report_all_transitions", false);

  const auto start_xyz = node->get_parameter("start_xyz").as_double_array();
  const auto direction_xyz = node->get_parameter("direction_xyz").as_double_array();
  const auto max_distance_m = node->get_parameter("max_distance_m").as_double();
  const auto step_m = node->get_parameter("step_m").as_double();
  const auto report_all_transitions = node->get_parameter("report_all_transitions").as_bool();

  if (start_xyz.size() != 3 || direction_xyz.size() != 3) {
    RCLCPP_FATAL(node->get_logger(), "start_xyz and direction_xyz must each have 3 elements.");
    rclcpp::shutdown();
    return 1;
  }
  Eigen::Vector3d start(start_xyz[0], start_xyz[1], start_xyz[2]);
  Eigen::Vector3d direction(direction_xyz[0], direction_xyz[1], direction_xyz[2]);
  if (direction.norm() < 1e-9) {
    RCLCPP_FATAL(node->get_logger(), "direction_xyz must be non-zero.");
    rclcpp::shutdown();
    return 1;
  }
  direction.normalize();

  auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  if (!client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_FATAL(node->get_logger(), "/get_planning_scene not available.");
    rclcpp::shutdown();
    return 1;
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  // WORLD_OBJECT_GEOMETRY (16) | OCTOMAP (32) -- see moveit_msgs::msg::PlanningSceneComponents.
  request->components.components = 16 | 32;
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_FATAL(node->get_logger(), "GetPlanningScene call failed/timed out.");
    rclcpp::shutdown();
    return 1;
  }
  const auto response = future.get();
  const auto & octomap_with_pose = response->scene.world.octomap;
  if (octomap_with_pose.octomap.data.empty()) {
    RCLCPP_FATAL(
      node->get_logger(),
      "planning scene's octomap is empty -- was push_world_geometry actually run?");
    rclcpp::shutdown();
    return 1;
  }
  const std::string octomap_frame_id = octomap_with_pose.header.frame_id;
  Eigen::Isometry3d octomap_origin;
  tf2::fromMsg(octomap_with_pose.origin, octomap_origin);
  std::unique_ptr<octomap::AbstractOcTree> abstract_tree(
    octomap_msgs::msgToMap(octomap_with_pose.octomap));
  std::unique_ptr<octomap::OcTree> octree(
    dynamic_cast<octomap::OcTree *>(abstract_tree.release()));
  if (!octree) {
    RCLCPP_FATAL(node->get_logger(), "octomap_msgs::msgToMap() failed.");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(
    node->get_logger(),
    "loaded octree (%zu leaf nodes, resolution=%.3fm, frame='%s'); marching from [%.4f %.4f %.4f] "
    "along [%.4f %.4f %.4f], step=%.4fm, max=%.4fm.",
    octree->getNumLeafNodes(), octree->getResolution(), octomap_frame_id.c_str(), start.x(),
    start.y(), start.z(), direction.x(), direction.y(), direction.z(), step_m, max_distance_m);

  bool found = false;
  bool previous_occupied = false;
  bool have_previous = false;
  for (double d = 0.0; d <= max_distance_m; d += step_m) {
    const Eigen::Vector3d point_world = start + d * direction;
    const Eigen::Vector3d point_in_octree_frame = octomap_origin.inverse() * point_world;
    const auto * node_ptr = octree->search(
      point_in_octree_frame.x(), point_in_octree_frame.y(), point_in_octree_frame.z());
    const bool occupied = node_ptr != nullptr && octree->isNodeOccupied(node_ptr);

    if (report_all_transitions && (!have_previous || occupied != previous_occupied)) {
      RCLCPP_INFO(
        node->get_logger(), "distance=%.4fm, point=[%.4f %.4f %.4f]: %s", d, point_world.x(),
        point_world.y(), point_world.z(), occupied ? "OCCUPIED" : "free");
    }
    have_previous = true;
    previous_occupied = occupied;

    if (occupied && !found) {
      RCLCPP_INFO(
        node->get_logger(),
        "FIRST OCCUPIED at distance=%.4fm, point=[%.4f %.4f %.4f] (world frame).", d,
        point_world.x(), point_world.y(), point_world.z());
      found = true;
      if (!report_all_transitions) {
        break;
      }
    }
  }
  if (!found) {
    RCLCPP_WARN(
      node->get_logger(), "No occupied voxel found within %.4fm along this ray.", max_distance_m);
  }

  rclcpp::shutdown();
  return found ? 0 : 1;
}
