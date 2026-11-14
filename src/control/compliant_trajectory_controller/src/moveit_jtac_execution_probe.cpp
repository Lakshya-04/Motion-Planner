/**
 * @file moveit_jtac_execution_probe.cpp
 * @brief Gate T10 (Level 2): proves moveit::planning_interface::MoveGroupInterface can plan AND
 *        execute a real joint-space goal through JtacController, exactly as it would through
 *        stock joint_trajectory_controller/JointTrajectoryController -- no MoveItExecutionAdapter
 *        changes needed, since that class only ever calls MoveGroupInterface::execute().
 * @details Run against 7dof_serum_moveit.launch.py with use_jtac_controller:=true. Uses the
 *          SAME validated joint-space target (VALIDATED_POSE) already proven collision-free,
 *          in-bounds, and well-conditioned by reference_pose_collision_probe/
 *          pose_conditioning_probe (this package) and reused across the T7/T8/T9 raw-trajectory
 *          tests -- reused here rather than re-derived, so a failure here is attributable to the
 *          MoveIt integration itself, not an unvalidated target pose.
 *
 *          Reusable diagnostic, matching pose_conditioning_probe/reference_pose_collision_probe's
 *          own established pattern in this package -- not a throwaway script.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include "moveit/move_group_interface/move_group_interface.h"
#include "mobile_manipulator_profile/profile.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
// Default target, in the group's joint order (lift, then 6 arm joints): the validated pose used
// throughout T7/T8/T9's own raw-trajectory tests. Another DOF needs the `target_joint_pose`
// parameter (or the profile's home pose via `use_profile_home`).
const std::vector<double> kValidatedPose{0.40, 0.0, -0.15, -0.15, 0.0, -0.15, 0.0};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // NOT automatically_declare_parameters_from_overrides(true): that flag auto-declares every
  // -p override BEFORE this file's own declare_parameter() call runs, so passing
  // -p planning_group:=... would throw ParameterAlreadyDeclaredException here. declare_parameter()
  // alone already picks up a matching -p override as its initial value -- no extra flag needed.
  auto node = rclcpp::Node::make_shared("moveit_jtac_execution_probe");

  // Names come from the robot profile (`profile` parameter, config/profiles/<name>.yaml); the
  // explicit parameters below still win when set.
  node->declare_parameter("profile", std::string(mobile_manipulator_profile::kDefaultProfile));
  std::string profile_warning;
  const auto profile = mobile_manipulator_profile::loadProfileOrDefaults(
    node->get_parameter("profile").as_string(), profile_warning);
  if (!profile_warning.empty()) {
    RCLCPP_WARN(node->get_logger(), "%s", profile_warning.c_str());
  }
  node->declare_parameter("planning_group", profile.whole_body_group);
  node->declare_parameter("target_joint_pose", std::vector<double>{});
  node->declare_parameter("use_profile_home", false);
  const auto planning_group = node->get_parameter("planning_group").as_string();
  std::vector<double> target_pose = node->get_parameter("target_joint_pose").as_double_array();
  if (target_pose.empty() && node->get_parameter("use_profile_home").as_bool()) {
    target_pose = profile.homePositions();
  }
  if (target_pose.empty()) {
    target_pose = kValidatedPose;
  }

  // MoveGroupInterface needs its node spinning (planning scene monitor, action clients) on a
  // separate thread -- standard requirement, documented in every MoveGroupInterface example.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
  move_group.setPlanningTime(10.0);

  const auto start_state = move_group.getCurrentState(5.0);
  if (!start_state) {
    RCLCPP_FATAL(node->get_logger(), "Failed to get current robot state within timeout.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }
  std::vector<double> before_positions;
  start_state->copyJointGroupPositions(planning_group, before_positions);
  std::string before_str;
  for (double v : before_positions) {
    before_str += std::to_string(v) + " ";
  }
  RCLCPP_INFO(node->get_logger(), "Before: [ %s]", before_str.c_str());

  const auto kJointNames = move_group.getActiveJoints();
  if (kJointNames.size() != target_pose.size()) {
    RCLCPP_FATAL(
      node->get_logger(),
      "Group '%s' has %zu joints but the target pose has %zu values: pass target_joint_pose "
      "(or use_profile_home:=true).", planning_group.c_str(), kJointNames.size(),
      target_pose.size());
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }
  std::map<std::string, double> target;
  for (size_t i = 0; i < kJointNames.size(); ++i) {
    target[kJointNames[i]] = target_pose[i];
  }
  move_group.setJointValueTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const bool planned = static_cast<bool>(move_group.plan(plan));
  RCLCPP_INFO(node->get_logger(), "plan() success = %s", planned ? "true" : "false");
  if (!planned) {
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  const auto execute_result = move_group.execute(plan);
  const bool executed = execute_result == moveit::core::MoveItErrorCode::SUCCESS;
  RCLCPP_INFO(
    node->get_logger(), "execute() error_code = %d (SUCCESS = %d)",
    execute_result.val, moveit::core::MoveItErrorCode::SUCCESS);

  const auto end_state = move_group.getCurrentState(5.0);
  std::vector<double> after_positions;
  end_state->copyJointGroupPositions(planning_group, after_positions);
  std::string after_str;
  double max_abs_error = 0.0;
  for (size_t i = 0; i < after_positions.size(); ++i) {
    after_str += std::to_string(after_positions[i]) + " ";
    max_abs_error = std::max(max_abs_error, std::abs(after_positions[i] - kValidatedPose[i]));
  }
  RCLCPP_INFO(node->get_logger(), "After:  [ %s]", after_str.c_str());
  RCLCPP_INFO(node->get_logger(), "max_abs_error_from_target = %.5f rad", max_abs_error);

  const bool within_tolerance = max_abs_error < 0.02;
  RCLCPP_INFO(
    node->get_logger(), "=== %s ===",
    (executed && within_tolerance) ? "T10 LEVEL 2 PASSED" : "T10 LEVEL 2 FAILED");

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return (executed && within_tolerance) ? 0 : 1;
}
