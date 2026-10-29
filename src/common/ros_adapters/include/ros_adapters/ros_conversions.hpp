/**
 * @file ros_conversions.hpp
 * @brief Boundary conversions between core_types and ROS message types.
 * @details This package is the only place core_types and ROS message
 *          types should meet -- core logic (core_types, and any
 *          *_core algorithm package built on it) never includes a ROS
 *          message header directly. Shared across ROS adapter packages
 *          (motion_planner, coverage_planner, etc.) so each one
 *          doesn't duplicate or depend on another just to reuse this.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <utility>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/duration.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "core_types/pose.hpp"
#include "core_types/trajectory.hpp"

namespace ros_adapters
{

/**
 * @brief Convert a ROS pose message to the pure core Pose type.
 * @param[in] pose Source geometry_msgs pose.
 * @return Equivalent core_types::Pose.
 */
inline core_types::Pose toCore(const geometry_msgs::msg::Pose & pose)
{
  return core_types::Pose{
    pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
}

/**
 * @brief Convert a core Pose back to a ROS pose message.
 * @param[in] pose Source core_types::Pose.
 * @return Equivalent geometry_msgs::msg::Pose.
 */
inline geometry_msgs::msg::Pose toRos(const core_types::Pose & pose)
{
  geometry_msgs::msg::Pose ros_pose;
  ros_pose.position.x = pose.x;
  ros_pose.position.y = pose.y;
  ros_pose.position.z = pose.z;
  ros_pose.orientation.x = pose.qx;
  ros_pose.orientation.y = pose.qy;
  ros_pose.orientation.z = pose.qz;
  ros_pose.orientation.w = pose.qw;
  return ros_pose;
}

/**
 * @brief Convert a ROS joint trajectory message to the pure core Trajectory type.
 * @param[in] trajectory Source trajectory_msgs joint trajectory.
 * @return Equivalent core_types::Trajectory.
 */
inline core_types::Trajectory toCore(
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  core_types::Trajectory core_trajectory;
  core_trajectory.joint_names = trajectory.joint_names;
  core_trajectory.points.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    core_types::TrajectoryPoint core_point;
    core_point.positions = point.positions;
    core_point.velocities = point.velocities;
    core_point.accelerations = point.accelerations;
    core_point.time_from_start_sec = rclcpp::Duration(point.time_from_start).seconds();
    core_trajectory.points.push_back(std::move(core_point));
  }
  return core_trajectory;
}

/**
 * @brief Convert a core Trajectory back to a ROS joint trajectory message.
 * @param[in] trajectory Source core_types::Trajectory.
 * @return Equivalent trajectory_msgs::msg::JointTrajectory.
 */
inline trajectory_msgs::msg::JointTrajectory toRos(const core_types::Trajectory & trajectory)
{
  trajectory_msgs::msg::JointTrajectory ros_trajectory;
  ros_trajectory.joint_names = trajectory.joint_names;
  ros_trajectory.points.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    trajectory_msgs::msg::JointTrajectoryPoint ros_point;
    ros_point.positions = point.positions;
    ros_point.velocities = point.velocities;
    ros_point.accelerations = point.accelerations;
    ros_point.time_from_start = rclcpp::Duration::from_seconds(point.time_from_start_sec);
    ros_trajectory.points.push_back(std::move(ros_point));
  }
  return ros_trajectory;
}

}  // namespace ros_adapters
