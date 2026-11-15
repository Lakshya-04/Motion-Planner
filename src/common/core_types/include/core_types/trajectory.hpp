/**
 * @file trajectory.hpp
 * @brief Plain time-parameterized joint trajectory, independent of ROS.
 * @details Adapters (e.g. ros_adapters) translate to/from
 *          trajectory_msgs::msg::JointTrajectory at the ROS boundary -- this
 *          type never crosses that boundary itself. This is the type that
 *          lets a Planner (see motion_planner::Planner) return what it
 *          planned without also executing it -- see the ports-and-adapters
 *          split and the plan/execute decoupling (docs/design_principles.md).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <string>
#include <vector>

namespace core_types
{

/**
 * @struct TrajectoryPoint
 * @brief One time-stamped waypoint of a joint trajectory.
 */
struct TrajectoryPoint
{
  std::vector<double> positions;      ///< Per-joint position, ordered per Trajectory::joint_names.
  std::vector<double> velocities;     ///< Per-joint velocity; may be empty if unavailable.
  std::vector<double> accelerations;  ///< Per-joint acceleration; may be empty if unavailable.
  double time_from_start_sec{0.0};    ///< Time from trajectory start, in seconds.
};

/**
 * @struct Trajectory
 * @brief Plain time-parameterized joint trajectory (no ROS message dependency).
 */
struct Trajectory
{
  std::vector<std::string> joint_names;   ///< Joint order shared by every point's vectors.
  std::vector<TrajectoryPoint> points;    ///< Ordered waypoints, increasing time_from_start_sec.
};

}  // namespace core_types
