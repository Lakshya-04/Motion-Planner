/**
 * @file pose.hpp
 * @brief Pure position + orientation type, with no ROS message dependency.
 * @details Adapters (e.g. motion_planner) translate to/from
 *          geometry_msgs::msg::Pose at the ROS boundary -- this type never
 *          crosses that boundary itself. See the ports-and-adapters split
 *          described in docs/design_principles.md.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

namespace core_types
{

/**
 * @struct Pose
 * @brief Plain position + orientation (quaternion), independent of ROS.
 */
struct Pose
{
  double x{0.0};   ///< X position.
  double y{0.0};   ///< Y position.
  double z{0.0};   ///< Z position.
  double qx{0.0};  ///< Quaternion X component.
  double qy{0.0};  ///< Quaternion Y component.
  double qz{0.0};  ///< Quaternion Z component.
  double qw{1.0};  ///< Quaternion W component (defaults to identity rotation).
};

}  // namespace core_types
