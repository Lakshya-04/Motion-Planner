// Conversions between scene_model types and ROS messages (sensor_msgs / shape_msgs / moveit_msgs).
// Messages only: no rclcpp here, so these are usable from any node or from tests.
#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "scene_model/core/scene_model.hpp"
#include "scene_model/core/types.hpp"

namespace scene_model
{
namespace ros_conv
{

geometry_msgs::msg::Pose toRos(const Transform & pose);
Transform fromRos(const geometry_msgs::msg::Pose & pose);

/// Reads x, y, z (float32) and, when present, `rgb` / `rgba` (packed) and `label` (uint32).
/// Points with a non-finite coordinate are dropped (with their color / label). Throws
/// std::runtime_error if x, y or z is missing or is not float32.
PointCloud fromRos(const sensor_msgs::msg::PointCloud2 & msg);

/// Same, also reporting for every returned point its index in the message (so a mask computed on
/// the returned cloud can be mapped back onto the message, which still holds the dropped points).
PointCloud fromRos(const sensor_msgs::msg::PointCloud2 & msg, std::vector<size_t> & source_index);

/// Unorganized cloud with fields x, y, z and, when the cloud carries them, rgb and label.
sensor_msgs::msg::PointCloud2 toRos(const PointCloud & cloud, const std::string & frame_id);

/// Keeps the points of `msg` whose `removed` entry is false, copying every field untouched
/// (intensity, ring, time, ...). The result is unorganized (height 1). `removed` must have one
/// entry per point (width * height).
sensor_msgs::msg::PointCloud2 keepPoints(
  const sensor_msgs::msg::PointCloud2 & msg, const std::vector<bool> & removed);

/// One collision object per scene object. Boxes, spheres, cylinders and meshes map to MoveIt
/// primitives / meshes; capsules become cylinders (MoveIt has none), point clouds, voxel grids and
/// ESDFs have no collision-object form and are listed in `skipped` when given.
std::vector<moveit_msgs::msg::CollisionObject> toRos(
  const SceneModel & scene, std::vector<std::string> * skipped = nullptr);

/// Objects of a collision object (primitives and meshes, each with its own pose composed with the
/// object's pose). Object ids are `<object id>` for a single shape and `<id>#<k>` for several.
std::vector<SceneObject> fromRos(const moveit_msgs::msg::CollisionObject & object);

/// The world of a planning scene: every collision object. The octomap, if any, is not converted.
SceneModel fromRos(const moveit_msgs::msg::PlanningScene & scene);

}  // namespace ros_conv
}  // namespace scene_model
