/**
 * @file planned_path_marker_utils.hpp
 * @brief RViz marker helpers for visualizing an FK'd TCP curve, colored by
 *        which planner produced it.
 * @details Shared by motion_planner's PlannerBenchmark and
 *          manipulation_manager's MotionValidator -- both need to
 *          publish the exact same "planned path" visualization (a
 *          LINE_STRIP for the overall shape, a dense per-point SPHERE_LIST
 *          trail, and a larger two-point SPHERE_LIST for the chunk's own
 *          start/end), and neither package may depend on the other (see
 *          motion_planner::MotionPlanner's own doc comment for the
 *          established one-way dependency direction) -- so this lives here,
 *          in the shared adapter package both already depend on, instead of
 *          being duplicated in each.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "Eigen/Geometry"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace ros_adapters
{

/// Which planner produced a marker chunk determines its color -- yellow for
/// Cartesian, green for FSP (a bridged Cartesian pass's own fsp legs must
/// stay visually distinguishable from a purely fsp-routed segment, so they
/// can't share a color). Anything else (shouldn't happen, but never
/// silently reuse a real color for an unexpected value) gets a distinct gray.
struct PlannedPathMarkerColor
{
  float r;
  float g;
  float b;
};

inline PlannedPathMarkerColor colorForPlanner(const std::string & planner_name)
{
  if (planner_name == "cartesian") {
    return {1.0f, 1.0f, 0.0f};
  }
  if (planner_name == "fsp") {
    return {0.0f, 1.0f, 0.0f};
  }
  return {0.5f, 0.5f, 0.5f};
}

inline visualization_msgs::msg::Marker makePlannedLineStripMarker(
  const std::string & frame_id, int32_t id, const std::string & planner_name,
  const std::vector<Eigen::Vector3d> & curve)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "planned";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.012;
  const PlannedPathMarkerColor color = colorForPlanner(planner_name);
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  marker.color.a = 1.0f;
  marker.points.reserve(curve.size());
  for (const auto & sample : curve) {
    geometry_msgs::msg::Point point;
    point.x = sample.x();
    point.y = sample.y();
    point.z = sample.z();
    marker.points.push_back(point);
  }
  return marker;
}

// SPHERE_LIST dots along the *whole* curve, not just its endpoints --
// RViz's LINE_STRIP is a camera-facing billboard quad whose on-screen width
// comes from cross(view_direction, local_tangent), so a segment viewed
// nearly end-on (view direction ~parallel to its tangent) collapses to zero
// width and vanishes; a sphere has no such degenerate viewing angle. Since
// different segments (especially the curved fsp bridge legs) have
// different tangents, which segments vanish shifts with camera angle --
// this is what made the line disappear unpredictably depending on viewpoint
// when only the two endpoints were dotted. A dense dotted trail never
// vanishes, regardless of angle.
inline visualization_msgs::msg::Marker makePlannedPathDotsMarker(
  const std::string & frame_id, int32_t id, const std::string & planner_name,
  const std::vector<Eigen::Vector3d> & curve)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "planned_dots";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.007;
  marker.scale.y = 0.007;
  marker.scale.z = 0.007;
  const PlannedPathMarkerColor color = colorForPlanner(planner_name);
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  marker.color.a = 1.0f;
  marker.points.reserve(curve.size());
  for (const auto & sample : curve) {
    geometry_msgs::msg::Point point;
    point.x = sample.x();
    point.y = sample.y();
    point.z = sample.z();
    marker.points.push_back(point);
  }
  return marker;
}

// Larger start/end markers on top of the dense trail above -- lets a
// glance at the chunk's two ends tell you where a hand-off actually
// happened (e.g. a bridge's start point vs. its resume point) without
// having to count small dots.
inline visualization_msgs::msg::Marker makePlannedEndpointMarkers(
  const std::string & frame_id, int32_t id, const std::string & planner_name,
  const std::vector<Eigen::Vector3d> & curve)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "planned_endpoints";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.026;
  marker.scale.y = 0.026;
  marker.scale.z = 0.026;
  const PlannedPathMarkerColor color = colorForPlanner(planner_name);
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  marker.color.a = 1.0f;
  if (curve.empty()) {
    return marker;
  }
  geometry_msgs::msg::Point start;
  start.x = curve.front().x();
  start.y = curve.front().y();
  start.z = curve.front().z();
  marker.points.push_back(start);
  if (curve.size() > 1) {
    geometry_msgs::msg::Point end;
    end.x = curve.back().x();
    end.y = curve.back().y();
    end.z = curve.back().z();
    marker.points.push_back(end);
  }
  return marker;
}

}  // namespace ros_adapters
