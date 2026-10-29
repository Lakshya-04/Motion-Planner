// Conversions between the ROS message types and the ROS-free core types.
// The only place in this package where both worlds are visible at once.
#pragma once

#include <vector>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include "coverage_planner/core/types.hpp"
#include "coverage_planner_msgs/msg/motion_plan_segment.hpp"
#include "coverage_planner_msgs/msg/obstacle_region.hpp"
#include "coverage_planner_msgs/msg/wall_plane.hpp"

namespace coverage_planner
{
namespace ros_conv
{

inline Pose toCore(const geometry_msgs::msg::Pose & m)
{
  Pose p;
  p.position = {m.position.x, m.position.y, m.position.z};
  p.orientation = {m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w};
  return p;
}

inline geometry_msgs::msg::Pose toRos(const Pose & p)
{
  geometry_msgs::msg::Pose m;
  m.position.x = p.position.x;
  m.position.y = p.position.y;
  m.position.z = p.position.z;
  m.orientation.x = p.orientation.x;
  m.orientation.y = p.orientation.y;
  m.orientation.z = p.orientation.z;
  m.orientation.w = p.orientation.w;
  return m;
}

inline PoseStamped toCore(const geometry_msgs::msg::PoseStamped & m)
{
  return PoseStamped{Header{m.header.frame_id}, toCore(m.pose)};
}

inline geometry_msgs::msg::PoseStamped toRos(const PoseStamped & p)
{
  geometry_msgs::msg::PoseStamped m;
  m.header.frame_id = p.header.frame_id;
  m.pose = toRos(p.pose);
  return m;
}

inline Path toCore(const nav_msgs::msg::Path & m)
{
  Path p;
  p.header.frame_id = m.header.frame_id;
  p.poses.reserve(m.poses.size());
  for (const auto & pose : m.poses) {
    p.poses.push_back(toCore(pose));
  }
  return p;
}

/// The core carries no time stamps; the caller stamps the message with its own clock.
inline nav_msgs::msg::Path toRos(const Path & p)
{
  nav_msgs::msg::Path m;
  m.header.frame_id = p.header.frame_id;
  m.poses.reserve(p.poses.size());
  for (const auto & pose : p.poses) {
    m.poses.push_back(toRos(pose));
  }
  return m;
}

inline OccupancyGrid toCore(const nav_msgs::msg::OccupancyGrid & m)
{
  OccupancyGrid g;
  g.header.frame_id = m.header.frame_id;
  g.info.resolution = m.info.resolution;
  g.info.width = m.info.width;
  g.info.height = m.info.height;
  g.info.origin = toCore(m.info.origin);
  g.data.assign(m.data.begin(), m.data.end());
  return g;
}

inline nav_msgs::msg::OccupancyGrid toRos(const OccupancyGrid & g)
{
  nav_msgs::msg::OccupancyGrid m;
  m.header.frame_id = g.header.frame_id;
  m.info.resolution = static_cast<float>(g.info.resolution);
  m.info.width = g.info.width;
  m.info.height = g.info.height;
  m.info.origin = toRos(g.info.origin);
  m.data.assign(g.data.begin(), g.data.end());
  return m;
}

inline PolygonStamped toCore(const geometry_msgs::msg::PolygonStamped & m)
{
  PolygonStamped p;
  p.header.frame_id = m.header.frame_id;
  p.polygon.points.reserve(m.polygon.points.size());
  for (const auto & pt : m.polygon.points) {
    p.polygon.points.push_back(Point32{pt.x, pt.y, pt.z});
  }
  return p;
}

inline MotionPlanSegment toCore(const coverage_planner_msgs::msg::MotionPlanSegment & m)
{
  MotionPlanSegment s;
  s.label = m.label;
  s.start_index = m.start_index;
  s.end_index = m.end_index;
  s.render_start_index = m.render_start_index;
  s.render_end_index = m.render_end_index;
  s.frame_id = m.frame_id;
  s.mode = m.mode;
  s.planner = m.planner;
  s.requires_contact = m.requires_contact;
  s.requires_retraction = m.requires_retraction;
  s.priority = m.priority;
  s.tags = m.tags;
  return s;
}

inline coverage_planner_msgs::msg::MotionPlanSegment toRos(const MotionPlanSegment & s)
{
  coverage_planner_msgs::msg::MotionPlanSegment m;
  m.label = s.label;
  m.start_index = s.start_index;
  m.end_index = s.end_index;
  m.render_start_index = s.render_start_index;
  m.render_end_index = s.render_end_index;
  m.frame_id = s.frame_id;
  m.mode = s.mode;
  m.planner = s.planner;
  m.requires_contact = s.requires_contact;
  m.requires_retraction = s.requires_retraction;
  m.priority = s.priority;
  m.tags = s.tags;
  return m;
}

inline std::vector<coverage_planner_msgs::msg::MotionPlanSegment> toRos(
  const std::vector<MotionPlanSegment> & segments)
{
  std::vector<coverage_planner_msgs::msg::MotionPlanSegment> out;
  out.reserve(segments.size());
  for (const auto & s : segments) {
    out.push_back(toRos(s));
  }
  return out;
}

inline WallPlane toCore(const coverage_planner_msgs::msg::WallPlane & m)
{
  WallPlane w;
  w.id = m.id;
  w.pose = toCore(m.pose);
  w.u_min = m.u_min;
  w.u_max = m.u_max;
  w.v_min = m.v_min;
  w.v_max = m.v_max;
  w.plane_rms_mm = m.plane_rms_mm;
  w.spread_p95_mm = m.spread_p95_mm;
  w.spread_p99_mm = m.spread_p99_mm;
  return w;
}

inline coverage_planner_msgs::msg::WallPlane toRos(const WallPlane & w)
{
  coverage_planner_msgs::msg::WallPlane m;
  m.id = w.id;
  m.pose = toRos(w.pose);
  m.u_min = w.u_min;
  m.u_max = w.u_max;
  m.v_min = w.v_min;
  m.v_max = w.v_max;
  m.plane_rms_mm = w.plane_rms_mm;
  m.spread_p95_mm = w.spread_p95_mm;
  m.spread_p99_mm = w.spread_p99_mm;
  return m;
}

inline ObstacleRegion toCore(const coverage_planner_msgs::msg::ObstacleRegion & m)
{
  return ObstacleRegion{m.id, m.parent_wall, m.u_min, m.u_max, m.v_min, m.v_max};
}

inline coverage_planner_msgs::msg::ObstacleRegion toRos(const ObstacleRegion & o)
{
  coverage_planner_msgs::msg::ObstacleRegion m;
  m.id = o.id;
  m.parent_wall = o.parent_wall;
  m.u_min = o.u_min;
  m.u_max = o.u_max;
  m.v_min = o.v_min;
  m.v_max = o.v_max;
  return m;
}

}  // namespace ros_conv
}  // namespace coverage_planner
