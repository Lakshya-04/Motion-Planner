#include <gtest/gtest.h>

#include "coverage_planner/ros/conversions.hpp"

using namespace coverage_planner;  // NOLINT

TEST(Conversions, PoseStampedRoundTrip)
{
  PoseStamped in;
  in.header.frame_id = "map";
  in.pose.position = {1.0, -2.0, 3.5};
  in.pose.orientation = {0.1, 0.2, 0.3, 0.9};

  const PoseStamped out = ros_conv::toCore(ros_conv::toRos(in));
  EXPECT_EQ(out.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(out.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(out.pose.position.y, -2.0);
  EXPECT_DOUBLE_EQ(out.pose.position.z, 3.5);
  EXPECT_DOUBLE_EQ(out.pose.orientation.w, 0.9);
}

TEST(Conversions, PathKeepsOrderAndFrame)
{
  Path in;
  in.header.frame_id = "wall";
  for (int i = 0; i < 5; ++i) {
    PoseStamped p;
    p.header.frame_id = "wall";
    p.pose.position.x = i;
    in.poses.push_back(p);
  }
  const Path out = ros_conv::toCore(ros_conv::toRos(in));
  ASSERT_EQ(out.poses.size(), 5U);
  EXPECT_EQ(out.header.frame_id, "wall");
  for (int i = 0; i < 5; ++i) {
    EXPECT_DOUBLE_EQ(out.poses[i].pose.position.x, i);
  }
}

TEST(Conversions, OccupancyGridRoundTrip)
{
  OccupancyGrid in;
  in.header.frame_id = "map";
  in.info.resolution = 0.25;  // exactly representable as float
  in.info.width = 3;
  in.info.height = 2;
  in.info.origin.position = {1.0, 2.0, 0.0};
  in.data = {0, 100, -1, 0, 0, 100};

  const OccupancyGrid out = ros_conv::toCore(ros_conv::toRos(in));
  EXPECT_EQ(out.info.width, 3U);
  EXPECT_EQ(out.info.height, 2U);
  EXPECT_DOUBLE_EQ(out.info.resolution, 0.25);
  EXPECT_DOUBLE_EQ(out.info.origin.position.y, 2.0);
  EXPECT_EQ(out.data, in.data);
}

TEST(Conversions, MotionPlanSegmentRoundTrip)
{
  MotionPlanSegment in;
  in.label = "strip_3";
  in.start_index = 4;
  in.end_index = 9;
  in.render_start_index = 5;
  in.render_end_index = 8;
  in.frame_id = "wall";
  in.mode = "cartesian";
  in.planner = "ompl";
  in.requires_contact = true;
  in.requires_retraction = true;
  in.priority = 2.0F;
  in.tags = {"strip", "obstacle_split"};

  const MotionPlanSegment out = ros_conv::toCore(ros_conv::toRos(in));
  EXPECT_EQ(out.label, in.label);
  EXPECT_EQ(out.start_index, 4U);
  EXPECT_EQ(out.end_index, 9U);
  EXPECT_EQ(out.render_start_index, 5U);
  EXPECT_EQ(out.mode, "cartesian");
  EXPECT_TRUE(out.requires_contact);
  EXPECT_TRUE(out.requires_retraction);
  EXPECT_FLOAT_EQ(out.priority, 2.0F);
  EXPECT_EQ(out.tags, in.tags);
}

TEST(Conversions, WallPlaneAndObstacleRoundTrip)
{
  WallPlane wall;
  wall.id = 7;
  wall.pose.header.frame_id = "map";
  wall.pose.pose.orientation = {0.0, 0.0, 0.0, 1.0};
  wall.u_min = -1.0;
  wall.u_max = 2.0;
  wall.v_min = 0.5;
  wall.v_max = 3.0;
  wall.plane_rms_mm = 4.2;
  const WallPlane w2 = ros_conv::toCore(ros_conv::toRos(wall));
  EXPECT_EQ(w2.id, 7U);
  EXPECT_DOUBLE_EQ(w2.u_max, 2.0);
  EXPECT_DOUBLE_EQ(w2.plane_rms_mm, 4.2);
  EXPECT_EQ(w2.pose.header.frame_id, "map");

  const ObstacleRegion o{3, 7, 0.1, 0.2, 0.3, 0.4};
  const ObstacleRegion o2 = ros_conv::toCore(ros_conv::toRos(o));
  EXPECT_EQ(o2.parent_wall, 7U);
  EXPECT_DOUBLE_EQ(o2.v_max, 0.4);
}

TEST(Conversions, PolygonToCore)
{
  geometry_msgs::msg::PolygonStamped m;
  m.header.frame_id = "map";
  geometry_msgs::msg::Point32 p;
  p.x = 1.0F;
  p.y = 2.0F;
  m.polygon.points.push_back(p);
  const PolygonStamped out = ros_conv::toCore(m);
  ASSERT_EQ(out.polygon.points.size(), 1U);
  EXPECT_FLOAT_EQ(out.polygon.points[0].y, 2.0F);
  EXPECT_EQ(out.header.frame_id, "map");
}
