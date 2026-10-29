#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "scene_model/ros/conversions.hpp"

using namespace scene_model;  // NOLINT

TEST(Conversions, PoseRoundTrip)
{
  Transform T = Transform::Identity();
  T.translation() = Vec3(1, -2, 3);
  T.linear() = Eigen::AngleAxisd(0.7, Vec3(1, 2, 3).normalized()).toRotationMatrix();
  const Transform back = ros_conv::fromRos(ros_conv::toRos(T));
  EXPECT_NEAR((back.translation() - T.translation()).norm(), 0.0, 1e-12);
  EXPECT_NEAR((back.linear() - T.linear()).norm(), 0.0, 1e-12);
}

TEST(Conversions, AnAllZeroQuaternionMeansIdentity)
{
  geometry_msgs::msg::Pose p;  // orientation 0 0 0 0
  p.orientation.w = 0.0;
  EXPECT_NEAR((ros_conv::fromRos(p).linear() - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-12);
}

TEST(Conversions, PointCloudRoundTripKeepsColorsAndLabels)
{
  PointCloud c;
  c.points = {Vec3(1, 2, 3), Vec3(-1, 0.5, 4), Vec3(0, 0, 0)};
  c.colors = {{255, 0, 0}, {0, 128, 255}, {7, 8, 9}};
  c.labels = {1, 42, 7};
  const auto msg = ros_conv::toRos(c, "map");
  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_EQ(msg.width, 3U);
  EXPECT_EQ(msg.height, 1U);
  const PointCloud back = ros_conv::fromRos(msg);
  ASSERT_EQ(back.points.size(), 3U);
  EXPECT_NEAR(back.points[1].y(), 0.5, 1e-6);
  EXPECT_EQ(back.colors, c.colors);
  EXPECT_EQ(back.labels, c.labels);
}

TEST(Conversions, PlainXyzCloudHasNoColorOrLabel)
{
  PointCloud c;
  c.points = {Vec3(1, 2, 3)};
  const auto msg = ros_conv::toRos(c, "base");
  EXPECT_EQ(msg.fields.size(), 3U);
  const PointCloud back = ros_conv::fromRos(msg);
  EXPECT_TRUE(back.colors.empty());
  EXPECT_TRUE(back.labels.empty());
}

TEST(Conversions, NonFinitePointsAreDroppedTogetherWithTheirFields)
{
  PointCloud c;
  c.points = {Vec3(1, 0, 0), Vec3(0, 0, 0), Vec3(3, 0, 0)};
  c.labels = {10, 20, 30};
  auto msg = ros_conv::toRos(c, "m");
  const float nan = std::nanf("");
  std::memcpy(msg.data.data() + 1 * msg.point_step, &nan, 4);  // x of point 1
  const PointCloud back = ros_conv::fromRos(msg);
  ASSERT_EQ(back.points.size(), 2U);
  EXPECT_EQ(back.labels, (std::vector<uint32_t>{10, 30}));
}

TEST(Conversions, SourceIndexMapsKeptPointsBackToTheMessage)
{
  PointCloud c;
  c.points = {Vec3(1, 0, 0), Vec3(0, 0, 0), Vec3(3, 0, 0), Vec3(4, 0, 0)};
  auto msg = ros_conv::toRos(c, "m");
  const float nan = std::nanf("");
  std::memcpy(msg.data.data() + 1 * msg.point_step, &nan, 4);
  std::vector<size_t> index;
  const PointCloud back = ros_conv::fromRos(msg, index);
  EXPECT_EQ(index, (std::vector<size_t>{0, 2, 3}));
  EXPECT_EQ(back.points.size(), index.size());
}

TEST(Conversions, BadCloudsFailWithAMessage)
{
  sensor_msgs::msg::PointCloud2 empty;
  EXPECT_THROW(ros_conv::fromRos(empty), std::runtime_error);  // no fields

  PointCloud c;
  c.points = {Vec3(1, 2, 3)};
  auto wrong_type = ros_conv::toRos(c, "m");
  wrong_type.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT64;
  EXPECT_THROW(ros_conv::fromRos(wrong_type), std::runtime_error);

  auto short_data = ros_conv::toRos(c, "m");
  short_data.data.resize(4);
  EXPECT_THROW(ros_conv::fromRos(short_data), std::runtime_error);
}

TEST(Conversions, KeepPointsPreservesEveryFieldAndFlattensOrganizedClouds)
{
  // 2 x 2 organized cloud with x y z and an extra intensity field.
  sensor_msgs::msg::PointCloud2 m;
  m.header.frame_id = "lidar";
  m.height = 2;
  m.width = 2;
  m.point_step = 16;
  m.row_step = 32;
  auto field = [&](const char * name, uint32_t offset) {
      sensor_msgs::msg::PointField f;
      f.name = name;
      f.offset = offset;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count = 1;
      m.fields.push_back(f);
    };
  field("x", 0); field("y", 4); field("z", 8); field("intensity", 12);
  m.data.resize(64);
  for (int i = 0; i < 4; ++i) {
    const float row[4] = {float(i), 0.f, 0.f, 100.f + float(i)};
    std::memcpy(m.data.data() + i * 16, row, 16);
  }
  const auto kept = ros_conv::keepPoints(m, {false, true, false, true});
  EXPECT_EQ(kept.width, 2U);
  EXPECT_EQ(kept.height, 1U);
  EXPECT_EQ(kept.fields.size(), 4U);
  EXPECT_EQ(kept.header.frame_id, "lidar");
  float intensity;
  std::memcpy(&intensity, kept.data.data() + 12, 4);
  EXPECT_FLOAT_EQ(intensity, 100.f);       // point 0
  std::memcpy(&intensity, kept.data.data() + 16 + 12, 4);
  EXPECT_FLOAT_EQ(intensity, 102.f);       // point 2
  EXPECT_THROW(ros_conv::keepPoints(m, {true}), std::runtime_error);
}

namespace
{
SceneObject object(const std::string & id, Geometry g, Vec3 at)
{
  SceneObject o;
  o.id = id;
  o.geometry = std::move(g);
  o.pose.translation() = at;
  return o;
}
}  // namespace

TEST(Conversions, PrimitivesRoundTripThroughCollisionObjects)
{
  SceneModel s("world");
  s.add(object("box", Box{Vec3(1, 2, 3)}, Vec3(0, 0, 1)));
  s.add(object("ball", Sphere{0.25}, Vec3(1, 0, 0)));
  s.add(object("post", Cylinder{0.1, 1.5}, Vec3(2, 0, 0)));
  std::vector<std::string> skipped;
  const auto objects = ros_conv::toRos(s, &skipped);
  ASSERT_EQ(objects.size(), 3U);
  EXPECT_TRUE(skipped.empty());
  EXPECT_EQ(objects[0].header.frame_id, "world");
  EXPECT_EQ(objects[2].primitives[0].dimensions[0], 1.5);  // height first
  EXPECT_EQ(objects[2].primitives[0].dimensions[1], 0.1);  // then radius

  const auto back = ros_conv::fromRos(objects[2]);
  ASSERT_EQ(back.size(), 1U);
  EXPECT_EQ(back[0].id, "post");
  const auto & cyl = std::get<Cylinder>(back[0].geometry);
  EXPECT_DOUBLE_EQ(cyl.radius, 0.1);
  EXPECT_DOUBLE_EQ(cyl.height, 1.5);
  EXPECT_NEAR(back[0].pose.translation().x(), 2.0, 1e-12);
}

TEST(Conversions, MeshesAndCapsulesAndSkippedKinds)
{
  SceneModel s("w");
  Mesh m;
  m.vertices = {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0)};
  m.faces = {{0, 1, 2}};
  s.add(object("tri", m, Vec3(0, 0, 0)));
  s.add(object("pill", Capsule{0.1, 0.4}, Vec3(0, 1, 0)));
  s.add(object("scan", PointCloud{{Vec3(0, 0, 0)}, {}, {}}, Vec3(0, 0, 0)));
  VoxelGrid g;
  g.voxel_size = 0.1;
  g.nx = g.ny = g.nz = 1;
  g.occupied = {1};
  s.add(object("grid", g, Vec3(0, 0, 0)));
  std::vector<std::string> skipped;
  const auto objects = ros_conv::toRos(s, &skipped);
  ASSERT_EQ(objects.size(), 2U);
  EXPECT_EQ(objects[0].meshes.size(), 1U);
  EXPECT_EQ(objects[1].primitives[0].type, shape_msgs::msg::SolidPrimitive::CYLINDER);
  EXPECT_NEAR(objects[1].primitives[0].dimensions[0], 0.6, 1e-12);  // full extent incl. caps
  ASSERT_EQ(skipped.size(), 2U);
  EXPECT_EQ(skipped[0], "scan (pointcloud)");

  const auto back = ros_conv::fromRos(objects[0]);
  ASSERT_EQ(back.size(), 1U);
  EXPECT_EQ(std::get<Mesh>(back[0].geometry).faces[0], (std::array<int, 3>{0, 1, 2}));
}

TEST(Conversions, ObjectPoseIsComposedWithEachShapePose)
{
  moveit_msgs::msg::CollisionObject co;
  co.id = "table";
  co.header.frame_id = "map";
  co.pose.position.x = 10.0;
  co.pose.orientation.w = 1.0;
  shape_msgs::msg::SolidPrimitive p;
  p.type = shape_msgs::msg::SolidPrimitive::BOX;
  p.dimensions = {1, 1, 1};
  co.primitives = {p, p};
  geometry_msgs::msg::Pose a, b;
  a.position.y = 1.0; a.orientation.w = 1.0;
  b.position.y = -1.0; b.orientation.w = 1.0;
  co.primitive_poses = {a, b};
  const auto objs = ros_conv::fromRos(co);
  ASSERT_EQ(objs.size(), 2U);
  EXPECT_EQ(objs[0].id, "table#0");
  EXPECT_EQ(objs[1].id, "table#1");
  EXPECT_NEAR(objs[0].pose.translation().x(), 10.0, 1e-12);
  EXPECT_NEAR(objs[0].pose.translation().y(), 1.0, 1e-12);
  EXPECT_NEAR(objs[1].pose.translation().y(), -1.0, 1e-12);
  EXPECT_EQ(objs[0].frame_id, "map");
}

TEST(Conversions, PlanningSceneWorldBecomesASceneModel)
{
  moveit_msgs::msg::PlanningScene ps;
  moveit_msgs::msg::CollisionObject co;
  co.id = "wall";
  co.header.frame_id = "map";
  co.pose.orientation.w = 1.0;
  shape_msgs::msg::SolidPrimitive p;
  p.type = shape_msgs::msg::SolidPrimitive::BOX;
  p.dimensions = {4, 0.1, 3};
  co.primitives = {p};
  ps.world.collision_objects = {co};
  const SceneModel s = ros_conv::fromRos(ps);
  EXPECT_EQ(s.frameId(), "map");
  ASSERT_EQ(s.size(), 1U);
  EXPECT_NE(s.find("wall"), nullptr);
}
