#include <gtest/gtest.h>

#include "scene_model/core/export.hpp"
#include "scene_model/core/io.hpp"

using namespace scene_model;  // NOLINT

TEST(Export, CuroboYamlRoundTripsThroughTheLoader)
{
  SceneModel s;
  SceneObject table;
  table.id = "table";
  table.geometry = Box{Vec3(1.0, 0.6, 0.05)};
  table.pose.translation() = Vec3(0, 0.5, 0.4);
  table.pose.rotate(Eigen::AngleAxisd(0.7, Vec3::UnitZ()));
  s.add(table);
  SceneObject ball;
  ball.id = "ball";
  ball.geometry = Sphere{0.15};
  ball.pose.translation() = Vec3(1, 2, 3);
  s.add(ball);
  SceneObject cyl;
  cyl.id = "post";
  cyl.geometry = Cylinder{0.05, 1.2};
  s.add(cyl);

  const SceneModel back = loadSceneYaml(toCuroboWorldYaml(s));
  ASSERT_EQ(back.size(), 3U);
  const auto & t2 = *back.find("table");
  EXPECT_NEAR((std::get<Box>(t2.geometry).size - Vec3(1.0, 0.6, 0.05)).norm(), 0.0, 1e-9);
  EXPECT_NEAR((t2.pose.translation() - table.pose.translation()).norm(), 0.0, 1e-9);
  EXPECT_NEAR((t2.pose.linear() - table.pose.linear()).norm(), 0.0, 1e-9);
  EXPECT_NEAR(std::get<Sphere>(back.find("ball")->geometry).radius, 0.15, 1e-12);
  EXPECT_NEAR(std::get<Cylinder>(back.find("post")->geometry).height, 1.2, 1e-12);
}

TEST(Export, InlineMeshIsEmittedAndUnsupportedKindsAreReported)
{
  SceneModel s;
  SceneObject tri;
  tri.id = "tri";
  Mesh m;
  m.vertices = {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0)};
  m.faces = {{0, 1, 2}};
  tri.geometry = m;
  s.add(tri);
  SceneObject cloud;
  cloud.id = "scan";
  cloud.geometry = PointCloud{{Vec3(0, 0, 0)}, {}, {}};
  s.add(cloud);

  std::vector<std::string> skipped;
  const std::string text = toCuroboWorldYaml(s, &skipped);
  EXPECT_NE(text.find("vertices"), std::string::npos);
  EXPECT_NE(text.find("faces"), std::string::npos);
  ASSERT_EQ(skipped.size(), 1U);
  EXPECT_EQ(skipped[0], "scan (pointcloud)");
}

TEST(Export, EmptySceneIsValidYaml)
{
  const SceneModel back = loadSceneYaml(toCuroboWorldYaml(SceneModel()));
  EXPECT_TRUE(back.empty());
}
