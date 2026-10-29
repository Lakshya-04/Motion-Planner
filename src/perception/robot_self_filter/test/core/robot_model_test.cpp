#include <gtest/gtest.h>

#include "robot_self_filter/core/robot_model.hpp"

using namespace robot_self_filter;  // NOLINT

namespace
{
const char * kRobot = R"(<?xml version="1.0"?>
<robot name="r">
  <link name="base"><collision><geometry><box size="0.4 0.2 0.1"/></geometry></collision></link>
  <link name="lift"><collision><origin xyz="0 0 0.05" rpy="0 0 0"/><geometry><cylinder radius="0.05" length="0.1"/></geometry></collision></link>
  <link name="stage"><collision><geometry><sphere radius="0.03"/></geometry></collision></link>
  <link name="arm"><collision><geometry><box size="0.1 0.1 0.6"/></geometry></collision></link>
  <link name="tool"/>
  <joint name="lift_joint" type="prismatic">
    <parent link="base"/><child link="lift"/><origin xyz="0 0 0.05"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="stage_joint" type="prismatic">
    <parent link="base"/><child link="stage"/><origin xyz="0.1 0 0.05"/><axis xyz="0 0 1"/>
    <mimic joint="lift_joint" multiplier="0.5" offset="0.01"/>
  </joint>
  <joint name="shoulder" type="revolute">
    <parent link="lift"/><child link="arm"/><origin xyz="0 0 0.1"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="tool_joint" type="fixed">
    <parent link="arm"/><child link="tool"/><origin xyz="0 0 0.3"/>
  </joint>
</robot>)";
}  // namespace

TEST(UrdfRobot, ParsesShapesRootAndJoints)
{
  const auto r = UrdfRobot::fromUrdf(kRobot);
  EXPECT_EQ(r.rootLink(), "base");
  ASSERT_EQ(r.shapes().size(), 5U);  // every link has an entry, `tool` an empty one
  const auto & base = r.shapes()[0];
  ASSERT_EQ(base.shapes.size(), 1U);
  EXPECT_EQ(base.shapes[0].kind, Shape::Kind::kBox);
  EXPECT_NEAR(base.shapes[0].size.x(), 0.4, 1e-12);
  EXPECT_EQ(r.shapes()[1].shapes[0].kind, Shape::Kind::kCylinder);
  EXPECT_EQ(r.shapes()[2].shapes[0].kind, Shape::Kind::kSphere);
  EXPECT_EQ(r.jointNames().size(), 3U);  // fixed joints are not listed
  EXPECT_TRUE(r.hasLink("tool"));
  EXPECT_FALSE(r.hasLink("nope"));
}

TEST(UrdfRobot, ForwardKinematicsFollowsPrismaticRevoluteFixedAndMimic)
{
  const auto r = UrdfRobot::fromUrdf(kRobot);
  const auto zero = r.linkPoses({});
  EXPECT_NEAR(zero.at("lift").translation().z(), 0.05, 1e-12);
  EXPECT_NEAR(zero.at("tool").translation().z(), 0.05 + 0.1 + 0.3, 1e-12);

  const auto q = r.linkPoses({{"lift_joint", 0.4}});
  EXPECT_NEAR(q.at("lift").translation().z(), 0.45, 1e-12);
  EXPECT_NEAR(q.at("stage").translation().z(), 0.05 + 0.5 * 0.4 + 0.01, 1e-12);  // mimic
  EXPECT_NEAR(q.at("tool").translation().z(), 0.45 + 0.1 + 0.3, 1e-12);

  // Shoulder 90 degrees about +y turns the arm's +z (and the tool offset 0.3) toward +x.
  const auto bent = r.linkPoses({{"shoulder", M_PI / 2}});
  EXPECT_NEAR(bent.at("tool").translation().x(), 0.3, 1e-12);
  EXPECT_NEAR(bent.at("tool").translation().z(), 0.05 + 0.1, 1e-12);
}

TEST(UrdfRobot, MeshesBecomeBoundingBoxesOnlyWhenResolved)
{
  const std::string urdf = R"(<robot name="m"><link name="a"><collision><origin xyz="0 0 1"/><geometry>
    <mesh filename="package://x/mesh.stl" scale="2 2 2"/></geometry></collision></link></robot>)";
  const auto skipped = UrdfRobot::fromUrdf(urdf);
  EXPECT_TRUE(skipped.shapes()[0].shapes.empty());
  ASSERT_EQ(skipped.skippedMeshes().size(), 1U);
  EXPECT_NE(skipped.skippedMeshes()[0].find("package://x/mesh.stl"), std::string::npos);

  const auto resolved = UrdfRobot::fromUrdf(urdf, [](const std::string &) {
    return std::optional<MeshBounds>(MeshBounds{Vec3(-0.1, -0.2, 0.0), Vec3(0.1, 0.2, 0.5)});
  });
  ASSERT_EQ(resolved.shapes()[0].shapes.size(), 1U);
  const Shape & s = resolved.shapes()[0].shapes[0];
  EXPECT_EQ(s.kind, Shape::Kind::kBox);
  EXPECT_NEAR(s.size.x(), 0.4, 1e-12);                      // scaled by 2
  EXPECT_NEAR(s.size.z(), 1.0, 1e-12);
  EXPECT_NEAR(s.local.translation().z(), 1.0 + 0.5, 1e-12);  // origin + scaled centre
  EXPECT_TRUE(resolved.skippedMeshes().empty());
}

TEST(UrdfRobot, MalformedInputFailsWithAReason)
{
  auto message = [](const std::string & xml) {
      try {UrdfRobot::fromUrdf(xml);} catch (const std::exception & e) {return std::string(e.what());}
      return std::string("no error");
    };
  EXPECT_NE(message("<robot><link").find("cannot parse"), std::string::npos);
  EXPECT_NE(message("<notrobot/>").find("no <robot>"), std::string::npos);
  EXPECT_NE(message("<robot><link name='a'/><link name='b'/></robot>").find("more than one root"), std::string::npos);
  EXPECT_NE(message("<robot><link name='a'/><joint name='j' type='floating'><parent link='a'/><child link='a'/></joint></robot>")
    .find("unsupported type"), std::string::npos);
  EXPECT_NE(message("<robot><link name='a'/><joint name='j' type='fixed'><parent link='a'/><child link='zz'/></joint></robot>")
    .find("missing link"), std::string::npos);
  EXPECT_NE(message("<robot><link name='a'/><link name='b'/><joint name='j' type='revolute'><parent link='a'/><child link='b'/>"
    "<axis xyz='0 0 0'/></joint></robot>").find("zero axis"), std::string::npos);
  EXPECT_NE(message("<robot><link name='a'><collision><geometry><box size='1 2'/></geometry></collision></link></robot>")
    .find("box size"), std::string::npos);
}
