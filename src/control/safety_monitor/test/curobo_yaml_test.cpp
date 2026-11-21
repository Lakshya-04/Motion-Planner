/**
 * @file curobo_yaml_test.cpp
 * @brief Loading a SphereModel from the cuRobo robot yml sphere format (tiny inline sample).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#include <gtest/gtest.h>

#include "safety_monitor/curobo_yaml.hpp"

using namespace safety_monitor;

namespace
{
const char * kYml = R"(
robot_cfg:
  kinematics:
    collision_link_names: [base_link, link1, link2, link3]
    collision_spheres:
      link3:
        - {center: [0.0, 0.0, 0.1], radius: 0.05}
      base_link:
        - {center: [0.0, 0.0, 0.0], radius: 0.10}
      link1:
        - {center: [0.0, 0.0, 0.2], radius: 0.06}
        - {center: [0.0, 0.0, 0.4], radius: 0.06}
      link2:
        - {center: [0.1, 0.0, 0.0], radius: 0.04}
    self_collision_ignore:
      base_link: [link1]
      link1: [link2]
)";
}  // namespace

TEST(CuroboYaml, LoadsLinksSpheresAndOrder)
{
  const SphereModel m = loadSphereModelFromCuroboYaml(kYml);
  ASSERT_EQ(m.linkCount(), 4);
  EXPECT_EQ(m.linkName(0), "base_link");  // collision_link_names order
  EXPECT_EQ(m.linkName(3), "link3");
  EXPECT_EQ(m.sphereCount(), 5);
  std::vector<Eigen::Isometry3d> T(4, Eigen::Isometry3d::Identity());
  std::vector<WorldSphere> ws;
  ASSERT_TRUE(m.worldSpheres(T, ws));
  EXPECT_DOUBLE_EQ(ws[2].center.z(), 0.4);
  EXPECT_DOUBLE_EQ(ws[1].radius, 0.06);
}

TEST(CuroboYaml, IgnoreTableBecomesAllowedPairs)
{
  const SphereModel m = loadSphereModelFromCuroboYaml(kYml);
  EXPECT_FALSE(m.pairChecked(m.linkIndex("base_link"), m.linkIndex("link1")));
  EXPECT_FALSE(m.pairChecked(m.linkIndex("link1"), m.linkIndex("link2")));
  EXPECT_TRUE(m.pairChecked(m.linkIndex("base_link"), m.linkIndex("link2")));
  EXPECT_TRUE(m.pairChecked(m.linkIndex("link2"), m.linkIndex("link3")));
}

TEST(CuroboYaml, ParentsExcludeAdjacentPairsAndSkipMissingLinks)
{
  // link2 and link3 are not adjacent in the table, but link3's parent chain goes through a link
  // without spheres ("wrist") to link2.
  const SphereModel m = loadSphereModelFromCuroboYaml(
    kYml, {{"link3", "wrist"}, {"wrist", "link2"}, {"link2", "link1"}, {"link1", "base_link"}});
  EXPECT_FALSE(m.pairChecked(m.linkIndex("link2"), m.linkIndex("link3")));
  EXPECT_TRUE(m.pairChecked(m.linkIndex("link1"), m.linkIndex("link3")));
}

TEST(CuroboYaml, ParentListedAfterChildStillWorks)
{
  const SphereModel m = loadSphereModelFromCuroboYaml(
    kYml, {{"base_link", "link3"}, {"link3", "link2"}});
  EXPECT_FALSE(m.pairChecked(m.linkIndex("base_link"), m.linkIndex("link3")));
}

TEST(CuroboYaml, MalformedInputThrows)
{
  EXPECT_THROW(loadSphereModelFromCuroboYaml("foo: 1"), std::runtime_error);
  EXPECT_THROW(
    loadSphereModelFromCuroboYaml("collision_spheres:\n  a: [{center: [0, 0], radius: 1}]"),
    std::runtime_error);
  EXPECT_THROW(
    loadSphereModelFromCuroboYaml(
      "collision_spheres:\n  a: [{center: [0,0,0], radius: 1}]\n  b: [{center: [0,0,0], radius: 1}]",
      {{"a", "b"}, {"b", "a"}}),
    std::runtime_error);
}
