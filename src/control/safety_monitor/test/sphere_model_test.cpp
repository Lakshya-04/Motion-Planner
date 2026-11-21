/**
 * @file sphere_model_test.cpp
 * @brief SphereModel: world spheres, attach/detach, the self-pair table, twists.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#include <gtest/gtest.h>

#include <algorithm>

#include "safety_monitor/sphere_model.hpp"

using namespace safety_monitor;

namespace
{
// base(0) - link1(1) - link2(2) - link3(3), plus a tool link (4) on link3.
SphereModel chain()
{
  SphereModel m;
  const int b = m.addLink("base", -1, {{Vec3(0, 0, 0.1), 0.1}});
  const int l1 = m.addLink("l1", b, {{Vec3(0, 0, 0), 0.05}});
  const int l2 = m.addLink("l2", l1, {{Vec3(0, 0, 0), 0.05}, {Vec3(0.1, 0, 0), 0.04}});
  const int l3 = m.addLink("l3", l2, {{Vec3(0, 0, 0), 0.05}});
  m.addLink("tool", l3, {{Vec3(0, 0, 0.05), 0.03}});
  return m;
}
}  // namespace

TEST(SphereModel, WorldSpheresApplyTransforms)
{
  SphereModel m = chain();
  std::vector<Eigen::Isometry3d> T(m.linkCount(), Eigen::Isometry3d::Identity());
  T[2] = Eigen::Translation3d(1, 2, 3) * Eigen::AngleAxisd(M_PI / 2, Vec3::UnitZ());
  std::vector<WorldSphere> ws;
  ASSERT_TRUE(m.worldSpheres(T, ws));
  ASSERT_EQ(static_cast<int>(ws.size()), m.sphereCount());
  EXPECT_EQ(m.sphereCount(), 6);
  // Second sphere of l2 at link x = 0.1 -> world (1, 2.1, 3) after a 90 degree yaw.
  const auto it = std::find_if(ws.begin(), ws.end(), [](const WorldSphere & s) {
      return s.link == 2 && s.index_in_link == 1;});
  ASSERT_NE(it, ws.end());
  EXPECT_NEAR(it->center.x(), 1.0, 1e-12);
  EXPECT_NEAR(it->center.y(), 2.1, 1e-12);
  EXPECT_NEAR(it->center.z(), 3.0, 1e-12);
  EXPECT_DOUBLE_EQ(it->radius, 0.04);
}

TEST(SphereModel, SizeMismatchFails)
{
  SphereModel m = chain();
  std::vector<WorldSphere> ws;
  EXPECT_FALSE(m.worldSpheres({Eigen::Isometry3d::Identity()}, ws));
  EXPECT_TRUE(ws.empty());
}

TEST(SphereModel, SelfPairTableExcludesParentChildAndSelf)
{
  SphereModel m = chain();
  EXPECT_FALSE(m.pairChecked(1, 1));
  EXPECT_FALSE(m.pairChecked(0, 1));  // parent-child
  EXPECT_FALSE(m.pairChecked(2, 1));  // child-parent, either order
  EXPECT_TRUE(m.pairChecked(0, 2));
  EXPECT_TRUE(m.pairChecked(1, 3));
  EXPECT_TRUE(m.pairChecked(0, 4));
  // 5 links: 10 pairs, 4 parent-child -> 6 checked
  EXPECT_EQ(m.selfPairs().size(), 6u);
}

TEST(SphereModel, AllowedPairsAreExcluded)
{
  SphereModel m = chain();
  m.allowPair("l3", "base");
  m.allowPair("l1", "l3");
  EXPECT_FALSE(m.pairChecked(3, 0));
  EXPECT_FALSE(m.pairChecked(1, 3));
  EXPECT_TRUE(m.pairChecked(0, 2));
  EXPECT_EQ(m.selfPairs().size(), 4u);
  EXPECT_THROW(m.allowPair("l3", "nope"), std::invalid_argument);
}

TEST(SphereModel, AttachAndDetachToolSpheres)
{
  SphereModel m = chain();
  const int n0 = m.sphereCount();
  m.attachSphere("tool", "payload", {Vec3(0, 0, 0.2), 0.08});
  m.attachSphere("tool", "payload", {Vec3(0, 0, 0.3), 0.08});
  m.attachSphere("l3", "clamp", {Vec3(0, 0.1, 0), 0.02});
  EXPECT_EQ(m.sphereCount(), n0 + 3);
  std::vector<Eigen::Isometry3d> T(m.linkCount(), Eigen::Isometry3d::Identity());
  std::vector<WorldSphere> ws;
  m.worldSpheres(T, ws);
  EXPECT_EQ(static_cast<int>(ws.size()), n0 + 3);
  EXPECT_EQ(m.detachSpheres("payload"), 2);
  EXPECT_EQ(m.sphereCount(), n0 + 1);
  EXPECT_EQ(m.detachSpheres("payload"), 0);
}

TEST(SphereModel, DuplicateAndBadParentThrow)
{
  SphereModel m = chain();
  EXPECT_THROW(m.addLink("l1", -1, {}), std::invalid_argument);
  EXPECT_THROW(m.addLink("x", 99, {}), std::invalid_argument);
}

TEST(SphereModel, ContactExemptFlag)
{
  SphereModel m = chain();
  EXPECT_FALSE(m.isContactExempt(4));
  m.setContactExempt("tool");
  EXPECT_TRUE(m.isContactExempt(4));
  EXPECT_FALSE(m.isContactExempt(3));
}

TEST(SphereModel, SphereVelocitiesFromTwists)
{
  SphereModel m;
  m.addLink("a", -1, {{Vec3(1, 0, 0), 0.1}});
  std::vector<Eigen::Isometry3d> T{Eigen::Isometry3d::Identity()};
  std::vector<LinkTwist> tw(1);
  tw[0].linear = Vec3(0, 0, 1);
  tw[0].angular = Vec3(0, 0, 2);  // v = (0,0,1) + (0,0,2) x (1,0,0) = (0, 2, 1)
  std::vector<Vec3> v;
  ASSERT_TRUE(sphereVelocities(m, T, tw, v));
  ASSERT_EQ(v.size(), 1u);
  EXPECT_NEAR((v[0] - Vec3(0, 2, 1)).norm(), 0.0, 1e-12);
  EXPECT_FALSE(sphereVelocities(m, T, {}, v));
}
