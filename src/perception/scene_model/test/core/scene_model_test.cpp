#include <gtest/gtest.h>

#include "scene_model/core/scene_model.hpp"

using namespace scene_model;  // NOLINT

namespace
{
SceneObject box(const std::string & id, Vec3 size, Vec3 at, Role role = Role::kObstacle)
{
  SceneObject o;
  o.id = id;
  o.geometry = Box{size};
  o.pose.translation() = at;
  o.role = role;
  return o;
}
}  // namespace

TEST(SceneModel, AddRejectsDuplicateAndEmptyIds)
{
  SceneModel s("map");
  EXPECT_TRUE(s.add(box("a", {1, 1, 1}, {0, 0, 0})));
  EXPECT_FALSE(s.add(box("a", {2, 2, 2}, {0, 0, 0})));
  EXPECT_FALSE(s.add(box("", {1, 1, 1}, {0, 0, 0})));
  EXPECT_EQ(s.size(), 1U);
}

TEST(SceneModel, UpsertReplacesInPlaceKeepingOrder)
{
  SceneModel s;
  s.add(box("a", {1, 1, 1}, {0, 0, 0}));
  s.add(box("b", {1, 1, 1}, {1, 0, 0}));
  s.upsert(box("a", {3, 3, 3}, {0, 0, 0}));
  ASSERT_EQ(s.size(), 2U);
  EXPECT_EQ(s.objects()[0].id, "a");
  EXPECT_DOUBLE_EQ(std::get<Box>(s.find("a")->geometry).size.x(), 3.0);
}

TEST(SceneModel, RolesStayDistinct)
{
  SceneModel s;
  s.add(box("wall", {4, 0.1, 3}, {0, 0, 0}, Role::kContactSurface));
  s.add(box("table", {1, 1, 1}, {0, 1, 0}));
  EXPECT_EQ(s.withRole(Role::kContactSurface).size(), 1U);
  EXPECT_EQ(s.withRole(Role::kObstacle).size(), 1U);
  EXPECT_EQ(s.withRole(Role::kContactSurface)[0]->id, "wall");
}

TEST(SceneModel, RemoveDynamicKeepsStatic)
{
  SceneModel s;
  s.add(box("fixed", {1, 1, 1}, {0, 0, 0}));
  auto moving = box("person", {0.5, 0.5, 1.8}, {2, 0, 0});
  moving.is_static = false;
  s.add(moving);
  s.removeDynamic();
  EXPECT_EQ(s.size(), 1U);
  EXPECT_NE(s.find("fixed"), nullptr);
}

TEST(SceneModel, BoundsApplyPoseIncludingRotation)
{
  SceneModel s;
  auto o = box("b", {2, 1, 1}, {10, 0, 0});
  o.pose.rotate(Eigen::AngleAxisd(M_PI / 2, Vec3::UnitZ()));  // swaps x and y extents
  s.add(o);
  const Aabb b = s.bounds();
  EXPECT_NEAR(b.min.x(), 9.5, 1e-9);
  EXPECT_NEAR(b.max.x(), 10.5, 1e-9);
  EXPECT_NEAR(b.min.y(), -1.0, 1e-9);
  EXPECT_NEAR(b.max.y(), 1.0, 1e-9);
}

TEST(SceneModel, EmptySceneHasEmptyBounds)
{
  EXPECT_TRUE(SceneModel().bounds().empty());
}

TEST(SceneModel, MergeRenamesClashingIds)
{
  SceneModel a, b;
  a.add(box("x", {1, 1, 1}, {0, 0, 0}));
  b.add(box("x", {1, 1, 1}, {5, 0, 0}));
  b.add(box("y", {1, 1, 1}, {6, 0, 0}));
  a.merge(b);
  EXPECT_EQ(a.size(), 3U);
  EXPECT_NE(a.find("x_1"), nullptr);
  EXPECT_NE(a.find("y"), nullptr);
}

TEST(SceneModel, VoxelAndPointCloudBounds)
{
  SceneModel s;
  SceneObject pc;
  pc.id = "pc";
  pc.geometry = PointCloud{{Vec3(0, 0, 0), Vec3(1, 2, 3)}, {}, {}};
  s.add(pc);
  SceneObject vg;
  vg.id = "vg";
  VoxelGrid g;
  g.voxel_size = 0.1;
  g.origin = Vec3(10, 10, 10);
  g.nx = g.ny = g.nz = 4;
  g.occupied.assign(64, 0);
  vg.geometry = g;
  s.add(vg);
  const Aabb b = s.bounds();
  EXPECT_NEAR(b.max.x(), 10.35, 1e-9);  // last voxel centre 10.3 + half voxel
  EXPECT_NEAR(b.min.x(), 0.0, 1e-9);
}
