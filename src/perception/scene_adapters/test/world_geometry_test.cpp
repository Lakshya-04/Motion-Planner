#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <string>

#include "scene_adapters/world_geometry.hpp"

namespace sm = scene_model;
using scene_adapters::ConversionReport;
using scene_adapters::toWorldGeometry;
using scene_adapters::WorldGeometryOptions;

namespace
{
sm::SceneObject object(const std::string & id, sm::Geometry g, sm::Role role = sm::Role::kObstacle)
{
  sm::SceneObject o;
  o.id = id;
  o.geometry = std::move(g);
  o.role = role;
  return o;
}

sm::Transform at(double x, double y, double z)
{
  sm::Transform t = sm::Transform::Identity();
  t.translation() = sm::Vec3(x, y, z);
  return t;
}
}  // namespace

TEST(WorldGeometry, ObstacleBoxKeepsPoseAndSize)
{
  sm::SceneModel scene("base");
  auto o = object("crate", sm::Box{sm::Vec3(0.2, 0.4, 0.6)});
  o.pose = at(1.0, 2.0, 3.0) * Eigen::AngleAxisd(M_PI / 2, sm::Vec3::UnitZ());
  scene.add(o);
  ConversionReport rep;
  const auto w = toWorldGeometry(scene, {}, &rep);
  EXPECT_EQ(w.frame_id, "base");
  ASSERT_EQ(w.boxes.size(), 1U);
  EXPECT_EQ(w.boxes[0].id, "crate");
  EXPECT_DOUBLE_EQ(w.boxes[0].center_x, 1.0);
  EXPECT_DOUBLE_EQ(w.boxes[0].size_y, 0.4);
  EXPECT_NEAR(w.boxes[0].orientation_z, std::sqrt(0.5), 1e-9);
  EXPECT_NEAR(w.boxes[0].orientation_w, std::sqrt(0.5), 1e-9);
  EXPECT_TRUE(w.points.empty());
  EXPECT_TRUE(rep.approximated.empty() && rep.skipped.empty());
  EXPECT_EQ(rep.boxes, 1U);
}

TEST(WorldGeometry, PrimitivesBecomeConservativeBoxesAndAreReported)
{
  sm::SceneModel scene("base");
  scene.add(object("ball", sm::Sphere{0.25}));
  scene.add(object("pipe", sm::Cylinder{0.1, 0.8}));
  scene.add(object("pill", sm::Capsule{0.1, 0.5}));
  ConversionReport rep;
  const auto w = toWorldGeometry(scene, {}, &rep);
  ASSERT_EQ(w.boxes.size(), 3U);
  EXPECT_DOUBLE_EQ(w.boxes[0].size_x, 0.5);
  EXPECT_DOUBLE_EQ(w.boxes[0].size_z, 0.5);
  EXPECT_DOUBLE_EQ(w.boxes[1].size_x, 0.2);
  EXPECT_DOUBLE_EQ(w.boxes[1].size_z, 0.8);
  EXPECT_DOUBLE_EQ(w.boxes[2].size_z, 0.7);      // length + both end caps
  EXPECT_EQ(rep.approximated.size(), 3U);
  EXPECT_NE(rep.approximated[0].find("sphere"), std::string::npos);
}

TEST(WorldGeometry, ContactSurfaceBecomesPointsNeverBoxes)
{
  sm::SceneModel scene("base");
  sm::PointCloud wall;
  for (int i = 0; i < 10; ++i) {wall.points.emplace_back(2.0, 0.1 * i, 1.0);}
  auto o = object("wall", wall, sm::Role::kContactSurface);
  o.pose = at(0.5, 0.0, 0.0);
  scene.add(o);
  const auto w = toWorldGeometry(scene);
  EXPECT_TRUE(w.boxes.empty());
  ASSERT_EQ(w.points.size(), 10U);
  EXPECT_DOUBLE_EQ(w.points[3].x, 2.5);          // pose applied
  EXPECT_NEAR(w.points[3].y, 0.3, 1e-12);
}

TEST(WorldGeometry, ContactSurfaceBoxIsSampledOnItsFaces)
{
  sm::SceneModel scene("base");
  scene.add(object("slab", sm::Box{sm::Vec3(0.1, 1.0, 1.0)}, sm::Role::kContactSurface));
  WorldGeometryOptions opt;
  opt.surface_spacing = 0.25;
  const auto w = toWorldGeometry(scene, opt);
  EXPECT_TRUE(w.boxes.empty());
  ASSERT_FALSE(w.points.empty());
  // every sample lies on the surface of the slab and the big faces are covered to their corners
  bool corner = false;
  for (const auto & p : w.points) {
    const bool on_x = std::abs(std::abs(p.x) - 0.05) < 1e-9;
    const bool on_y = std::abs(std::abs(p.y) - 0.5) < 1e-9;
    const bool on_z = std::abs(std::abs(p.z) - 0.5) < 1e-9;
    EXPECT_TRUE(on_x || on_y || on_z);
    corner = corner || (on_x && on_y && on_z);
  }
  EXPECT_TRUE(corner);
}

TEST(WorldGeometry, ObstacleCloudBecomesCubesNotContactPoints)
{
  sm::SceneModel scene("base");
  sm::PointCloud pc;
  pc.points = {{0.01, 0.01, 0.01}, {0.02, 0.03, 0.04}, {0.31, 0.0, 0.0}};
  scene.add(object("clutter", pc));
  WorldGeometryOptions opt;
  opt.obstacle_voxel = 0.1;
  ConversionReport rep;
  const auto w = toWorldGeometry(scene, opt, &rep);
  EXPECT_TRUE(w.points.empty());                 // never the excludable blob
  ASSERT_EQ(w.boxes.size(), 2U);
  EXPECT_DOUBLE_EQ(w.boxes[0].size_x, 0.1);
  EXPECT_EQ(w.boxes[0].id, "clutter#c0");
  EXPECT_TRUE(rep.coarsened.empty());
}

TEST(WorldGeometry, CapGrowsTheCubesAndEveryPointStaysCovered)
{
  sm::SceneModel scene("base");
  sm::PointCloud pc;
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) {pc.points.emplace_back(0.05 * i, 0.05 * j, 0.0);}
  }
  scene.add(object("grid", pc));
  WorldGeometryOptions opt;
  opt.obstacle_voxel = 0.05;
  opt.max_obstacle_boxes = 30;
  ConversionReport rep;
  const auto w = toWorldGeometry(scene, opt, &rep);
  EXPECT_LE(w.boxes.size(), 30U);
  EXPECT_FALSE(rep.coarsened.empty());
  const double edge = w.boxes[0].size_x;
  EXPECT_GT(edge, 0.05);
  for (const auto & p : pc.points) {
    bool covered = false;
    for (const auto & b : w.boxes) {
      covered = covered || (std::abs(p.x() - b.center_x) <= edge / 2 + 1e-9 &&
        std::abs(p.y() - b.center_y) <= edge / 2 + 1e-9 && std::abs(p.z() - b.center_z) <= edge / 2 + 1e-9);
    }
    ASSERT_TRUE(covered);
  }
}

TEST(WorldGeometry, MeshSurfaceIsSampledAtTheRequestedSpacing)
{
  sm::SceneModel scene("base");
  sm::Mesh quad;
  quad.vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
  quad.faces = {{0, 1, 2}, {0, 2, 3}};
  scene.add(object("panel", quad, sm::Role::kContactSurface));
  WorldGeometryOptions opt;
  opt.surface_spacing = 0.1;
  const auto w = toWorldGeometry(scene, opt);
  EXPECT_GT(w.points.size(), 100U);
  for (const auto & p : w.points) {
    EXPECT_NEAR(p.z, 0.0, 1e-12);
    EXPECT_GE(p.x, -1e-12);
    EXPECT_LE(p.x, 1.0 + 1e-12);
  }
}

TEST(WorldGeometry, VoxelGridUsesOccupiedCellsAndHonoursUnknown)
{
  sm::SceneModel scene("base");
  sm::VoxelGrid g;
  g.voxel_size = 0.1;
  g.nx = 3;
  g.ny = 1;
  g.nz = 1;
  g.occupied = {sm::kVoxelOccupied, sm::kVoxelUnknown, sm::kVoxelFree};
  scene.add(object("map", g, sm::Role::kContactSurface));
  EXPECT_EQ(toWorldGeometry(scene).points.size(), 1U);
  WorldGeometryOptions opt;
  opt.unknown_voxels_occupied = true;
  EXPECT_EQ(toWorldGeometry(scene, opt).points.size(), 2U);
}

TEST(WorldGeometry, ObstacleVoxelGridIsCoveredWholeNotJustAtItsCentre)
{
  // one 0.1 m voxel converted with 0.02 m cubes: every point of the voxel must stay inside a cube
  sm::SceneModel scene("base");
  sm::VoxelGrid g;
  g.voxel_size = 0.1;
  g.origin = sm::Vec3(0.013, -0.007, 0.0);
  g.nx = g.ny = g.nz = 1;
  g.occupied = {sm::kVoxelOccupied};
  scene.add(object("map", g));
  WorldGeometryOptions opt;
  opt.obstacle_voxel = 0.02;
  const auto w = toWorldGeometry(scene, opt);
  ASSERT_FALSE(w.boxes.empty());
  for (const double fx : {-0.05, 0.0, 0.0499}) {
    for (const double fy : {-0.05, 0.0, 0.0499}) {
      for (const double fz : {-0.05, 0.0, 0.0499}) {
        const double px = g.origin.x() + fx, py = g.origin.y() + fy, pz = g.origin.z() + fz;
        bool covered = false;
        for (const auto & b : w.boxes) {
          covered = covered || (std::abs(px - b.center_x) <= b.size_x / 2 + 1e-9 &&
            std::abs(py - b.center_y) <= b.size_y / 2 + 1e-9 &&
            std::abs(pz - b.center_z) <= b.size_z / 2 + 1e-9);
        }
        EXPECT_TRUE(covered) << fx << " " << fy << " " << fz;
      }
    }
  }
}

TEST(WorldGeometry, BadMeshIndexAndSpacingDoNotThrowOrHang)
{
  sm::SceneModel scene("base");
  sm::Mesh m;
  m.vertices = {sm::Vec3(0, 0, 0), sm::Vec3(1, 0, 0), sm::Vec3(0, 1, 0)};
  m.faces = {{0, 1, 2}, {0, 1, 7}};
  scene.add(object("tri", m));
  WorldGeometryOptions opt;
  opt.surface_spacing = 0.0;
  ConversionReport rep;
  EXPECT_NO_THROW(toWorldGeometry(scene, opt, &rep));
  EXPECT_FALSE(rep.skipped.empty());
  EXPECT_FALSE(rep.approximated.empty());
}

TEST(WorldGeometry, EsdfIsSkippedAndReported)
{
  sm::SceneModel scene("base");
  scene.add(object("field", sm::EsdfGrid{}));
  ConversionReport rep;
  const auto w = toWorldGeometry(scene, {}, &rep);
  EXPECT_TRUE(w.boxes.empty() && w.points.empty());
  ASSERT_EQ(rep.skipped.size(), 1U);
  EXPECT_NE(rep.skipped[0].find("field"), std::string::npos);
}

TEST(WorldGeometry, EmptySceneGivesAnEmptyUpdate)
{
  const auto w = toWorldGeometry(sm::SceneModel("base"));
  EXPECT_EQ(w.frame_id, "base");
  EXPECT_TRUE(w.boxes.empty() && w.points.empty());
}

TEST(WorldGeometry, BoxIdsStayUniqueAfterASceneMergeRenamesObjects)
{
  // SceneModel::merge renames a clashing id to "<id>_<n>", which is also the pattern cube boxes
  // used: a cloud "crate" merged with a box "crate" gave crate_0, crate_1, crate_1.
  sm::SceneModel a("base");
  sm::PointCloud pc;
  pc.points = {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}};
  a.add(object("crate", pc));
  sm::SceneModel b("base");
  b.add(object("crate", sm::Box{sm::Vec3(0.1, 0.1, 0.1)}));
  a.merge(b);
  WorldGeometryOptions opt;
  opt.obstacle_voxel = 0.1;
  const auto w = toWorldGeometry(a, opt);
  std::set<std::string> ids;
  for (const auto & box : w.boxes) {ids.insert(box.id);}
  EXPECT_EQ(ids.size(), w.boxes.size());
  EXPECT_EQ(w.boxes.size(), 3U);
}
