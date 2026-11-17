#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "coverage_planner/core/coverage_from_scan.hpp"
#include "coverage_planner/core/coverage_verification.hpp"
#include "coverage_planner/core/surface_patch.hpp"

using namespace coverage_planner;  // NOLINT

namespace
{
ToolCoverageProfile contactTool()
{
  ToolCoverageProfile t;
  t.name = "wipe";
  t.task.contact = true;
  t.task.standoff_m = 0.0;
  t.coverage.swath_m = 0.3;
  t.coverage.overlap_pct = 20.0;
  t.coverage.sweep_direction = "vertical";
  t.coverage.obstacle_inflation_radius_m = 0.1;
  t.coverage.passes = {PassVariant{}};
  return t;
}

ScanCoverageOptions options()
{
  ScanCoverageOptions o;
  o.resolution = 0.05;
  return o;
}

scene_model::SceneObject box(const std::string & id, scene_model::Vec3 size, scene_model::Vec3 at)
{
  scene_model::SceneObject o;
  o.id = id;
  o.geometry = scene_model::Box{size};
  o.pose.translation() = at;
  return o;
}

Eigen::Vector3d toolZ(const Pose & p)
{
  const Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  return q * Eigen::Vector3d::UnitZ();
}

Eigen::Vector3d toVec(const Pose & p)
{
  return Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
}

/// Replays a lifted plan on the patch raster: every segment is one stroke through its poses
/// projected into (u, v). Target = the cells the planner saw as free.
VerificationReport verify(const SurfacePatch & patch, const ScanCoverageResult & r, double swath)
{
  const double res = r.grid.info.resolution;
  std::vector<bool> target(r.grid.data.size());
  for (size_t i = 0; i < target.size(); ++i) {target[i] = r.grid.data[i] == 0;}
  CoverageVerifier verifier(
    patch.u_min, patch.u_max, patch.v_min, patch.v_max, res, target);
  EXPECT_EQ(static_cast<size_t>(verifier.width()) * verifier.height(), r.grid.data.size());
  for (const auto & seg : r.final_plan.segments) {
    ToolStroke stroke;
    stroke.swath_m = swath;
    for (uint32_t i = seg.start_index; i <= seg.end_index; ++i) {
      const Eigen::Vector3d uvn = patch.project(toVec(r.final_plan.path.poses[i].pose));
      stroke.samples.push_back({uvn.x(), uvn.y(), true});
    }
    verifier.addStroke(stroke);
  }
  return verifier.report();
}
}  // namespace

TEST(SurfacePatch, KindFollowsTheNormal)
{
  EXPECT_EQ(classifySurfaceKind(Eigen::Vector3d(0, 0, 1)), SurfaceKind::kFloor);
  EXPECT_EQ(classifySurfaceKind(Eigen::Vector3d(0, 0, -1)), SurfaceKind::kCeiling);
  EXPECT_EQ(classifySurfaceKind(Eigen::Vector3d(0, -1, 0)), SurfaceKind::kWall);
  EXPECT_EQ(classifySurfaceKind(Eigen::Vector3d(0.05, 0, 1)), SurfaceKind::kFloor);
  EXPECT_EQ(classifySurfaceKind(Eigen::Vector3d(1, 0, 1)), SurfaceKind::kWall);
  EXPECT_EQ(surfaceKindName(SurfaceKind::kCeiling), "ceiling");
}

TEST(SurfacePatch, FramesAreRightHandedForEveryKind)
{
  const SurfacePatch patches[] = {
    makeFloorPatch(0, 4, 0, 3, 0.0), makeCeilingPatch(0, 4, 0, 3, 2.8),
    makeSurfacePatch({0, 2, 0}, {0, -1, 0}, 0, 4, 0, 3),
    makeSurfacePatch({1, 1, 0}, {1, 1, 0}, 0, 1, 0, 2.5)};
  for (const auto & p : patches) {
    EXPECT_NEAR(p.u.cross(p.v).dot(p.normal), 1.0, 1e-9) << surfaceKindName(p.kind);
    EXPECT_NEAR(p.u.dot(p.normal), 0.0, 1e-9);
    EXPECT_NEAR(p.v.dot(p.normal), 0.0, 1e-9);
  }
  EXPECT_EQ(patches[0].kind, SurfaceKind::kFloor);
  EXPECT_EQ(patches[1].kind, SurfaceKind::kCeiling);
  EXPECT_EQ(patches[2].kind, SurfaceKind::kWall);
  // A wall keeps v pointing up and u horizontal.
  EXPECT_NEAR(patches[2].v.z(), 1.0, 1e-9);
  EXPECT_NEAR(patches[3].v.z(), 1.0, 1e-9);
  EXPECT_THROW(makeSurfacePatch({0, 0, 0}, {0, 0, 0}, 0, 1, 0, 1), std::invalid_argument);
  EXPECT_THROW(makeFloorPatch(1, 1, 0, 1), std::invalid_argument);
}

TEST(SurfacePatch, ProjectAndPointAreInverse)
{
  const SurfacePatch p = makeCeilingPatch(0, 4, 0, 3, 2.8);
  const Eigen::Vector3d w = p.point(1.5, 0.7, 0.2);
  const Eigen::Vector3d uvn = p.project(w);
  EXPECT_NEAR(uvn.x(), 1.5, 1e-9);
  EXPECT_NEAR(uvn.y(), 0.7, 1e-9);
  EXPECT_NEAR(uvn.z(), 0.2, 1e-9);
  EXPECT_NEAR(w.z(), 2.8 - 0.2, 1e-9);  // outward is down for a ceiling
  EXPECT_TRUE(p.contains(1.0, 1.0));
  EXPECT_FALSE(p.contains(4.1, 1.0));
  EXPECT_TRUE(p.contains(4.1, 1.0, 0.2));
}

TEST(SurfacePatch, WallPlaneRoundTrip)
{
  const SurfacePatch p = makeSurfacePatch({0, 2, 0}, {0, -1, 0}, 0, 4, 0, 3, 7, "odom");
  const SurfacePatch q = patchFromWallPlane(patchToWallPlane(p));
  EXPECT_EQ(q.id, 7U);
  EXPECT_EQ(q.frame_id, "odom");
  EXPECT_LT((q.u - p.u).norm(), 1e-9);
  EXPECT_LT((q.v - p.v).norm(), 1e-9);
  EXPECT_LT((q.normal - p.normal).norm(), 1e-9);
  EXPECT_EQ(q.kind, SurfaceKind::kWall);
}

TEST(SurfacePatch, LiftPutsTheToolAxisAlongTheNormalWithStandoff)
{
  const SurfacePatch floor = makeFloorPatch(0, 4, 0, 3, 0.0);
  const SurfacePatch ceiling = makeCeilingPatch(0, 4, 0, 3, 2.8);
  // Contact: on the surface, tool z into it.
  Pose f = liftPatchPose(floor, 1.0, 2.0, 0.0);
  EXPECT_NEAR(toVec(f).z(), 0.0, 1e-9);
  EXPECT_GT(toolZ(f).dot(Eigen::Vector3d(0, 0, -1)), 0.999);
  // Standoff 0.25: above the floor, below the ceiling; z still into the surface.
  f = liftPatchPose(floor, 1.0, 2.0, 0.25);
  EXPECT_NEAR(toVec(f).x(), 1.0, 1e-9);
  EXPECT_NEAR(toVec(f).y(), 2.0, 1e-9);
  EXPECT_NEAR(toVec(f).z(), 0.25, 1e-9);
  const Pose c = liftPatchPose(ceiling, 1.0, 2.0, 0.25);
  EXPECT_NEAR(toVec(c).z(), 2.55, 1e-9);
  EXPECT_GT(toolZ(c).dot(Eigen::Vector3d(0, 0, 1)), 0.999);
  // x axis follows u (world +x on both), and the frame is a proper rotation.
  const Eigen::Quaterniond q(c.orientation.w, c.orientation.x, c.orientation.y, c.orientation.z);
  EXPECT_NEAR((q * Eigen::Vector3d::UnitX()).dot(ceiling.u), 1.0, 1e-9);
  EXPECT_NEAR(q.toRotationMatrix().determinant(), 1.0, 1e-9);
}

TEST(SurfacePatch, LiftedPathTiltsTowardTheEdgeForANonContactTool)
{
  const SurfacePatch floor = makeFloorPatch(0, 4, 0, 3, 0.0);
  ToolTask task;
  task.contact = false;
  task.standoff_m = 0.3;
  task.tool_normal_policy = "tilt_at_edges";
  task.max_edge_tilt_deg = 20.0;
  const Path path = liftPatchPath(floor, {{2.0, 1.5}, {0.0, 1.5}}, task);
  ASSERT_EQ(path.poses.size(), 2U);
  EXPECT_GT(toolZ(path.poses[0].pose).dot(Eigen::Vector3d(0, 0, -1)), 0.9999);  // mid-floor: none
  const double angle =
    std::acos(std::min(1.0, toolZ(path.poses[1].pose).dot(Eigen::Vector3d(0, 0, -1))));
  EXPECT_NEAR(angle * 180.0 / M_PI, 20.0, 0.5);  // at the u_min edge: full tilt
  EXPECT_NEAR(toVec(path.poses[0].pose).z(), 0.3, 1e-9);
}

TEST(SurfacePatch, FloorRectangleIsCoveredAroundAnObstacle)
{
  const SurfacePatch floor = makeFloorPatch(0, 4, 0, 3, 0.0);
  scene_model::SceneModel scene("map");
  scene.add(box("crate", {0.6, 0.6, 0.5}, {2.0, 1.5, 0.25}));  // stands on the floor
  const auto regions = patchObstacles(floor, scene);
  ASSERT_EQ(regions.size(), 1U);
  EXPECT_NEAR(regions[0].u_min, 1.7, 1e-6);
  EXPECT_NEAR(regions[0].v_max, 1.8, 1e-6);

  const auto tool = contactTool();
  const auto r = planPatchCoverage(floor, regions, tool, options());
  ASSERT_TRUE(r.success) << r.message;
  ASSERT_GT(r.final_plan.path.poses.size(), 20U);

  for (const auto & p : r.final_plan.path.poses) {
    EXPECT_NEAR(p.pose.position.z, 0.0, 1e-6);
    EXPECT_GE(p.pose.position.x, -0.05);
    EXPECT_LE(p.pose.position.x, 4.05);
    EXPECT_GE(p.pose.position.y, -0.05);
    EXPECT_LE(p.pose.position.y, 3.05);
    EXPECT_GT(toolZ(p.pose).dot(Eigen::Vector3d(0, 0, -1)), 0.999);
  }
  // No stroke crosses the crate footprint (sampled along each segment).
  for (const auto & seg : r.final_plan.segments) {
    const Eigen::Vector3d a = toVec(r.final_plan.path.poses[seg.start_index].pose);
    const Eigen::Vector3d b = toVec(r.final_plan.path.poses[seg.end_index].pose);
    for (int i = 0; i <= 50; ++i) {
      const Eigen::Vector3d s = a + (b - a) * (i / 50.0);
      EXPECT_FALSE(s.x() > 1.7 && s.x() < 2.3 && s.y() > 1.2 && s.y() < 1.8) << s.transpose();
    }
  }
  const auto report = verify(floor, r, 0.3);
  EXPECT_GT(report.target_cells, 100U);
  EXPECT_GT(report.covered_fraction, 0.95);
}

TEST(SurfacePatch, CeilingRectangleIsCoveredWithTheToolPointingUp)
{
  const SurfacePatch ceiling = makeCeilingPatch(0, 4, 0, 3, 2.8);
  scene_model::SceneModel scene("map");
  // A lamp hanging 0.2 m off the ceiling: world y in [-1.8, -1.2] is patch v in [1.2, 1.8].
  scene.add(box("lamp", {0.6, 0.6, 0.2}, {2.0, -1.5, 2.7}));
  const auto regions = patchObstacles(ceiling, scene);
  ASSERT_EQ(regions.size(), 1U);
  EXPECT_NEAR(regions[0].v_min, 1.2, 1e-6);
  EXPECT_NEAR(regions[0].v_max, 1.8, 1e-6);

  ToolCoverageProfile spray = contactTool();
  spray.task.contact = false;
  spray.task.standoff_m = 0.3;
  const auto r = planPatchCoverage(ceiling, regions, spray, options());
  ASSERT_TRUE(r.success) << r.message;
  for (const auto & p : r.final_plan.path.poses) {
    EXPECT_NEAR(p.pose.position.z, 2.5, 1e-6);  // 0.3 m below the ceiling
    EXPECT_GT(toolZ(p.pose).dot(Eigen::Vector3d(0, 0, 1)), 0.999);
    EXPECT_GE(p.pose.position.x, -0.05);
    EXPECT_LE(p.pose.position.x, 4.05);
    EXPECT_LE(p.pose.position.y, 0.05);
    EXPECT_GE(p.pose.position.y, -3.05);
  }
  for (const auto & seg : r.final_plan.segments) {
    const Eigen::Vector3d a = toVec(r.final_plan.path.poses[seg.start_index].pose);
    const Eigen::Vector3d b = toVec(r.final_plan.path.poses[seg.end_index].pose);
    for (int i = 0; i <= 50; ++i) {
      const Eigen::Vector3d s = a + (b - a) * (i / 50.0);
      EXPECT_FALSE(s.x() > 1.7 && s.x() < 2.3 && s.y() > -1.8 && s.y() < -1.2) << s.transpose();
    }
  }
  const auto report = verify(ceiling, r, toolSwath(spray));
  EXPECT_GT(report.covered_fraction, 0.95);
}

TEST(SurfacePatch, ColumnFacePlansLikeAWall)
{
  // One face of a square column at x = 1, facing +x, 0.5 m wide and 2.5 m tall.
  const SurfacePatch face = makeSurfacePatch({1.0, 0.0, 0.0}, {1, 0, 0}, 0, 0.5, 0, 2.5);
  EXPECT_EQ(face.kind, SurfaceKind::kWall);
  const auto r = planPatchCoverage(face, {}, contactTool(), options());
  ASSERT_TRUE(r.success) << r.message;
  for (const auto & p : r.final_plan.path.poses) {
    EXPECT_NEAR(p.pose.position.x, 1.0, 1e-6);
    EXPECT_GT(toolZ(p.pose).dot(Eigen::Vector3d(-1, 0, 0)), 0.999);
  }
  EXPECT_GT(verify(face, r, 0.3).covered_fraction, 0.95);
}

TEST(SurfacePatch, FloorScanPlansThroughThePipelineNow)
{
  // A scanned floor (plane z = 0) used to be rejected by the wall-only assumptions; the same
  // pipeline now returns floor poses with the tool pointing down.
  scene_model::PointCloud cloud;
  std::mt19937 rng(3);
  std::normal_distribution<double> noise(0.0, 0.002);
  for (double x = 0.0; x <= 4.0; x += 0.02) {
    for (double y = 0.0; y <= 3.0; y += 0.02) {
      cloud.points.emplace_back(x, y, noise(rng));
    }
  }
  ScanCoverageOptions o = options();
  o.plane.normal_hint = scene_model::Vec3(0, 0, 1);
  const auto r = planCoverageFromScan(cloud, scene_model::SceneModel(), contactTool(), o);
  ASSERT_TRUE(r.success) << r.message;
  const SurfacePatch patch = patchFromWallPlane(r.wall);
  EXPECT_EQ(patch.kind, SurfaceKind::kFloor);
  for (const auto & p : r.final_plan.path.poses) {
    EXPECT_NEAR(p.pose.position.z, 0.0, 0.02);
    EXPECT_GT(toolZ(p.pose).dot(Eigen::Vector3d(0, 0, -1)), 0.99);
  }
  EXPECT_GT(verify(patch, r, 0.3).covered_fraction, 0.95);
}

namespace
{
// A wall in the plane y = 2, x in [0, 4], z in [0, 3].
scene_model::PointCloud wallScan()
{
  scene_model::PointCloud cloud;
  for (double x = 0.0; x <= 4.0; x += 0.02) {
    for (double z = 0.0; z <= 3.0; z += 0.02) {
      cloud.points.emplace_back(x, 2.0, z);
    }
  }
  return cloud;
}
}  // namespace

TEST(SurfacePatch, WallThroughThePatchPathMatchesTheScanPath)
{
  // Same wall two ways: the scan pipeline, and the patch API fed the wall it fitted and the
  // same obstacles and observed points. Poses must agree exactly.
  scene_model::SceneModel scene("map");
  scene.add(box("cabinet", {0.5, 0.3, 0.6}, {2.0, 1.85, 1.5}));
  ScanCoverageOptions o = options();
  o.plane.normal_hint = scene_model::Vec3(0, -1, 0);
  const auto scan = wallScan();
  const auto a = planCoverageFromScan(scan, scene, contactTool(), o);
  ASSERT_TRUE(a.success) << a.message;

  const SurfacePatch patch = patchFromWallPlane(a.wall);
  EXPECT_EQ(patch.kind, SurfaceKind::kWall);
  const auto regions = patchObstacles(patch, scene);
  const auto b = planPatchCoverage(patch, regions, contactTool(), o);
  ASSERT_TRUE(b.success) << b.message;
  // The scan pipeline also masks unobserved cells; this scan observes the whole wall, so the
  // grids agree up to the border cells the fit extents clip.
  ASSERT_EQ(a.final_plan.path.poses.size(), b.final_plan.path.poses.size());
  for (size_t i = 0; i < a.final_plan.path.poses.size(); ++i) {
    EXPECT_LT((toVec(a.final_plan.path.poses[i].pose) - toVec(b.final_plan.path.poses[i].pose)).norm(),
      1e-9) << i;
  }
  // Wall strokes keep the wall conventions: on the plane y = 2, tool z into the wall (+y).
  for (const auto & p : a.final_plan.path.poses) {
    EXPECT_NEAR(p.pose.position.y, 2.0, 0.02);
    EXPECT_GT(toolZ(p.pose).dot(Eigen::Vector3d(0, 1, 0)), 0.99);
  }
}
