#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

#include "wall_detector/core/prior_map.hpp"

using namespace wall_detector;  // NOLINT

namespace
{
const std::string kData = PRIOR_MAP_TEST_DATA;
constexpr double kDeg = 3.14159265358979323846 / 180.0;

std::string tempFile(const std::string & name, const std::string & text)
{
  const auto p = std::filesystem::temp_directory_path() / ("prior_map_test_" + name);
  std::ofstream(p) << text;
  return p.string();
}

const PriorSurface * byNormal(const PriorMap & m, const Vec3 & n)
{
  for (const auto & s : m.surfaces) {
    if (s.normal.dot(n) > 0.999) {return &s;}
  }
  return nullptr;
}

// The measured fit of the north wall (y = 3, normal -y toward the room) with a given offset / tilt.
scene_model::PlaneFit measuredNorth(double offset_m, double tilt_deg)
{
  scene_model::PlaneFit fit;
  const Eigen::Matrix3d R = Eigen::AngleAxisd(tilt_deg * kDeg, Vec3::UnitZ()).toRotationMatrix();
  fit.normal = R * Vec3(0, -1, 0);
  fit.u = Vec3::UnitZ().cross(fit.normal).normalized();  // up x normal, as in PlaneFit
  fit.v = fit.normal.cross(fit.u);
  fit.origin = Vec3(2.0, 3.0 - offset_m, 1.25);  // moved toward the room by offset_m
  fit.u_min = -2; fit.u_max = 2; fit.v_min = -1.25; fit.v_max = 1.25;
  return fit;
}
}  // namespace

TEST(PriorMapMesh, BoxRoomObjGivesFourWallsFacingTheRobot)
{
  PriorMapParams p;
  p.robot_position = Vec3(2.0, 1.5, 1.0);  // in the middle of the room
  const auto map = loadPriorMap(kData + "/room.obj", p);
  ASSERT_EQ(map.surfaces.size(), 4u);  // floor and ceiling dropped
  EXPECT_EQ(map.frame_id, "map");
  const auto * north = byNormal(map, Vec3(0, -1, 0));
  const auto * south = byNormal(map, Vec3(0, 1, 0));
  const auto * east = byNormal(map, Vec3(-1, 0, 0));
  const auto * west = byNormal(map, Vec3(1, 0, 0));
  ASSERT_TRUE(north && south && east && west);
  EXPECT_NEAR(north->origin.y(), 3.0, 1e-9);
  EXPECT_NEAR(east->origin.x(), 4.0, 1e-9);
  const auto f = surfaceFrame(*north);
  EXPECT_NEAR(f.u_max - f.u_min, 4.0, 1e-9);
  EXPECT_NEAR(f.v_max - f.v_min, 2.5, 1e-9);
  EXPECT_NEAR(f.u.cross(f.v).dot(f.normal), 1.0, 1e-9);
  EXPECT_NEAR(std::abs(f.v.z()), 1.0, 1e-9);
}

TEST(PriorMapMesh, FlipsAnOutwardFacingMeshTowardTheRobotAndSaysSo)
{
  // Same room, faces wound so the normals point out of the room.
  std::string obj = "v 0 0 0\nv 4 0 0\nv 4 3 0\nv 0 3 0\nv 0 0 2.5\nv 4 0 2.5\nv 4 3 2.5\nv 0 3 2.5\n"
    "f 1 2 6 5\nf 2 3 7 6\nf 3 4 8 7\nf 4 1 5 8\n";
  PriorMapParams p;
  p.robot_position = Vec3(2.0, 1.5, 1.0);
  const auto map = loadPriorMap(tempFile("outward.obj", obj), p);
  ASSERT_EQ(map.surfaces.size(), 4u);
  for (const auto & s : map.surfaces) {
    EXPECT_GT((*p.robot_position - s.origin).dot(s.normal), 0.0);
  }
  EXPECT_EQ(map.warnings.size(), 4u);
}

TEST(PriorMapMesh, PlyQuadsAndScale)
{
  const std::string ply =
    "ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\n"
    "element face 1\nproperty list uchar int vertex_indices\nend_header\n"
    "0 0 0\n4000 0 0\n4000 0 2500\n0 0 2500\n4 0 1 2 3\n";  // millimetres, wall in y = 0
  PriorMapParams p;
  p.mesh_scale = 0.001;
  p.robot_position = Vec3(2, 2, 1);
  const auto map = loadPriorMap(tempFile("wall.ply", ply), p);
  ASSERT_EQ(map.surfaces.size(), 1u);
  EXPECT_NEAR(map.surfaces[0].normal.y(), 1.0, 1e-9);
  const auto f = surfaceFrame(map.surfaces[0]);
  EXPECT_NEAR(f.u_max - f.u_min, 4.0, 1e-9);
}

TEST(PriorMapMesh, DisconnectedCoplanarWallsStayTwoSurfaces)
{
  const std::string obj =
    "v 0 0 0\nv 2 0 0\nv 2 0 2\nv 0 0 2\nv 5 0 0\nv 7 0 0\nv 7 0 2\nv 5 0 2\n"
    "f 1 2 3 4\nf 5 6 7 8\n";
  PriorMapParams p;
  p.robot_position = Vec3(3, -2, 1);
  const auto map = loadPriorMap(tempFile("two.obj", obj), p);
  EXPECT_EQ(map.surfaces.size(), 2u);
}

TEST(PriorMapJson, WindowWallBecomesWallSceneWithOpeningsAndBox)
{
  const auto map = loadPriorMap(kData + "/window_wall.json");
  ASSERT_EQ(map.surfaces.size(), 1u);
  ASSERT_EQ(map.surfaces[0].openings.size(), 2u);
  EXPECT_NEAR(map.surfaces[0].normal.y(), -1.0, 1e-12);
  const auto scene = priorMapToWallSceneJson(map, "window_wall.json");
  ASSERT_EQ(scene["walls"].size(), 1u);
  const auto & w = scene["walls"][0];
  EXPECT_NEAR(w["u_range_m"][1].get<double>() - w["u_range_m"][0].get<double>(), 4.0, 1e-9);
  EXPECT_NEAR(w["v_range_m"][1].get<double>() - w["v_range_m"][0].get<double>(), 2.5, 1e-9);
  // Same keys the coverage planner's loader reads.
  for (const char * k : {"id", "origin_xyz", "basis_u", "basis_v", "normal_outward", "u_range_m",
      "v_range_m", "plane_rms_mm", "spread_p95_mm", "spread_p99_mm"}) {
    EXPECT_TRUE(w.contains(k)) << k;
  }
  ASSERT_EQ(scene["obstacles"].size(), 3u);  // window, door, cabinet
  const auto & win = scene["obstacles"][0];
  EXPECT_EQ(win["kind"], "window");
  EXPECT_EQ(win["parent_wall"], 0);
  EXPECT_NEAR(win["uv_on_wall"]["u"][1].get<double>() - win["uv_on_wall"]["u"][0].get<double>(), 1.0, 1e-9);
  EXPECT_NEAR(win["uv_on_wall"]["v"][1].get<double>() - win["uv_on_wall"]["v"][0].get<double>(), 1.0, 1e-9);
  const auto & box = scene["obstacles"][2];
  EXPECT_EQ(box["kind"], "box");
  // Cabinet centre y 2.8, depth 0.4: its back is at y 3.0 (on the wall), its front 0.4 m off it.
  EXPECT_NEAR(box["protrusion_max_m"].get<double>(), 0.4, 1e-9);
  EXPECT_NEAR(box["uv_on_wall"]["u"][1].get<double>() - box["uv_on_wall"]["u"][0].get<double>(), 0.6, 1e-9);
}

TEST(PriorMapJson, WindowSitsWhereTheJsonPutsIt)
{
  const auto map = loadPriorMap(kData + "/window_wall.json");
  const auto f = surfaceFrame(map.surfaces[0]);
  // Window centre (1.5, 3, 1.5) in the wall's (u, v).
  const Vec3 d = Vec3(1.5, 3.0, 1.5) - f.origin;
  const auto scene = priorMapToWallSceneJson(map, "x");
  const auto & uv = scene["obstacles"][0]["uv_on_wall"];
  const double uc = 0.5 * (uv["u"][0].get<double>() + uv["u"][1].get<double>());
  const double vc = 0.5 * (uv["v"][0].get<double>() + uv["v"][1].get<double>());
  EXPECT_NEAR(uc, d.dot(f.u), 1e-9);
  EXPECT_NEAR(vc, d.dot(f.v), 1e-9);
}

TEST(PriorMapMerge, TwoCmOneDegreeKeepsMeasuredPoseAndPriorOpenings)
{
  const auto map = loadPriorMap(kData + "/window_wall.json");
  const auto measured = measuredNorth(0.02, 1.0);
  const auto r = mergeWithScan(map.surfaces[0], measured);
  EXPECT_TRUE(r.same_plane);
  EXPECT_NEAR(r.angle_deg, 1.0, 1e-6);
  EXPECT_NEAR(r.offset_m, 0.02, 1e-9);  // 2 cm along the prior normal (-y, toward the room)
  // The pose is the measurement, untouched.
  EXPECT_EQ(r.plane.origin, measured.origin);
  EXPECT_EQ(r.plane.normal, measured.normal);
  ASSERT_EQ(r.openings.size(), 2u);
  EXPECT_EQ(r.openings[0].kind, "window");
  ASSERT_EQ(r.openings[0].uv.size(), 4u);
  EXPECT_NE(r.message.find("2"), std::string::npos);
  // The window's width in the measured plane is still about 1 m.
  double u0 = 1e9, u1 = -1e9;
  for (const auto & p : r.openings[0].uv) {u0 = std::min(u0, p[0]); u1 = std::max(u1, p[0]);}
  EXPECT_NEAR(u1 - u0, 1.0, 0.02);

  const auto scene = mergedWallSceneJson(measured, map.surfaces[0], r, 7, "map", "scan+prior");
  EXPECT_EQ(scene["walls"][0]["id"], 7);
  EXPECT_EQ(scene["obstacles"].size(), 2u);
  EXPECT_TRUE(scene["walls"][0]["prior_merge"]["same_plane"].get<bool>());
}

TEST(PriorMapMerge, DisagreementIsReportedNotHidden)
{
  const auto map = loadPriorMap(kData + "/window_wall.json");
  const auto far = mergeWithScan(map.surfaces[0], measuredNorth(0.20, 0.0));
  EXPECT_FALSE(far.same_plane);
  EXPECT_NEAR(std::abs(far.offset_m), 0.20, 1e-9);
  EXPECT_TRUE(far.openings.empty());
  EXPECT_NE(far.message.find("NOT the same plane"), std::string::npos);
  EXPECT_NEAR((far.plane.origin - measuredNorth(0.20, 0.0).origin).norm(), 0.0, 0.0);

  const auto tilted = mergeWithScan(map.surfaces[0], measuredNorth(0.0, 8.0));
  EXPECT_FALSE(tilted.same_plane);
  EXPECT_NEAR(tilted.angle_deg, 8.0, 1e-6);

  auto flipped = measuredNorth(0.0, 0.0);
  flipped.normal = -flipped.normal;
  const auto fr = mergeWithScan(map.surfaces[0], flipped);
  EXPECT_FALSE(fr.same_plane);
  EXPECT_TRUE(fr.normal_flipped);
}

TEST(PriorMapReject, MalformedInput)
{
  const std::string good_poly = "[[0,3,0],[4,3,0],[4,3,2.5],[0,3,2.5]]";
  auto surf = [&](const std::string & body) {
      return "{\"surfaces\":[{\"id\":\"a\"," + body + "}]}";
    };
  EXPECT_THROW(parsePriorSurfacesJson("{not json"), std::runtime_error);
  EXPECT_THROW(parsePriorSurfacesJson("[]"), std::runtime_error);
  EXPECT_THROW(parsePriorSurfacesJson("{}"), std::runtime_error);
  EXPECT_THROW(parsePriorSurfacesJson(surf("\"polygon\":[[0,0,0],[1,0,0]]")), std::runtime_error);
  EXPECT_THROW(parsePriorSurfacesJson(surf("\"polygon\":[[0,0,0],[1,0,0],[2,0,0]]")), std::runtime_error);   // zero area
  EXPECT_THROW(parsePriorSurfacesJson(surf("\"polygon\":[[0,0,0],[1,0,0],[1,1,0.5],[0,1,0]]")), std::runtime_error);  // not planar
  EXPECT_THROW(parsePriorSurfacesJson(surf("\"polygon\":[[0,0,0],[1,0,\"x\"],[1,1,0]]")), std::runtime_error);
  EXPECT_THROW(parsePriorSurfacesJson(surf("\"polygon\":" + good_poly + ",\"normal\":[0,0,1]")), std::runtime_error);  // contradicts the polygon
  EXPECT_THROW(  // opening outside the wall
    parsePriorSurfacesJson(surf(
      "\"polygon\":" + good_poly +
      ",\"openings\":[{\"polygon\":[[5,3,1],[6,3,1],[6,3,2]]}]")), std::runtime_error);
  EXPECT_THROW(  // opening off the plane
    parsePriorSurfacesJson(surf(
      "\"polygon\":" + good_poly +
      ",\"openings\":[{\"polygon\":[[1,2,1],[2,2,1],[2,2,2]]}]")), std::runtime_error);
  EXPECT_THROW(  // duplicate ids
    parsePriorSurfacesJson(
      "{\"surfaces\":[{\"id\":\"a\",\"polygon\":" + good_poly + "},{\"id\":\"a\",\"polygon\":" + good_poly + "}]}"),
    std::runtime_error);
  EXPECT_THROW(parsePriorSurfacesJson("{\"units\":\"feet\",\"surfaces\":[]}"), std::runtime_error);
  EXPECT_THROW(
    parsePriorSurfacesJson("{\"surfaces\":[],\"obstacles\":[{\"id\":\"b\",\"center\":[0,0,0],\"size\":[1,0,1]}]}"),
    std::runtime_error);

  // Meshes.
  EXPECT_THROW(parsePlyMesh("ply\nformat binary_little_endian 1.0\nend_header\n"), std::runtime_error);
  EXPECT_THROW(parsePlyMesh("not a ply"), std::runtime_error);
  EXPECT_THROW(
    parsePlyMesh("ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\n"
      "element face 1\nproperty list uchar int vertex_indices\nend_header\n0 0 0\n1 0 0\n0 1 0\n3 0 1 9\n"),
    std::runtime_error);
  EXPECT_THROW(loadPriorMap(tempFile("bad.obj", "v 0 0 0\nv 1 0\nf 1 1 1\n")), std::runtime_error);
  EXPECT_THROW(loadPriorMap(tempFile("idx.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 7\n")), std::runtime_error);
  EXPECT_THROW(loadPriorMap(tempFile("empty.obj", "# nothing\n")), std::runtime_error);
  EXPECT_THROW(loadPriorMap(kData + "/missing.json"), std::runtime_error);
  EXPECT_THROW(loadPriorMap(tempFile("model.ifc", "ISO-10303-21;")), std::runtime_error);
  // Only a floor: nothing left after the default filter.
  EXPECT_THROW(
    loadPriorMap(tempFile("floor.obj", "v 0 0 0\nv 3 0 0\nv 3 3 0\nv 0 3 0\nf 1 2 3 4\n")), std::runtime_error);
}

TEST(PriorMapFrames, TransformKeepsGeometryAndRenamesFrame)
{
  const auto map = loadPriorMap(kData + "/window_wall.json");
  scene_model::Transform T = scene_model::Transform::Identity();
  T.translate(Vec3(10, 0, 0));
  const auto moved = transformPriorMap(map, T, "odom");
  EXPECT_EQ(moved.frame_id, "odom");
  EXPECT_NEAR(moved.surfaces[0].origin.x(), map.surfaces[0].origin.x() + 10.0, 1e-12);
  EXPECT_NEAR(moved.surfaces[0].normal.y(), -1.0, 1e-12);
  EXPECT_THROW(transformPriorMap(map, T, ""), std::runtime_error);
}
