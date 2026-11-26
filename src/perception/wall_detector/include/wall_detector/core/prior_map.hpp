// Prior map input: an external wall / room model (OBJ or ASCII PLY mesh, or the simple
// "prior_surfaces" JSON) turned into the planes and openings the wall_scene.json already carries,
// plus a merge of a prior plane with a measured one. ROS-free; reuses scene_model (loadObj, plane
// frame convention) and wall_detector's wallSceneJson. Format, conventions and the list of what is
// not supported: docs/prior_map.md.
//
// Conventions (same as scene_model::PlaneFit and wall_scene.json):
//   normal = outward, toward the robot / the room side; u = up x normal (horizontal along a wall),
//   v = normal x u (up the wall), u x v = normal; metres; every point in PriorMap::frame_id.
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "scene_model/core/plane_fit.hpp"
#include "scene_model/core/types.hpp"

namespace wall_detector
{

using scene_model::Vec3;

/// A hole in a surface (door, window, vent): a planar polygon lying in the surface's plane.
struct PriorOpening
{
  std::string kind{"opening"};  ///< Free text: "door", "window", "opening", ...
  std::vector<Vec3> polygon;    ///< >= 3 vertices, in the map's frame.
};

/// One planar patch: a polygon in a plane with optional openings.
struct PriorSurface
{
  std::string id;
  Vec3 origin{Vec3::Zero()};  ///< Centroid of the polygon, on the plane.
  Vec3 normal{Vec3::UnitZ()}; ///< Unit, toward the robot side (see PriorMapParams::robot_position).
  std::vector<Vec3> polygon;  ///< Outer boundary, >= 3 vertices, coplanar.
  std::vector<PriorOpening> openings;
};

/// An obstacle box standing in front of the surfaces (a column, a cabinet, a scaffold).
struct PriorBox
{
  std::string id;
  Vec3 center{Vec3::Zero()};
  Vec3 size{Vec3::Zero()};  ///< Full extents along the box's local x, y, z.
  double yaw_deg{0.0};      ///< Rotation about +z of the map's frame.
};

struct PriorMap
{
  std::string frame_id;  ///< Named frame the coordinates are in (e.g. "map"; REP-105).
  std::vector<PriorSurface> surfaces;
  std::vector<PriorBox> obstacles;
  std::vector<std::string> warnings;  ///< Things that were changed or dropped on the way in.
};

struct PriorMapParams
{
  /// Frame to record when the input carries none (OBJ / PLY have no frame notion).
  std::string frame_id{"map"};
  /// Uniform scale applied to a mesh (e.g. 0.001 for millimetre exports).
  double mesh_scale{1.0};
  /// When set, every normal is flipped to face this point (recorded as a warning when it flips).
  std::optional<Vec3> robot_position;
  Vec3 up{Vec3::UnitZ()};
  /// Mesh -> planes: triangles whose normals differ by less than this (either sign) and whose
  /// plane offsets differ by less than `plane_offset_tol_m` belong to one plane.
  double normal_tol_deg{2.0};
  double plane_offset_tol_m{0.01};
  /// Vertices closer than this are one vertex (connectivity of triangles).
  double weld_tol_m{0.001};
  /// Mesh planes smaller than this (m^2) are dropped with a warning.
  double min_area_m2{0.1};
  /// Mesh planes that are not vertical (floors, ceilings) are dropped unless set.
  bool include_horizontal{false};
  /// Max distance of a JSON polygon vertex from its plane (m).
  double coplanar_tol_m{0.01};
};

/// Throws std::runtime_error ("prior map: ..." with the reason) on any malformed input.
PriorMap parsePriorSurfacesJson(const std::string & text, const PriorMapParams & params = {});
PriorMap loadPriorSurfacesJson(const std::string & path, const PriorMapParams & params = {});

/// ASCII PLY with vertices and faces (polygons fan-triangulated). Binary PLY is rejected.
scene_model::Mesh parsePlyMesh(const std::string & text, const std::string & source = "");
scene_model::Mesh loadPlyMesh(const std::string & path);

/// Groups the triangles into planes (see PriorMapParams), one surface per connected coplanar
/// group; the polygon is the bounding rectangle of the group in the plane's (u, v). Openings
/// are not recovered from a mesh. Surfaces are ordered by decreasing area, ids "<prefix>_<k>".
PriorMap priorMapFromMesh(
  const scene_model::Mesh & mesh, const PriorMapParams & params = {},
  const std::string & id_prefix = "plane");

/// Picks the loader from the extension: .obj, .ply (meshes) or .json (prior_surfaces).
PriorMap loadPriorMap(const std::string & path, const PriorMapParams & params = {});

/// Re-expresses the map in `new_frame` (`new_T_old`: points in the old frame -> the new one).
PriorMap transformPriorMap(
  const PriorMap & map, const scene_model::Transform & new_T_old, const std::string & new_frame);

/// In-plane frame of a surface: u = up x normal (fallback for a horizontal plane: world x
/// projected), v = normal x u.
struct SurfaceFrame
{
  Vec3 origin, u, v, normal;
  double u_min{0}, u_max{0}, v_min{0}, v_max{0};  ///< Polygon extents about `origin`.
};
SurfaceFrame surfaceFrame(const PriorSurface & surface, const Vec3 & up = Vec3::UnitZ());

/// The prior as the coverage planner's wall_scene.json: one wall per surface (id = index in
/// `map.surfaces`, plus "name"), every opening as a flush obstacle region (bounding box in the
/// wall's (u, v); height 0, "kind" kept) and every PriorBox within `max_standoff_m` in front of
/// a wall (or `behind_tol_m` behind it) as an obstacle region with its protrusion.
nlohmann::json priorMapToWallSceneJson(
  const PriorMap & map, const std::string & source, const Vec3 & up = Vec3::UnitZ(),
  double max_standoff_m = 0.6, double behind_tol_m = 0.05);

struct MergeParams
{
  double max_offset_m{0.05};   ///< Larger plane-to-plane distance = not the same plane.
  double max_angle_deg{5.0};   ///< Larger angle between the normals = not the same plane.
};

struct UvPolygon
{
  std::string kind;
  std::vector<std::array<double, 2>> uv;  ///< In the measured plane's (u, v) about its origin.
};

struct MergeResult
{
  bool same_plane{false};
  double offset_m{0.0};   ///< Measured origin minus prior plane, along the prior normal (signed).
  double angle_deg{0.0};  ///< Angle between the normals, ignoring sign, 0..90.
  bool normal_flipped{false};  ///< The measured normal points away from the prior's (dot < 0).
  /// The pose to use. Always the measured fit, unchanged: a prior never moves a measurement.
  scene_model::PlaneFit plane;
  /// The prior's openings re-expressed in the measured plane's (u, v); empty when !same_plane.
  std::vector<UvPolygon> openings;
  std::string message;    ///< Human-readable, always set; says when the planes disagree.
};

/// Reconciles a prior surface with a measured plane. Same plane (within `params`) -> keep the
/// measured pose and carry the prior's openings over; otherwise nothing is carried over and the
/// message says by how much they disagree. Never replaces the measurement.
MergeResult mergeWithScan(
  const PriorSurface & prior, const scene_model::PlaneFit & measured, const MergeParams & params = {});

/// wall_scene.json for a measured wall with the prior's openings (when the merge found the same
/// plane) as obstacle regions; adds "prior_merge" {prior_id, offset_m, angle_deg, same_plane,
/// message} to the wall.
nlohmann::json mergedWallSceneJson(
  const scene_model::PlaneFit & measured, const PriorSurface & prior, const MergeResult & merge,
  uint32_t wall_id, const std::string & frame_id, const std::string & source);

}  // namespace wall_detector
