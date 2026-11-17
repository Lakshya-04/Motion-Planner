// A planar patch of a surface: a floor, a ceiling, a wall or one face of a column, as a plane
// with a right-handed (u, v, normal) frame and a rectangular extent in (u, v).
//
// This is the planar implementation of the `SurfaceModel` design in docs/architecture.md:
//   project(point)      -> (u, v, n)         position in patch coordinates (n = height off the plane)
//   lift(u, v, offset)  -> Pose              tool pose: tool z axis along -normal (into the
//                                            surface), `offset` along +normal (the standoff)
//   contains(u, v)
// Coverage planning stays 2D in (u, v) and reuses today's grid planners; only the lift knows
// the patch is horizontal or vertical. `planPatchCoverage` runs the whole chain for one patch.
//
// Convention: `normal` points out of the surface toward the side the tool works from. A floor
// has normal +z, a ceiling -z (it faces down into the room). u x v = normal, always.
// Pure core: no ROS.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "coverage_planner/core/coverage_from_scan.hpp"
#include "coverage_planner/core/tool_coverage_profile.hpp"
#include "coverage_planner/core/types.hpp"

namespace coverage_planner
{

enum class SurfaceKind { kWall, kFloor, kCeiling };

/// "wall" | "floor" | "ceiling".
std::string surfaceKindName(SurfaceKind kind);

/// A normal within `tolerance_deg` of `up` is a floor, within it of `-up` a ceiling, else a wall.
SurfaceKind classifySurfaceKind(
  const Eigen::Vector3d & normal, const Eigen::Vector3d & up = Eigen::Vector3d::UnitZ(),
  double tolerance_deg = 10.0);

struct SurfacePatch
{
  uint32_t id{0};
  std::string frame_id{"map"};
  SurfaceKind kind{SurfaceKind::kWall};
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
  Eigen::Vector3d u{Eigen::Vector3d::UnitX()};        ///< In-plane axis, unit.
  Eigen::Vector3d v{Eigen::Vector3d::UnitZ()};        ///< In-plane axis, unit, u x v = normal.
  Eigen::Vector3d normal{-Eigen::Vector3d::UnitY()};  ///< Outward, unit.
  double u_min{0.0};
  double u_max{0.0};
  double v_min{0.0};
  double v_max{0.0};

  /// World point of patch coordinates (u, v) plus `n` along the normal.
  Eigen::Vector3d point(double pu, double pv, double n = 0.0) const;
  /// Patch coordinates of a world point: (u, v, n).
  Eigen::Vector3d project(const Eigen::Vector3d & world) const;
  /// True when (u, v) lies inside the extent (inclusive, with `margin` allowed outside).
  bool contains(double pu, double pv, double margin = 0.0) const;
  double width() const {return u_max - u_min;}
  double height() const {return v_max - v_min;}
};

/**
 * @brief Patch from a plane and a rectangular extent in its (u, v) frame.
 * @details The in-plane axes are built from `up`: u = up x normal (horizontal along a wall, v
 *          points up). For a horizontal plane (normal parallel to `up`) u = world x projected
 *          into the plane and v = normal x u. `kind` is classified from the normal.
 * @throws std::invalid_argument for a zero normal or an empty extent.
 */
SurfacePatch makeSurfacePatch(
  const Eigen::Vector3d & origin, const Eigen::Vector3d & normal, double u_min, double u_max,
  double v_min, double v_max, uint32_t id = 0, const std::string & frame_id = "map",
  const Eigen::Vector3d & up = Eigen::Vector3d::UnitZ());

/// Floor rectangle x in [x_min, x_max], y in [y_min, y_max] at height z. u = +x, v = +y.
SurfacePatch makeFloorPatch(
  double x_min, double x_max, double y_min, double y_max, double z = 0.0, uint32_t id = 0,
  const std::string & frame_id = "map");

/// Ceiling rectangle at height z, normal -z (facing down). u = +x, v = -y, so the patch
/// (u, v) in [x_min, x_max] x [y_min, y_max] covers world x in [x_min, x_max] and
/// y in [-y_max, -y_min].
SurfacePatch makeCeilingPatch(
  double x_min, double x_max, double y_min, double y_max, double z, uint32_t id = 0,
  const std::string & frame_id = "map");

/// The patch as the WallPlane the existing planners take (and back).
WallPlane patchToWallPlane(const SurfacePatch & patch);
SurfacePatch patchFromWallPlane(
  const WallPlane & wall, const Eigen::Vector3d & up = Eigen::Vector3d::UnitZ());

/// One tool pose at patch coordinates (u, v): tool z axis along -normal, position `offset` along
/// +normal off the surface, x axis along u. No edge tilt (that needs the tool profile); use
/// liftPatchPath for paths.
Pose liftPatchPose(const SurfacePatch & patch, double pu, double pv, double offset = 0.0);

/// 2D waypoints (u, v) lifted to oriented tool poses: standoff, edge tilt and yaw from the tool
/// task, exactly as liftCoveragePathPose does for a wall.
Path liftPatchPath(
  const SurfacePatch & patch, const std::vector<Eigen::Vector2d> & uv_waypoints,
  const ToolTask & tool_task);

/// Obstacle footprints on the patch (see obstacleRegionsFromScene; works for any orientation).
std::vector<ObstacleRegion> patchObstacles(
  const SurfacePatch & patch, const scene_model::SceneModel & scene, double max_standoff = 0.6,
  double behind_tolerance = 0.05);

/**
 * @brief Whole 2D coverage plan of one patch with today's planners: rasterize the extent with
 *        the obstacles (inflated per the tool), plan with the grid planner and pattern in
 *        `options`, lift every part, expand passes. Same chain as planCoverageFromScan from
 *        the rasterization on. `options.resolution`, `planner`, `wall_pattern` and the strip /
 *        corner options apply; the plane-fit options are ignored.
 * @param obstacles Footprints in the patch (u, v), e.g. from patchObstacles().
 */
ScanCoverageResult planPatchCoverage(
  const SurfacePatch & patch, const std::vector<ObstacleRegion> & obstacles,
  const ToolCoverageProfile & tool, const ScanCoverageOptions & options = {});

}  // namespace coverage_planner
