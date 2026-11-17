#include "coverage_planner/core/surface_patch.hpp"

#include <cmath>
#include <stdexcept>

#include "coverage_planner/core/wall_basis.hpp"
#include "coverage_planner/core/wall_pose_lift.hpp"

namespace coverage_planner
{

std::string surfaceKindName(SurfaceKind kind)
{
  switch (kind) {
    case SurfaceKind::kFloor: return "floor";
    case SurfaceKind::kCeiling: return "ceiling";
    default: return "wall";
  }
}

SurfaceKind classifySurfaceKind(
  const Eigen::Vector3d & normal, const Eigen::Vector3d & up, double tolerance_deg)
{
  const double c = normal.normalized().dot(up.normalized());
  const double limit = std::cos(tolerance_deg * M_PI / 180.0);
  if (c >= limit) {return SurfaceKind::kFloor;}
  if (c <= -limit) {return SurfaceKind::kCeiling;}
  return SurfaceKind::kWall;
}

Eigen::Vector3d SurfacePatch::point(double pu, double pv, double n) const
{
  return origin + u * pu + v * pv + normal * n;
}

Eigen::Vector3d SurfacePatch::project(const Eigen::Vector3d & world) const
{
  const Eigen::Vector3d rel = world - origin;
  return Eigen::Vector3d(rel.dot(u), rel.dot(v), rel.dot(normal));
}

bool SurfacePatch::contains(double pu, double pv, double margin) const
{
  return pu >= u_min - margin && pu <= u_max + margin && pv >= v_min - margin &&
         pv <= v_max + margin;
}

SurfacePatch makeSurfacePatch(
  const Eigen::Vector3d & origin, const Eigen::Vector3d & normal, double u_min, double u_max,
  double v_min, double v_max, uint32_t id, const std::string & frame_id,
  const Eigen::Vector3d & up)
{
  if (normal.norm() < 1e-9) {
    throw std::invalid_argument("makeSurfacePatch: zero normal");
  }
  if (!(u_max > u_min) || !(v_max > v_min)) {
    throw std::invalid_argument("makeSurfacePatch: empty extent");
  }
  SurfacePatch p;
  p.id = id;
  p.frame_id = frame_id;
  p.origin = origin;
  p.normal = normal.normalized();
  Eigen::Vector3d u = up.cross(p.normal);
  if (u.norm() < 1e-6) {
    // Horizontal plane: world x projected into the plane.
    const Eigen::Vector3d x = Eigen::Vector3d::UnitX();
    u = x - p.normal * p.normal.dot(x);
  }
  p.u = u.normalized();
  p.v = p.normal.cross(p.u).normalized();
  p.u_min = u_min;
  p.u_max = u_max;
  p.v_min = v_min;
  p.v_max = v_max;
  p.kind = classifySurfaceKind(p.normal, up);
  return p;
}

SurfacePatch makeFloorPatch(
  double x_min, double x_max, double y_min, double y_max, double z, uint32_t id,
  const std::string & frame_id)
{
  return makeSurfacePatch(
    Eigen::Vector3d(0, 0, z), Eigen::Vector3d::UnitZ(), x_min, x_max, y_min, y_max, id, frame_id);
}

SurfacePatch makeCeilingPatch(
  double x_min, double x_max, double y_min, double y_max, double z, uint32_t id,
  const std::string & frame_id)
{
  return makeSurfacePatch(
    Eigen::Vector3d(0, 0, z), -Eigen::Vector3d::UnitZ(), x_min, x_max, y_min, y_max, id,
    frame_id);
}

WallPlane patchToWallPlane(const SurfacePatch & patch)
{
  WallPlane wall;
  wall.id = patch.id;
  wall.pose.header.frame_id = patch.frame_id;
  wall.pose.pose = encodeWallBasis(patch.origin, patch.u, patch.v, patch.normal);
  wall.u_min = patch.u_min;
  wall.u_max = patch.u_max;
  wall.v_min = patch.v_min;
  wall.v_max = patch.v_max;
  return wall;
}

SurfacePatch patchFromWallPlane(const WallPlane & wall, const Eigen::Vector3d & up)
{
  const WallBasis b = decomposeWallBasis(wall);
  SurfacePatch p;
  p.id = wall.id;
  p.frame_id = wall.pose.header.frame_id;
  p.origin = b.origin;
  p.u = b.basis_u.normalized();
  p.v = b.basis_v.normalized();
  p.normal = b.normal_outward.normalized();
  p.u_min = wall.u_min;
  p.u_max = wall.u_max;
  p.v_min = wall.v_min;
  p.v_max = wall.v_max;
  p.kind = classifySurfaceKind(p.normal, up);
  return p;
}

Path liftPatchPath(
  const SurfacePatch & patch, const std::vector<Eigen::Vector2d> & uv_waypoints,
  const ToolTask & tool_task)
{
  Path flat;
  flat.header.frame_id = patch.frame_id;
  flat.poses.reserve(uv_waypoints.size());
  for (const auto & w : uv_waypoints) {
    const Eigen::Vector3d p = patch.point(w.x(), w.y());
    PoseStamped ps;
    ps.header.frame_id = patch.frame_id;
    ps.pose.position.x = p.x();
    ps.pose.position.y = p.y();
    ps.pose.position.z = p.z();
    flat.poses.push_back(ps);
  }
  return liftCoveragePathPose(flat, patchToWallPlane(patch), tool_task);
}

Pose liftPatchPose(const SurfacePatch & patch, double pu, double pv, double offset)
{
  ToolTask task;
  task.contact = offset <= 0.0;
  task.standoff_m = offset;
  task.tool_normal_policy = "constant";
  task.max_edge_tilt_deg = 0.0;
  task.tool_frame_yaw_deg = 0.0;
  return liftPatchPath(patch, {Eigen::Vector2d(pu, pv)}, task).poses.front().pose;
}

std::vector<ObstacleRegion> patchObstacles(
  const SurfacePatch & patch, const scene_model::SceneModel & scene, double max_standoff,
  double behind_tolerance)
{
  return obstacleRegionsFromScene(scene, patchToWallPlane(patch), max_standoff, behind_tolerance);
}

ScanCoverageResult planPatchCoverage(
  const SurfacePatch & patch, const std::vector<ObstacleRegion> & obstacles,
  const ToolCoverageProfile & tool, const ScanCoverageOptions & options)
{
  ScanCoverageOptions o = options;
  o.wall_id = patch.id;
  o.frame_id = patch.frame_id;
  return planCoverageOnWall(patchToWallPlane(patch), obstacles, nullptr, tool, o);
}

}  // namespace coverage_planner
