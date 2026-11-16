#include "coverage_planner/core/coverage_from_scan.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "coverage_planner/core/wall_basis.hpp"
#include "coverage_planner/core/wall_rasterizer.hpp"

namespace coverage_planner
{

WallPlane wallFromPlaneFit(
  const scene_model::PlaneFit & fit, uint32_t id, const std::string & frame_id)
{
  WallPlane wall;
  wall.id = id;
  wall.pose.header.frame_id = frame_id;
  wall.pose.pose = encodeWallBasis(fit.origin, fit.u, fit.v, fit.normal);
  wall.u_min = fit.u_min;
  wall.u_max = fit.u_max;
  wall.v_min = fit.v_min;
  wall.v_max = fit.v_max;
  wall.plane_rms_mm = 1000.0 * fit.rms;
  wall.spread_p95_mm = 1000.0 * fit.spread_p95;
  wall.spread_p99_mm = 1000.0 * fit.spread_p99;
  return wall;
}

std::vector<ObstacleRegion> obstacleRegionsFromScene(
  const scene_model::SceneModel & scene, const WallPlane & wall, double max_standoff,
  double behind_tolerance)
{
  const WallBasis basis = decomposeWallBasis(wall);
  std::vector<ObstacleRegion> regions;
  uint32_t next_id = 0;
  for (const auto & object : scene.objects()) {
    if (object.role != scene_model::Role::kObstacle) {
      continue;
    }
    const scene_model::Aabb box = scene_model::objectBounds(object);
    if (box.empty()) {
      continue;
    }
    double n_min = 1e18, n_max = -1e18, u_min = 1e18, u_max = -1e18, v_min = 1e18, v_max = -1e18;
    for (int i = 0; i < 8; ++i) {
      const Eigen::Vector3d corner(
        (i & 1) ? box.max.x() : box.min.x(), (i & 2) ? box.max.y() : box.min.y(),
        (i & 4) ? box.max.z() : box.min.z());
      const Eigen::Vector3d rel = corner - basis.origin;
      const double n = rel.dot(basis.normal_outward);
      const double u = rel.dot(basis.basis_u);
      const double v = rel.dot(basis.basis_v);
      n_min = std::min(n_min, n);
      n_max = std::max(n_max, n);
      u_min = std::min(u_min, u);
      u_max = std::max(u_max, u);
      v_min = std::min(v_min, v);
      v_max = std::max(v_max, v);
    }
    // Skip objects that are entirely in front of the working zone or entirely behind the wall.
    if (n_max < -behind_tolerance || n_min > max_standoff) {
      continue;
    }
    ObstacleRegion region;
    region.id = next_id++;
    region.parent_wall = wall.id;
    region.u_min = std::max(u_min, wall.u_min);
    region.u_max = std::min(u_max, wall.u_max);
    region.v_min = std::max(v_min, wall.v_min);
    region.v_max = std::min(v_max, wall.v_max);
    if (region.u_min < region.u_max && region.v_min < region.v_max) {
      regions.push_back(region);
    }
  }
  return regions;
}

std::vector<bool> observedMaskFromPoints(
  const std::vector<scene_model::Vec3> & points, const WallPlane & wall, int width, int height,
  double resolution)
{
  const WallBasis basis = decomposeWallBasis(wall);
  std::vector<bool> mask(static_cast<size_t>(width) * static_cast<size_t>(height), false);
  for (const auto & p : points) {
    const Eigen::Vector3d rel = p - basis.origin;
    const int x = static_cast<int>(std::floor((rel.dot(basis.basis_u) - wall.u_min) / resolution));
    const int y = static_cast<int>(std::floor((rel.dot(basis.basis_v) - wall.v_min) / resolution));
    if (x >= 0 && x < width && y >= 0 && y < height) {
      mask[static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)] = true;
    }
  }
  return mask;
}

ScanCoverageResult planCoverageOnWall(
  const WallPlane & wall, const std::vector<ObstacleRegion> & regions,
  const std::vector<scene_model::Vec3> * observed_points, const ToolCoverageProfile & tool,
  const ScanCoverageOptions & options)
{
  ScanCoverageResult out;
  out.wall = wall;

  // Grid: observed cells from the points (when given), obstacles from the regions.
  const double res = options.resolution;
  const int width = std::max(1, static_cast<int>(std::ceil((out.wall.u_max - out.wall.u_min) / res)));
  const int height = std::max(1, static_cast<int>(std::ceil((out.wall.v_max - out.wall.v_min) / res)));
  const std::vector<bool> observed = observed_points ?
    observedMaskFromPoints(*observed_points, out.wall, width, height, res) : std::vector<bool>{};
  try {
    out.grid = rasterizeWallCoverageGrid(
      out.wall, regions, tool.coverage, res, options.frame_id, observed);
  } catch (const std::exception & e) {
    out.message = std::string("rasterization failed: ") + e.what();
    return out;
  }

  // Plan on the wall plane (obstacle inflation is already part of the rasterized grid).
  CoveragePlannerParams params;
  params.swath = toolSwath(tool);
  params.step = toolLineSpacing(tool);
  params.sweep_direction = tool.coverage.sweep_direction;
  params.start_corner = options.start_corner;
  params.obstacle_inflation_radius = 0.0;
  params.allow_unknown = options.allow_unknown;
  params.enable_boundary_pass = tool.coverage.enable_boundary_pass;
  params.boundary_offset = tool.coverage.boundary_offset_m;
  params.add_strip_connectors = options.add_strip_connectors;
  params.pose_mode = "identity";
  params.map_frame = options.frame_id;
  params.use_region = false;
  params.plan_all_components = true;
  params.min_component_cells = 25;
  params.optimize_strip_order = false;
  params.requires_contact = tool.task.contact;
  params.ensure_coverage_within_boundary = tool.coverage.ensure_coverage_within_boundary;

  PatternOptions pattern = options.wall_pattern;
  pattern.planner = options.planner;
  out.pattern = planWallPattern(out.wall, out.grid, tool, pattern, params);
  if (!out.pattern.success) {
    out.message = "coverage planning failed: " + out.pattern.message;
    return out;
  }

  // Lift each part onto the wall with its own tool profile (panels use a turned tool),
  //    orient the tool, expand passes, and concatenate in execution order.
  out.final_plan.path.header.frame_id = options.frame_id;
  for (const auto & part : out.pattern.parts) {
    ExpandedCoverage piece = expandCoveragePasses(part.path, part.segments, out.wall, part.tool);
    const uint32_t offset = static_cast<uint32_t>(out.final_plan.path.poses.size());
    for (auto & segment : piece.segments) {
      segment.start_index += offset;
      segment.end_index += offset;
      segment.render_start_index += offset;
      segment.render_end_index += offset;
      segment.tags.push_back("part:" + part.name);
      out.final_plan.segments.push_back(std::move(segment));
    }
    out.final_plan.path.poses.insert(
      out.final_plan.path.poses.end(), piece.path.poses.begin(), piece.path.poses.end());
  }
  out.success = true;
  out.message = "ok";
  return out;
}

ScanCoverageResult planCoverageFromScan(
  const scene_model::PointCloud & scan, const scene_model::SceneModel & obstacles,
  const ToolCoverageProfile & tool, const ScanCoverageOptions & options)
{
  ScanCoverageResult out;

  // 1. Wall plane from the scan.
  const auto fit = options.label ?
    scene_model::fitPlaneForLabel(scan, *options.label, options.plane) :
    scene_model::fitPlane(scan.points, options.plane);
  if (!fit) {
    out.message = "no wall found: the scan has no plane with enough inliers";
    return out;
  }
  if (fit->u_max - fit->u_min <= 0.0 || fit->v_max - fit->v_min <= 0.0) {
    out.message = "fitted plane has no extent";
    return out;
  }
  const WallPlane wall = wallFromPlaneFit(*fit, options.wall_id, options.frame_id);

  // 2. Obstacles and observed points on that plane, then the shared planning chain.
  std::vector<scene_model::Vec3> wall_points;
  wall_points.reserve(fit->inliers.size());
  for (size_t i : fit->inliers) {
    wall_points.push_back(scan.points[i]);
  }
  return planCoverageOnWall(
    wall, obstacleRegionsFromScene(obstacles, wall), &wall_points, tool, options);
}

}  // namespace coverage_planner
