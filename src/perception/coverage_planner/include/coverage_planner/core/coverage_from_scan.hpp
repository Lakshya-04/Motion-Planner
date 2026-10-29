// Raw scan -> coverage plan, without a hand-written wall description.
//
//   point cloud --fitPlane--> wall plane + extents
//   obstacle objects (SceneModel) --project--------> grid --boustrophedon--> flat path
//   --lift + pass expansion--> oriented tool poses
//
// Pure core (scene_model + coverage_planner core): no ROS, no PCL.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "coverage_planner/core/coverage_pass_expander.hpp"
#include "coverage_planner/core/coverage_patterns.hpp"
#include "coverage_planner/core/coverage_planner_base.hpp"
#include "coverage_planner/core/tool_coverage_profile.hpp"
#include "coverage_planner/core/types.hpp"
#include "scene_model/core/plane_fit.hpp"
#include "scene_model/core/scene_model.hpp"
#include "scene_model/core/types.hpp"

namespace coverage_planner
{

/// Wall description from a plane fit: pose = the plane frame (x = u, y = v, z = outward normal),
/// extents and flatness metrics (mm) copied over.
WallPlane wallFromPlaneFit(const scene_model::PlaneFit & fit, uint32_t id, const std::string & frame_id);

/// Obstacle footprints on the wall: every `kObstacle` object whose bounds come within
/// `max_standoff` in front of the wall (or up to `behind_tolerance` behind it) is projected into
/// the wall's (u, v) and clipped to its extents. Conservative: uses axis-aligned bounds.
std::vector<ObstacleRegion> obstacleRegionsFromScene(
  const scene_model::SceneModel & scene, const WallPlane & wall, double max_standoff = 0.6,
  double behind_tolerance = 0.05);

/// Row-major (v * width + u) mask of grid cells that contain at least one of `points`.
std::vector<bool> observedMaskFromPoints(
  const std::vector<scene_model::Vec3> & points, const WallPlane & wall, int width, int height,
  double resolution);

struct ScanCoverageOptions
{
  double resolution{0.05};  ///< Grid cell size (m).
  uint32_t wall_id{0};
  std::string frame_id{"map"};
  /// Fit only the points with this label (e.g. one wall of a segmented scan); empty = all points.
  std::optional<uint32_t> label;
  scene_model::PlaneFitParams plane;
  bool add_strip_connectors{false};
  bool allow_unknown{false};
  std::string start_corner{"bottom_left"};
  /// Grid planner underneath the pattern: "boustrophedon" or "sweepline".
  std::string planner{"boustrophedon"};
  /// Wall-level pattern (vertical, horizontal, zigzag) and its panel / bottom-pass settings.
  PatternOptions wall_pattern;
};

struct ScanCoverageResult
{
  bool success{false};
  std::string message;
  WallPlane wall;
  OccupancyGrid grid;          ///< The rasterized coverage grid (obstacles and unobserved cells set).
  PatternPlan pattern;         ///< The plan on the wall plane, before lifting: parts and panels.
  ExpandedCoverage final_plan; ///< Lifted, oriented, pass-expanded tool poses.
};

/// Whole pipeline. `obstacles` may be empty. Never throws for "no wall found" or "planning
/// failed": the reason is in `message`.
ScanCoverageResult planCoverageFromScan(
  const scene_model::PointCloud & scan, const scene_model::SceneModel & obstacles,
  const ToolCoverageProfile & tool, const ScanCoverageOptions & options = {});

}  // namespace coverage_planner
