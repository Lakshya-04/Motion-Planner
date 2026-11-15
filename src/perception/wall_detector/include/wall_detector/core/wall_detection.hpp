// Wall detection from a (live) point cloud: plane fit + the wall_scene.json the coverage planner reads.
//
//   points (in the target frame) --scene_model::fitPlane (RANSAC + PCA)--> plane + extents
//   --> wall_scene.json (schema of coverage_planner test/data/wall_scene.json) + walls.pcd (inliers,
//       label = wall id), which wall_coverage_service_node and the manager's coverage path read.
//
// ROS-free, no PCL: reuses scene_model's plane fit; swap in PCL multi-plane segmentation behind
// the same function when several walls per scan are needed (docs/perception_live.md).
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "scene_model/core/plane_fit.hpp"
#include "scene_model/core/types.hpp"

namespace wall_detector
{

struct WallDetectionParams
{
  scene_model::PlaneFitParams plane;
  /// Points farther than this from the viewpoint are ignored (0 = no limit).
  double max_range_m{0.0};
  /// Reject a fit whose inliers are fewer than this fraction of the points.
  double min_inlier_fraction{0.3};
};

struct WallDetection
{
  scene_model::PlaneFit fit;
  double inlier_fraction{0.0};
  size_t num_points{0};
  std::vector<scene_model::Vec3> inlier_points;  ///< The inliers (what walls.pcd holds).
};

/// Dominant plane of `points`. `viewpoint` (the sensor origin, same frame) orients the normal
/// toward the sensor (the wall's outward normal faces the robot) and applies `max_range_m`.
/// Returns nullopt when there is no plane with enough inliers.
std::optional<WallDetection> detectWall(
  const std::vector<scene_model::Vec3> & points, const scene_model::Vec3 & viewpoint,
  const WallDetectionParams & params = {});

std::vector<scene_model::Vec3> transformPoints(
  const std::vector<scene_model::Vec3> & points, const scene_model::Transform & transform);

/// The wall as the coverage planner's wall_scene.json (one wall, no obstacles).
/// plane_abcd = (n, -n . origin); origin / basis / extents / flatness (mm) from the fit.
nlohmann::json wallSceneJson(
  const scene_model::PlaneFit & fit, uint32_t wall_id, const std::string & frame_id,
  const std::string & source);

/// Writes `dir/wall_scene.json` and `dir/walls.pcd` (the inlier points, label = wall id), each
/// through a temporary file and a rename so a reader never sees half a file. Returns the json path.
std::string writeWallScene(
  const std::string & dir, const scene_model::PlaneFit & fit,
  const std::vector<scene_model::Vec3> & points, uint32_t wall_id, const std::string & frame_id,
  const std::string & source);

}  // namespace wall_detector
