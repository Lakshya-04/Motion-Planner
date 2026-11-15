// SceneModel -> motion_planner::WorldGeometryUpdate.
//
// The planner ports take two kinds of geometry and a contact segment may only ever exclude one of
// them from collision checking (see motion_planner/core/scene_port.hpp):
//   points  one blob (an octomap in MoveIt): the contact surface, e.g. the wall being worked;
//   boxes   discrete named obstacles: never excludable, always a real hazard.
// So the split follows the object's Role, not its shape: contact-surface objects become points,
// obstacles become boxes. Anything that is not a box is approximated conservatively (it only ever
// grows) and reported, so a caller can see what was changed.
#pragma once

#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include "motion_planner/core/scene_port.hpp"
#include "scene_model/core/scene_model.hpp"

namespace scene_adapters
{

struct WorldGeometryOptions
{
  double surface_spacing{0.05};         ///< Sampling spacing for contact-surface points (m).
  double obstacle_voxel{0.05};          ///< Cube size for cloud / mesh / voxel obstacles (m).
  size_t max_obstacle_boxes{2000};      ///< Cap on boxes made from one object (at least 8); the cube grows to fit.
  bool unknown_voxels_occupied{false};  ///< Treat kVoxelUnknown cells as occupied.
  /// Pose of another frame in the scene frame (scene_T_frame), keyed by frame id. An object whose
  /// own frame_id is set and differs from the scene frame is transformed with its entry; without
  /// one it is skipped and reported, never placed as if it were in the scene frame.
  std::map<std::string, scene_model::Transform> frame_to_scene;
};

struct ConversionReport
{
  size_t boxes{0};
  size_t points{0};
  std::vector<std::string> approximated;  ///< "<id>: sphere -> cube", ... (always conservative)
  std::vector<std::string> skipped;       ///< Objects that cannot be expressed (e.g. an ESDF grid).
  std::vector<std::string> coarsened;     ///< Objects whose obstacle cubes grew to respect the cap.
};

/// An object with an empty frame_id (or the scene's own) is in the scene frame. One in another
/// frame is moved into the scene frame with `options.frame_to_scene`, or skipped with an entry in
/// `report->skipped` when no transform is given.
motion_planner::WorldGeometryUpdate toWorldGeometry(
  const scene_model::SceneModel & scene, const WorldGeometryOptions & options = {},
  ConversionReport * report = nullptr);

}  // namespace scene_adapters
