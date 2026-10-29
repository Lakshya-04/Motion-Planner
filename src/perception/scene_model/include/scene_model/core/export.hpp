#pragma once

#include <string>

#include "scene_model/core/scene_model.hpp"

namespace scene_model
{

/// cuRobo world dict as YAML text (`cuboid` / `sphere` / `capsule` / `cylinder` / `mesh`, poses
/// as [x, y, z, qw, qx, qy, qz]) for `curobo.scene.Scene.create(...)`. Meshes that came from a
/// file are referenced by `file_path`; others carry inline vertices and faces. Point clouds,
/// voxel grids and ESDF grids are not expressible as a cuRobo primitive: they are listed in
/// `skipped` (when non-null) so the caller can route them through a voxelizer / ESDF mapper.
std::string toCuroboWorldYaml(const SceneModel & scene, std::vector<std::string> * skipped = nullptr);

}  // namespace scene_model
