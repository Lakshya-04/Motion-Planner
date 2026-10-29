// File loaders. No PCL, no ROS: point clouds (.pcd, .ply), meshes (.obj, .stl) and scene
// descriptions (YAML / JSON). All throw std::runtime_error with the file name and the reason.
#pragma once

#include <string>

#include "scene_model/core/scene_model.hpp"
#include "scene_model/core/types.hpp"

namespace scene_model
{

/// ASCII and binary PCD (not binary_compressed). Reads x, y, z; also `rgb` / `rgba` (packed)
/// and `label` when present. NaN points (unorganised-cloud placeholders) are dropped.
PointCloud loadPcd(const std::string & path);

/// ASCII and binary_little_endian PLY. Reads x, y, z and red / green / blue when present.
PointCloud loadPly(const std::string & path);

/// Picks the loader from the extension (.pcd or .ply).
PointCloud loadPointCloud(const std::string & path);

/// Wavefront OBJ (v / f, with v/vt/vn forms, negative indices, polygons fan-triangulated).
Mesh loadObj(const std::string & path);

/// ASCII and binary STL. Triangle corners are merged when they coincide exactly.
Mesh loadStl(const std::string & path);

/// Picks the loader from the extension (.obj or .stl).
Mesh loadMesh(const std::string & path);

/// Writes an ASCII PCD (x y z, plus label when the cloud has labels). Mainly for tests and
/// for saving a filtered or fused cloud.
void savePcdAscii(const std::string & path, const PointCloud & cloud);

/// Loads a scene description. Two layouts are accepted:
///
///  1. Native:
///       frame_id: map
///       objects:
///         - id: table
///           type: box            # box | sphere | cylinder | capsule | mesh | pointcloud
///           size: [1.0, 0.6, 0.05]
///           pose: {position: [0, 0, 0.4], orientation_xyzw: [0, 0, 0, 1]}
///           role: obstacle       # obstacle | contact_surface
///           static: true
///
///  2. cuRobo-style world dict (name -> properties under cuboid / sphere / capsule /
///     cylinder / mesh), pose as [x, y, z, qw, qx, qy, qz]:
///       cuboid: {table: {dims: [1.0, 0.6, 0.05], pose: [0, 0, 0.4, 1, 0, 0, 0]}}
///
/// Mesh and point-cloud files are resolved relative to the description's directory.
SceneModel loadSceneFile(const std::string & path);

/// Same, from text (relative file references resolve against `base_dir`).
SceneModel loadSceneYaml(const std::string & yaml_text, const std::string & base_dir = "");

}  // namespace scene_model
