#pragma once

#include <octomap/OcTree.h>

#include "scene_model/core/types.hpp"

namespace wall_detector
{

/// Window of the distance field, in the octomap's own frame: centre and edge lengths (m), voxel size (m).
struct EsdfWindow
{
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  Eigen::Vector3d dims{2.0, 2.0, 2.0};
  double voxel_size{0.05};
};

/// Occupied leaves of an octomap rasterised onto the window (a leaf larger than a voxel fills every
/// voxel it covers; only leaves the octree itself calls occupied count), then scene_model's exact
/// Euclidean distance transform. Returns the signed field: metres, positive outside obstacles, negative
/// inside; space the octomap never saw counts as free.
scene_model::EsdfGrid octreeToEsdf(const octomap::OcTree & tree, const EsdfWindow & window);

}  // namespace wall_detector
