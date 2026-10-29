#pragma once

#include "scene_model/core/types.hpp"

namespace scene_model
{

/// Bins points into an occupancy grid covering their bounding box (plus `margin`). A voxel is
/// occupied when it contains at least `min_points` points. Throws if the grid would exceed
/// `max_voxels` (guards against a mistyped voxel size).
VoxelGrid voxelize(
  const PointCloud & cloud, double voxel_size, int min_points = 1, double margin = 0.0,
  size_t max_voxels = 200'000'000);

/// Centres of the occupied voxels (the inverse direction: a point cloud for backends that take
/// points).
PointCloud occupiedCentres(const VoxelGrid & grid);

/// How computeEsdf treats voxels marked kVoxelUnknown (same three choices as nvblox's
/// `unobserved_esdf_policy`, so maps from either source behave the same):
enum class UnobservedPolicy
{
  /// Unknown space counts as free; distances propagate through it. Optimistic: plans may route
  /// through space nobody has looked at.
  kFree,
  /// Unknown space counts as an obstacle. Conservative: plans stay inside observed free space.
  kOccupied,
  /// Distances are computed as for kFree, but the output at unknown voxels is NaN so a consumer
  /// can tell "unobserved" from a real distance (see isUnobserved()).
  kMarkUnobserved,
};

inline bool isUnobserved(float distance) {return distance != distance;}

/// Exact Euclidean signed distance field of an occupancy grid (Felzenszwalb separable distance
/// transform, O(n)). Positive in free space = distance to the nearest occupied voxel centre,
/// negative inside = minus the distance to the nearest free voxel centre; metres.
EsdfGrid computeEsdf(const VoxelGrid & grid, UnobservedPolicy policy = UnobservedPolicy::kFree);

}  // namespace scene_model
