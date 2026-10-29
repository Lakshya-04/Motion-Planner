#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "scene_model/core/types.hpp"

namespace scene_model
{

struct PlaneFitParams
{
  int ransac_iterations{300};
  double inlier_threshold{0.02};  ///< Max point-to-plane distance counted as an inlier (m).
  size_t min_inliers{50};
  uint32_t seed{1};
  /// Direction the normal should point toward (e.g. toward the robot). Zero = unconstrained
  /// (then the sign follows the +z-up convention below).
  Vec3 normal_hint{Vec3::Zero()};
  /// Used to pick a horizontal in-plane u axis: u = up x normal. For a vertical wall that is
  /// the horizontal direction along the wall, so v points up.
  Vec3 up{Vec3::UnitZ()};
};

/// A fitted plane with the in-plane frame a coverage planner works in: x = u (along the wall),
/// y = v (up the wall), z = normal; right-handed (u x v = normal).
struct PlaneFit
{
  Vec3 origin{Vec3::Zero()};  ///< Centroid of the inliers.
  Vec3 u{Vec3::UnitX()};
  Vec3 v{Vec3::UnitY()};
  Vec3 normal{Vec3::UnitZ()};
  double u_min{0}, u_max{0}, v_min{0}, v_max{0};  ///< Inlier extents in (u, v) about `origin`.
  double rms{0};      ///< RMS point-to-plane distance of the inliers (m).
  double spread_p95{0};  ///< 95th-percentile absolute distance (m).
  double spread_p99{0};
  std::vector<size_t> inliers;  ///< Indices into the input cloud.

  /// Pose of the plane frame in the cloud frame.
  Transform pose() const;
};

/// RANSAC plane fit refined by a least-squares (PCA) fit on the inliers. Returns nullopt when
/// no plane with at least `min_inliers` inliers exists.
std::optional<PlaneFit> fitPlane(const std::vector<Vec3> & points, const PlaneFitParams & params = {});

/// Fit a plane to the points of one label (e.g. one wall of a segmented cloud).
std::optional<PlaneFit> fitPlaneForLabel(
  const PointCloud & cloud, uint32_t label, const PlaneFitParams & params = {});

}  // namespace scene_model
