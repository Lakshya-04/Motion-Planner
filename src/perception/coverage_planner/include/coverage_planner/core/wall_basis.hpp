#ifndef COVERAGE_PLANNER__WALL_BASIS_HPP_
#define COVERAGE_PLANNER__WALL_BASIS_HPP_

#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

#include "coverage_planner_msgs/msg/wall_plane.hpp"

namespace coverage_planner
{

// The interfaces live in coverage_planner_msgs; alias them into this package's own
// namespace so `msg::`/`srv::` still mean what they always did here.
namespace msg = coverage_planner_msgs::msg;

/// A wall's own origin + orthonormal (u, v, normal) basis, decomposed out of WallPlane.pose.
struct WallBasis
{
  Eigen::Vector3d origin;
  Eigen::Vector3d basis_u;
  Eigen::Vector3d basis_v;
  Eigen::Vector3d normal_outward;
};

/**
 * @brief Decomposes WallPlane.pose.pose (position + quaternion) back into origin + the (basis_u,
 *        basis_v, normal_outward) rotation-matrix columns it was built from -- see
 *        encodeWallBasis() for the inverse.
 */
inline WallBasis decomposeWallBasis(const msg::WallPlane & wall)
{
  const auto & pose = wall.pose.pose;
  const Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

  WallBasis basis;
  basis.origin = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  basis.basis_u = rotation.col(0);
  basis.basis_v = rotation.col(1);
  basis.normal_outward = rotation.col(2);
  return basis;
}

/**
 * @brief Encodes an origin + orthonormal (u, v, normal) basis into a plain Pose -- the inverse of
 *        decomposeWallBasis(). Returns a bare Pose (not WallPlane.pose's own PoseStamped) since
 *        this is also used for OccupancyGrid.info.origin, which has no frame_id of its own.
 */
inline geometry_msgs::msg::Pose encodeWallBasis(
  const Eigen::Vector3d & origin, const Eigen::Vector3d & basis_u, const Eigen::Vector3d & basis_v,
  const Eigen::Vector3d & normal_outward)
{
  // Eigen::Quaterniond(Matrix3d) assumes a proper (orthonormal, right-handed) rotation and
  // silently produces a meaningless result otherwise -- catch a malformed basis (e.g. a flipped
  // normal from a perception export) here rather than aiming the tool at the wrong side of the
  // wall with no error raised anywhere downstream.
  constexpr double kOrthonormalTolerance = 1e-3;
  if (
    std::abs(basis_u.norm() - 1.0) > kOrthonormalTolerance ||
    std::abs(basis_v.norm() - 1.0) > kOrthonormalTolerance ||
    std::abs(normal_outward.norm() - 1.0) > kOrthonormalTolerance ||
    std::abs(basis_u.dot(basis_v)) > kOrthonormalTolerance ||
    std::abs(basis_u.dot(normal_outward)) > kOrthonormalTolerance ||
    std::abs(basis_v.dot(normal_outward)) > kOrthonormalTolerance ||
    basis_u.cross(basis_v).dot(normal_outward) <= 0.0) {
    throw std::runtime_error(
      "encodeWallBasis: basis_u/basis_v/normal_outward is not an orthonormal, right-handed "
      "basis");
  }

  Eigen::Matrix3d rotation;
  rotation.col(0) = basis_u;
  rotation.col(1) = basis_v;
  rotation.col(2) = normal_outward;
  Eigen::Quaterniond quaternion(rotation);
  quaternion.normalize();

  geometry_msgs::msg::Pose pose;
  pose.position.x = origin.x();
  pose.position.y = origin.y();
  pose.position.z = origin.z();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__WALL_BASIS_HPP_
