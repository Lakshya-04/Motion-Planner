/**
 * @file frame_transform.hpp
 * @brief Rigid transform of poses and of the wall direction between two frames (ROS-free).
 * @details The manager can take its coverage poses in a fixed world frame and express them in the
 *          planning frame (base_footprint) when each column starts, so the plan stays correct
 *          wherever the base stopped. The transform is the TF lookup `planning <- world`.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <array>
#include <cmath>

#include "core_types/pose.hpp"
#include "manipulation_manager/core/wall_direction.hpp"

namespace manipulation_manager
{

/// Rigid transform: a point p in the source frame is `rotate(q, p) + t` in the target frame.
struct FrameTransform
{
  std::array<double, 3> t{0.0, 0.0, 0.0};
  std::array<double, 4> q{0.0, 0.0, 0.0, 1.0};  ///< Unit quaternion (x, y, z, w).

  /// Rotation about z by `yaw` and translation (x, y, z).
  static FrameTransform fromYaw(double x, double y, double z, double yaw)
  {
    return FrameTransform{{x, y, z}, {0.0, 0.0, std::sin(0.5 * yaw), std::cos(0.5 * yaw)}};
  }

  /// The transform that undoes this one.
  FrameTransform inverse() const
  {
    const std::array<double, 4> qi{-q[0], -q[1], -q[2], q[3]};
    const auto r = rotateBy(qi, t);
    return FrameTransform{{-r[0], -r[1], -r[2]}, qi};
  }

  core_types::Pose apply(const core_types::Pose & p) const
  {
    const auto pos = rotateBy(q, {p.x, p.y, p.z});
    // q_out = q * q_pose (Hamilton product, x y z w)
    const double ax = q[0], ay = q[1], az = q[2], aw = q[3];
    const double bx = p.qx, by = p.qy, bz = p.qz, bw = p.qw;
    core_types::Pose out;
    out.x = pos[0] + t[0];
    out.y = pos[1] + t[1];
    out.z = pos[2] + t[2];
    out.qw = aw * bw - ax * bx - ay * by - az * bz;
    out.qx = aw * bx + ax * bw + ay * bz - az * by;
    out.qy = aw * by - ax * bz + ay * bw + az * bx;
    out.qz = aw * bz + ax * by - ay * bx + az * bw;
    return out;
  }

  /// The wall direction rotated into the target frame (no translation).
  WallDirection apply(const WallDirection & w) const
  {
    const auto d = rotateBy(q, w.d);
    return WallDirection::from(d[0], d[1], d[2]);
  }

  /// Yaw of the rotation (about z), for a mostly planar transform.
  double yaw() const
  {
    return std::atan2(2.0 * (q[3] * q[2] + q[0] * q[1]), 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
  }

private:
  static std::array<double, 3> rotateBy(const std::array<double, 4> & qq, const std::array<double, 3> & v)
  {
    // v' = v + 2 w (u x v) + 2 u x (u x v), u = (x, y, z)
    const double ux = qq[0], uy = qq[1], uz = qq[2], w = qq[3];
    const double cx = uy * v[2] - uz * v[1];
    const double cy = uz * v[0] - ux * v[2];
    const double cz = ux * v[1] - uy * v[0];
    return {
      v[0] + 2.0 * (w * cx + uy * cz - uz * cy),
      v[1] + 2.0 * (w * cy + uz * cx - ux * cz),
      v[2] + 2.0 * (w * cz + ux * cy - uy * cx)};
  }
};

/// Every pose of an intent, moved into the target frame.
template<typename Intent>
Intent transformIntent(const FrameTransform & tf, Intent intent)
{
  for (auto & p : intent.poses) {
    p = tf.apply(p);
  }
  return intent;
}

}  // namespace manipulation_manager
