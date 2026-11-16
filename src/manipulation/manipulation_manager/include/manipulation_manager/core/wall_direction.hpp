#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

#include "core_types/pose.hpp"

namespace manipulation_manager
{

/// Which way the wall lies from the robot, as a unit vector in the planning frame. The default
/// (0, 1, 0) is the layout the manager was first written for (wall on the +y side); any direction
/// works, so a wall at +x (the MuJoCo and Gazebo scenes) or at an angle needs no code change.
struct WallDirection
{
  std::array<double, 3> d{0.0, 1.0, 0.0};

  /// Normalises; throws std::invalid_argument for a (near) zero vector.
  static WallDirection from(double x, double y, double z)
  {
    const double n = std::sqrt(x * x + y * y + z * z);
    if (n < 1e-9) {
      throw std::invalid_argument("wall direction must not be the zero vector");
    }
    return WallDirection{{x / n, y / n, z / n}};
  }

  /// `p` moved `distance` metres towards the wall (negative: away from it).
  core_types::Pose shifted(core_types::Pose p, double distance) const
  {
    p.x += d[0] * distance;
    p.y += d[1] * distance;
    p.z += d[2] * distance;
    return p;
  }

  /// The pose `extra_m` short of `dock_pose` (further from the wall, same orientation): where a free-space
  /// move can stop clear of the wall before a short straight move takes the tool to `dock_pose`. A free-space
  /// plan to a pose a few cm from the wall tends to end with the tool mount clipping the wall voxels.
  core_types::Pose approachPose(const core_types::Pose & dock_pose, double extra_m) const
  {
    return shifted(dock_pose, -std::fabs(extra_m));
  }

  /// Unit quaternion (x, y, z, w) whose local Z axis is the wall's outward normal (-d): the shortest
  /// rotation from +Z to -d. For d = +y this is (sqrt(.5), 0, 0, sqrt(.5)), the constant the manager
  /// used before the direction was configurable.
  std::array<double, 4> outwardNormalQuaternion() const
  {
    const double nx = -d[0], ny = -d[1], nz = -d[2];
    // rotation taking (0,0,1) to n: axis = z x n = (-ny, nx, 0), cos = nz
    const double c = nz;
    if (c < -1.0 + 1e-9) {  // n = -z: half turn about x
      return {1.0, 0.0, 0.0, 0.0};
    }
    // q = (axis, 1 + c) normalised (half-angle form)
    double qx = -ny, qy = nx, qz = 0.0, qw = 1.0 + c;
    const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    return {qx / n, qy / n, qz / n, qw / n};
  }
};

}  // namespace manipulation_manager
