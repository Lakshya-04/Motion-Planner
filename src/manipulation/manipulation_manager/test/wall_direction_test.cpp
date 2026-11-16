#include <cmath>
#include <array>
#include <vector>

#include <gtest/gtest.h>

#include "manipulation_manager/core/wall_direction.hpp"

namespace manipulation_manager
{
namespace
{
// Rotate v by unit quaternion (x, y, z, w).
std::array<double, 3> rotate(const std::array<double, 4> & q, std::array<double, 3> v)
{
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  const double tx = 2 * (y * v[2] - z * v[1]);
  const double ty = 2 * (z * v[0] - x * v[2]);
  const double tz = 2 * (x * v[1] - y * v[0]);
  return {v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz),
    v[2] + w * tz + (x * ty - y * tx)};
}
}  // namespace

TEST(WallDirectionTest, DefaultIsTheOriginalPlusYWallAndKeepsItsQuaternion)
{
  const WallDirection w;
  const auto q = w.outwardNormalQuaternion();
  EXPECT_NEAR(q[0], std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(q[1], 0.0, 1e-12);
  EXPECT_NEAR(q[2], 0.0, 1e-12);
  EXPECT_NEAR(q[3], std::sqrt(0.5), 1e-12);
}

TEST(WallDirectionTest, LocalZIsTheOutwardNormalForAnyDirection)
{
  for (const auto & dir : std::vector<std::array<double, 3>>{
      {1, 0, 0}, {-1, 0, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}, {1, 1, 0}, {0.3, -0.5, 0.2}})
  {
    const auto w = WallDirection::from(dir[0], dir[1], dir[2]);
    const auto q = w.outwardNormalQuaternion();
    EXPECT_NEAR(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3], 1.0, 1e-12);
    const auto z = rotate(q, {0, 0, 1});
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(z[i], -w.d[i], 1e-12) << "dir " << dir[0] << "," << dir[1] << "," << dir[2];
    }
  }
}

TEST(WallDirectionTest, ShiftsTowardsAndAwayFromTheWall)
{
  const auto w = WallDirection::from(2, 0, 0);  // normalised to +x
  core_types::Pose p;
  p.x = 1.0; p.y = 0.2; p.z = 0.5;
  const auto toward = w.shifted(p, 0.1);
  EXPECT_NEAR(toward.x, 1.1, 1e-12);
  EXPECT_NEAR(toward.y, 0.2, 1e-12);
  const auto away = w.shifted(p, -0.12);
  EXPECT_NEAR(away.x, 0.88, 1e-12);
}

TEST(WallDirectionTest, ZeroVectorThrows)
{
  EXPECT_THROW(WallDirection::from(0, 0, 0), std::invalid_argument);
}


TEST(WallDirectionTest, ApproachPoseBacksOffAlongTheWallNormalAndKeepsTheOrientation)
{
  const auto w = WallDirection::from(1.0, 1.0, 0.0);
  core_types::Pose dock;
  dock.x = 0.9; dock.y = -0.3; dock.z = 1.1;
  dock.qx = 0.1; dock.qy = 0.2; dock.qz = 0.3; dock.qw = 0.9;
  const auto a = w.approachPose(dock, 0.1);
  const double s = std::sqrt(0.5) * 0.1;
  EXPECT_NEAR(a.x, 0.9 - s, 1e-12);
  EXPECT_NEAR(a.y, -0.3 - s, 1e-12);
  EXPECT_NEAR(a.z, 1.1, 1e-12);
  EXPECT_DOUBLE_EQ(a.qx, 0.1);
  EXPECT_DOUBLE_EQ(a.qw, 0.9);
  // The sign of the distance does not matter: the approach pose is always on the robot's side.
  const auto b = w.approachPose(dock, -0.1);
  EXPECT_NEAR(b.x, a.x, 1e-12);
  EXPECT_NEAR(b.y, a.y, 1e-12);
  // Zero extra is the dock pose itself.
  const auto c = w.approachPose(dock, 0.0);
  EXPECT_NEAR(c.x, dock.x, 1e-12);
}

}  // namespace manipulation_manager
