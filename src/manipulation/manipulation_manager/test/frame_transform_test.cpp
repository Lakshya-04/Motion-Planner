#include <cmath>

#include <gtest/gtest.h>

#include "manipulation_manager/core/coverage_source.hpp"
#include "manipulation_manager/core/frame_transform.hpp"

namespace manipulation_manager
{
namespace
{
core_types::Pose pose(double x, double y, double z, double yaw)
{
  core_types::Pose p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.qz = std::sin(0.5 * yaw);
  p.qw = std::cos(0.5 * yaw);
  return p;
}
}  // namespace

TEST(FrameTransformTest, IdentityLeavesPosesAndDirectionAlone)
{
  const FrameTransform id;
  const auto p = id.apply(pose(0.98, 0.1, 1.2, 0.4));
  EXPECT_DOUBLE_EQ(p.x, 0.98);
  EXPECT_DOUBLE_EQ(p.y, 0.1);
  EXPECT_DOUBLE_EQ(p.z, 1.2);
  EXPECT_NEAR(p.qz, std::sin(0.2), 1e-12);
  const auto d = id.apply(WallDirection::from(1, 0, 0));
  EXPECT_NEAR(d.d[0], 1.0, 1e-12);
}

TEST(FrameTransformTest, TranslationOnlyShiftsPositions)
{
  // base 0.4 m back and 0.3 m right of the world origin, facing the same way: world -> base
  const auto world_in_base = FrameTransform::fromYaw(0.4, 0.3, 0.0, 0.0);
  const auto p = world_in_base.apply(pose(0.98, 0.0, 1.2, 0.0));
  EXPECT_NEAR(p.x, 1.38, 1e-12);
  EXPECT_NEAR(p.y, 0.3, 1e-12);
  EXPECT_NEAR(p.z, 1.2, 1e-12);
}

TEST(FrameTransformTest, YawRotatesPositionOrientationAndWallDirection)
{
  // The base at (1, 2) turned +90 degrees in the world: base_in_world. world_in_base is its inverse.
  const auto base_in_world = FrameTransform::fromYaw(1.0, 2.0, 0.0, M_PI / 2);
  const auto world_in_base = base_in_world.inverse();
  // A wall point 1 m ahead of the base's heading in the world: (1, 3)
  const auto p = world_in_base.apply(pose(1.0, 3.0, 1.0, M_PI / 2));
  EXPECT_NEAR(p.x, 1.0, 1e-12);
  EXPECT_NEAR(p.y, 0.0, 1e-12);
  EXPECT_NEAR(std::atan2(2 * p.qw * p.qz, 1 - 2 * p.qz * p.qz), 0.0, 1e-12);  // yaw 90 - 90 = 0
  // A wall at world +y is straight ahead (+x) for that base.
  const auto d = world_in_base.apply(WallDirection::from(0, 1, 0));
  EXPECT_NEAR(d.d[0], 1.0, 1e-12);
  EXPECT_NEAR(d.d[1], 0.0, 1e-12);
  EXPECT_NEAR(world_in_base.yaw(), -M_PI / 2, 1e-12);
}

TEST(FrameTransformTest, InverseRoundTrips)
{
  const auto tf = FrameTransform::fromYaw(-0.12, -0.25, 0.05, 1.57);
  const auto orig = pose(0.98, 0.17, 1.15, 0.3);
  const auto back = tf.inverse().apply(tf.apply(orig));
  EXPECT_NEAR(back.x, orig.x, 1e-12);
  EXPECT_NEAR(back.y, orig.y, 1e-12);
  EXPECT_NEAR(back.z, orig.z, 1e-12);
  EXPECT_NEAR(back.qz, orig.qz, 1e-12);
  EXPECT_NEAR(back.qw, orig.qw, 1e-12);
}

TEST(FrameTransformTest, TransformIntentMovesEveryPoseAndKeepsTheRest)
{
  CoverageSegmentIntent intent;
  intent.label = "col0";
  intent.target_eef_speed = 0.01;
  intent.poses = {pose(0.98, 0.0, 1.0, 0.0), pose(0.98, 0.0, 1.3, 0.0)};
  const auto moved = transformIntent(FrameTransform::fromYaw(0.5, 0.0, 0.0, 0.0), intent);
  EXPECT_EQ(moved.label, "col0");
  EXPECT_DOUBLE_EQ(moved.target_eef_speed, 0.01);
  ASSERT_EQ(moved.poses.size(), 2u);
  EXPECT_NEAR(moved.poses[0].x, 1.48, 1e-12);
  EXPECT_NEAR(moved.poses[1].z, 1.3, 1e-12);
  EXPECT_DOUBLE_EQ(intent.poses[0].x, 0.98);  // the original is untouched
}

}  // namespace manipulation_manager
