/**
 * @file distance_source_test.cpp
 * @brief Distance queries at fixed poses with known answers.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#include <gtest/gtest.h>

#include "safety_monitor/distance_source.hpp"

using namespace safety_monitor;

TEST(StaticSource, SphereDistanceAndGradient)
{
  StaticPrimitiveSource s;
  s.addSphere(Vec3(1, 0, 0), 0.5);
  const auto q = s.query(Vec3(3, 0, 0), 0.0);
  ASSERT_TRUE(q.valid);
  EXPECT_NEAR(q.distance, 1.5, 1e-12);
  EXPECT_NEAR((q.gradient - Vec3::UnitX()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(s.query(Vec3(1, 0, 0), 0.0).distance, -0.5, 1e-12);  // inside: negative
}

TEST(StaticSource, BoxOutsideFaceEdgeCorner)
{
  StaticPrimitiveSource s;
  s.addBox(Eigen::Isometry3d::Identity(), Vec3(1, 1, 1));
  EXPECT_NEAR(s.query(Vec3(3, 0, 0), 0).distance, 2.0, 1e-12);            // face
  EXPECT_NEAR(s.query(Vec3(2, 2, 0), 0).distance, std::sqrt(2.0), 1e-12);  // edge
  EXPECT_NEAR(s.query(Vec3(2, 2, 2), 0).distance, std::sqrt(3.0), 1e-12);  // corner
  const auto q = s.query(Vec3(0, 0, -4), 0);
  EXPECT_NEAR((q.gradient + Vec3::UnitZ()).norm(), 0.0, 1e-12);
}

TEST(StaticSource, BoxInsideAndRotated)
{
  StaticPrimitiveSource s;
  s.addBox(Eigen::Translation3d(0, 0, 0) * Eigen::AngleAxisd(M_PI / 2, Vec3::UnitZ()),
    Vec3(2, 0.5, 1));  // after a yaw the long axis points along world y
  EXPECT_NEAR(s.query(Vec3(0, 3, 0), 0).distance, 1.0, 1e-12);
  EXPECT_NEAR(s.query(Vec3(3, 0, 0), 0).distance, 2.5, 1e-12);
  EXPECT_NEAR(s.query(Vec3(0.1, 0, 0), 0).distance, -0.4, 1e-12);  // inside, nearest face x=+-0.5
}

TEST(StaticSource, PlaneSignedDistance)
{
  StaticPrimitiveSource s(true);
  s.addPlane(Vec3(0, 0, 2), 0.0);  // un-normalised input; solid below z = 0
  EXPECT_NEAR(s.query(Vec3(5, 5, 0.3), 0).distance, 0.3, 1e-12);
  EXPECT_NEAR(s.query(Vec3(0, 0, -0.2), 0).distance, -0.2, 1e-12);
  EXPECT_TRUE(s.isWorkSurface());
}

TEST(StaticSource, MinimumOverPrimitivesAndEmpty)
{
  StaticPrimitiveSource s;
  EXPECT_FALSE(s.query(Vec3::Zero(), 0).valid);
  s.addSphere(Vec3(10, 0, 0), 1.0);
  s.addSphere(Vec3(0, 2, 0), 0.5);
  EXPECT_NEAR(s.query(Vec3::Zero(), 0).distance, 1.5, 1e-12);
}

TEST(EsdfGrid, TrilinearInterpolatesALinearFieldExactly)
{
  EsdfGrid g(Vec3(0, 0, 0), 0.1, 11, 11, 11);
  g.fill([](const Vec3 & c) {return c.x() - 0.5 + 0.0 * c.y();});  // plane x = 0.5
  const auto q = g.query(Vec3(0.234, 0.5, 0.77), 0);
  ASSERT_TRUE(q.valid);
  EXPECT_NEAR(q.distance, 0.234 - 0.5, 1e-6);
  EXPECT_NEAR((q.gradient - Vec3::UnitX()).norm(), 0.0, 1e-6);
}

TEST(EsdfGrid, MatchesSphereSdfNearVoxelCentres)
{
  EsdfGrid g(Vec3(-1, -1, -1), 0.05, 41, 41, 41);
  g.fill([](const Vec3 & c) {return c.norm() - 0.3;});
  const auto q = g.query(Vec3(0.5, 0.0, 0.0), 0);
  ASSERT_TRUE(q.valid);
  EXPECT_NEAR(q.distance, 0.2, 1e-3);
  EXPECT_NEAR(q.gradient.x(), 1.0, 5e-3);
  // exactly on a voxel centre the lookup returns the stored value
  EXPECT_NEAR(g.query(g.center(30, 20, 20), 0).distance, g.center(30, 20, 20).norm() - 0.3, 1e-6);
}

TEST(EsdfGrid, OutsideGridAndEmptyAreInvalid)
{
  EsdfGrid g(Vec3(0, 0, 0), 0.1, 5, 5, 5);
  EXPECT_FALSE(g.query(Vec3(-0.01, 0.2, 0.2), 0).valid);
  EXPECT_FALSE(g.query(Vec3(0.2, 0.2, 0.41), 0).valid);
  EXPECT_TRUE(g.query(Vec3(0.4, 0.4, 0.4), 0).valid);  // upper corner centre is inside
  EXPECT_FALSE(EsdfGrid().query(Vec3::Zero(), 0).valid);
}

TEST(CapsuleSource, DistanceToSegmentAndCaps)
{
  MovingCapsuleSource s;
  s.update({{Vec3(0, 0, 0), Vec3(0, 0, 1.8), 0.2, Vec3::Zero()}}, 1.0);
  EXPECT_NEAR(s.query(Vec3(1, 0, 0.9), 0).distance, 0.8, 1e-12);    // side
  EXPECT_NEAR(s.query(Vec3(0, 0, 2.5), 0).distance, 0.5, 1e-12);    // above the top cap
  EXPECT_EQ(s.kind(), SourceKind::kPerson);
  EXPECT_DOUBLE_EQ(s.stamp(), 1.0);
}

TEST(CapsuleSource, PredictsAlongVelocityAndReportsIt)
{
  MovingCapsuleSource s;
  s.update({{Vec3(0, 0, 0), Vec3(0, 0, 1), 0.1, Vec3(1.0, 0, 0)}}, 0.0);
  const auto now = s.query(Vec3(2, 0, 0.5), 0.0);
  const auto later = s.query(Vec3(2, 0, 0.5), 0.5);
  EXPECT_NEAR(now.distance, 1.9, 1e-12);
  EXPECT_NEAR(later.distance, 1.4, 1e-12);
  EXPECT_NEAR(later.velocity.x(), 1.0, 1e-12);
}

TEST(CapsuleSource, NeverUpdatedHasInvalidStamp)
{
  MovingCapsuleSource s;
  EXPECT_LT(s.stamp(), 0.0);
  EXPECT_FALSE(s.query(Vec3::Zero(), 0).valid);
}
