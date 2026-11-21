#include <gtest/gtest.h>

#include <cmath>

#include "diff_ik_core/pose_error.hpp"

using namespace diff_ik_core;

namespace
{
Eigen::Matrix3d rotZ(double a) {return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()).toRotationMatrix();}
}  // namespace

TEST(RotationLog, IdentityIsZero)
{
  EXPECT_LT(rotationLog(Eigen::Matrix3d::Identity()).norm(), 1e-12);
}

TEST(RotationLog, AxisAngleAndSign)
{
  const Eigen::Vector3d axis = Eigen::Vector3d(1, 2, -2).normalized();
  const Eigen::Vector3d w = rotationLog(Eigen::AngleAxisd(0.7, axis).toRotationMatrix());
  EXPECT_LT((w - 0.7 * axis).norm(), 1e-12);
  const Eigen::Vector3d wn = rotationLog(Eigen::AngleAxisd(-0.7, axis).toRotationMatrix());
  EXPECT_LT((wn + 0.7 * axis).norm(), 1e-12);
}

TEST(RotationLog, TinyRotationIsStable)
{
  const Eigen::Vector3d w = rotationLog(Eigen::AngleAxisd(1e-9, Eigen::Vector3d::UnitX()).toRotationMatrix());
  EXPECT_NEAR(w.x(), 1e-9, 1e-15);
}

TEST(RotationLog, WrapsNearPiToTheShortWay)
{
  const double pi = M_PI;
  const Eigen::Vector3d under = rotationLog(rotZ(pi - 0.01));
  const Eigen::Vector3d over = rotationLog(rotZ(pi + 0.01));   // = -(pi - 0.01)
  EXPECT_NEAR(under.z(), pi - 0.01, 1e-9);
  EXPECT_NEAR(over.z(), -(pi - 0.01), 1e-9);
  EXPECT_LE(under.norm(), pi);
  EXPECT_LE(over.norm(), pi);
}

TEST(RotationLog, ExactlyPiHasMagnitudePi)
{
  const Eigen::Vector3d w = rotationLog(rotZ(M_PI));
  EXPECT_NEAR(w.norm(), M_PI, 1e-9);
  EXPECT_NEAR(std::abs(w.z()), M_PI, 1e-9);
}

TEST(RotationLog, ExponentialOfLogRoundTrips)
{
  for (double a : {0.1, 1.0, 2.5, 3.1}) {
    const Eigen::Matrix3d R = Eigen::AngleAxisd(a, Eigen::Vector3d(0.3, -0.5, 0.8).normalized()).toRotationMatrix();
    const Eigen::Vector3d w = rotationLog(R);
    const Eigen::Matrix3d R2 = Eigen::AngleAxisd(w.norm(), w.normalized()).toRotationMatrix();
    EXPECT_LT((R - R2).norm(), 1e-9);
  }
}

TEST(PoseError, PositionAndOrientationParts)
{
  Eigen::Isometry3d cur = Eigen::Isometry3d::Identity();
  cur.translation() = Eigen::Vector3d(1, 2, 3);
  cur.linear() = rotZ(0.2);
  Eigen::Isometry3d tgt = cur;
  tgt.translation() += Eigen::Vector3d(0.1, -0.2, 0.3);
  tgt.linear() = rotZ(0.5);
  const Vector6d e = poseError(cur, tgt);
  EXPECT_NEAR(e(0), 0.1, 1e-12);
  EXPECT_NEAR(e(1), -0.2, 1e-12);
  EXPECT_NEAR(e(2), 0.3, 1e-12);
  EXPECT_LT((e.tail<3>() - Eigen::Vector3d(0, 0, 0.3)).norm(), 1e-9);
  EXPECT_LT(poseError(cur, cur).norm(), 1e-12);
}

TEST(Feedback, ProportionalPlusFeedforward)
{
  FeedbackParams p;
  p.kp_linear = 2.0; p.kp_angular = 3.0; p.max_linear = 0.0; p.max_angular = 0.0;
  Vector6d ff = Vector6d::Zero(); ff(0) = 0.01;
  Vector6d e = Vector6d::Zero(); e(0) = 0.1; e(5) = 0.2;
  const Vector6d v = trackingTwist(ff, e, p);
  EXPECT_NEAR(v(0), 0.01 + 0.2, 1e-12);
  EXPECT_NEAR(v(5), 0.6, 1e-12);
}

TEST(Feedback, GainLimitBoundsCorrectionNotFeedforward)
{
  FeedbackParams p;
  p.kp_linear = 10.0; p.max_linear = 0.05; p.kp_angular = 10.0; p.max_angular = 0.1;
  Vector6d ff = Vector6d::Zero(); ff(0) = 0.2;   // larger than the limit: must pass through
  Vector6d e = Vector6d::Zero(); e(0) = 1.0; e(1) = 1.0; e(3) = 2.0;
  const Vector6d v = trackingTwist(ff, e, p);
  const Eigen::Vector3d fb = v.head<3>() - ff.head<3>();
  EXPECT_NEAR(fb.norm(), 0.05, 1e-12);
  EXPECT_NEAR(fb.x(), fb.y(), 1e-12);   // direction kept
  EXPECT_NEAR(v.tail<3>().norm(), 0.1, 1e-12);
}
