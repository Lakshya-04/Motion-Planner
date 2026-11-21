#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "chain_helper.hpp"
#include "diff_ik_core/diff_ik.hpp"
#include "diff_ik_core/jacobian_quality.hpp"
#include "diff_ik_core/pose_error.hpp"

using namespace diff_ik_core;
using namespace chain_test;

namespace
{
DiffIkLimits limitsFor(int n, double lim = 1.5, double v = 10.0)
{
  DiffIkLimits l;
  l.q_min = Eigen::VectorXd::Constant(n, -lim);
  l.q_max = Eigen::VectorXd::Constant(n, lim);
  l.qd_max = Eigen::VectorXd::Constant(n, v);
  l.qdd_max = Eigen::VectorXd::Constant(n, 100.0);
  return l;
}

Vector6d twist(double a, double b, double c, double d, double e, double f)
{
  Vector6d v; v << a, b, c, d, e, f; return v;
}

const AxisMask kXY = {true, true, false, false, false, false};
const AxisMask kXYYaw = {true, true, false, false, false, true};
const AxisMask kPosOnly = {true, true, true, false, false, false};

const Eigen::VectorXd kQ6 = vec({0.3, -1.1, 1.3, -0.8, 1.0, 0.4});
const Eigen::VectorXd kQ7 = vec({0.3, 0.8, -0.4, -1.2, 0.5, 0.9, 0.2});
}  // namespace

TEST(DiffIk, PlanarTwoLinkMatchesAnalyticInverse)
{
  const Chain c = planar({1.0, 0.7});
  const Eigen::VectorXd q = vec({0.3, 1.1});
  const Eigen::MatrixXd J = c.jacobian(q);
  const Vector6d v = twist(0.05, -0.02, 0, 0, 0, 0);
  const auto r = computeStep(J, v, kXY, q, limitsFor(2), DiffIkParams());
  ASSERT_TRUE(r.valid);
  const Eigen::Vector2d analytic = J.topRows(2).inverse() * Eigen::Vector2d(0.05, -0.02);
  EXPECT_LT((r.qd - analytic).norm(), 1e-9);
  EXPECT_EQ(r.damping, 0.0);
  EXPECT_EQ(r.scale, 1.0);
}

TEST(DiffIk, PlanarThreeLinkPositionAndYawMatchesAnalyticInverse)
{
  const Chain c = planar({0.8, 0.6, 0.4});
  const Eigen::VectorXd q = vec({0.2, 0.9, -0.7});
  const Eigen::MatrixXd J = c.jacobian(q);
  Eigen::Matrix3d J3;
  J3.row(0) = J.row(0); J3.row(1) = J.row(1); J3.row(2) = J.row(5);
  const Vector6d v = twist(0.03, 0.04, 0, 0, 0, 0.1);
  const auto r = computeStep(J, v, kXYYaw, q, limitsFor(3), DiffIkParams());
  const Eigen::Vector3d analytic = J3.inverse() * Eigen::Vector3d(0.03, 0.04, 0.1);
  EXPECT_LT((r.qd - analytic).norm(), 1e-9);
}

TEST(DiffIk, SpatialSixRReproducesFullTwist)
{
  const Chain c = arm6();
  const Eigen::MatrixXd J = c.jacobian(kQ6);
  ASSERT_GT(minSingularValue(J), 0.05);
  const Vector6d v = twist(0.02, -0.01, 0.03, 0.05, -0.04, 0.02);
  const auto r = computeStep(J, v, kAllAxes, kQ6, limitsFor(6), DiffIkParams());
  EXPECT_LT((r.achieved_twist - v).norm(), 1e-9);
  EXPECT_LT((J * r.qd - v).norm(), 1e-9);
}

TEST(DiffIk, SevenRReproducesTwistWithAndWithoutNullSpaceCosts)
{
  const Chain c = arm7();
  const Eigen::MatrixXd J = c.jacobian(kQ7);
  ASSERT_GT(minSingularValue(J), 0.05);
  const Vector6d v = twist(0.02, -0.01, 0.03, 0.05, -0.04, 0.02);
  DiffIkParams p;
  const auto plain = computeStep(J, v, kAllAxes, kQ7, limitsFor(7), p);
  EXPECT_LT((plain.achieved_twist - v).norm(), 1e-9);
  p.null_space.push_back({std::make_shared<JointCenteringCost>(), 5.0});
  p.null_space.push_back({std::make_shared<LimitAvoidanceCost>(0.3), 1.0});
  const auto with = computeStep(J, v, kAllAxes, kQ7, limitsFor(7), p);
  EXPECT_LT((with.achieved_twist - v).norm(), 1e-9);
  EXPECT_GT((with.qd - plain.qd).norm(), 1e-6);   // the null-space term did move something
}

TEST(DiffIk, NullSpaceMotionLeavesTaskTwistUnchanged)
{
  const Chain c = arm7();
  const Eigen::MatrixXd J = c.jacobian(kQ7);
  DiffIkParams p;
  p.null_space.push_back({std::make_shared<JointCenteringCost>(), 1.0});
  const auto r = computeStep(J, Vector6d::Zero(), kAllAxes, kQ7, limitsFor(7), p);
  EXPECT_GT(r.qd.norm(), 1e-4);                 // self-motion exists
  EXPECT_LT((J * r.qd).norm(), 1e-9);           // and does not move the tool
}

TEST(DiffIk, NullSpaceCenteringReducesDistanceFromCentreOverSteps)
{
  const Chain c = arm7();
  Eigen::VectorXd q = kQ7;
  const DiffIkLimits lim = limitsFor(7, 2.0);
  DiffIkParams p;
  p.null_space.push_back({std::make_shared<JointCenteringCost>(), 20.0});
  const Eigen::Isometry3d T0 = c.fk(q);
  const double d0 = q.norm();
  for (int k = 0; k < 400; ++k) {
    const auto r = computeStep(c.jacobian(q), Vector6d::Zero(), kAllAxes, q, lim, p);
    q += r.qd * 0.01;
  }
  EXPECT_LT(q.norm(), d0);
  EXPECT_LT((c.fk(q).translation() - T0.translation()).norm(), 5e-3);   // tool stayed put
}

TEST(DiffIk, DampingIsZeroAboveThresholdAndRisesSmoothlyBelow)
{
  const Chain c = planar({1.0, 0.7});
  DiffIkParams p;
  p.sigma_threshold = 0.1;
  p.lambda_max = 0.2;
  const Vector6d v = twist(0.01, 0.0, 0, 0, 0, 0);
  double prev = -1.0;
  // q2 from well-conditioned towards the stretched singularity
  for (double q2 : {1.0, 0.5, 0.2, 0.1, 0.05, 0.02, 0.0}) {
    const Eigen::VectorXd q = vec({0.2, q2});
    const auto r = computeStep(c.jacobian(q), v, kXY, q, limitsFor(2), p);
    if (r.sigma_min >= p.sigma_threshold) {EXPECT_EQ(r.damping, 0.0);}
    EXPECT_GE(r.damping, prev);   // non-decreasing towards the singularity
    prev = r.damping;
  }
  EXPECT_NEAR(prev, 0.2, 1e-9);   // lambda_max at sigma_min = 0
}

TEST(DiffIk, PathThroughSingularityStopsWithoutJointVelocitySpike)
{
  const Chain c = planar({1.0, 0.7});
  DiffIkParams p;                       // eps 0.05, lambda_max 0.1
  const double speed = 0.05;            // m/s towards the stretched pose along the radial direction
  Eigen::VectorXd q = vec({0.4, 0.6});
  double max_qd = 0.0, last_radial = 1e9;
  const double bound = speed / (2.0 * p.lambda_max) * 1.05;   // DLS gain is at most 1/(2 lambda)
  for (int k = 0; k < 4000; ++k) {
    const Eigen::Isometry3d T = c.fk(q);
    const Eigen::Vector2d radial = T.translation().head<2>().normalized();
    const Vector6d v = twist(speed * radial.x(), speed * radial.y(), 0, 0, 0, 0);
    const auto r = computeStep(c.jacobian(q), v, kXY, q, limitsFor(2, 3.0), p);
    max_qd = std::max(max_qd, r.qd.cwiseAbs().maxCoeff());
    last_radial = r.achieved_twist.head<2>().dot(radial);
    q += r.qd * 0.005;
  }
  EXPECT_LT(max_qd, std::max(bound, 3.0 * speed / 0.1));   // no spike (a plain inverse diverges)
  EXPECT_LT(std::abs(last_radial), 1e-3);                  // slowed to a stop
  EXPECT_GT(q(1), -0.05);                                  // never flipped through the singularity
}

TEST(DiffIk, ExactSingularityGivesZeroVelocityTowardsTheUnreachableDirection)
{
  const Chain c = planar({1.0, 0.7});
  const Eigen::VectorXd q = vec({0.0, 0.0});
  const auto r = computeStep(c.jacobian(q), twist(0.05, 0, 0, 0, 0, 0), kXY, q, limitsFor(2), DiffIkParams());
  EXPECT_LT(r.qd.norm(), 1e-9);
  EXPECT_NEAR(r.damping, 0.1, 1e-9);
  EXPECT_LT(r.sigma_min, 1e-9);
}

TEST(DiffIk, UniformScalingKeepsTheDirectionOfTheTask)
{
  const Chain c = arm6();
  const Eigen::MatrixXd J = c.jacobian(kQ6);
  const Vector6d v = twist(0.3, -0.2, 0.25, 0.4, -0.3, 0.2);
  DiffIkLimits lim = limitsFor(6, 3.0, 100.0);
  const auto free_r = computeStep(J, v, kAllAxes, kQ6, lim, DiffIkParams());
  lim.qd_max = Eigen::VectorXd::Constant(6, 0.25 * free_r.qd.cwiseAbs().maxCoeff());
  const auto r = computeStep(J, v, kAllAxes, kQ6, lim, DiffIkParams());
  EXPECT_NEAR(r.scale, 0.25, 1e-9);
  EXPECT_LT((r.qd - r.scale * free_r.qd).norm(), 1e-9);
  EXPECT_LT((r.achieved_twist - r.scale * v).norm(), 1e-9);   // same direction, shorter
  EXPECT_NEAR(r.qd.cwiseAbs().maxCoeff(), lim.qd_max(0), 1e-12);
  EXPECT_GE(r.velocity_limit_hits, 1);
}

TEST(DiffIk, VelocityLimitsAreNeverExceededOverRandomConfigurations)
{
  const Chain c = arm7();
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  DiffIkLimits lim = limitsFor(7, 2.5, 0.8);
  DiffIkParams p;
  p.null_space.push_back({std::make_shared<JointCenteringCost>(), 50.0});
  p.null_space.push_back({std::make_shared<LimitAvoidanceCost>(0.3), 50.0});
  for (int k = 0; k < 300; ++k) {
    Eigen::VectorXd q(7);
    for (int i = 0; i < 7; ++i) {q(i) = 2.0 * u(rng);}
    const Vector6d v = twist(u(rng), u(rng), u(rng), 3 * u(rng), 3 * u(rng), 3 * u(rng));
    const auto r = computeStep(c.jacobian(q), v, kAllAxes, q, lim, p);
    ASSERT_TRUE(r.valid);
    EXPECT_LE(r.qd.cwiseAbs().maxCoeff(), 0.8 + 1e-12);
    EXPECT_TRUE(r.qd.allFinite());
  }
}

TEST(DiffIk, PositionZoneZeroesVelocityIntoTheLimitOnly)
{
  const Chain c = planar({1.0, 0.7});
  DiffIkLimits lim = limitsFor(2, 1.0);
  lim.position_margin = 0.05;
  const Eigen::VectorXd q = vec({-0.97, 0.8});   // joint 0 inside the lower zone
  Eigen::MatrixXd J = c.jacobian(q);
  // A twist whose solution drives joint 0 negative and one that drives it positive.
  const Eigen::Vector2d dir_neg = J.topRows(2) * vec({-1.0, 0.0});
  const Eigen::Vector2d dir_pos = J.topRows(2) * vec({1.0, 0.0});
  const auto neg = computeStep(J, twist(dir_neg.x(), dir_neg.y(), 0, 0, 0, 0), kXY, q, lim, DiffIkParams());
  EXPECT_EQ(neg.qd(0), 0.0);
  EXPECT_EQ(neg.saturation[0], JointSaturation::PositionLimit);
  EXPECT_EQ(neg.position_limit_hits, 1);
  const auto pos = computeStep(J, twist(dir_pos.x(), dir_pos.y(), 0, 0, 0, 0), kXY, q, lim, DiffIkParams());
  EXPECT_GT(pos.qd(0), 0.5);
  EXPECT_EQ(pos.position_limit_hits, 0);
}

TEST(DiffIk, ContinuousJointHasNoPositionStop)
{
  const Chain c = planar({1.0, 0.7});
  DiffIkLimits lim = limitsFor(2, M_PI);   // range 2*pi: continuous
  lim.position_margin = 0.5;
  const Eigen::VectorXd q = vec({3.1, 0.8});
  const Eigen::Vector2d d = c.jacobian(q).topRows(2) * vec({1.0, 0.0});
  const auto r = computeStep(c.jacobian(q), twist(d.x(), d.y(), 0, 0, 0, 0), kXY, q, lim, DiffIkParams());
  EXPECT_NEAR(r.qd(0), 1.0, 1e-9);
  EXPECT_EQ(r.position_limit_hits, 0);
}

TEST(DiffIk, HeavilyWeightedLiftMovesLess)
{
  const Chain c = liftArm6();
  Eigen::VectorXd q(7);
  q << 0.5, 0.3, -1.1, 1.3, -0.8, 1.0, 0.4;
  const Eigen::MatrixXd J = c.jacobian(q);
  const Vector6d v = twist(0.0, 0.0, 0.05, 0, 0, 0);
  DiffIkLimits lim = limitsFor(7, 3.0);
  DiffIkParams eq, heavy;
  heavy.joint_weights = Eigen::VectorXd::Ones(7);
  heavy.joint_weights(0) = 1000.0;
  const auto a = computeStep(J, v, kPosOnly, q, lim, eq);
  const auto b = computeStep(J, v, kPosOnly, q, lim, heavy);
  EXPECT_LT(std::abs(b.qd(0)), 0.05 * std::abs(a.qd(0)));
  EXPECT_LT((J.topRows(3) * b.qd - v.head<3>()).norm(), 1e-6);   // the task is still done
  EXPECT_LT((J.topRows(3) * a.qd - v.head<3>()).norm(), 1e-9);
}

TEST(DiffIk, LiftEndBiasPushesLiftAwayFromItsTopInTheNullSpace)
{
  const Chain c = liftArm6();
  Eigen::VectorXd q(7);
  q << 0.9, 0.3, -1.1, 1.3, -0.8, 1.0, 0.4;   // lift near the top of [0, 1]
  DiffIkLimits lim = limitsFor(7, 3.0);
  lim.q_min(0) = 0.0;
  lim.q_max(0) = 1.0;
  DiffIkParams p;
  p.null_space.push_back({std::make_shared<JointEndBiasCost>(0, 0.4), 2.0});
  const Eigen::MatrixXd J = c.jacobian(q);
  const auto r = computeStep(J, Vector6d::Zero(), kAllAxes, q, lim, p);
  EXPECT_LT(r.qd(0), -1e-4);
  EXPECT_LT((J * r.qd).norm(), 1e-8);
  // Mid-travel: no push.
  q(0) = 0.5;
  const auto mid = computeStep(c.jacobian(q), Vector6d::Zero(), kAllAxes, q, lim, p);
  EXPECT_LT(mid.qd.norm(), 1e-12);
}

TEST(DiffIk, MaskedAxesIgnoreTheFreeAxes)
{
  const Chain c = arm6();
  const Eigen::MatrixXd J = c.jacobian(kQ6);
  const auto a = computeStep(J, twist(0.02, 0.01, -0.03, 0.0, 0.0, 0.0), kPosOnly, kQ6, limitsFor(6), DiffIkParams());
  const auto b = computeStep(J, twist(0.02, 0.01, -0.03, 9.0, -9.0, 9.0), kPosOnly, kQ6, limitsFor(6), DiffIkParams());
  EXPECT_LT((a.qd - b.qd).norm(), 1e-12);
  EXPECT_LT((J.topRows(3) * a.qd - Eigen::Vector3d(0.02, 0.01, -0.03)).norm(), 1e-9);
}

TEST(DiffIk, MaskedTaskGivesTheNullSpaceRoomOfTheFreeAxes)
{
  const Chain c = arm6();
  const Eigen::MatrixXd J = c.jacobian(kQ6);
  DiffIkParams p;
  p.null_space.push_back({std::make_shared<JointCenteringCost>(), 1.0});
  const auto r = computeStep(J, Vector6d::Zero(), kPosOnly, kQ6, limitsFor(6), p);
  EXPECT_GT(r.qd.norm(), 1e-4);                     // 3 free DOF: self-motion exists
  EXPECT_LT((J.topRows(3) * r.qd).norm(), 1e-9);    // position held
  // Full-pose task on a 6-joint arm has no null space.
  const auto full = computeStep(J, Vector6d::Zero(), kAllAxes, kQ6, limitsFor(6), p);
  EXPECT_LT(full.qd.norm(), 1e-9);
}

TEST(DiffIk, EmptyMaskWithoutCostsIsZeroAndSizeMismatchIsInvalid)
{
  const Chain c = arm6();
  const Eigen::MatrixXd J = c.jacobian(kQ6);
  const AxisMask none = {false, false, false, false, false, false};
  const auto r = computeStep(J, twist(1, 1, 1, 1, 1, 1), none, kQ6, limitsFor(6), DiffIkParams());
  EXPECT_TRUE(r.valid);
  EXPECT_EQ(r.qd.norm(), 0.0);
  const auto bad = computeStep(J, twist(1, 1, 1, 1, 1, 1), kAllAxes, vec({0, 0}), limitsFor(6), DiffIkParams());
  EXPECT_FALSE(bad.valid);
  EXPECT_EQ(bad.qd.norm(), 0.0);
}

TEST(DiffIk, IntegrateClampsAccelerationAndIsSemiImplicit)
{
  DiffIkLimits lim = limitsFor(1, 10.0, 5.0);
  lim.qdd_max(0) = 2.0;
  const auto r = integrate(vec({0.0}), vec({0.0}), vec({4.0}), 0.1, lim);
  EXPECT_NEAR(r.qd(0), 0.2, 1e-12);          // 2 * 0.1
  EXPECT_NEAR(r.q(0), 0.02, 1e-12);          // advanced with the NEW velocity
  const auto fast = integrate(vec({0.0}), vec({4.9}), vec({50.0}), 1.0, limitsFor(1, 100.0, 5.0));
  EXPECT_NEAR(fast.qd(0), 5.0, 1e-12);       // velocity limit
}

TEST(DiffIk, IntegrateStopsAtPositionLimit)
{
  DiffIkLimits lim = limitsFor(1, 1.0, 5.0);
  const auto r = integrate(vec({0.99}), vec({2.0}), vec({2.0}), 0.1, lim);
  EXPECT_EQ(r.q(0), 1.0);
  EXPECT_EQ(r.qd(0), 0.0);
}

TEST(DiffIk, ClosedLoopTrackingConvergesToTheTargetPose)
{
  const Chain c = arm6();
  Eigen::VectorXd q = kQ6, qd = Eigen::VectorXd::Zero(6);
  Eigen::Isometry3d target = c.fk(q);
  target.translation() += Eigen::Vector3d(0.08, -0.05, 0.06);
  target.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d(0, 0, 1)).toRotationMatrix() * target.linear();
  DiffIkLimits lim = limitsFor(6, 3.0, 2.0);
  FeedbackParams fb;
  fb.kp_linear = 4.0; fb.kp_angular = 4.0; fb.max_linear = 0.1; fb.max_angular = 0.3;
  for (int k = 0; k < 1500; ++k) {
    const Vector6d e = poseError(c.fk(q), target);
    const Vector6d v = trackingTwist(Vector6d::Zero(), e, fb);
    const auto r = computeStep(c.jacobian(q), v, kAllAxes, q, lim, DiffIkParams());
    const auto s = integrate(q, qd, r.qd, 0.005, lim);
    q = s.q; qd = s.qd;
  }
  const Vector6d e = poseError(c.fk(q), target);
  EXPECT_LT(e.head<3>().norm(), 1e-3);
  EXPECT_LT(e.tail<3>().norm(), 2e-3);
}

TEST(DiffIk, IsDeterministic)
{
  const Chain c = arm7();
  const Eigen::MatrixXd J = c.jacobian(kQ7);
  DiffIkParams p;
  p.null_space.push_back({std::make_shared<JointCenteringCost>(), 3.0});
  const Vector6d v = twist(0.02, -0.01, 0.03, 0.05, -0.04, 0.02);
  const auto a = computeStep(J, v, kAllAxes, kQ7, limitsFor(7), p);
  const auto b = computeStep(J, v, kAllAxes, kQ7, limitsFor(7), p);
  EXPECT_EQ(a.qd, b.qd);
  EXPECT_EQ(a.sigma_min, b.sigma_min);
}
