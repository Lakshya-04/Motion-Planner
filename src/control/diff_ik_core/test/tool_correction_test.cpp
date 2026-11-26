#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "chain_helper.hpp"
#include "diff_ik_core/diff_ik.hpp"
#include "diff_ik_core/pose_error.hpp"
#include "diff_ik_core/tool_correction.hpp"

using namespace diff_ik_core;
using namespace chain_test;

namespace
{
const double kInf = std::numeric_limits<double>::infinity();

DiffIkLimits openLimits(int n, double v = kInf)
{
  DiffIkLimits l;
  l.q_min = Eigen::VectorXd::Constant(n, -10.0);   // range over 2*pi: no position stop
  l.q_max = Eigen::VectorXd::Constant(n, 10.0);
  l.qd_max = Eigen::VectorXd::Constant(n, v);
  l.qdd_max = Eigen::VectorXd::Constant(n, kInf);
  return l;
}

ToolOffset sampleOffset()
{
  ToolOffset o;
  o.xyz = Eigen::Vector3d(0.01, -0.005, 0.02);
  o.rpy = Eigen::Vector3d(0.1, -0.2, 0.3);
  return o;
}

Eigen::Vector3d contactPoint(const Chain & c, const Eigen::VectorXd & q, const ToolOffset & o)
{
  return toolPoseFromFlange(c.fk(q), o).translation();
}

// Contact point velocity per joint by central differences on forward kinematics.
Eigen::MatrixXd numericContactJacobian(const Chain & c, const Eigen::VectorXd & q, const ToolOffset & o)
{
  const double h = 1e-6;
  Eigen::MatrixXd J(3, c.dof());
  for (int i = 0; i < c.dof(); ++i) {
    Eigen::VectorXd qp = q, qm = q;
    qp(i) += h;
    qm(i) -= h;
    J.col(i) = (contactPoint(c, qp, o) - contactPoint(c, qm, o)) / (2 * h);
  }
  return J;
}

void expectShiftMatchesFk(const Chain & c, const Eigen::VectorXd & q)
{
  const ToolOffset o = sampleOffset();
  const Eigen::Isometry3d T = c.fk(q);
  const Eigen::MatrixXd Jf = c.jacobian(q);
  const Jacobian6 Jt = shiftJacobianToTool(Jf, T.linear(), o);
  ASSERT_EQ(Jt.cols(), c.dof());
  EXPECT_LT((Jt.topRows<3>() - numericContactJacobian(c, q, o)).cwiseAbs().maxCoeff(), 1e-6);
  EXPECT_LT((Jt.bottomRows<3>() - Jf.bottomRows<3>()).cwiseAbs().maxCoeff(), 1e-15);
  // the shift is not a no-op for this offset
  EXPECT_GT((Jt.topRows<3>() - Jf.topRows<3>()).cwiseAbs().maxCoeff(), 1e-4);
}
}  // namespace

TEST(ToolOffset, RpyIsRzRyRx)
{
  ToolOffset o;
  o.rpy = Eigen::Vector3d(0.3, -0.4, 0.5);
  const Eigen::Matrix3d R =
    (Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX())).toRotationMatrix();
  EXPECT_LT((o.rotation() - R).norm(), 1e-12);
  o.rpy = Eigen::Vector3d(0, 0, M_PI / 2);   // yaw only turns x into y
  EXPECT_LT((o.rotation() * Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitY()).norm(), 1e-12);
}

TEST(ShiftJacobian, MatchesFiniteDifferencePlanar)
{
  expectShiftMatchesFk(planar({0.5, 0.4}), vec({0.3, 0.8}));
  expectShiftMatchesFk(planar({0.5, 0.4, 0.3}), vec({0.3, -0.6, 0.9}));
}

TEST(ShiftJacobian, MatchesFiniteDifferenceSpatial)
{
  expectShiftMatchesFk(arm6(), vec({0.3, -1.2, 1.4, -0.7, 0.9, 0.4}));
  expectShiftMatchesFk(arm7(), vec({0.2, 0.6, -0.4, 1.1, 0.3, -0.8, 0.5}));
}

TEST(ShiftJacobian, RejectsBadShape)
{
  EXPECT_EQ(shiftJacobianToTool(Eigen::MatrixXd::Zero(3, 4), Eigen::Matrix3d::Identity(), {}).cols(), 0);
  EXPECT_EQ(shiftJacobianToTool(Eigen::MatrixXd::Zero(6, 9), Eigen::Matrix3d::Identity(), {}).cols(), 0);
}

TEST(ShiftJacobian, WrongOffsetLeavesTipErrorCorrectedOffsetDoesNot)
{
  const Chain c = arm6();
  const Eigen::VectorXd q = vec({0.3, -1.2, 1.4, -0.7, 0.9, 0.4});
  const ToolOffset truth = sampleOffset();
  ToolOffset wrong = truth;
  wrong.xyz += Eigen::Vector3d(0.012, 0.016, 0.0);   // 2 cm off

  const Eigen::Isometry3d T = c.fk(q);
  const Eigen::MatrixXd Jf = c.jacobian(q);
  const Jacobian6 J_true = shiftJacobianToTool(Jf, T.linear(), truth);
  const Jacobian6 J_wrong = shiftJacobianToTool(Jf, T.linear(), wrong);

  Vector6d v;
  v << 0.02, -0.01, 0.015, 0.2, -0.1, 0.3;
  const DiffIkLimits lim = openLimits(6);
  const DiffIkParams params;

  const auto bad = computeStep(J_wrong, v, kAllAxes, q, lim, params);
  const auto good = computeStep(J_true, v, kAllAxes, q, lim, params);
  ASSERT_TRUE(bad.valid);
  ASSERT_TRUE(good.valid);

  // true tip twist for each command, from the true contact Jacobian
  const Vector6d tip_bad = J_true * Eigen::VectorXd(bad.qd);
  const Vector6d tip_good = J_true * Eigen::VectorXd(good.qd);
  EXPECT_GT((tip_bad - v).head<3>().norm(), 1e-3);
  EXPECT_LT((tip_good - v).norm(), 1e-9);
}

TEST(ShiftJacobian, ContactFrameTwistIsRotatedWorldTwist)
{
  const Chain c = arm6();
  const Eigen::VectorXd q = vec({0.3, -1.2, 1.4, -0.7, 0.9, 0.4});
  const ToolOffset o = sampleOffset();
  const Eigen::Isometry3d T = c.fk(q);
  const Eigen::MatrixXd Jf = c.jacobian(q);
  const Jacobian6 Jw = shiftJacobianToTool(Jf, T.linear(), o, TwistFrame::World);
  const Jacobian6 Jc = shiftJacobianToTool(Jf, T.linear(), o, TwistFrame::Contact);
  const Eigen::Matrix3d Rwc = T.linear() * o.rotation();

  const Eigen::VectorXd qd = vec({0.1, -0.2, 0.3, 0.05, -0.15, 0.25});
  const Vector6d tw = Jw * qd;
  const Vector6d tc = Jc * qd;
  EXPECT_LT((tc.head<3>() - Rwc.transpose() * tw.head<3>()).norm(), 1e-12);
  EXPECT_LT((tc.tail<3>() - Rwc.transpose() * tw.tail<3>()).norm(), 1e-12);
  EXPECT_GT((tc - tw).norm(), 1e-3);   // the rotation is not the identity here
}

TEST(PlaneServo, ZeroAtTarget)
{
  const Eigen::Vector3d n = Eigen::Vector3d(0.3, -0.5, 0.8).normalized();
  const Vector6d v = planeServoTwist(n, 0.0, -n, PlaneServoParams());
  EXPECT_EQ(v.norm(), 0.0);
}

TEST(PlaneServo, SignsAndLimits)
{
  const Eigen::Vector3d n = Eigen::Vector3d::UnitX();
  PlaneServoParams p;
  p.kp_distance = 1.0;
  p.max_linear = 0.01;
  const Vector6d away = planeServoTwist(n, 0.005, -n, p);   // too close: move along +n
  EXPECT_NEAR(away(0), 0.005, 1e-15);
  const Vector6d clamped = planeServoTwist(n, -0.5, -n, p);
  EXPECT_NEAR(clamped(0), -0.01, 1e-15);

  // axis tilted towards +y: the twist must turn it back, angular dot (d axis / dt) < 0 along y
  const Eigen::Vector3d a = Eigen::Vector3d(-1.0, 0.2, 0.0).normalized();   // anti-parallel would be (-1,0,0)
  const Vector6d t = planeServoTwist(n, 0.0, -a, p);
  const Eigen::Vector3d adot = t.tail<3>().cross(-a);
  EXPECT_LT(adot.dot(Eigen::Vector3d::UnitY()), 0.0);
  EXPECT_LE(t.tail<3>().norm(), p.max_angular + 1e-15);
}

TEST(PlaneServo, ClosedLoopConvergesWithinLimits)
{
  const Chain c = arm6();
  const ToolOffset o = sampleOffset();
  const Eigen::VectorXd q_goal = vec({0.3, -1.2, 1.4, -0.7, 0.9, 0.4});
  const Eigen::Isometry3d T_goal = toolPoseFromFlange(c.fk(q_goal), o);
  const Eigen::Vector3d axis_goal = T_goal.linear().col(2);   // tool axis points at the wall
  const Eigen::Vector3d n = -axis_goal;
  const double target_standoff = 0.05;
  const Eigen::Vector3d wall_point = T_goal.translation() - target_standoff * n;

  const DiffIkLimits lim = openLimits(6, 1.0);
  const DiffIkParams params;

  // start 1.5 cm too far out and 8 degrees tilted, reached with the same diff IK
  Eigen::Isometry3d T0 = T_goal;
  T0.translation() += 0.015 * n;
  const Eigen::Vector3d tilt_axis = n.cross(Eigen::Vector3d::UnitZ()).normalized();
  T0.linear() = Eigen::AngleAxisd(8.0 * M_PI / 180.0, tilt_axis).toRotationMatrix() * T_goal.linear();
  Eigen::VectorXd q = q_goal;
  for (int k = 0; k < 2000; ++k) {
    const Eigen::Isometry3d Tc = toolPoseFromFlange(c.fk(q), o);
    const Vector6d e = poseError(Tc, T0);
    if (e.norm() < 1e-10) {break;}
    const Jacobian6 J = shiftJacobianToTool(c.jacobian(q), c.fk(q).linear(), o);
    const auto r = computeStep(J, 0.5 * e, kAllAxes, q, lim, params);
    ASSERT_TRUE(r.valid);
    q += r.qd;
  }

  auto standoff = [&](const Eigen::VectorXd & qq) {
      return (toolPoseFromFlange(c.fk(qq), o).translation() - wall_point).dot(n);
    };
  auto tiltDeg = [&](const Eigen::VectorXd & qq) {
      const Eigen::Vector3d a = toolPoseFromFlange(c.fk(qq), o).linear().col(2);
      return std::acos(std::max(-1.0, std::min(1.0, -a.dot(n)))) * 180.0 / M_PI;
    };
  ASSERT_NEAR(target_standoff - standoff(q), -0.015, 1e-6);
  ASSERT_NEAR(tiltDeg(q), 8.0, 1e-3);

  PlaneServoParams sp;
  sp.kp_distance = 4.0;
  sp.kp_tilt = 4.0;
  sp.max_linear = 0.02;
  sp.max_angular = 0.1;
  const double dt = 0.01;
  for (int k = 0; k < 600; ++k) {
    const Eigen::Isometry3d Tf = c.fk(q);
    const Eigen::Isometry3d Tc = toolPoseFromFlange(Tf, o);
    const Vector6d v = planeServoTwist(n, target_standoff - standoff(q), Tc.linear().col(2), sp);
    ASSERT_LE(std::abs(v.head<3>().dot(n)), sp.max_linear + 1e-12);
    ASSERT_LE(v.tail<3>().norm(), sp.max_angular + 1e-12);
    const Jacobian6 J = shiftJacobianToTool(c.jacobian(q), Tf.linear(), o);
    const auto r = computeStep(J, v, kAllAxes, q, lim, params);
    ASSERT_TRUE(r.valid);
    ASSERT_LE(r.qd.cwiseAbs().maxCoeff(), 1.0 + 1e-12);
    q += dt * r.qd;
  }
  EXPECT_LT(std::abs(standoff(q) - target_standoff), 1e-3);
  EXPECT_LT(tiltDeg(q), 0.5);
}
