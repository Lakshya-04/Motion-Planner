#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "chain_helper.hpp"
#include "diff_ik_core/cartesian_tracker.hpp"

using namespace diff_ik_core;
using namespace chain_test;

namespace
{
const double kInf = std::numeric_limits<double>::infinity();
const double kDt = 0.002;

DiffIkLimits limitsFor(int n, double vmax)
{
  DiffIkLimits l;
  l.q_min = Eigen::VectorXd::Constant(n, -10.0);   // range over 2*pi: no position stop
  l.q_max = Eigen::VectorXd::Constant(n, 10.0);
  l.qd_max = Eigen::VectorXd::Constant(n, vmax);
  l.qdd_max = Eigen::VectorXd::Constant(n, kInf);
  return l;
}

ToolOffset trueOffset()
{
  ToolOffset o;
  o.xyz = Eigen::Vector3d(0.01, -0.005, 0.15);
  o.rpy = Eigen::Vector3d(0.1, -0.2, 0.3);
  return o;
}

ToolOffset wrongBy(const ToolOffset & o, const Eigen::Vector3d & d)
{
  ToolOffset w = o;
  w.xyz += d;
  return w;
}

TrackerParams paramsFor(int n, const ToolOffset & tool, double vmax = 2.0)
{
  TrackerParams p;
  p.limits = limitsFor(n, vmax);
  p.tool = tool;
  p.dt = kDt;
  return p;
}

const Eigen::VectorXd kQ6 = vec({0.3, -1.2, 1.4, -0.7, 0.9, 0.4});

struct Stroke
{
  Eigen::Isometry3d start;
  Eigen::Vector3d dir;
  double speed;
  double length;

  TrackerReference at(double t) const
  {
    const double s = std::min(speed * t, length);
    TrackerReference r;
    r.pose = start;
    r.pose.translation() = start.translation() + dir * s;
    if (speed * t < length) {r.twist.head<3>() = dir * speed;}
    return r;
  }
};

struct Trace
{
  std::vector<double> pos_err;   // contact error against the true offset, after each step
  std::vector<double> ang_err;
  std::vector<TrackerOutput> out;
  Eigen::VectorXd q_end;
};

// Kinematic loop: the arm goes exactly where the tracker's q_next says.
Trace runStroke(
  const Chain & c, const Eigen::VectorXd & q0, CartesianTracker & trk, const ToolOffset & truth,
  const Stroke & s, int steps, int swap_step = -1, const ToolOffset & swap_to = {})
{
  Trace tr;
  Eigen::VectorXd q = q0;
  for (int k = 0; k < steps; ++k) {
    if (k == swap_step) {trk.setToolOffset(swap_to);}
    const TrackerOutput o = trk.step(q, c.jacobian(q), c.fk(q), s.at(k * kDt));
    EXPECT_TRUE(o.ok());
    q = o.q_next;
    const Eigen::Isometry3d T = toolPoseFromFlange(c.fk(q), truth);
    const Vector6d e = poseError(T, s.at((k + 1) * kDt).pose);
    tr.pos_err.push_back(e.head<3>().norm());
    tr.ang_err.push_back(e.tail<3>().norm());
    tr.out.push_back(o);
  }
  tr.q_end = q;
  return tr;
}

Stroke strokeFrom(const Chain & c, const Eigen::VectorXd & q0, const ToolOffset & truth)
{
  return {toolPoseFromFlange(c.fk(q0), truth), Eigen::Vector3d(0.0, 1.0, 0.0), 0.05, 0.3};
}

double maxOf(const std::vector<double> & v, size_t from = 0)
{
  return *std::max_element(v.begin() + static_cast<long>(from), v.end());
}
}  // namespace

TEST(CartesianTracker, StraightStrokeWithCorrectOffsetStaysUnder1mmAnd01Deg)
{
  const Chain c = arm6();
  CartesianTracker trk(paramsFor(6, trueOffset()));
  const Trace tr = runStroke(c, kQ6, trk, trueOffset(), strokeFrom(c, kQ6, trueOffset()), 3200);
  EXPECT_LT(maxOf(tr.pos_err), 1e-3);
  EXPECT_LT(maxOf(tr.ang_err), 0.1 * M_PI / 180.0);
  // the stroke really moved the tool 0.3 m
  const Eigen::Isometry3d T = toolPoseFromFlange(c.fk(tr.q_end), trueOffset());
  EXPECT_NEAR((T.translation() - strokeFrom(c, kQ6, trueOffset()).start.translation()).norm(), 0.3, 1e-3);
}

TEST(CartesianTracker, TurningAboutTheContactPointKeepsThePointPut)
{
  const Chain c = arm6();
  CartesianTracker trk(paramsFor(6, trueOffset()));
  const Eigen::Isometry3d T0 = toolPoseFromFlange(c.fk(kQ6), trueOffset());
  const double w = 0.2;   // rad/s about world z; the 0.15 m tool makes the flange swing 3 cm/s
  Eigen::VectorXd q = kQ6;
  double worst_pos = 0.0, worst_ang = 0.0;
  for (int k = 0; k < 2500; ++k) {
    TrackerReference r;
    r.pose = T0;
    r.pose.linear() = Eigen::AngleAxisd(w * k * kDt, Eigen::Vector3d::UnitZ()) * T0.linear();
    r.twist.tail<3>() = Eigen::Vector3d(0.0, 0.0, w);
    const TrackerOutput o = trk.step(q, c.jacobian(q), c.fk(q), r);
    ASSERT_TRUE(o.ok());
    worst_pos = std::max(worst_pos, o.position_error);
    worst_ang = std::max(worst_ang, o.orientation_error);
    q = o.q_next;
  }
  EXPECT_LT(worst_pos, 1e-3);
  EXPECT_LT(worst_ang, 0.1 * M_PI / 180.0);
}

TEST(CartesianTracker, WrongOffsetMissesTheContactPointCorrectOneDoesNot)
{
  const Chain c = arm6();
  const Stroke s = strokeFrom(c, kQ6, trueOffset());
  const ToolOffset wrong = wrongBy(trueOffset(), Eigen::Vector3d(0.0, 0.0, 0.02));
  CartesianTracker bad(paramsFor(6, wrong));
  const Trace tb = runStroke(c, kQ6, bad, trueOffset(), s, 3200);
  // once the believed contact has settled on the path, the real one is ~2 cm off
  EXPECT_GT(maxOf(tb.pos_err, 1000), 0.01);
  EXPECT_NEAR(tb.pos_err.back(), 0.02, 2e-3);

  CartesianTracker good(paramsFor(6, trueOffset()));
  const Trace tg = runStroke(c, kQ6, good, trueOffset(), s, 3200);
  EXPECT_LT(maxOf(tg.pos_err), 1e-3);
}

TEST(CartesianTracker, HotSwapToCorrectOffsetConvergesWithin3s)
{
  const Chain c = arm6();
  const Stroke s = strokeFrom(c, kQ6, trueOffset());
  const ToolOffset wrong = wrongBy(trueOffset(), Eigen::Vector3d(0.0, 0.0, 0.02));
  CartesianTracker trk(paramsFor(6, wrong));
  const int swap = 500;   // 1 s in, the 2 cm error is there by then
  const Trace tr = runStroke(c, kQ6, trk, trueOffset(), s, 3200, swap, trueOffset());
  EXPECT_GT(maxOf(tr.pos_err), 0.01);
  EXPECT_GT(tr.pos_err[swap - 1], 0.015);
  const size_t settled = static_cast<size_t>(swap) + static_cast<size_t>(3.0 / kDt);
  EXPECT_LT(maxOf(tr.pos_err, settled), 1e-3);
  EXPECT_EQ(trk.toolOffset().xyz, trueOffset().xyz);
}

TEST(CartesianTracker, ThroughFullExtensionVelocitiesStayBoundedAndSigmaMinIsSmall)
{
  const Chain c = planar({0.4, 0.3, 0.3});   // reach 1.0 m
  const Eigen::VectorXd q0 = vec({0.3, -0.6, 0.4});
  TrackerParams p = paramsFor(3, ToolOffset{}, 1.0);
  p.mask = {true, true, false, false, false, false};
  CartesianTracker trk(p);
  const Eigen::Isometry3d T0 = c.fk(q0);
  // radial stroke that runs past the reach
  const Eigen::Vector3d dir = T0.translation().normalized();
  const double len = 1.1 - T0.translation().norm();
  Stroke s{T0, dir, 0.05, len};
  const int steps = static_cast<int>(len / 0.05 / kDt) + 1500;
  const Trace tr = runStroke(c, q0, trk, ToolOffset{}, s, steps);
  double min_sigma = kInf;
  for (const auto & o : tr.out) {
    ASSERT_TRUE(o.qdot.allFinite());
    EXPECT_LE(o.qdot.cwiseAbs().maxCoeff(), 1.0 + 1e-12);
    min_sigma = std::min(min_sigma, o.sigma_min);
  }
  EXPECT_LT(min_sigma, 0.02);
  EXPECT_GT(tr.out.front().sigma_min, 0.05);   // starts outside the damped zone
}

TEST(CartesianTracker, NonFiniteInputHoldsQAndReportsInvalid)
{
  const Chain c = arm6();
  CartesianTracker trk(paramsFor(6, trueOffset()));
  const Stroke s = strokeFrom(c, kQ6, trueOffset());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Eigen::MatrixXd J = c.jacobian(kQ6);

  auto expectHeld = [&](const TrackerOutput & o) {
      EXPECT_EQ(o.status, TrackerStatus::Invalid);
      EXPECT_FALSE(o.ok());
      EXPECT_EQ(o.qdot.size(), 6);
      EXPECT_EQ(o.qdot.norm(), 0.0);
      EXPECT_EQ((o.q_next - kQ6).norm(), 0.0);
    };

  Eigen::VectorXd qn = kQ6;
  qn(2) = nan;
  const TrackerOutput oq = trk.step(qn, J, c.fk(kQ6), s.at(0));
  EXPECT_EQ(oq.status, TrackerStatus::Invalid);
  EXPECT_EQ(oq.qdot.norm(), 0.0);

  Eigen::MatrixXd Jn = J;
  Jn(0, 1) = nan;
  expectHeld(trk.step(kQ6, Jn, c.fk(kQ6), s.at(0)));

  Eigen::Isometry3d Tn = c.fk(kQ6);
  Tn.translation()(0) = nan;
  expectHeld(trk.step(kQ6, J, Tn, s.at(0)));

  TrackerReference r = s.at(0);
  r.twist(1) = kInf;
  expectHeld(trk.step(kQ6, J, c.fk(kQ6), r));

  PlaneMeasurement m;
  m.standoff_error = nan;
  expectHeld(trk.step(kQ6, J, c.fk(kQ6), s.at(0), m));

  // and it still works afterwards
  EXPECT_TRUE(trk.step(kQ6, J, c.fk(kQ6), s.at(0)).ok());
}

TEST(CartesianTracker, TwoRunsAreBitwiseIdentical)
{
  const Chain c = arm6();
  const Stroke s = strokeFrom(c, kQ6, trueOffset());
  const ToolOffset wrong = wrongBy(trueOffset(), Eigen::Vector3d(0.0, 0.0, 0.02));
  CartesianTracker a(paramsFor(6, wrong)), b(paramsFor(6, wrong));
  const Trace ta = runStroke(c, kQ6, a, trueOffset(), s, 800, 400, trueOffset());
  const Trace tb = runStroke(c, kQ6, b, trueOffset(), s, 800, 400, trueOffset());
  ASSERT_EQ(ta.q_end.size(), tb.q_end.size());
  EXPECT_EQ(std::memcmp(ta.q_end.data(), tb.q_end.data(), sizeof(double) * 6), 0);
  EXPECT_EQ(std::memcmp(ta.pos_err.data(), tb.pos_err.data(), sizeof(double) * ta.pos_err.size()), 0);
  for (size_t i = 0; i < ta.out.size(); ++i) {
    EXPECT_EQ(std::memcmp(ta.out[i].qdot.data(), tb.out[i].qdot.data(), sizeof(double) * 6), 0);
    EXPECT_EQ(ta.out[i].sigma_min, tb.out[i].sigma_min);
  }
}

TEST(CartesianTracker, PlaneServoClosesStandoffErrorInsideTwistLimits)
{
  const Chain c = arm6();
  TrackerParams p = paramsFor(6, trueOffset());
  p.plane_enabled = true;
  p.max_linear_twist = 0.015;   // below the servo's own 0.02, so the limit has to act
  CartesianTracker trk(p);

  // wall in front of the tool; the tool axis is the contact frame z
  const Eigen::Isometry3d T0 = toolPoseFromFlange(c.fk(kQ6), trueOffset());
  const Eigen::Vector3d axis = T0.linear().col(2);
  const Eigen::Vector3d normal = -axis;
  const double target = 0.05;
  const Eigen::Vector3d wall = T0.translation() + axis * (target - 0.015);   // starts 1.5 cm too far

  TrackerReference ref;
  ref.pose = T0;
  Eigen::VectorXd q = kQ6;
  double first_err = 0.0, err = 0.0, worst_lin = 0.0, worst_ang = 0.0, min_scale = 1.0;
  for (int k = 0; k < 4000; ++k) {
    const Eigen::Isometry3d T = toolPoseFromFlange(c.fk(q), trueOffset());
    PlaneMeasurement m;
    m.normal = normal;
    m.tool_axis = T.linear().col(2);
    const double standoff = (T.translation() - wall).dot(normal);
    m.standoff_error = target - standoff;
    err = m.standoff_error;
    if (k == 0) {first_err = err;}
    const TrackerOutput o = trk.step(q, c.jacobian(q), c.fk(q), ref, m);
    ASSERT_TRUE(o.ok());
    worst_lin = std::max(worst_lin, o.twist.head<3>().norm());
    worst_ang = std::max(worst_ang, o.twist.tail<3>().norm());
    min_scale = std::min(min_scale, o.twist_scale);
    q = o.q_next;
  }
  EXPECT_NEAR(std::abs(first_err), 0.015, 1e-9);
  EXPECT_LT(std::abs(err), 1e-3);
  EXPECT_LE(worst_lin, p.max_linear_twist + 1e-12);
  EXPECT_LE(worst_ang, p.max_angular_twist + 1e-12);
  EXPECT_LT(min_scale, 1.0);
}

TEST(CartesianTracker, DiagnosticsAreFilled)
{
  const Chain c = arm6();
  CartesianTracker trk(paramsFor(6, trueOffset()));
  const Stroke s = strokeFrom(c, kQ6, trueOffset());
  TrackerReference r = s.at(0);
  r.pose.translation() += Eigen::Vector3d(0.003, 0.0, 0.004);
  const TrackerOutput o = trk.step(kQ6, c.jacobian(kQ6), c.fk(kQ6), r);
  ASSERT_TRUE(o.ok());
  EXPECT_NEAR(o.position_error, 0.005, 1e-9);
  EXPECT_NEAR(o.orientation_error, 0.0, 1e-9);
  EXPECT_GT(o.sigma_min, 0.0);
  EXPECT_GE(o.condition, 1.0);
  EXPECT_GT(o.manipulability, 0.0);
  EXPECT_DOUBLE_EQ(o.joint_limit_margin, 0.5);   // all joints continuous
  EXPECT_EQ(o.task_scale, 1.0);
}
