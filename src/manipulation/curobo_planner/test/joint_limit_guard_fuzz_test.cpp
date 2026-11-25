// Property / fuzz tests of the joint limit guard (ROS-free): seeded, 20000 random trajectories and policies, with
// NaN / inf / huge values, ragged points and non-monotone time. Invariants: a refused trajectory is untouched; a
// retimed one has finite, strictly increasing time, the same path, and sits inside limit * safety_factor (velocity,
// acceleration, jerk) when measured again; reject mode never changes a trajectory.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "curobo_planner/joint_limit_guard.hpp"
#include "fuzz_util.hpp"

namespace cp = curobo_planner;
namespace mp = motion_planner;
using fuzz::kCases;
using fuzz::Rng;

namespace
{
bool same(const core_types::Trajectory & a, const core_types::Trajectory & b)
{
  if (a.joint_names != b.joint_names || a.points.size() != b.points.size()) {return false;}
  for (size_t i = 0; i < a.points.size(); ++i) {
    const auto & p = a.points[i];
    const auto & q = b.points[i];
    auto eq = [](const std::vector<double> & x, const std::vector<double> & y) {
        if (x.size() != y.size()) {return false;}
        for (size_t k = 0; k < x.size(); ++k) {
          if (!(x[k] == y[k]) && !(std::isnan(x[k]) && std::isnan(y[k]))) {return false;}
        }
        return true;
      };
    if (!eq(p.positions, q.positions) || !eq(p.velocities, q.velocities) || !eq(p.accelerations, q.accelerations)) {return false;}
    if (!(p.time_from_start_sec == q.time_from_start_sec) &&
      !(std::isnan(p.time_from_start_sec) && std::isnan(q.time_from_start_sec)))
    {
      return false;
    }
  }
  return true;
}

// peak finite-difference ratios of a valid trajectory, written independently of the guard
struct Peaks
{
  double v{0}, a{0}, j{0};
};

Peaks measure(const core_types::Trajectory & t, const cp::JointLimitPolicy & pol)
{
  Peaks k;
  const size_t n = t.joint_names.size(), m = t.points.size();
  std::vector<std::vector<double>> v;
  std::vector<double> dt;
  for (size_t i = 0; i + 1 < m; ++i) {
    dt.push_back(t.points[i + 1].time_from_start_sec - t.points[i].time_from_start_sec);
    v.emplace_back(n);
    for (size_t j = 0; j < n; ++j) {
      v[i][j] = (t.points[i + 1].positions[j] - t.points[i].positions[j]) / dt[i];
      k.v = std::max(k.v, std::fabs(v[i][j]) / pol.limits.at(t.joint_names[j]).velocity);
    }
  }
  for (size_t i = 0; i + 2 < m; ++i) {
    for (size_t j = 0; j < n; ++j) {
      const double a = (v[i + 1][j] - v[i][j]) / (0.5 * (dt[i] + dt[i + 1]));
      k.a = std::max(k.a, std::fabs(a) / pol.limits.at(t.joint_names[j]).acceleration);
    }
  }
  for (size_t i = 0; i + 3 < m; ++i) {
    for (size_t j = 0; j < n; ++j) {
      const double jl = pol.limits.at(t.joint_names[j]).jerk;
      if (!(jl > 0.0)) {continue;}
      const double a0 = (v[i + 1][j] - v[i][j]) / (0.5 * (dt[i] + dt[i + 1]));
      const double a1 = (v[i + 2][j] - v[i + 1][j]) / (0.5 * (dt[i + 1] + dt[i + 2]));
      k.j = std::max(k.j, std::fabs((a1 - a0) / dt[i + 1]) / jl);
    }
  }
  return k;
}
}  // namespace

TEST(JointLimitGuardFuzz, RetimeAndRejectKeepTheInvariants)
{
  Rng rng(0xC0B0001);
  int retimed = 0, rejected = 0, untouched = 0, refused = 0;
  for (int c = 0; c < kCases; ++c) {
    SCOPED_TRACE("case " + std::to_string(c));
    const size_t n = static_cast<size_t>(rng.integer(1, 4));
    cp::JointLimitPolicy pol;
    pol.safety_factor = rng.chance(0.05) ? rng.wild(0.0, 1.5, 1.0) : rng.uni(0.3, 1.0);
    pol.mode = rng.chance(0.4) ? cp::LimitMode::kReject : cp::LimitMode::kRetime;
    core_types::Trajectory t;
    for (size_t j = 0; j < n; ++j) {
      const std::string name = "j" + std::to_string(j);
      t.joint_names.push_back(name);
      if (rng.chance(0.03)) {continue;}   // a joint without a limit
      mp::JointLimit l;
      l.velocity = rng.wild(0.2, 3.0, 0.03);
      l.acceleration = rng.wild(0.5, 10.0, 0.03);
      l.jerk = rng.chance(0.5) ? 0.0 : rng.wild(1.0, 100.0, 0.03);
      pol.limits[name] = l;
    }
    if (rng.chance(0.02)) {pol.limits.clear();}
    const int np = rng.integer(0, 12);
    double time = 0.0, scale = rng.chance(0.2) ? 0.3 : rng.uni(0.01, 0.3);
    std::vector<double> q = rng.vec(n, -2.0, 2.0);
    for (int i = 0; i < np; ++i) {
      core_types::TrajectoryPoint p;
      time += scale * rng.uni(0.2, 1.5);
      p.time_from_start_sec = rng.chance(0.03) ? rng.wild(0.0, 1.0, 1.0) : time;
      for (auto & x : q) {x += rng.uni(-1.0, 1.0) * (rng.chance(0.3) ? 0.5 : 0.05);}
      p.positions = q;
      if (rng.chance(0.02)) {p.positions[0] = rng.wild(0.0, 0.0, 1.0);}
      if (rng.chance(0.02)) {p.positions.push_back(0.0);}
      if (rng.chance(0.15)) {p.velocities = rng.vec(n, -3.0, 3.0, 0.03);}
      if (rng.chance(0.15)) {p.accelerations = rng.vec(rng.chance(0.03) ? n + 1 : n, -10.0, 10.0, 0.03);}
      t.points.push_back(p);
    }
    const auto before = t;
    auto work = t;
    const auto r = cp::enforceJointLimits(work, pol);
    if (!r.ok) {
      ++refused;
      ASSERT_FALSE(r.reason_code.empty());
      ASSERT_FALSE(r.message.empty());
      ASSERT_TRUE(same(work, before)) << "a refused trajectory was modified: " << r.reason_code;
      continue;
    }
    ASSERT_TRUE(r.reason_code.empty());
    // an ok result is a valid, finite trajectory
    ASSERT_TRUE(std::isfinite(r.time_scale));
    ASSERT_GE(r.time_scale, 1.0);
    if (pol.enabled()) {
      for (size_t i = 0; i < work.points.size(); ++i) {
        ASSERT_TRUE(std::isfinite(work.points[i].time_from_start_sec)) << "non-finite time after an ok result";
        if (i > 0) {ASSERT_GT(work.points[i].time_from_start_sec, work.points[i - 1].time_from_start_sec);}
        for (const double v : work.points[i].velocities) {ASSERT_TRUE(std::isfinite(v));}
        for (const double a : work.points[i].accelerations) {ASSERT_TRUE(std::isfinite(a));}
        ASSERT_EQ(work.points[i].positions, before.points[i].positions);   // the path is never changed
      }
    }
    if (r.time_scale == 1.0) {
      ++untouched;
      ASSERT_TRUE(same(work, before));
    } else {
      ++retimed;
      ASSERT_EQ(pol.mode, cp::LimitMode::kRetime) << "reject mode changed a trajectory";
      ASSERT_NEAR(work.points.back().time_from_start_sec, before.points.back().time_from_start_sec * r.time_scale,
        1e-9 * work.points.back().time_from_start_sec);
    }
    if (pol.enabled() && work.points.size() >= 2) {
      // measured again, the result is inside limit * factor (rounding slack 1e-6)
      const Peaks pk = measure(work, pol);
      const double lim = pol.safety_factor * (1.0 + 1e-6);
      ASSERT_LE(pk.v, lim);
      ASSERT_LE(pk.a, lim);
      ASSERT_LE(pk.j, lim);
      for (const auto & p : work.points) {
        for (size_t j = 0; j < p.velocities.size(); ++j) {
          ASSERT_LE(std::fabs(p.velocities[j]), pol.limits.at(work.joint_names[j]).velocity * lim);
        }
        for (size_t j = 0; j < p.accelerations.size(); ++j) {
          ASSERT_LE(std::fabs(p.accelerations[j]), pol.limits.at(work.joint_names[j]).acceleration * lim);
        }
      }
      // retiming a retimed trajectory again changes nothing
      auto again = work;
      const auto r2 = cp::enforceJointLimits(again, pol);
      ASSERT_TRUE(r2.ok);
      ASSERT_LE(r2.time_scale, 1.0 + 1e-6);
    }
    // reject mode on the original: ok exactly when retime mode would leave it alone
    cp::JointLimitPolicy rej = pol;
    rej.mode = cp::LimitMode::kReject;
    auto w2 = before;
    const auto rr = cp::enforceJointLimits(w2, rej);
    ASSERT_TRUE(same(w2, before));
    cp::JointLimitPolicy ret = pol;
    ret.mode = cp::LimitMode::kRetime;
    auto w3 = before;
    const auto rt = cp::enforceJointLimits(w3, ret);
    ASSERT_EQ(rr.ok, rt.ok && rt.time_scale == 1.0);
    if (!rr.ok && !(rt.ok && rt.time_scale == 1.0) && !rt.ok) {
      ASSERT_EQ(rr.reason_code, rt.reason_code);
    }
    if (rr.reason_code == "jerk_violation") {ASSERT_GE(rr.max_jerk_ratio, pol.safety_factor);}
  }
  RecordProperty("retimed", retimed);
  EXPECT_GT(retimed, 500);     // the generator really exercises every branch
  EXPECT_GT(refused, 500);
  EXPECT_GT(untouched, 500);
}

TEST(JointLimitGuardFuzz, RegressionOverflowAndNonFiniteVectors)
{
  cp::JointLimitPolicy pol;
  pol.limits["a"] = {1.0, 1.0, 0.0};
  core_types::Trajectory t;
  t.joint_names = {"a"};
  auto pt = [](double q, double time) {
      core_types::TrajectoryPoint p;
      p.positions = {q};
      p.time_from_start_sec = time;
      return p;
    };
  // positions 1e300 apart: the finite-difference velocity overflows, the scale was inf and every time inf / NaN
  t.points = {pt(-1e300, 0.0), pt(1e300, 1.0)};
  auto copy = t;
  auto r = cp::enforceJointLimits(t, pol);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason_code, "invalid_trajectory");
  EXPECT_TRUE(same(t, copy));
  // a NaN velocity vector from the planner was silently ignored
  t.points = {pt(0.0, 0.0), pt(0.1, 1.0)};
  t.points[1].velocities = {std::numeric_limits<double>::quiet_NaN()};
  r = cp::enforceJointLimits(t, pol);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.reason_code, "invalid_trajectory");
  // a denormal time step: v = dq / dt overflows
  t.points = {pt(0.0, 0.0), pt(0.1, 5e-324)};
  r = cp::enforceJointLimits(t, pol);
  EXPECT_FALSE(r.ok);
}
