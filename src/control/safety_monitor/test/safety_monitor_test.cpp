/**
 * @file safety_monitor_test.cpp
 * @brief SafetyMonitor: scale, thresholds, sweep, latch, fail-safe, exemption, watchdog, timing.
 * @details Default Params: T_stop 0.3 s, T_reaction 0.1 s, static margin 0.05 m, static slow band
 *          0.30 m, sweep inflation 0.02 m. A single sphere of radius 0.1 m gives a surface distance
 *          d = h - 0.12 above the plane z = 0, so d_stop = 0.05 (h = 0.17) and d_slow = 0.35
 *          (h = 0.47) for a robot at rest.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include "safety_monitor/heartbeat_watchdog.hpp"
#include "safety_monitor/safety_monitor.hpp"

using namespace safety_monitor;

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

struct OneSphere
{
  SphereModel model;
  OneSphere()
  {
    model.addLink("tip", -1, {{Vec3::Zero(), 0.1}});
  }
};

std::vector<Eigen::Isometry3d> at(double x, double y, double z)
{
  return {Eigen::Isometry3d(Eigen::Translation3d(x, y, z))};
}

StaticPrimitiveSource floorPlane(bool work_surface = false)
{
  StaticPrimitiveSource s(work_surface);
  s.addPlane(Vec3::UnitZ(), 0.0);
  return s;
}

Result run(
  SafetyMonitor & m, const std::vector<Eigen::Isometry3d> & T, const std::vector<Vec3> & v,
  const std::vector<const DistanceSource *> & src, double t)
{
  return m.evaluate(T, v, src, t, t);
}

}  // namespace

TEST(Monitor, OkWhenFar)
{
  OneSphere r;
  SafetyMonitor m(r.model, Params{});
  const auto floor = floorPlane();
  const auto res = run(m, at(0, 0, 1.0), {}, {&floor}, 1.0);
  EXPECT_EQ(res.state, State::kOk);
  EXPECT_EQ(res.reason, Reason::kNone);
  EXPECT_DOUBLE_EQ(res.speed_scale, 1.0);
  EXPECT_NEAR(res.min_distance, 1.0 - 0.12, 1e-12);
}

TEST(Monitor, StopsAtOrBelowProtectiveDistance)
{
  OneSphere r;
  SafetyMonitor m(r.model, Params{});
  const auto floor = floorPlane();
  const auto res = run(m, at(0, 0, 0.16), {}, {&floor}, 1.0);  // d = 0.04 < d_stop = 0.05
  EXPECT_EQ(res.state, State::kProtectiveStop);
  EXPECT_EQ(res.reason, Reason::kEnvironment);
  EXPECT_DOUBLE_EQ(res.speed_scale, 0.0);
}

TEST(Monitor, ScaleIsLinearInTheSlowBand)
{
  OneSphere r;
  SafetyMonitor m(r.model, Params{});
  const auto floor = floorPlane();
  // d = 0.20 -> (0.20 - 0.05) / 0.30 = 0.5
  const auto res = run(m, at(0, 0, 0.32), {}, {&floor}, 1.0);
  EXPECT_EQ(res.state, State::kSlow);
  EXPECT_EQ(res.reason, Reason::kEnvironment);
  EXPECT_NEAR(res.speed_scale, 0.5, 1e-9);
}

TEST(Monitor, ScaleMonotonicInDistance)
{
  OneSphere r;
  const auto floor = floorPlane();
  double prev = -1.0;
  for (double h = 0.10; h < 0.8; h += 0.01) {
    SafetyMonitor m(r.model, Params{});  // fresh: no latch
    const double s = run(m, at(0, 0, h), {}, {&floor}, 1.0).speed_scale;
    EXPECT_GE(s, prev - 1e-12) << "h=" << h;
    EXPECT_GE(s, 0.0);
    EXPECT_LE(s, 1.0);
    prev = s;
  }
  EXPECT_DOUBLE_EQ(prev, 1.0);
}

TEST(Monitor, ApproachSpeedWidensTheProtectiveDistance)
{
  OneSphere r;
  const auto floor = floorPlane();
  Params p;
  p.sweep_samples = 1;
  p.stop_time_s = 1e-6;  // isolate the reaction term from the sweep
  SafetyMonitor rest(r.model, p), moving(r.model, p);
  const auto a = run(rest, at(0, 0, 0.32), {}, {&floor}, 1.0);
  const auto b = run(moving, at(0, 0, 0.32), {Vec3(0, 0, -1.0)}, {&floor}, 1.0);
  EXPECT_LT(b.speed_scale, a.speed_scale);
  // d_stop grows by v * T_reaction = 0.1: (0.20 - 0.15) / 0.30
  EXPECT_NEAR(b.speed_scale, 0.05 / 0.30, 1e-4);
  // Moving away does not widen it.
  SafetyMonitor away(r.model, p);
  EXPECT_NEAR(run(away, at(0, 0, 0.32), {Vec3(0, 0, 1.0)}, {&floor}, 1.0).speed_scale, 0.5, 1e-6);
}

TEST(Monitor, PersonMarginIsStricterThanStatic)
{
  OneSphere r;
  // The same capsule shape as a static sphere obstacle and as a person, same surface distance.
  StaticPrimitiveSource obstacle;
  obstacle.addSphere(Vec3(1.0, 0, 0), 0.2);
  MovingCapsuleSource person;
  person.update({{Vec3(1.0, 0, 0), Vec3(1.0, 0, 0), 0.2, Vec3::Zero()}}, 1.0);
  SafetyMonitor ms(r.model, Params{}), mp(r.model, Params{});
  // d = 1.0 - 0.2 - 0.12 = 0.68: static is far past d_slow = 0.35, a person is not.
  const auto s = run(ms, at(0, 0, 0), {}, {&obstacle}, 1.0);
  const auto p = run(mp, at(0, 0, 0), {}, {&person}, 1.0);
  EXPECT_EQ(s.state, State::kOk);
  EXPECT_LT(p.speed_scale, s.speed_scale);
  EXPECT_EQ(p.reason, Reason::kPerson);
}

TEST(Monitor, SweepCatchesAnApproachingObstacleAStaticCheckMisses)
{
  OneSphere r;
  const auto floor = floorPlane();
  SafetyMonitor still(r.model, Params{}), diving(r.model, Params{});
  // At rest at h = 1.0 the floor is far away; at 3 m/s the sphere is on the floor after 0.3 s.
  EXPECT_EQ(run(still, at(0, 0, 1.0), {}, {&floor}, 1.0).state, State::kOk);
  const auto res = run(diving, at(0, 0, 1.0), {Vec3(0, 0, -3.0)}, {&floor}, 1.0);
  EXPECT_EQ(res.state, State::kProtectiveStop);
  EXPECT_LT(res.min_distance, 0.0);  // the swept positions penetrate
}

TEST(Monitor, SweepCatchesAnApproachingPerson)
{
  OneSphere r;
  MovingCapsuleSource idle, walking;
  idle.update({{Vec3(2, 0, -1), Vec3(2, 0, 1), 0.2, Vec3::Zero()}}, 1.0);
  walking.update({{Vec3(2, 0, -1), Vec3(2, 0, 1), 0.2, Vec3(-3.0, 0, 0)}}, 1.0);
  SafetyMonitor a(r.model, Params{}), b(r.model, Params{});
  EXPECT_EQ(run(a, at(0, 0, 0), {}, {&idle}, 1.0).state, State::kOk);
  EXPECT_EQ(run(b, at(0, 0, 0), {}, {&walking}, 1.0).state, State::kProtectiveStop);
}

TEST(Monitor, EsdfSourceDrivesTheScale)
{
  OneSphere r;
  EsdfGrid g(Vec3(-1, -1, -1), 0.05, 41, 41, 41);
  g.fill([](const Vec3 & c) {return 0.9 - c.x();});  // solid for x > 0.9
  SafetyMonitor near_m(r.model, Params{}), far_m(r.model, Params{});
  EXPECT_EQ(run(far_m, at(0, 0, 0), {}, {&g}, 1.0).state, State::kOk);   // d = 0.78 > d_slow
  // sphere at x = 0.7 -> d = 0.2 - 0.12 = 0.08, inside the band but past d_stop
  const auto res = run(near_m, at(0.7, 0, 0), {}, {&g}, 1.0);
  EXPECT_EQ(res.state, State::kSlow);
  EXPECT_NEAR(res.speed_scale, (0.08 - 0.05) / 0.30, 1e-6);
}

TEST(Monitor, ReportsWorstSphereAndLink)
{
  SphereModel model;
  model.addLink("a", -1, {{Vec3::Zero(), 0.1}});
  model.addLink("b", 0, {{Vec3::Zero(), 0.1}, {Vec3(0, 0, -0.3), 0.1}});
  std::vector<Eigen::Isometry3d> T{
    Eigen::Isometry3d(Eigen::Translation3d(0, 0, 2.0)),
    Eigen::Isometry3d(Eigen::Translation3d(0, 0, 0.6))};
  const auto floor = floorPlane();
  SafetyMonitor m(model, Params{});
  const auto res = run(m, T, {}, {&floor}, 1.0);
  EXPECT_EQ(res.worst_sphere, 2);  // third world sphere: link b, second sphere, at z = 0.3
  EXPECT_EQ(res.worst_link, 1);
  EXPECT_NEAR(res.min_distance, 0.3 - 0.12, 1e-12);
}

TEST(Monitor, SelfCollisionStopsNonAdjacentLinks)
{
  SphereModel model;
  model.addLink("l0", -1, {{Vec3::Zero(), 0.1}});
  model.addLink("l1", 0, {{Vec3::Zero(), 0.1}});
  model.addLink("l2", 1, {{Vec3::Zero(), 0.1}});
  auto pose = [](double x) {return Eigen::Isometry3d(Eigen::Translation3d(x, 0, 0));};
  SafetyMonitor m(model, Params{});
  // l0 and l2 overlap (distance 0.1 < 0.2); l1 is far. Parent-child pairs (0,1),(1,2) are ignored.
  auto res = run(m, {pose(0), pose(1.0), pose(0.1)}, {}, {}, 1.0);
  EXPECT_EQ(res.state, State::kProtectiveStop);
  EXPECT_EQ(res.reason, Reason::kSelfCollision);
  // A parent-child overlap alone is not a self collision.
  SafetyMonitor m2(model, Params{});
  res = run(m2, {pose(0), pose(0.05), pose(3.0)}, {}, {}, 1.0);
  EXPECT_EQ(res.state, State::kOk);
}

TEST(Monitor, AllowedPairIsNotChecked)
{
  SphereModel model;
  model.addLink("l0", -1, {{Vec3::Zero(), 0.1}});
  model.addLink("l1", 0, {{Vec3::Zero(), 0.1}});
  model.addLink("l2", 1, {{Vec3::Zero(), 0.1}});
  model.allowPair("l0", "l2");
  auto pose = [](double x) {return Eigen::Isometry3d(Eigen::Translation3d(x, 0, 0));};
  SafetyMonitor m(model, Params{});
  EXPECT_EQ(run(m, {pose(0), pose(1.0), pose(0.1)}, {}, {}, 1.0).state, State::kOk);
}

TEST(Monitor, ToolIsExemptOnTheWorkPlaneButNotAgainstOthers)
{
  SphereModel model;
  model.addLink("arm", -1, {{Vec3(0, 0, 1.0), 0.1}});
  model.addLink("tool", 0, {{Vec3::Zero(), 0.05}});
  model.setContactExempt("tool");
  const auto wall = floorPlane(true);
  // Tool touching the work plane (centre 0.05 above it), arm high up.
  std::vector<Eigen::Isometry3d> T{
    Eigen::Isometry3d(Eigen::Translation3d(0, 0, 1.0)),
    Eigen::Isometry3d(Eigen::Translation3d(0, 0, 0.05))};
  SafetyMonitor m(model, Params{});
  EXPECT_EQ(run(m, T, {}, {&wall}, 1.0).state, State::kOk);

  // The same plane NOT flagged as the work surface stops the robot.
  const auto plain = floorPlane(false);
  SafetyMonitor m2(model, Params{});
  EXPECT_EQ(run(m2, T, {}, {&plain}, 1.0).state, State::kProtectiveStop);

  // A non-exempt link gets no exemption on the work plane.
  SphereModel model2;
  model2.addLink("arm", -1, {{Vec3(0, 0, 1.0), 0.1}});
  model2.addLink("tool", 0, {{Vec3::Zero(), 0.05}});
  SafetyMonitor m3(model2, Params{});
  EXPECT_EQ(run(m3, T, {}, {&wall}, 1.0).state, State::kProtectiveStop);
}

TEST(Monitor, ToolIsNotExemptAgainstAPerson)
{
  SphereModel model;
  model.addLink("tool", -1, {{Vec3::Zero(), 0.05}});
  model.setContactExempt("tool");
  MovingCapsuleSource person;
  person.update({{Vec3(0.2, 0, -1), Vec3(0.2, 0, 1), 0.15, Vec3::Zero()}}, 1.0);
  SafetyMonitor m(model, Params{});
  const auto res = run(m, at(0, 0, 0), {}, {&person}, 1.0);
  EXPECT_EQ(res.state, State::kProtectiveStop);
  EXPECT_EQ(res.reason, Reason::kPerson);
  // ... nor against another static obstacle.
  StaticPrimitiveSource box;
  box.addSphere(Vec3(0.1, 0, 0), 0.05);
  SafetyMonitor m2(model, Params{});
  EXPECT_EQ(run(m2, at(0, 0, 0), {}, {&box}, 1.0).state, State::kProtectiveStop);
}

TEST(Monitor, StaleRobotDataFailsSafe)
{
  OneSphere r;
  SafetyMonitor m(r.model, Params{});
  const auto floor = floorPlane();
  const auto res = m.evaluate(at(0, 0, 1.0), {}, {&floor}, 1.0, 1.5);  // 0.5 s old > 0.25 s
  EXPECT_EQ(res.state, State::kProtectiveStop);
  EXPECT_EQ(res.reason, Reason::kStaleData);
  EXPECT_DOUBLE_EQ(res.speed_scale, 0.0);
}

TEST(Monitor, StalePersonSourceFailsSafe)
{
  OneSphere r;
  MovingCapsuleSource person;
  person.update({{Vec3(5, 5, 0), Vec3(5, 5, 1), 0.2, Vec3::Zero()}}, 1.0);
  SafetyMonitor m(r.model, Params{});
  EXPECT_EQ(m.evaluate(at(0, 0, 1), {}, {&person}, 1.4, 1.4).reason, Reason::kStaleData);
  MovingCapsuleSource never;  // never updated: unstamped
  SafetyMonitor m2(r.model, Params{});
  EXPECT_EQ(run(m2, at(0, 0, 1), {}, {&never}, 1.0).reason, Reason::kInvalidInput);
}

TEST(Monitor, NanAndBadStampsFailSafe)
{
  OneSphere r;
  const auto floor = floorPlane();
  struct Case {std::vector<Eigen::Isometry3d> T; std::vector<Vec3> v; double stamp, now;};
  const auto good = at(0, 0, 1.0);
  auto nan_T = good;
  nan_T[0].translation().x() = kNaN;
  const std::vector<Case> cases = {
    {nan_T, {}, 1.0, 1.0},
    {good, {Vec3(kNaN, 0, 0)}, 1.0, 1.0},
    {good, {}, kNaN, 1.0},
    {good, {}, -1.0, 1.0},     // absent / negative stamp
    {good, {}, 1.0, kNaN},
    {good, {}, 1.0, -2.0},
    {good, {Vec3::Zero(), Vec3::Zero()}, 1.0, 1.0},  // velocity count mismatch
    {{}, {}, 1.0, 1.0},                               // transform count mismatch
  };
  for (size_t i = 0; i < cases.size(); ++i) {
    SafetyMonitor m(r.model, Params{});
    const auto res = m.evaluate(cases[i].T, cases[i].v, {&floor}, cases[i].stamp, cases[i].now);
    EXPECT_EQ(res.state, State::kProtectiveStop) << i;
    EXPECT_EQ(res.reason, Reason::kInvalidInput) << i;
    EXPECT_DOUBLE_EQ(res.speed_scale, 0.0) << i;
  }
}

TEST(Monitor, NullSourceFailsSafe)
{
  OneSphere r;
  SafetyMonitor m(r.model, Params{});
  EXPECT_EQ(run(m, at(0, 0, 1), {}, {nullptr}, 1.0).reason, Reason::kInvalidInput);
}

TEST(Monitor, StopLatchesAndNeedsClearTimeAndExplicitReset)
{
  OneSphere r;
  SafetyMonitor m(r.model, Params{});  // min_clear_time 1.0 s
  const auto floor = floorPlane();
  EXPECT_EQ(run(m, at(0, 0, 0.1), {}, {&floor}, 1.0).state, State::kProtectiveStop);
  EXPECT_TRUE(m.latched());
  // The hazard is gone, but the stop holds (no reset, clear time not elapsed).
  auto res = run(m, at(0, 0, 1.0), {}, {&floor}, 1.1);
  EXPECT_EQ(res.state, State::kProtectiveStop);
  EXPECT_DOUBLE_EQ(res.speed_scale, 0.0);
  EXPECT_EQ(res.reason, Reason::kEnvironment);
  // Clear time elapsed but no reset: still latched.
  EXPECT_EQ(run(m, at(0, 0, 1.0), {}, {&floor}, 2.5).state, State::kProtectiveStop);
  // Explicit reset releases it on the next cycle.
  m.reset();
  res = run(m, at(0, 0, 1.0), {}, {&floor}, 2.6);
  EXPECT_EQ(res.state, State::kOk);
  EXPECT_FALSE(m.latched());
}

TEST(Monitor, ResetBeforeClearTimeWaitsForIt)
{
  OneSphere r;
  SafetyMonitor m(r.model, Params{});
  const auto floor = floorPlane();
  run(m, at(0, 0, 0.1), {}, {&floor}, 1.0);
  run(m, at(0, 0, 1.0), {}, {&floor}, 1.1);  // clear starts at 1.1
  m.reset();
  EXPECT_EQ(run(m, at(0, 0, 1.0), {}, {&floor}, 1.5).state, State::kProtectiveStop);  // 0.4 s
  EXPECT_EQ(run(m, at(0, 0, 1.0), {}, {&floor}, 2.2).state, State::kOk);              // 1.1 s
}

TEST(Monitor, ResetDuringTheHazardIsDiscarded)
{
  OneSphere r;
  SafetyMonitor m(r.model, Params{});
  const auto floor = floorPlane();
  run(m, at(0, 0, 0.1), {}, {&floor}, 1.0);
  m.reset();  // hazard still present: ignored
  run(m, at(0, 0, 0.1), {}, {&floor}, 1.1);
  EXPECT_EQ(run(m, at(0, 0, 1.0), {}, {&floor}, 5.0).state, State::kProtectiveStop);
  EXPECT_EQ(run(m, at(0, 0, 1.0), {}, {&floor}, 7.0).state, State::kProtectiveStop);
}

TEST(Monitor, HazardReturningDuringClearTimeRestartsIt)
{
  OneSphere r;
  SafetyMonitor m(r.model, Params{});
  const auto floor = floorPlane();
  run(m, at(0, 0, 0.1), {}, {&floor}, 1.0);
  run(m, at(0, 0, 1.0), {}, {&floor}, 1.1);
  run(m, at(0, 0, 0.1), {}, {&floor}, 1.8);  // hazard again: clear timer restarts
  run(m, at(0, 0, 1.0), {}, {&floor}, 1.9);
  m.reset();
  EXPECT_EQ(run(m, at(0, 0, 1.0), {}, {&floor}, 2.5).state, State::kProtectiveStop);
  EXPECT_EQ(run(m, at(0, 0, 1.0), {}, {&floor}, 3.0).state, State::kOk);
}

TEST(Monitor, FailSafeStopAlsoLatches)
{
  OneSphere r;
  SafetyMonitor m(r.model, Params{});
  const auto floor = floorPlane();
  EXPECT_EQ(m.evaluate(at(0, 0, 1), {}, {&floor}, 1.0, 3.0).reason, Reason::kStaleData);
  EXPECT_EQ(run(m, at(0, 0, 1), {}, {&floor}, 3.1).reason, Reason::kStaleData);  // latched reason
  run(m, at(0, 0, 1), {}, {&floor}, 3.2);
  m.reset();
  EXPECT_EQ(run(m, at(0, 0, 1), {}, {&floor}, 4.5).state, State::kOk);
}

TEST(Monitor, Deterministic)
{
  OneSphere r;
  StaticPrimitiveSource s;
  s.addSphere(Vec3(0.5, 0.1, 0), 0.1);
  s.addBox(Eigen::Isometry3d(Eigen::Translation3d(0, 1, 0)), Vec3(0.2, 0.2, 0.2));
  const std::vector<Vec3> v{Vec3(0.3, -0.1, 0.2)};
  SafetyMonitor a(r.model, Params{}), b(r.model, Params{});
  const auto x = run(a, at(0, 0.2, 0.1), v, {&s}, 1.0);
  const auto y = run(b, at(0, 0.2, 0.1), v, {&s}, 1.0);
  EXPECT_EQ(x.speed_scale, y.speed_scale);
  EXPECT_EQ(x.min_distance, y.min_distance);
  EXPECT_EQ(x.worst_sphere, y.worst_sphere);
  const auto x2 = run(a, at(0, 0.2, 0.1), v, {&s}, 1.0);
  EXPECT_EQ(x.speed_scale, x2.speed_scale);
}

TEST(Monitor, TimingSanity60SpheresAgainst200Obstacles)
{
  SphereModel model;
  int parent = -1;
  for (int l = 0; l < 6; ++l) {
    std::vector<Sphere> sp;
    for (int k = 0; k < 10; ++k) {sp.push_back({Vec3(0.05 * k, 0, 0), 0.05});}
    parent = model.addLink("l" + std::to_string(l), parent, sp);
  }
  std::vector<Eigen::Isometry3d> T;
  for (int l = 0; l < 6; ++l) {
    T.emplace_back(Eigen::Translation3d(0.0, 0.3 * l, 1.0));
  }
  StaticPrimitiveSource obstacles;
  for (int i = 0; i < 200; ++i) {
    obstacles.addSphere(Vec3(2.0 + 0.1 * (i % 20), -2.0 + 0.2 * (i / 20), 0.5), 0.05);
  }
  EXPECT_EQ(model.sphereCount(), 60);
  EXPECT_EQ(obstacles.size(), 200u);
  SafetyMonitor m(model, Params{});
  const std::vector<Vec3> v(60, Vec3(0.1, 0, 0));
  const auto t0 = std::chrono::steady_clock::now();
  Result res;
  for (int i = 0; i < 10; ++i) {res = run(m, T, v, {&obstacles}, 1.0);}
  const double ms_each =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / 10;
  EXPECT_LT(ms_each, 10.0);  // 60 spheres x 200 obstacles must fit a 100 Hz loop with room to spare
  EXPECT_TRUE(std::isfinite(res.min_distance));
}

TEST(Heartbeat, ExpiresWhenNeverBeatenOrLate)
{
  HeartbeatWatchdog w(0.5);
  EXPECT_TRUE(w.expired(0.0));  // never beaten
  w.beat(1.0);
  EXPECT_FALSE(w.expired(1.0));
  EXPECT_FALSE(w.expired(1.5));
  EXPECT_TRUE(w.expired(1.51));
  w.beat(1.6);
  EXPECT_FALSE(w.expired(1.7));
}

TEST(Heartbeat, InvalidTimesAreExpiredAndNotBeats)
{
  HeartbeatWatchdog w(0.5);
  w.beat(kNaN);
  w.beat(-1.0);
  EXPECT_TRUE(w.expired(0.1));
  w.beat(2.0);
  EXPECT_TRUE(w.expired(kNaN));
  EXPECT_TRUE(w.expired(1.0));  // clock went backwards
  EXPECT_FALSE(w.expired(2.2));
}

// ---------------------------------------------------------------------------------------------
// Regression tests for the fail-safe review findings.
// ---------------------------------------------------------------------------------------------

namespace
{
// A person source that is also flagged as the work surface (a misconfigured custom source).
struct PersonWorkSurface : DistanceSource
{
  MovingCapsuleSource c;
  DistanceQuery query(const Vec3 & p, double t) const override {return c.query(p, t);}
  SourceKind kind() const override {return SourceKind::kPerson;}
  bool isWorkSurface() const override {return true;}
  double stamp() const override {return c.stamp();}
};

MovingCapsuleSource personAt(double x, double vx, double stamp)
{
  MovingCapsuleSource p;
  p.update({{Vec3(x, 0, -1), Vec3(x, 0, 1), 0.2, Vec3(vx, 0, 0)}}, stamp);
  return p;
}
}  // namespace

TEST(Monitor, FutureDataStampFailsSafe)
{
  OneSphere r;
  SafetyMonitor m(r.model, Params{});
  const auto floor = floorPlane();
  const auto res = m.evaluate(at(0, 0, 1), {}, {&floor}, 1e6, 1.0);
  EXPECT_DOUBLE_EQ(res.speed_scale, 0.0);
  EXPECT_EQ(res.reason, Reason::kInvalidInput);
  // within the jitter epsilon is still accepted
  SafetyMonitor m2(r.model, Params{});
  EXPECT_EQ(m2.evaluate(at(0, 0, 1), {}, {&floor}, 1.0005, 1.0).state, State::kOk);
}

TEST(Monitor, FutureSourceStampFailsSafe)
{
  OneSphere r;
  SafetyMonitor m(r.model, Params{});
  const auto person = personAt(5.0, 0.0, 1e6);
  const auto res = m.evaluate(at(0, 0, 1), {}, {&person}, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(res.speed_scale, 0.0);
  EXPECT_EQ(res.reason, Reason::kInvalidInput);
}

TEST(Monitor, NaNSphereRadiusIsRejectedAtConstruction)
{
  SphereModel m;
  m.addLink("tip", -1, {});
  EXPECT_THROW(m.attachSphere("tip", "payload", {Vec3(0, 0, -0.3), kNaN}), std::invalid_argument);
  EXPECT_THROW(m.addLink("x", -1, {{Vec3::Zero(), kNaN}}), std::invalid_argument);
  EXPECT_THROW(
    m.addLink("y", -1, {{Vec3(kNaN, 0, 0), 0.1}}), std::invalid_argument);
}

TEST(Monitor, NegativeSphereRadiusIsRejected)
{
  SphereModel m;
  m.addLink("tip", -1, {});
  EXPECT_THROW(m.attachSphere("tip", "p", {Vec3::Zero(), -0.1}), std::invalid_argument);
  EXPECT_THROW(m.addLink("x", -1, {{Vec3::Zero(), -0.1}}), std::invalid_argument);
  EXPECT_EQ(m.sphereCount(), 0);
}

namespace
{
// Bypasses model validation: a source whose answer has a NaN gradient-free distance.
struct NaNScaleSource : DistanceSource
{
  DistanceQuery query(const Vec3 &, double) const override
  {
    DistanceQuery q;
    q.valid = true;
    q.distance = kNaN;
    return q;
  }
};
}  // namespace

TEST(Monitor, NaNDistanceIsAStop)
{
  OneSphere r;
  SafetyMonitor m(r.model, Params{});
  NaNScaleSource s;
  const auto res = m.evaluate(at(0, 0, 1), {}, {&s}, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(res.speed_scale, 0.0);
  EXPECT_EQ(res.state, State::kProtectiveStop);
}

TEST(Monitor, EsdfNaNVoxelFailsSafe)
{
  OneSphere r;
  EsdfGrid g(Vec3(-1, -1, -1), 0.1, 21, 21, 21);
  g.fill([](const Vec3 &) {return -0.5;});  // robot inside an obstacle
  g.set(10, 10, 10, static_cast<float>(kNaN));
  g.set(11, 10, 10, std::numeric_limits<float>::infinity());
  SafetyMonitor m(r.model, Params{});
  const auto res = m.evaluate(at(0.05, 0, 0), {}, {&g}, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(res.speed_scale, 0.0);
  EXPECT_EQ(res.reason, Reason::kInvalidInput);
  const DistanceQuery q = g.query(Vec3(0.05, 0, 0), 0.0);
  EXPECT_TRUE(q.valid);
  EXPECT_TRUE(std::isnan(q.distance));
}

TEST(Monitor, EsdfOutsideGridDefaultStopsAndFreeIsOptIn)
{
  OneSphere r;
  EsdfGrid g(Vec3(-1, -1, -1), 0.1, 21, 21, 21);
  g.fill([](const Vec3 &) {return 5.0;});
  SafetyMonitor m(r.model, Params{});
  EXPECT_EQ(g.outsideBehavior(), OutsideGrid::kUnknownFailsSafe);
  auto res = m.evaluate(at(10, 0, 0), {}, {&g}, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(res.speed_scale, 0.0);
  EXPECT_EQ(res.reason, Reason::kInvalidInput);
  g.setOutsideBehavior(OutsideGrid::kFree);
  SafetyMonitor m2(r.model, Params{});
  EXPECT_EQ(m2.evaluate(at(10, 0, 0), {}, {&g}, 1.0, 1.0).state, State::kOk);
  EXPECT_EQ(m2.evaluate(at(0, 0, 0), {}, {&g}, 1.0, 1.0).state, State::kOk);
}

TEST(Monitor, EsdfSetStampImpliesStaleness)
{
  OneSphere r;
  EsdfGrid g(Vec3(-1, -1, -1), 0.1, 21, 21, 21);
  g.fill([](const Vec3 &) {return 5.0;});
  g.setStamp(0.0);
  SafetyMonitor m(r.model, Params{});
  const auto res = m.evaluate(at(0, 0, 0), {}, {&g}, 100.0, 100.0);
  EXPECT_DOUBLE_EQ(res.speed_scale, 0.0);
  EXPECT_EQ(res.reason, Reason::kStaleData);
  g.setStamp(100.0);
  SafetyMonitor m2(r.model, Params{});
  EXPECT_EQ(m2.evaluate(at(0, 0, 0), {}, {&g}, 100.0, 100.0).state, State::kOk);
  g.setDynamic(false);  // explicit opt-out
  SafetyMonitor m3(r.model, Params{});
  EXPECT_EQ(m3.evaluate(at(0, 0, 0), {}, {&g}, 500.0, 500.0).state, State::kOk);
}

TEST(Monitor, ParamsValidation)
{
  OneSphere r;
  auto bad = [&](auto && set) {
      Params p;
      set(p);
      EXPECT_THROW(SafetyMonitor(r.model, p), std::invalid_argument);
    };
  bad([](Params & p) {p.stale_timeout_s = kNaN;});
  bad([](Params & p) {p.stop_time_s = -1.0;});
  bad([](Params & p) {p.reaction_time_s = -0.1;});
  bad([](Params & p) {p.static_slow_band_m = kNaN;});
  bad([](Params & p) {p.person_slow_band_m = -0.5;});  // inverted band
  bad([](Params & p) {p.self_slow_band_m = -0.01;});
  bad([](Params & p) {p.human_speed_mps = std::numeric_limits<double>::infinity();});
  bad([](Params & p) {p.static_margin_m = kNaN;});
  bad([](Params & p) {p.min_clear_time_s = -1.0;});
  bad([](Params & p) {p.sweep_samples = -1;});
  bad([](Params & p) {p.sweep_radius_inflation_m = kNaN;});
  EXPECT_NO_THROW(SafetyMonitor(r.model, Params{}));
}

TEST(Heartbeat, ConstructorValidatesTimeout)
{
  EXPECT_THROW(HeartbeatWatchdog{kNaN}, std::invalid_argument);
  EXPECT_THROW(HeartbeatWatchdog{-0.5}, std::invalid_argument);
  EXPECT_THROW(
    HeartbeatWatchdog{std::numeric_limits<double>::infinity()}, std::invalid_argument);
  EXPECT_NO_THROW(HeartbeatWatchdog{0.5});
}

TEST(Monitor, PersonIsExtrapolatedOverDataAge)
{
  OneSphere r;
  // Fresh data: the person is at x = 1.0 walking toward the robot at 1.6 m/s.
  const auto fresh = personAt(1.0, -1.6, 1.0);
  // The same person reported 0.25 s ago, when it was at x = 1.4.
  const auto aged = personAt(1.4, -1.6, 0.75);
  SafetyMonitor m1(r.model, Params{});
  SafetyMonitor m2(r.model, Params{});
  const auto a = m1.evaluate(at(0, 0, 0), {}, {&fresh}, 1.0, 1.0);
  const auto b = m2.evaluate(at(0, 0, 0), {}, {&aged}, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(a.speed_scale, 0.0);
  EXPECT_DOUBLE_EQ(b.speed_scale, a.speed_scale);
  EXPECT_EQ(b.state, a.state);
  EXPECT_EQ(b.reason, Reason::kPerson);
}

TEST(Monitor, IsoProtectiveDistanceIncludesHumanReactionTravel)
{
  OneSphere r;
  // Standing person, surface distance 0.78 m (centre 1.10 - capsule 0.2 - sphere 0.1 - inflation
  // 0.02). ISO form: 1.6 * (0.3 + 0.1) + 0.2 = 0.84 m > 0.78 m, so stop. (The old form without
  // the reaction term gave d_stop = 0.68 m and scale 0.167.)
  const auto person = personAt(1.10, 0.0, 1.0);
  SafetyMonitor m(r.model, Params{});
  const auto res = m.evaluate(at(0, 0, 0), {}, {&person}, 1.0, 1.0);
  EXPECT_NEAR(res.min_distance, 0.78, 1e-9);
  EXPECT_DOUBLE_EQ(res.speed_scale, 0.0);
  EXPECT_EQ(res.reason, Reason::kPerson);
  // just outside d_stop: 0.86 m slows but does not stop
  const auto farther = personAt(1.18, 0.0, 1.0);
  SafetyMonitor m2(r.model, Params{});
  const auto r2 = m2.evaluate(at(0, 0, 0), {}, {&farther}, 1.0, 1.0);
  EXPECT_GT(r2.speed_scale, 0.0);
  EXPECT_LT(r2.speed_scale, 1.0);
}

TEST(Monitor, ContactExemptionNeverAppliesToPersons)
{
  SphereModel model;
  model.addLink("tool", -1, {{Vec3::Zero(), 0.1}});
  model.setContactExempt("tool");
  PersonWorkSurface ps;
  ps.c.update({{Vec3(0, 0, -1), Vec3(0, 0, 1), 0.2, Vec3::Zero()}}, 1.0);
  SafetyMonitor m(model, Params{});
  const auto res = m.evaluate(at(0, 0, 0), {}, {&ps}, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(res.speed_scale, 0.0);
  EXPECT_EQ(res.reason, Reason::kPerson);
  // a plain work surface still exempts
  const auto floor = floorPlane(true);
  SafetyMonitor m2(model, Params{});
  EXPECT_EQ(m2.evaluate(at(0, 0, 0.05), {}, {&floor}, 1.0, 1.0).state, State::kOk);
}
