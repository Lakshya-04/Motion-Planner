// Host-only check that computeStep and integrate do not touch the heap. The global operator new
// is replaced by a counting one in this executable only; the count is read around the calls so
// gtest's own allocations do not matter.
#include <gtest/gtest.h>

#include <atomic>
#include <cstdlib>
#include <new>

#include "chain_helper.hpp"
#include "diff_ik_core/cartesian_tracker.hpp"
#include "diff_ik_core/diff_ik.hpp"

// The replaced operator new forwards to malloc, so GCC's pairing heuristic is wrong here.
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"

namespace
{
std::atomic<long> g_allocs{0};
}

void * operator new(std::size_t n)
{
  g_allocs.fetch_add(1, std::memory_order_relaxed);
  if (void * p = std::malloc(n ? n : 1)) {return p;}
  throw std::bad_alloc();
}
void * operator new[](std::size_t n) {return operator new(n);}
void operator delete(void * p) noexcept {std::free(p);}
void operator delete[](void * p) noexcept {std::free(p);}
void operator delete(void * p, std::size_t) noexcept {std::free(p);}
void operator delete[](void * p, std::size_t) noexcept {std::free(p);}

using namespace diff_ik_core;
using namespace chain_test;

namespace
{
DiffIkLimits limitsFor(int n)
{
  DiffIkLimits l;
  l.q_min = Eigen::VectorXd::Constant(n, -2.5);
  l.q_max = Eigen::VectorXd::Constant(n, 2.5);
  l.qd_max = Eigen::VectorXd::Constant(n, 0.5);
  l.qdd_max = Eigen::VectorXd::Constant(n, 5.0);
  l.position_margin = 0.05;
  return l;
}
}  // namespace

TEST(DiffIkAlloc, ComputeStepAndIntegrateDoNotAllocateForUpTo8Joints)
{
  const Chain c = arm7();
  const Eigen::VectorXd q = vec({0.3, 0.8, -0.4, -1.2, 0.5, 0.9, 0.2});
  const Eigen::MatrixXd J = c.jacobian(q);
  const DiffIkLimits lim = limitsFor(7);
  DiffIkParams p;
  p.null_space.push_back({std::make_shared<JointCenteringCost>(), 5.0});
  p.null_space.push_back({std::make_shared<LimitAvoidanceCost>(0.3), 1.0});
  p.null_space.push_back({std::make_shared<JointEndBiasCost>(0, 0.4), 1.0});
  Vector6d v; v << 0.05, -0.02, 0.03, 0.1, -0.05, 0.02;
  const Eigen::VectorXd qd0 = Eigen::VectorXd::Zero(7);

  // Warm up (first-call statics), then count.
  (void)computeStep(J, v, kAllAxes, q, lim, p);
  const long before = g_allocs.load();
  double sink = 0.0;
  for (int i = 0; i < 100; ++i) {
    const auto r = computeStep(J, v, kAllAxes, q, lim, p);
    const auto s = integrate(q, qd0, r.qd, 0.01, lim);
    sink += s.q(0) + r.sigma_min;
  }
  const long full = g_allocs.load() - before;
  EXPECT_EQ(full, 0) << "heap allocations in 100 calls (sink " << sink << ")";

  // Masked 3-row task, 7 joints, weights present.
  DiffIkParams pw = p;
  pw.joint_weights = Eigen::VectorXd::Ones(7);
  const AxisMask pos = {true, true, true, false, false, false};
  const long b2 = g_allocs.load();
  for (int i = 0; i < 100; ++i) {sink += computeStep(J, v, pos, q, lim, pw).sigma_min;}
  EXPECT_EQ(g_allocs.load() - b2, 0);

  // Rejected input also stays off the heap.
  Vector6d vn = v; vn(0) = std::nan("");
  const long b3 = g_allocs.load();
  EXPECT_FALSE(computeStep(J, vn, kAllAxes, q, lim, p).valid);
  EXPECT_EQ(g_allocs.load() - b3, 0);
}

TEST(DiffIkAlloc, EightJointsAreStillAllocationFree)
{
  const Chain c = planar(std::vector<double>(8, 0.3));
  const Eigen::VectorXd q = Eigen::VectorXd::Constant(8, 0.3);
  const Eigen::MatrixXd J = c.jacobian(q);
  const DiffIkLimits lim = limitsFor(8);
  DiffIkParams p;
  p.null_space.push_back({std::make_shared<JointCenteringCost>(), 5.0});
  Vector6d v = Vector6d::Zero(); v(0) = 0.02;
  const AxisMask xy = {true, true, false, false, false, true};
  (void)computeStep(J, v, xy, q, lim, p);
  const long before = g_allocs.load();
  EXPECT_TRUE(computeStep(J, v, xy, q, lim, p).valid);
  EXPECT_EQ(g_allocs.load() - before, 0);
}

TEST(DiffIkAlloc, CartesianTrackerStepIsAllocationFree)
{
  const Chain c = arm7();
  const Eigen::VectorXd q = vec({0.3, 0.8, -0.4, -1.2, 0.5, 0.9, 0.2});
  const Eigen::MatrixXd J = c.jacobian(q);
  const Eigen::Isometry3d T = c.fk(q);
  TrackerParams tp;
  tp.limits = limitsFor(7);
  tp.tool.xyz = Eigen::Vector3d(0.0, 0.01, 0.15);
  tp.plane_enabled = true;
  tp.ik.null_space.push_back({std::make_shared<JointCenteringCost>(), 1.0});
  CartesianTracker trk(tp);
  TrackerReference ref;
  ref.pose = T;
  ref.pose.translation() += Eigen::Vector3d(0.01, 0.0, 0.0);
  PlaneMeasurement m;
  m.standoff_error = 0.01;
  (void)trk.step(q, J, T, ref, m);
  const long before = g_allocs.load();
  double sink = 0.0;
  for (int i = 0; i < 100; ++i) {
    const TrackerOutput o = trk.step(q, J, T, ref, m);
    sink += o.q_next(0) + o.sigma_min + o.joint_limit_margin;
  }
  EXPECT_EQ(g_allocs.load() - before, 0) << "sink " << sink;
}

TEST(DiffIkAlloc, TheCounterSeesAHeapVector)
{
  const long before = g_allocs.load();
  double * p = new double[32];
  asm volatile("" : : "g"(p) : "memory");   // the compiler may not drop the allocation
  delete[] p;
  EXPECT_GT(g_allocs.load() - before, 0);
}
