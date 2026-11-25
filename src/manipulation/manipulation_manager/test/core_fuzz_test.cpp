// Property / fuzz tests of the ROS-free decision cores that gate real arm and lift motion.
//
// Fixed seeds, 20000 random cases per test, adversarial inputs (NaN, inf, huge, zero, negative, size mismatches,
// empty vectors). Every test states its invariants as asserts; a failure prints the seed and case number. Built to be
// run with -fsanitize=address,undefined as well (see the package CMakeLists / report for the standalone command).
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "fuzz_util.hpp"
#include "manipulation_manager/core/column_ik_gate.hpp"
#include "manipulation_manager/core/dock_ik_selector.hpp"
#include "manipulation_manager/core/geometric_coverage.hpp"
#include "manipulation_manager/core/recovery_branch.hpp"
#include "manipulation_manager/core/recovery_jog.hpp"
#include "manipulation_manager/core/stand_check.hpp"
#include "manipulation_manager/core/stand_gate.hpp"

namespace manipulation_manager
{
namespace
{
using fuzz::kCases;
using fuzz::Rng;
using V = std::vector<double>;
const double kNaN = std::numeric_limits<double>::quiet_NaN();

bool allFinite(const V & v)
{
  for (const double x : v) {
    if (!std::isfinite(x)) {return false;}
  }
  return true;
}

bool sameBits(const V & a, const V & b)
{
  return a.size() == b.size() && (a.empty() || std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0);
}

// ---------------------------------------------------------------------------------------------------- stand_check

TEST(CoreFuzz, StandCheckDecisionsAreConsistent)
{
  Rng rng(0x51A0D001);
  for (int c = 0; c < kCases; ++c) {
    SCOPED_TRACE("case " + std::to_string(c));
    StandCheckConfig cfg;
    cfg.min_tol_m = rng.wild(0.0, 0.1);
    cfg.max_tol_m = rng.wild(0.0, 0.5);
    cfg.margin_scale = rng.wild(0.0, 2.0);
    cfg.fallback_tol_m = rng.wild(0.0, 0.5);
    cfg.yaw_tol_rad = rng.wild(0.0, 0.5);
    cfg.max_redocks = rng.integer(-2, 4);
    const StandCheck check(cfg);
    const auto & k = check.config();
    // the sanitised config is usable
    ASSERT_GE(k.max_redocks, 0);
    ASSERT_FALSE(std::isnan(k.min_tol_m));
    ASSERT_FALSE(std::isnan(k.max_tol_m));
    ASSERT_FALSE(std::isnan(k.fallback_tol_m));
    ASSERT_FALSE(std::isnan(k.yaw_tol_rad));
    ASSERT_LE(k.min_tol_m, k.max_tol_m);

    const double margin = rng.wild(-0.2, 0.6);
    const double tol = check.toleranceM(margin);
    ASSERT_FALSE(std::isnan(tol)) << "margin " << margin;
    ASSERT_GE(tol, k.min_tol_m);
    ASSERT_LE(tol, k.max_tol_m);

    StandAchieved a;
    a.position_error_m = rng.wild(0.0, 0.4);
    a.yaw_error_rad = rng.wild(-0.6, 0.6);
    const int redocks = rng.integer(-1, 4);
    const auto r = check.evaluate(a, margin, redocks);
    ASSERT_FALSE(r.message.empty());
    const bool valid = std::isfinite(a.position_error_m) && std::isfinite(a.yaw_error_rad) && a.position_error_m >= 0.0;
    if (!valid) {
      ASSERT_EQ(r.decision, StandDecision::kAbort);
      ASSERT_EQ(r.reason, StandReason::kInvalidMeasurement);
      ASSERT_TRUE(r.stand_failed);
      continue;
    }
    const bool within = a.position_error_m <= r.tol_m && std::fabs(a.yaw_error_rad) <= r.tol_yaw_rad;
    ASSERT_EQ(r.decision == StandDecision::kGo, within) << "pos " << a.position_error_m << " tol " << r.tol_m;
    ASSERT_EQ(r.stand_failed, r.decision == StandDecision::kAbort);
    if (r.decision == StandDecision::kRedock) {ASSERT_LT(redocks, k.max_redocks);}
    if (r.decision == StandDecision::kAbort) {ASSERT_GE(redocks, k.max_redocks);}
  }
}

TEST(CoreFuzz, StandCheckRegressionNonFiniteToleranceNeverGoes)
{
  // fallback_tol_m = NaN made the tolerance NaN: every error compared "not bad" and the arm was allowed to move.
  StandCheckConfig cfg;
  cfg.fallback_tol_m = kNaN;
  StandCheck check(cfg);
  EXPECT_FALSE(std::isnan(check.toleranceM(-1.0)));
  EXPECT_EQ(check.evaluate({0.4, 0.0}, -1.0, 0).decision, StandDecision::kRedock);
  // margin_scale = inf with a zero margin: inf * 0 = NaN
  cfg = StandCheckConfig{};
  cfg.margin_scale = std::numeric_limits<double>::infinity();
  StandCheck check2(cfg);
  EXPECT_FALSE(std::isnan(check2.toleranceM(0.0)));
  EXPECT_NE(check2.evaluate({0.4, 0.0}, 0.0, 0).decision, StandDecision::kGo);
}

// ---------------------------------------------------------------------------------------------------- stand_gate

class FuzzNav : public StandNavigatorPort
{
public:
  bool navigate(std::string & message) override
  {
    ++drives;
    if (fail_drive) {
      message = fail_msg;
      return false;
    }
    if (next < seq.size()) {cur = seq[next++];}
    return true;
  }
  StandAchieved achieved() const override {return cur;}
  bool stopRequested() const override {return stop;}
  StandAchieved cur;
  std::vector<StandAchieved> seq;
  size_t next{0};
  bool fail_drive{false};
  std::string fail_msg;
  bool stop{false};
  int drives{0};
};

TEST(CoreFuzz, StandGateTerminatesAndOnlyMovesWithinTolerance)
{
  Rng rng(0x51A0D002);
  for (int c = 0; c < kCases; ++c) {
    SCOPED_TRACE("case " + std::to_string(c));
    StandCheckConfig cfg;
    cfg.max_redocks = rng.integer(0, 4);
    const StandCheck check(cfg);
    FuzzNav nav;
    nav.cur = {rng.wild(0.0, 0.3), rng.wild(-0.3, 0.3)};
    const int n = rng.integer(0, 6);
    for (int i = 0; i < n; ++i) {nav.seq.push_back({rng.wild(0.0, 0.3), rng.wild(-0.3, 0.3)});}
    nav.fail_drive = rng.chance(0.15);
    nav.fail_msg = rng.chance(0.5) ? "drive failed" : "";
    nav.stop = rng.chance(0.1);
    const double margin = rng.wild(-0.2, 0.5);
    const auto out = runStandGate(nav, check, margin);
    ASSERT_LE(out.redocks, cfg.max_redocks + 1);
    ASSERT_EQ(nav.drives, nav.stop ? 0 : out.redocks);
    ASSERT_FALSE(out.ok && out.abort);
    if (out.ok) {
      const auto r = check.evaluate(nav.achieved(), margin, out.redocks);
      ASSERT_EQ(r.decision, StandDecision::kGo) << "ok but the last arrival is outside the tolerance";
      ASSERT_LE(out.redocks, cfg.max_redocks);
    } else {
      ASSERT_FALSE(out.message.empty());
    }
    // resolveNavigation never answers kRetry for a stand the gate gave up
    const bool gate_abort = rng.chance(0.5);
    TaskStateMachine m2(rng.integer(1, 4));
    m2.startMulti();
    m2.standsPlanned(static_cast<size_t>(rng.integer(1, 3)));
    const auto v = resolveNavigation(m2, rng.chance(0.3), gate_abort);
    if (gate_abort) {ASSERT_NE(v, NavVerdict::kRetry);}
  }
}

// ------------------------------------------------------------------------------------------------- dock_ik_selector

struct IkCall
{
  V seed;
  std::optional<V> sol;
};

TEST(CoreFuzz, SelectDockIkMatchesAnIndependentOracleAndNeverReturnsABadCandidate)
{
  Rng rng(0x51A0D003);
  for (int c = 0; c < kCases; ++c) {
    SCOPED_TRACE("case " + std::to_string(c));
    const size_t n = static_cast<size_t>(rng.integer(2, 7));
    DockIkSelectorParams p;
    p.linear_joints.clear();
    if (rng.chance(0.8)) {p.linear_joints.push_back(0);}
    if (rng.chance(0.2)) {p.linear_joints.push_back(static_cast<size_t>(rng.integer(0, 8)));}
    const int nb = rng.integer(0, 3);
    for (int i = 0; i < nb; ++i) {p.branch_joints.push_back(static_cast<size_t>(rng.integer(0, static_cast<int>(n) + 1)));}
    p.branch_deadband_rad = rng.uni(0.0, 0.2);
    p.wrist_joint = rng.integer(-1, static_cast<int>(n));
    p.wrist_min_rad = rng.uni(-4.0, 0.0);
    p.wrist_max_rad = rng.uni(0.0, 4.0);
    p.max_dq_rad = rng.wild(0.1, 4.0, 0.03);
    const bool want_reach = rng.chance(0.5);
    DockReachContext ctx;
    ctx.horizontal_m = rng.wild(0.0, 1.5, 0.03);
    ctx.z_rel_m = rng.wild(-1.0, 2.0, 0.03);
    if (want_reach) {
      p.reach_target_fraction = rng.uni(0.2, 1.3);
      p.reach_m = rng.uni(0.3, 1.6);
      p.lift_joint = static_cast<size_t>(rng.integer(0, static_cast<int>(n) + 1));
      p.lift_min_m = rng.uni(-0.2, 0.5);
      p.lift_max_m = rng.uni(0.0, 1.8);
    }
    V reference = rng.vec(n, -3.0, 3.0, 0.02);
    if (rng.chance(0.03)) {reference.clear();}
    const int ns = rng.integer(0, 4);
    std::vector<V> seeds;
    for (int i = 0; i < ns; ++i) {
      seeds.push_back(rng.chance(0.1) ? V{} : rng.vec(rng.chance(0.1) ? n + 1 : n, -3.0, 3.0, 0.03));
    }
    std::vector<IkCall> calls;
    Rng ikr(rng.integer(0, 1 << 30));
    const DockIkFn ik = [&](const V & seed) -> std::optional<V> {
        IkCall call;
        call.seed = seed;
        const double r = ikr.uni(0.0, 1.0);
        if (r < 0.15) {
          call.sol = std::nullopt;
        } else if (r < 0.2) {
          call.sol = V{};
        } else if (r < 0.25) {
          call.sol = ikr.vec(n + 1, -3.0, 3.0);
        } else {
          V s = (!reference.empty() && ikr.chance(0.6)) ? reference : ikr.vec(n, -3.0, 3.0);
          if (s.size() != n) {s = ikr.vec(n, -3.0, 3.0);}
          for (auto & x : s) {x += ikr.uni(-1.0, 1.0) * (ikr.chance(0.5) ? 0.2 : 2.5);}
          if (ikr.chance(0.07)) {s[static_cast<size_t>(ikr.integer(0, static_cast<int>(n) - 1))] = ikr.wild(0.0, 0.0, 1.0);}
          call.sol = s;
        }
        calls.push_back(call);
        return call.sol;
      };
    const DockIkChoice ch = selectDockIk(ik, seeds, reference, p, want_reach ? &ctx : nullptr);

    // 1. independent invariants of an accepted candidate
    if (ch.ok) {
      ASSERT_FALSE(reference.empty());
      ASSERT_TRUE(allFinite(reference));
      ASSERT_EQ(ch.q.size(), reference.size());
      ASSERT_TRUE(allFinite(ch.q)) << "a non-finite IK solution was accepted";
      ASSERT_FALSE(dockBranchChanged(ch.q, reference, p));
      if (p.wrist_joint >= 0 && static_cast<size_t>(p.wrist_joint) < ch.q.size()) {
        const double w = ch.q[static_cast<size_t>(p.wrist_joint)];
        ASSERT_GE(w, p.wrist_min_rad);
        ASSERT_LE(w, p.wrist_max_rad);
      }
      ASSERT_TRUE(std::isfinite(ch.cost));
      ASSERT_TRUE(ch.cost <= p.max_dq_rad * std::max(1.0, p.lift_cost_weight)) << "cost " << ch.cost << " max " << p.max_dq_rad;
      ASSERT_DOUBLE_EQ(ch.cost, dockIkCost(ch.q, reference, p));
      ASSERT_LT(ch.seed_index, seeds.size() * 2);
    } else {
      ASSERT_TRUE(ch.q.empty());
    }
    ASSERT_LE(ch.rejected_branch + ch.rejected_wrist + ch.rejected_cost, ch.candidates);
    ASSERT_EQ(ch.ok, ch.candidates > ch.rejected_branch + ch.rejected_wrist + ch.rejected_cost);

    // 2. oracle over the recorded IK calls
    const bool prefer = want_reach && dockReachPreferenceOn(p) && p.lift_joint < reference.size();
    size_t cand = 0, rw = 0, rb = 0, rc = 0;
    const V * best = nullptr;
    double best_cost = 0.0;
    bool best_under = false;
    for (const auto & call : calls) {
      if (!call.sol || call.sol->size() != reference.size()) {continue;}
      ++cand;
      const V & s = *call.sol;
      if (p.wrist_joint >= 0 && static_cast<size_t>(p.wrist_joint) < s.size()) {
        const double w = s[static_cast<size_t>(p.wrist_joint)];
        if (w < p.wrist_min_rad || w > p.wrist_max_rad) {++rw; continue;}
      }
      if (dockBranchChanged(s, reference, p)) {++rb; continue;}
      const double cost = dockIkCost(s, reference, p);
      if (!(dockIkCost(s, reference, p, false) <= p.max_dq_rad)) {++rc; continue;}
      bool under = true;
      if (prefer) {under = dockReachFraction(s[p.lift_joint], ctx, p) <= p.reach_target_fraction + 1e-9;}
      if (best == nullptr || (under && !best_under) || (under == best_under && cost < best_cost)) {
        best = &s;
        best_cost = cost;
        best_under = under;
      }
    }
    if (!reference.empty() && !allFinite(reference)) {
      ASSERT_FALSE(ch.ok);
      continue;
    }
    ASSERT_EQ(ch.candidates, cand);
    ASSERT_EQ(ch.rejected_wrist, rw);
    ASSERT_EQ(ch.rejected_branch, rb);
    ASSERT_EQ(ch.rejected_cost, rc);
    ASSERT_EQ(ch.ok, best != nullptr);
    if (ch.ok) {
      ASSERT_TRUE(sameBits(ch.q, *best)) << "not the best acceptable candidate";
      // the reach preference: under the target unless the fallback is flagged
      if (prefer) {
        ASSERT_EQ(ch.reach_over_target, !best_under);
        if (!ch.reach_over_target) {ASSERT_LE(ch.reach_fraction, p.reach_target_fraction + 1e-9);}
      } else {
        ASSERT_FALSE(ch.reach_over_target);
      }
    }
  }
}

TEST(CoreFuzz, SelectDockIkRegressionNanSolutionIsRejected)
{
  // A NaN IK solution had cost 0 (std::max(0, NaN) keeps 0), passed the branch and wrist checks and won.
  DockIkSelectorParams p;
  const V ref{0.5, 0.1, -0.2};
  const DockIkFn ik = [&](const V &) -> std::optional<V> {return V{kNaN, 0.1, -0.2};};
  const auto ch = selectDockIk(ik, {ref}, ref, p);
  EXPECT_FALSE(ch.ok);
  // and a NaN reference accepts nothing
  const DockIkFn ik2 = [&](const V &) -> std::optional<V> {return V{0.5, 0.1, -0.2};};
  EXPECT_FALSE(selectDockIk(ik2, {ref}, V{kNaN, 0.1, -0.2}, p).ok);
  // a NaN max_dq accepts nothing
  p.max_dq_rad = kNaN;
  EXPECT_FALSE(selectDockIk(ik2, {ref}, ref, p).ok);
}

TEST(CoreFuzz, IdealLiftStaysInsideTheLiftRangeAndKeepsTheTargetWhenPossible)
{
  Rng rng(0x51A0D004);
  for (int c = 0; c < kCases; ++c) {
    SCOPED_TRACE("case " + std::to_string(c));
    DockIkSelectorParams p;
    p.reach_target_fraction = rng.uni(0.2, 1.2);
    p.reach_m = rng.uni(0.2, 2.0);
    p.lift_min_m = rng.uni(-0.3, 1.0);
    p.lift_max_m = rng.uni(-0.3, 1.8);
    DockReachContext ctx{rng.uni(0.0, 2.0), rng.uni(-1.0, 2.5)};
    const double cur = rng.uni(-1.0, 2.5);
    const double v = idealLift(cur, ctx, p);
    ASSERT_TRUE(std::isfinite(v));
    ASSERT_GE(v, std::min(p.lift_min_m, p.lift_max_m));
    ASSERT_LE(v, std::max(p.lift_min_m, p.lift_max_m));
    // when the target is reachable inside the range, the ideal lift reaches it
    const double lo = std::min(p.lift_min_m, p.lift_max_m), hi = std::max(p.lift_min_m, p.lift_max_m);
    double best = INFINITY;
    for (int i = 0; i <= 200; ++i) {best = std::min(best, dockReachFraction(lo + (hi - lo) * i / 200.0, ctx, p));}
    if (best <= p.reach_target_fraction - 1e-3) {
      ASSERT_LE(dockReachFraction(v, ctx, p), p.reach_target_fraction + 1e-6);
    }
    // never farther from `cur` than needed: if cur already satisfies the target (and is in range) it is kept
    if (cur >= lo && cur <= hi && dockReachFraction(cur, ctx, p) <= p.reach_target_fraction) {
      ASSERT_NEAR(v, cur, 1e-9);
    }
  }
}

// ----------------------------------------------------------------------------------------------- column_ik_gate

class FuzzPort : public ColumnIkCheckPort
{
public:
  ColumnIkVerdict check(
    const std::vector<core_types::Pose> &, const std::array<double, 3> &, const std::vector<double> & seed) override
  {
    seeds.push_back(seed);
    ColumnIkVerdict v;
    const double r = rng->uni(0.0, 1.0);
    if (r < 0.25) {
      v.solved = true;
      v.continuous = true;
    } else if (r < 0.6) {
      v.solved = true;
      v.failure = "branch_break";
      v.split_count = static_cast<size_t>(rng->integer(1, 4));
    } else {
      v.failure = "refused";
      v.failure_pose = static_cast<size_t>(rng->integer(0, 5));
    }
    v.max_continuity_ratio = rng->uni(0.0, 5.0);
    verdicts.push_back(v);
    return v;
  }
  Rng * rng{nullptr};
  std::vector<std::vector<double>> seeds;
  std::vector<ColumnIkVerdict> verdicts;
};

TEST(CoreFuzz, ColumnIkGateNeverRejectsAContinuousColumnAndNeverHandsOutABadSeed)
{
  Rng rng(0x51A0D005);
  for (int c = 0; c < kCases; ++c) {
    SCOPED_TRACE("case " + std::to_string(c));
    ColumnIkGateConfig cfg;
    cfg.lift_joint = static_cast<size_t>(rng.integer(0, 4));
    const int nl = rng.integer(0, 4);
    for (int i = 0; i < nl; ++i) {cfg.lift_candidates.push_back(rng.wild(0.0, 1.5, 0.1));}
    cfg.reject_on_refused = rng.chance(0.5);
    cfg.reject_on_break = rng.chance(0.7);
    const ColumnIkGate gate(cfg);
    FuzzPort port;
    port.rng = &rng;
    const std::vector<core_types::Pose> strokes(static_cast<size_t>(rng.integer(0, 3)));
    V seed = rng.vec(static_cast<size_t>(rng.integer(0, 4)), -1.0, 1.0);
    const bool null_port = rng.chance(0.03);
    const auto d = gate.decide(null_port ? nullptr : &port, strokes, {0.0, 0.0, 1.0}, seed);
    ASSERT_FALSE(d.reason.empty());
    if (null_port || strokes.empty() || seed.empty()) {
      ASSERT_EQ(d.action, ColumnIkAction::kNotChecked);
      ASSERT_EQ(d.attempts, 0u);
      ASSERT_TRUE(port.seeds.empty());
      continue;
    }
    ASSERT_EQ(d.attempts, port.seeds.size());
    ASSERT_GE(d.attempts, 1u);
    ASSERT_LE(d.attempts, 1 + cfg.lift_candidates.size());
    ASSERT_EQ(port.seeds[0], seed);
    bool any_cont = false;
    for (const auto & v : port.verdicts) {any_cont = any_cont || v.continuous;}
    for (const auto & s : port.seeds) {
      ASSERT_EQ(s.size(), seed.size());
      ASSERT_TRUE(allFinite(s)) << "a non-finite lift height was sent to the IK";
      for (size_t i = 0; i < s.size(); ++i) {
        if (i != cfg.lift_joint) {ASSERT_EQ(s[i], seed[i]);}
      }
    }
    if (any_cont) {
      // a column with a continuous chain from any seed must never be rejected
      ASSERT_TRUE(d.action == ColumnIkAction::kGo || d.action == ColumnIkAction::kGoWithSeed);
      ASSERT_TRUE(d.verdict.continuous);
    }
    if (d.action == ColumnIkAction::kGoWithSeed) {
      ASSERT_TRUE(d.verdict.continuous);
      ASSERT_TRUE(allFinite(d.seed));
      ASSERT_EQ(d.seed, port.seeds.back());   // the first continuous attempt ends the search
    }
    if (d.action == ColumnIkAction::kGo && d.verdict.continuous) {ASSERT_EQ(d.attempts, 1u);}
    if (d.action == ColumnIkAction::kReject) {ASSERT_FALSE(any_cont);}
    if (!any_cont) {ASSERT_EQ(d.seed, seed);}
  }
}

// --------------------------------------------------------------------------------------------------- recovery_jog

TEST(CoreFuzz, RecoveryJogStaysFiniteInsideLimitsAndTimed)
{
  Rng rng(0x51A0D006);
  for (int c = 0; c < kCases; ++c) {
    SCOPED_TRACE("case " + std::to_string(c));
    const size_t n = static_cast<size_t>(rng.integer(0, 7));
    RecoveryJogParams p;
    p.max_step_rad = rng.wild(0.001, 0.2, 0.05);
    p.max_speed = rng.wild(0.01, 1.0, 0.05);
    p.min_duration_s = rng.wild(0.0, 2.0, 0.05);
    p.max_total_rad = rng.wild(0.1, 4.0, 0.05);
    p.done_tolerance_rad = rng.wild(0.0, 0.01, 0.05);
    p.limit_margin = rng.wild(0.0, 0.05, 0.05);
    const size_t nl = rng.chance(0.2) ? static_cast<size_t>(rng.integer(0, 8)) : n;
    if (rng.chance(0.8)) {
      p.lower = rng.vec(nl, -3.0, 0.0);
      p.upper = rng.vec(rng.chance(0.2) ? nl + 1 : nl, -0.1, 3.0);
    }
    V cur = rng.vec(n, -3.2, 3.2, 0.03);
    V good = rng.vec(rng.chance(0.05) ? n + 1 : n, -3.2, 3.2, 0.03);
    if (rng.chance(0.3) && cur.size() == good.size()) {
      for (size_t i = 0; i < cur.size(); ++i) {good[i] = cur[i] + rng.uni(-0.3, 0.3);}
    }
    const RecoveryJog jog = planRecoveryJog(cur, good, p);
    if (!jog.ok) {
      ASSERT_FALSE(jog.reason.empty());
      ASSERT_TRUE(jog.points.empty());
      continue;
    }
    ASSERT_TRUE(allFinite(cur) && allFinite(good)) << "a jog was planned from a non-finite state";
    ASSERT_EQ(jog.target.size(), cur.size());
    ASSERT_TRUE(allFinite(jog.target));
    const double margin = std::max(0.0, p.limit_margin);
    for (size_t i = 0; i < jog.target.size(); ++i) {
      if (i < p.lower.size() && i < p.upper.size() && std::isfinite(p.lower[i]) && std::isfinite(p.upper[i])) {
        if (p.upper[i] - p.lower[i] > 2.0 * margin) {
          ASSERT_GE(jog.target[i], p.lower[i] + margin - 1e-12);
          ASSERT_LE(jog.target[i], p.upper[i] - margin + 1e-12);
        } else {
          // a range too narrow for the margin (or inverted) keeps its middle
          ASSERT_GE(jog.target[i], std::min(p.lower[i], p.upper[i]) - 1e-12);
          ASSERT_LE(jog.target[i], std::max(p.lower[i], p.upper[i]) + 1e-12);
        }
      }
    }
    if (!jog.needed) {
      ASSERT_TRUE(jog.points.empty());
      continue;
    }
    ASSERT_FALSE(jog.points.empty());
    double dmax = 0.0;
    for (size_t i = 0; i < cur.size(); ++i) {dmax = std::max(dmax, std::fabs(jog.target[i] - cur[i]));}
    ASSERT_LE(dmax, p.max_total_rad + 1e-12);
    V prev = cur;
    double prev_t = 0.0;
    for (const auto & pt : jog.points) {
      ASSERT_EQ(pt.positions.size(), cur.size());
      ASSERT_TRUE(allFinite(pt.positions));
      ASSERT_TRUE(std::isfinite(pt.time_from_start_s));
      ASSERT_GT(pt.time_from_start_s, prev_t) << "time must increase";
      double step = 0.0, w = 0.0;
      for (size_t i = 0; i < cur.size(); ++i) {step = std::max(step, std::fabs(pt.positions[i] - prev[i]));}
      ASSERT_LE(step, 1.5 * p.max_step_rad * (1.0 + 1e-9) + 1e-12);   // the smoothstep ramp peaks at slope 1.5
      // the speed between waypoints stays under max_speed (peak slope 1.5 is already in the duration)
      const double dt = pt.time_from_start_s - prev_t;
      for (size_t i = 0; i < cur.size(); ++i) {w = std::max(w, std::fabs(pt.positions[i] - prev[i]) / dt);}
      ASSERT_LE(w, p.max_speed * (1.0 + 1e-6));
      prev = pt.positions;
      prev_t = pt.time_from_start_s;
    }
    for (size_t i = 0; i < cur.size(); ++i) {ASSERT_NEAR(prev[i], jog.target[i], 1e-9 * (1.0 + std::fabs(jog.target[i])));}
    ASSERT_GE(prev_t, p.min_duration_s - 1e-12);
  }
}

TEST(CoreFuzz, RecoveryJogRegressionNonFiniteInputsAreRefused)
{
  RecoveryJogParams p;
  // a NaN joint in `good` was skipped by std::max: dmax stayed small and the jog was "ok" with a NaN target
  RecoveryJog j = planRecoveryJog({0.0, 0.0}, {kNaN, 0.0}, p);
  EXPECT_FALSE(j.ok);
  j = planRecoveryJog({kNaN, 0.0}, {0.0, 0.1}, p);
  EXPECT_FALSE(j.ok);
  // NaN step size: the step count cast from NaN was undefined behaviour
  p.max_step_rad = kNaN;
  EXPECT_FALSE(planRecoveryJog({0.0}, {0.5}, p).ok);
  p = RecoveryJogParams{};
  p.max_step_rad = 1e-300;   // 1e300 waypoints
  EXPECT_FALSE(planRecoveryJog({0.0}, {0.5}, p).ok);
  // a negative margin must not push the target outside the limits
  p = RecoveryJogParams{};
  p.lower = {-1.0};
  p.upper = {1.0};
  p.limit_margin = -0.5;
  j = planRecoveryJog({0.0}, {5.0}, p);
  ASSERT_TRUE(j.ok);
  EXPECT_LE(j.target[0], 1.0);
  // a non-finite joint is "near the limits"
  EXPECT_TRUE(nearJointLimits({kNaN}, {-1.0}, {1.0}, 0.01));
}

// ------------------------------------------------------------------------------------------------ recovery_branch

TEST(CoreFuzz, RecoveryBranchTargetsAreOnTheLiveBranch)
{
  Rng rng(0x51A0D007);
  for (int c = 0; c < kCases; ++c) {
    SCOPED_TRACE("case " + std::to_string(c));
    const size_t n = static_cast<size_t>(rng.integer(1, 7));
    DockIkSelectorParams p;
    p.linear_joints = {0};
    for (int i = 0; i < rng.integer(0, 3); ++i) {p.branch_joints.push_back(static_cast<size_t>(rng.integer(0, 8)));}
    p.wrist_joint = rng.integer(-1, static_cast<int>(n));
    p.wrist_min_rad = rng.uni(-4.0, 0.0);
    p.wrist_max_rad = rng.uni(0.0, 4.0);
    p.max_dq_rad = rng.uni(0.1, 4.0);
    p.branch_deadband_rad = rng.uni(0.0, 0.2);
    auto mk = [&]() {
        const int k = rng.integer(0, 9);
        return k == 0 ? V{} : rng.vec(k == 1 ? n + 1 : n, -3.2, 3.2, 0.04);
      };
    const V live = rng.chance(0.03) ? V{} : rng.vec(n, -3.2, 3.2, 0.03);
    const V good = mk(), seed = mk();
    const auto t = selectRecoveryJogTarget(live, good, seed, p);
    if (t.ok) {
      ASSERT_TRUE(allFinite(t.q)) << "a non-finite recovery configuration was chosen";
      ASSERT_TRUE(allFinite(live));
      ASSERT_TRUE(recoveryConfigOnBranch(t.q, live, p));
      ASSERT_FALSE(dockBranchChanged(t.q, live, p));
      ASSERT_TRUE(t.q == good || t.q == seed);
      ASSERT_EQ(t.from_branch_seed, !recoveryConfigOnBranch(good, live, p));
    } else {
      ASSERT_TRUE(t.q.empty());
      ASSERT_FALSE(recoveryConfigOnBranch(good, live, p));
      ASSERT_FALSE(recoveryConfigOnBranch(seed, live, p));
    }
    // the goal after a recovery: from the live state, on the live branch
    Rng ikr(static_cast<uint64_t>(c) + 99);
    const DockIkFn ik = [&](const V &) -> std::optional<V> {
        if (ikr.chance(0.2)) {return std::nullopt;}
        return ikr.vec(n, -3.2, 3.2, 0.05);
      };
    const auto g = selectRecoveryGoal(ik, live, seed, p);
    if (g.ok) {
      ASSERT_TRUE(allFinite(g.q));
      ASSERT_EQ(g.q.size(), live.size());
      ASSERT_FALSE(dockBranchChanged(g.q, live, p));
    }
  }
}

// ----------------------------------------------------------------------------------------- geometric_coverage

GeoPoint gp(Rng & r, double span, double p_bad)
{
  return {r.wild(-span, span, p_bad), r.wild(-span, span, p_bad), r.wild(-span, span, p_bad)};
}

TEST(CoreFuzz, GeometricCoverageIsBoundedAndSamplesOnlyHelp)
{
  Rng rng(0x51A0D008);
  const int cases = 6000;   // the grid work per case dominates; 6000 keeps the test under 5 s
  for (int c = 0; c < cases; ++c) {
    SCOPED_TRACE("case " + std::to_string(c));
    GeometricCoverageConfig cfg;
    cfg.swath_m = rng.wild(0.02, 0.3, 0.04);
    cfg.cell_m = rng.wild(0.02, 0.1, 0.04);
    cfg.contact_force_n = rng.wild(-1.0, 5.0, 0.04);
    cfg.max_gap_m = rng.wild(0.0, 0.3, 0.04);
    cfg.max_standoff_m = rng.wild(0.0, 0.1, 0.04);
    cfg.plane_origin = gp(rng, 0.5, 0.02);
    cfg.wall_normal = rng.chance(0.5) ? GeoPoint{1.0, 0.0, 0.0} : gp(rng, 1.0, 0.05);
    cfg.max_cells = static_cast<size_t>(rng.integer(0, 3) == 0 ? rng.integer(0, 50) : 20000);
    const int ncol = rng.integer(0, 4);
    std::vector<PlannedStroke> planned;
    for (int i = 0; i < rng.integer(0, 5); ++i) {
      PlannedStroke s;
      s.column = static_cast<size_t>(rng.integer(0, ncol));
      for (int k = 0; k < rng.integer(0, 5); ++k) {s.points.push_back(gp(rng, rng.chance(0.03) ? 1e6 : 0.6, 0.04));}
      planned.push_back(s);
    }
    std::vector<ToolSample> samples;
    for (int i = 0; i < rng.integer(0, 30); ++i) {
      ToolSample s;
      s.column = static_cast<size_t>(rng.integer(0, ncol));
      s.p = gp(rng, rng.chance(0.03) ? 1e30 : 0.6, 0.04);
      s.force_n = rng.wild(-1.0, 10.0, 0.05);
      samples.push_back(s);
    }
    if (rng.chance(0.3)) {cfg.groups[0] = {0, 1, 99};}
    const auto r = computeGeometricCoverage(planned, samples, cfg);
    ASSERT_EQ(r.samples, samples.size());
    ASSERT_LE(r.working_samples, samples.size());
    ASSERT_LE(r.covered_cells, r.planned_cells);
    ASSERT_TRUE(std::isfinite(r.percent));
    ASSERT_GE(r.percent, 0.0);
    ASSERT_LE(r.percent, 100.0 + 1e-9);
    if (!r.available) {
      ASSERT_FALSE(r.note.empty());
      continue;
    }
    ASSERT_GT(r.planned_cells, 0u);
    ASSERT_NEAR(r.percent, 100.0 * static_cast<double>(r.covered_cells) / static_cast<double>(r.planned_cells), 1e-9);
    for (const auto & kv : r.column_percent) {
      ASSERT_TRUE(std::isfinite(kv.second));
      ASSERT_GE(kv.second, 0.0);
      ASSERT_LE(kv.second, 100.0 + 1e-9);
    }
    for (const auto & kv : r.group) {
      ASSERT_LE(kv.second.covered_cells, kv.second.planned_cells);
      ASSERT_LE(kv.second.planned_cells, r.planned_cells);
      ASSERT_LE(kv.second.percent, 100.0 + 1e-9);
    }
    // more executed samples in a column of their own can only add coverage
    auto more = samples;
    for (int i = 0; i < 5; ++i) {
      ToolSample s;
      s.column = 1000;
      s.p = gp(rng, 0.6, 0.0);
      s.force_n = 100.0;
      more.push_back(s);
    }
    const auto r2 = computeGeometricCoverage(planned, more, cfg);
    ASSERT_EQ(r2.available, r.available);
    ASSERT_EQ(r2.planned_cells, r.planned_cells);
    ASSERT_GE(r2.covered_cells, r.covered_cells);
  }
}

TEST(CoreFuzz, GeometricCoverageDenseExecutionOfThePlanIsComplete)
{
  Rng rng(0x51A0D009);
  for (int c = 0; c < 1500; ++c) {
    SCOPED_TRACE("case " + std::to_string(c));
    GeometricCoverageConfig cfg;
    cfg.swath_m = rng.uni(0.05, 0.2);
    cfg.cell_m = rng.uni(0.02, 0.05);
    cfg.contact_force_n = 1.0;
    cfg.max_gap_m = 0.05;
    cfg.max_standoff_m = 0.02;
    cfg.max_cells = 100000;
    // wall: plane x = 0.5, normal +x; planned strokes are vertical lines on it
    cfg.plane_origin = {0.5, 0.0, 0.0};
    std::vector<PlannedStroke> planned;
    std::vector<ToolSample> samples;
    const int ncol = rng.integer(1, 4);
    for (int col = 0; col < ncol; ++col) {
      const double y = rng.uni(-0.5, 0.5), z0 = rng.uni(0.0, 0.5), z1 = z0 + rng.uni(0.0, 0.8);
      PlannedStroke s;
      s.column = static_cast<size_t>(col);
      s.points = {{0.5, y, z0}, {0.5, y, z1}};
      planned.push_back(s);
      for (double z = z0; z <= z1 + 1e-9; z += 0.04) {samples.push_back({static_cast<size_t>(col), {0.5, y, std::min(z, z1)}, 20.0});}
      samples.push_back({static_cast<size_t>(col), {0.5, y, z1}, 20.0});
    }
    const auto r = computeGeometricCoverage(planned, samples, cfg);
    ASSERT_TRUE(r.available);
    ASSERT_DOUBLE_EQ(r.percent, 100.0);
    // the same samples with no force cover nothing
    for (auto & s : samples) {s.force_n = 0.0;}
    ASSERT_DOUBLE_EQ(computeGeometricCoverage(planned, samples, cfg).percent, 0.0);
  }
}

TEST(CoreFuzz, GeometricCoverageRegressionDegenerateGrids)
{
  GeometricCoverageConfig cfg;
  const std::vector<PlannedStroke> planned{{0, {{0.0, 0.0, 0.0}, {0.0, 0.5, 0.0}}}};
  // max_cells = 0 never satisfied the coarsening loop: the cell size doubled to infinity (undefined behaviour)
  cfg.max_cells = 0;
  EXPECT_FALSE(computeGeometricCoverage(planned, {}, cfg).available);
  // a planned point 1e30 m away: the grid index overflowed a long
  const std::vector<PlannedStroke> far{{0, {{0.0, 0.0, 0.0}, {0.0, 1e30, 0.0}}}};
  cfg = GeometricCoverageConfig{};
  const auto r = computeGeometricCoverage(far, {}, cfg);
  EXPECT_LE(r.covered_cells, r.planned_cells);
  // a tool sample 1e30 m away from a normal patch
  const auto r2 = computeGeometricCoverage(planned, {{0, {0.0, 1e30, 0.0}, 10.0}}, cfg);
  EXPECT_LE(r2.covered_cells, r2.planned_cells);
}

}  // namespace
}  // namespace manipulation_manager
