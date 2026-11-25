// Monte Carlo benchmark of split avoidance on planar fake arms. ROS-free, header-only, shared by
// apps/split_benchmark.cpp and test/core/split_benchmark_test.cpp.
//
// Model (what is measured, and what is not):
//   - A planar arm in the x-y plane: a vertical prismatic lift (joint 0, y axis) plus 2 or 3 revolute links
//     (joints 1..n). The 2-link arm has a free tool angle; the 3-link arm must hold a tool angle phi. Closed
//     form IK returns both elbow branches (elbow sign of joint 2), filtered by joint limits.
//   - A column is a stroke along y on a wall at x = standoff, preceded by an approach pose and followed by a
//     retract pose (pulled back 0.15 m). Standoff, stroke start, stroke length and (3-link) tool angle are
//     drawn at random per column. Columns with a pose that has no IK solution on any branch are discarded
//     (counted), so every kept column is "whole" in the reachability sense and only the joint path can split.
//   - The lift tracks the pose height with a fixed gain (lift = clamp(0.5 y)), independent of the branch, so
//     the lift travel is nearly the same for every method here; it is reported but does not discriminate.
//   - Methods: (a) per-pose IK taking the first solution (fixed order: elbow + first; "rand": a random
//     order per pose, as a solver that converges to either basin), (b) per-pose IK taking the lowest cost
//     solution (sum |q|), (c) ColumnIkChain (previous-solution seeding, two folded seeds). (a) and (b) are
//     evaluated by the same ColumnIkChain metrics (an IK that returns the pre-selected solution and no
//     folded seeds), so all methods are scored by one yardstick.
//   Limits: planar fakes with a hand-picked break rule (ratio limit, elbow sign change); not a real arm, no
//   collision, no orientation redundancy, nothing here ran on hardware.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "reachability/core/column_ik_chain.hpp"
#include "reachability/core/path_quality_audit.hpp"

namespace reachability
{
namespace split_bench
{

struct Setting
{
  std::string name;
  int links{3};          ///< 2 or 3 revolute links.
  bool tight{false};     ///< Tighter joint limits (they take one branch away for part of the workspace).
};

inline std::vector<Setting> defaultSettings()
{
  return {{"2link_loose", 2, false}, {"2link_tight", 2, true},
    {"3link_loose", 3, false}, {"3link_tight", 3, true}};
}

enum Method {kFirstFixed = 0, kFirstRandom, kLowestCost, kChain, kMethodCount};

inline const char * methodName(int m)
{
  static const char * n[] = {"first_fixed", "first_random", "lowest_cost", "chain"};
  return n[m];
}

struct MethodStats
{
  std::string setting;
  std::string method;
  int columns{0};
  int discarded{0};          ///< Candidate columns dropped for a pose without any IK solution.
  int whole{0};              ///< Columns with a continuous joint path.
  int breaks{0};             ///< Sum of split_count.
  int branch_changes{0};
  double mean_ratio{0.0};    ///< Mean of per-column max continuity ratio.
  double p95_ratio{0.0};
  double mean_lift_m{0.0};
  int near_singular{0};      ///< Columns whose stroke auditPathQuality flags near_singular.
  int near_limit{0};
  int worse_than_chain{0};   ///< Columns where this method breaks more than the chain (chain row: 0).
  int better_than_chain{0};  ///< Columns where this method breaks less than the chain.
  bool operator==(const MethodStats & o) const
  {
    return setting == o.setting && method == o.method && columns == o.columns && discarded == o.discarded &&
           whole == o.whole && breaks == o.breaks && branch_changes == o.branch_changes &&
           mean_ratio == o.mean_ratio && p95_ratio == o.p95_ratio && mean_lift_m == o.mean_lift_m &&
           near_singular == o.near_singular && near_limit == o.near_limit &&
           worse_than_chain == o.worse_than_chain && better_than_chain == o.better_than_chain;
  }
};

namespace detail
{

struct Arm
{
  int n{3};
  std::array<double, 3> len{0.45, 0.40, 0.20};
  std::vector<double> lo, hi;     ///< Per joint including the lift at index 0.
};

inline Arm makeArm(const Setting & s)
{
  Arm a;
  a.n = s.links;
  if (s.links == 2) {a.len = {0.55, 0.50, 0.0};}
  if (s.tight) {
    a.lo = {-0.3, -1.0, -2.2, -2.0};
    a.hi = {0.3, 1.6, 2.2, 2.0};
  } else {
    a.lo = {-0.3, -2.8, -2.6, -2.6};
    a.hi = {0.3, 2.8, 2.6, 2.6};
  }
  a.lo.resize(static_cast<size_t>(s.links) + 1);
  a.hi.resize(static_cast<size_t>(s.links) + 1);
  return a;
}

inline double angleOf(const Pose & p) {return std::atan2(p.linear()(1, 0), p.linear()(0, 0));}

inline Pose makePose(double x, double y, double phi)
{
  Pose p = Pose::Identity();
  p.translation() = Vec3(x, y, 0.0);
  p.linear() = Eigen::AngleAxisd(phi, Vec3::UnitZ()).toRotationMatrix();
  return p;
}

inline double wrapPi(double a) {return std::remainder(a, 2.0 * M_PI);}

inline double liftFor(const Arm & a, double y) {return std::clamp(0.5 * y, a.lo[0], a.hi[0]);}

/// Both elbow branches (elbow + first) within the joint limits; may be empty.
inline std::vector<JointVec> solveAll(const Arm & a, const Pose & t)
{
  std::vector<JointVec> out;
  const double lift = liftFor(a, t.translation().y());
  const double x = t.translation().x(), y = t.translation().y() - lift;
  double wx = x, wy = y, phi = 0.0;
  double l1 = a.len[0], l2 = a.len[1];
  if (a.n == 3) {
    phi = angleOf(t);
    wx -= a.len[2] * std::cos(phi);
    wy -= a.len[2] * std::sin(phi);
  }
  const double c = (wx * wx + wy * wy - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
  if (c > 1.0 || c < -1.0) {return out;}
  for (const int br : {1, -1}) {
    const double q2 = br * std::acos(c);
    const double q1 = std::atan2(wy, wx) - std::atan2(l2 * std::sin(q2), l1 + l2 * std::cos(q2));
    JointVec q(a.n + 1);
    q[0] = lift;
    q[1] = wrapPi(q1);
    q[2] = q2;
    if (a.n == 3) {q[3] = wrapPi(phi - q1 - q2);}
    bool ok = true;
    for (Eigen::Index i = 0; i < q.size(); ++i) {
      if (q[i] < a.lo[static_cast<size_t>(i)] - 1e-9 || q[i] > a.hi[static_cast<size_t>(i)] + 1e-9) {ok = false;}
    }
    if (ok) {out.push_back(q);}
  }
  return out;
}

inline Eigen::MatrixXd jacobian(const Arm & a, const std::vector<double> & q)
{
  // Revolute joints only (lift locked): position rows, plus the tool angle row for 3 links.
  const int n = a.n;
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(n == 3 ? 3 : 2, n);
  double acc = 0.0;
  std::array<double, 3> ang{};
  for (int i = 0; i < n; ++i) {acc += q[static_cast<size_t>(i) + 1]; ang[static_cast<size_t>(i)] = acc;}
  for (int j = 0; j < n; ++j) {
    double dx = 0.0, dy = 0.0;
    for (int k = j; k < n; ++k) {
      dx -= a.len[static_cast<size_t>(k)] * std::sin(ang[static_cast<size_t>(k)]);
      dy += a.len[static_cast<size_t>(k)] * std::cos(ang[static_cast<size_t>(k)]);
    }
    J(0, j) = dx;
    J(1, j) = dy;
    if (n == 3) {J(2, j) = 1.0;}
  }
  return J;
}

inline double sumAbs(const JointVec & q) {return q.cwiseAbs().sum();}

struct Column
{
  std::vector<Pose> poses;
  size_t stroke_begin{0}, stroke_end{0};   ///< Stroke poses are [begin, end).
};

inline Column randomColumn(std::mt19937_64 & rng, int links)
{
  std::uniform_real_distribution<double> U(0.0, 1.0);
  const double d = 0.40 + 0.55 * U(rng);
  const double y0 = -0.45 + 0.80 * U(rng);
  const double len = 0.30 + 0.60 * U(rng);
  const double phi = links == 3 ? -0.4 + 0.8 * U(rng) : 0.0;
  Column c;
  const double pull = 0.15, step = 0.03;
  c.poses.push_back(makePose(d - pull, y0, phi));
  c.stroke_begin = 1;
  for (double y = y0; y <= y0 + len + 1e-9; y += step) {c.poses.push_back(makePose(d, y, phi));}
  c.stroke_end = c.poses.size();
  c.poses.push_back(makePose(d - pull, y0 + len, phi));
  return c;
}

inline ColumnIkChainParams chainParams(const Arm & a)
{
  ColumnIkChainParams p;
  p.joint_kinds.push_back(JointKind::kPrismatic);
  for (int i = 0; i < a.n; ++i) {p.joint_kinds.push_back(JointKind::kRevolute);}
  p.branch_joints = {2};
  p.initial_seed = JointVec::Zero(a.n + 1);
  p.initial_seed[2] = 1.2;
  JointVec s1 = JointVec::Zero(a.n + 1), s2 = s1;
  s1[1] = 0.3; s1[2] = 1.4;
  s2[1] = 0.3; s2[2] = -1.4;
  p.folded_seeds = {s1, s2};
  return p;
}

struct ColumnResult
{
  bool whole{false};
  int breaks{0};
  int branch_changes{0};
  double ratio{0.0};
  double lift{0.0};
  bool near_singular{false};
  bool near_limit{false};
};

inline ColumnResult score(const Arm & a, const Column & col, const ColumnIkChainResult & r)
{
  ColumnResult o;
  o.whole = r.continuous();
  o.breaks = static_cast<int>(r.split_count);
  o.branch_changes = static_cast<int>(r.branch_changes);
  o.ratio = r.max_continuity_ratio;
  o.lift = r.lift_travel_m;
  PathStroke st;
  st.id = "stroke";
  for (size_t i = col.stroke_begin; i < col.stroke_end && i < r.steps.size(); ++i) {
    if (r.steps[i].solved) {st.waypoints.emplace_back(r.steps[i].q.data(), r.steps[i].q.data() + r.steps[i].q.size());}
  }
  JacobianFn jf = [&a](const std::vector<double> & q, Eigen::MatrixXd & J) {J = jacobian(a, q); return true;};
  LimitMarginFn mf = [&a](const std::vector<double> & q) {
      // Revolute joints only: the lift saturating at its range is the lift policy, not an arm limit.
      return limitMarginFromBounds(
        std::vector<double>(a.lo.begin() + 1, a.lo.end()), std::vector<double>(a.hi.begin() + 1, a.hi.end()),
        std::vector<double>(q.begin() + 1, q.end()));
    };
  const auto rep = auditPathQuality({st}, jf, mf);
  o.near_singular = rep.near_singular > 0;
  o.near_limit = rep.near_limit > 0;
  return o;
}

}  // namespace detail

/// Runs `n` kept columns of one setting through all methods. Deterministic in (setting, n, seed).
inline std::vector<MethodStats> runSetting(const Setting & s, int n, uint64_t seed)
{
  using namespace detail;
  const Arm arm = makeArm(s);
  std::mt19937_64 rng(seed);
  std::mt19937_64 order_rng(seed ^ 0x9e3779b97f4a7c15ULL);
  const ColumnIkChainParams full = chainParams(arm);
  ColumnIkChainParams plain = full;
  plain.folded_seeds.clear();

  std::vector<MethodStats> out(kMethodCount);
  std::vector<std::vector<double>> ratios(kMethodCount);
  std::vector<std::vector<ColumnResult>> per(kMethodCount);
  int discarded = 0;
  int guard = 0;
  while (static_cast<int>(per[0].size()) < n && ++guard < n * 200) {
    const Column col = randomColumn(rng, s.links);
    std::vector<std::vector<JointVec>> sols;
    bool ok = true;
    for (const auto & p : col.poses) {
      sols.push_back(solveAll(arm, p));
      if (sols.back().empty()) {ok = false; break;}
    }
    if (!ok) {++discarded; continue;}

    // The pre-selected solution per pose for (a) and (b).
    std::array<std::vector<JointVec>, 3> pick;
    for (const auto & ss : sols) {
      pick[kFirstFixed].push_back(ss.front());
      std::vector<JointVec> shuffled = ss;
      std::shuffle(shuffled.begin(), shuffled.end(), order_rng);
      pick[kFirstRandom].push_back(shuffled.front());
      pick[kLowestCost].push_back(*std::min_element(ss.begin(), ss.end(),
        [](const JointVec & u, const JointVec & v) {return sumAbs(u) < sumAbs(v);}));
    }
    for (int m = 0; m < 3; ++m) {
      const auto & chosen = pick[static_cast<size_t>(m)];
      auto lookup = [&col, &chosen](const Pose & t, const JointVec &) -> std::optional<JointVec> {
          for (size_t i = 0; i < col.poses.size(); ++i) {
            if (col.poses[i].translation() == t.translation()) {return chosen[i];}
          }
          return std::nullopt;
        };
      ColumnIkChain chain(plain, lookup);
      per[static_cast<size_t>(m)].push_back(score(arm, col, chain.solve(col.poses)));
    }
    auto nearest = [&arm](const Pose & t, const JointVec & seed) -> std::optional<JointVec> {
        const auto ss = solveAll(arm, t);
        if (ss.empty()) {return std::nullopt;}
        return *std::min_element(ss.begin(), ss.end(), [&seed](const JointVec & u, const JointVec & v) {
            return (u - seed).norm() < (v - seed).norm();
          });
      };
    ColumnIkChain chain(full, nearest);
    per[kChain].push_back(score(arm, col, chain.solve(col.poses)));
  }

  for (int m = 0; m < kMethodCount; ++m) {
    MethodStats & st = out[static_cast<size_t>(m)];
    st.setting = s.name;
    st.method = methodName(m);
    st.columns = static_cast<int>(per[static_cast<size_t>(m)].size());
    st.discarded = discarded;
    std::vector<double> rs;
    for (size_t i = 0; i < per[static_cast<size_t>(m)].size(); ++i) {
      const auto & c = per[static_cast<size_t>(m)][i];
      const auto & ch = per[kChain][i];
      st.whole += c.whole ? 1 : 0;
      st.breaks += c.breaks;
      st.branch_changes += c.branch_changes;
      st.mean_ratio += c.ratio;
      st.mean_lift_m += c.lift;
      st.near_singular += c.near_singular ? 1 : 0;
      st.near_limit += c.near_limit ? 1 : 0;
      st.worse_than_chain += c.breaks > ch.breaks ? 1 : 0;
      st.better_than_chain += c.breaks < ch.breaks ? 1 : 0;
      rs.push_back(c.ratio);
    }
    if (st.columns > 0) {
      st.mean_ratio /= st.columns;
      st.mean_lift_m /= st.columns;
      std::sort(rs.begin(), rs.end());
      st.p95_ratio = rs[static_cast<size_t>(0.95 * (rs.size() - 1))];
    }
  }
  return out;
}

}  // namespace split_bench
}  // namespace reachability
