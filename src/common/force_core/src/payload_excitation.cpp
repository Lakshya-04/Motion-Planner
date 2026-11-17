#include "force_core/payload_excitation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace force_core
{
namespace
{
// Eigenvalues of a symmetric n x n matrix (row-major) by cyclic Jacobi; n is 4 or 6 here.
std::vector<double> symmetricEigenvalues(std::vector<double> a, std::size_t n)
{
  for (int sweep = 0; sweep < 60; ++sweep) {
    double off = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = i + 1; j < n; ++j) {off += a[i * n + j] * a[i * n + j];}
    }
    if (off < 1e-26) {break;}
    for (std::size_t p = 0; p < n; ++p) {
      for (std::size_t q = p + 1; q < n; ++q) {
        const double apq = a[p * n + q];
        if (std::abs(apq) < 1e-300) {continue;}
        const double theta = (a[q * n + q] - a[p * n + p]) / (2.0 * apq);
        const double t = (theta >= 0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
        const double c = 1.0 / std::sqrt(t * t + 1.0), s = t * c;
        for (std::size_t k = 0; k < n; ++k) {
          const double akp = a[k * n + p], akq = a[k * n + q];
          a[k * n + p] = c * akp - s * akq;
          a[k * n + q] = s * akp + c * akq;
        }
        for (std::size_t k = 0; k < n; ++k) {
          const double apk = a[p * n + k], aqk = a[q * n + k];
          a[p * n + k] = c * apk - s * aqk;
          a[q * n + k] = s * apk + c * aqk;
        }
      }
    }
  }
  std::vector<double> ev(n);
  for (std::size_t i = 0; i < n; ++i) {ev[i] = a[i * n + i];}
  return ev;
}

double condFromRows(const std::vector<std::vector<double>> & rows, std::size_t n)
{
  std::vector<double> ata(n * n, 0.0);
  for (const auto & r : rows) {
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = 0; j < n; ++j) {ata[i * n + j] += r[i] * r[j];}
    }
  }
  const auto ev = symmetricEigenvalues(ata, n);
  const double hi = *std::max_element(ev.begin(), ev.end());
  const double lo = *std::min_element(ev.begin(), ev.end());
  if (hi <= 0.0 || lo <= 1e-12 * hi) {return std::numeric_limits<double>::infinity();}
  return std::sqrt(hi / lo);
}

double angleBetween(const Vec3 & a, const Vec3 & b)
{
  const double d = a.x * b.x + a.y * b.y + a.z * b.z;
  const double na = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
  const double nb = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
  if (na < 1e-12 || nb < 1e-12) {return 0.0;}
  return std::acos(std::clamp(d / (na * nb), -1.0, 1.0));
}

struct Lcg
{
  std::uint32_t s;
  double next() {s = s * 1664525u + 1013904223u; return (s >> 8) / 16777216.0;}
};

double quintic(double tau) {return tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau);}
}  // namespace

double regressorConditionNumber(const std::vector<Mat3> & rs, const Vec3 & gravity_world)
{
  const double gn = std::sqrt(gravity_world.x * gravity_world.x + gravity_world.y * gravity_world.y +
      gravity_world.z * gravity_world.z);
  if (rs.empty() || gn < 1e-9) {return std::numeric_limits<double>::infinity();}
  std::vector<std::vector<double>> frows, trows;
  for (const auto & r : rs) {
    const Vec3 g = (1.0 / gn) * rotateInverse(r, gravity_world);
    frows.push_back({g.x, 1, 0, 0});
    frows.push_back({g.y, 0, 1, 0});
    frows.push_back({g.z, 0, 0, 1});
    trows.push_back({0, g.z, -g.y, 1, 0, 0});
    trows.push_back({-g.z, 0, g.x, 0, 1, 0});
    trows.push_back({g.y, -g.x, 0, 0, 0, 1});
  }
  return std::max(condFromRows(frows, 4), condFromRows(trows, 6));
}

ExcitationPlan generateExcitationPoses(
  const std::vector<double> & q_start, const ExcitationLimits & lim, const ForwardKinematics & fk,
  const SafetyProxy & is_safe, const Vec3 & gravity, const ExcitationOptions & opt)
{
  ExcitationPlan plan;
  const std::size_t n = q_start.size();
  if (n == 0 || lim.q_min.size() != n || lim.q_max.size() != n || lim.v_max.size() != n ||
    lim.a_max.size() != n || !fk)
  {
    plan.message = "limits and start configuration must have the same, non-zero length, and fk is required";
    return plan;
  }
  const auto safe = [&](const std::vector<double> & q) {return !is_safe || is_safe(q);};
  const auto inside = [&](const std::vector<double> & q) {
      for (std::size_t j = 0; j < n; ++j) {
        if (q[j] < lim.q_min[j] + lim.margin_rad || q[j] > lim.q_max[j] - lim.margin_rad) {return false;}
      }
      return true;
    };
  if (!inside(q_start) || !safe(q_start)) {
    plan.message = "the start configuration is outside the limits (with margin) or fails the safety proxy";
    return plan;
  }
  for (std::size_t j = 0; j < n; ++j) {
    if (lim.v_max[j] <= 0 || lim.a_max[j] <= 0 || lim.velocity_scale <= 0) {
      plan.message = "velocity and acceleration bounds must be positive";
      return plan;
    }
  }
  const auto pathSafe = [&](const std::vector<double> & a, const std::vector<double> & b) {
      double d = 0.0;
      for (std::size_t j = 0; j < n; ++j) {d = std::max(d, std::abs(b[j] - a[j]));}
      const int steps = std::max(1, static_cast<int>(std::ceil(d / opt.path_step_rad)));
      std::vector<double> q(n);
      for (int k = 1; k <= steps; ++k) {
        const double s = static_cast<double>(k) / steps;
        for (std::size_t j = 0; j < n; ++j) {q[j] = a[j] + s * (b[j] - a[j]);}
        if (!safe(q)) {return false;}
      }
      return true;
    };

  // Candidates: safe, in-bounds joint configurations with their gravity direction in the sensor frame.
  struct Cand {std::vector<double> q; Vec3 dir;};
  std::vector<Cand> cands;
  Lcg rng{opt.seed};
  for (std::size_t i = 0; i < opt.num_candidates; ++i) {
    std::vector<double> q(n);
    for (std::size_t j = 0; j < n; ++j) {
      const double lo = lim.q_min[j] + lim.margin_rad, hi = lim.q_max[j] - lim.margin_rad;
      q[j] = lo + (hi - lo) * rng.next();
    }
    if (!safe(q)) {continue;}
    cands.push_back({q, rotateInverse(fk(q), gravity)});
  }

  std::vector<std::vector<double>> chosen{q_start};
  std::vector<Vec3> dirs{rotateInverse(fk(q_start), gravity)};
  while (chosen.size() < opt.num_poses) {
    double best_score = -1.0;
    std::size_t best = cands.size();
    for (std::size_t c = 0; c < cands.size(); ++c) {
      double mn = 1e9;
      for (const auto & d : dirs) {mn = std::min(mn, angleBetween(d, cands[c].dir));}
      if (mn <= best_score) {continue;}
      if (!pathSafe(chosen.back(), cands[c].q)) {continue;}
      best_score = mn;
      best = c;
    }
    if (best == cands.size() || best_score < 0.05) {break;}
    chosen.push_back(cands[best].q);
    dirs.push_back(cands[best].dir);
  }
  if (chosen.size() < 6) {
    plan.message = "only " + std::to_string(chosen.size()) +
      " reachable, safe, well-spread poses found (need 6): relax the proxy or the margin, or raise num_candidates";
    return plan;
  }

  std::vector<Mat3> rs;
  for (const auto & q : chosen) {rs.push_back(fk(q));}
  plan.condition_number = regressorConditionNumber(rs, gravity);
  plan.min_pair_angle_rad = 1e9;
  for (std::size_t i = 0; i < dirs.size(); ++i) {
    for (std::size_t j = i + 1; j < dirs.size(); ++j) {
      plan.min_pair_angle_rad = std::min(plan.min_pair_angle_rad, angleBetween(dirs[i], dirs[j]));
    }
  }

  double t = 0.0;
  for (std::size_t k = 0; k < chosen.size(); ++k) {
    ExcitationWaypoint w;
    w.q = chosen[k];
    if (k > 0) {
      double dur = opt.min_duration_s;
      for (std::size_t j = 0; j < n; ++j) {
        const double d = std::abs(chosen[k][j] - chosen[k - 1][j]);
        dur = std::max(dur, 1.875 * d / (lim.velocity_scale * lim.v_max[j]));
        dur = std::max(dur, std::sqrt(5.7735 * d / (lim.velocity_scale * lim.a_max[j])));
      }
      for (std::size_t j = 0; j < n; ++j) {
        const double d = std::abs(chosen[k][j] - chosen[k - 1][j]);
        w.peak_velocity_ratio = std::max(w.peak_velocity_ratio, 1.875 * d / dur / lim.v_max[j]);
        w.peak_accel_ratio = std::max(w.peak_accel_ratio, 5.7735 * d / (dur * dur) / lim.a_max[j]);
      }
      t += dur;
    }
    w.t_arrive_s = t;
    w.t_leave_s = t + opt.dwell_s;
    t = w.t_leave_s;
    plan.waypoints.push_back(w);
  }
  plan.duration_s = plan.waypoints.back().t_arrive_s;
  plan.ok = true;
  plan.message = plan.condition_number > opt.max_condition ?
    "plan ok but the regressor condition number " + std::to_string(plan.condition_number) +
    " is above " + std::to_string(opt.max_condition) + ": add poses or candidates" : "ok";
  return plan;
}

std::vector<double> sampleExcitationPlan(const ExcitationPlan & plan, double t)
{
  if (plan.waypoints.empty()) {return {};}
  if (t <= plan.waypoints.front().t_leave_s) {return plan.waypoints.front().q;}
  for (std::size_t k = 1; k < plan.waypoints.size(); ++k) {
    const auto & a = plan.waypoints[k - 1], & b = plan.waypoints[k];
    if (t <= b.t_arrive_s) {
      const double tau = (t - a.t_leave_s) / (b.t_arrive_s - a.t_leave_s);
      const double s = quintic(std::clamp(tau, 0.0, 1.0));
      std::vector<double> q(a.q.size());
      for (std::size_t j = 0; j < q.size(); ++j) {q[j] = a.q[j] + s * (b.q[j] - a.q[j]);}
      return q;
    }
    if (t <= b.t_leave_s) {return b.q;}
  }
  return plan.waypoints.back().q;
}

}  // namespace force_core
