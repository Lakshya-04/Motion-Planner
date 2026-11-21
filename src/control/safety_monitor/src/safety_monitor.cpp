/**
 * @file safety_monitor.cpp
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#include "safety_monitor/safety_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace safety_monitor
{

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

Result failSafe(Reason r)
{
  Result out;
  out.speed_scale = 0.0;
  out.state = State::kProtectiveStop;
  out.reason = r;
  return out;
}

// Finite, non-negative time.
bool validTime(double t) {return std::isfinite(t) && t >= 0.0;}

void requireRange(bool ok, const char * name)
{
  if (!ok) {
    throw std::invalid_argument(std::string("safety_monitor::Params: invalid ") + name);
  }
}

struct Worst
{
  double scale{1.0};
  double min_distance{kInf};
  int sphere{-1};
  Reason reason{Reason::kNone};
};

void consider(
  Worst & w, double scale, double dist, int sphere, Reason reason)
{
  if (std::isnan(scale) || std::isnan(dist)) {
    scale = 0.0;  // fail safe: an undefined distance is a stop, never a skipped sphere
    dist = -kInf;
  }
  if (dist < w.min_distance) {w.min_distance = dist;}
  if (scale < w.scale) {
    w.scale = scale;
    w.sphere = sphere;
    w.reason = reason;
  }
}

double scaleOf(double d, double d_stop, double band)
{
  if (band <= 0.0) {return d > d_stop ? 1.0 : 0.0;}
  return std::clamp((d - d_stop) / band, 0.0, 1.0);
}

}  // namespace

SafetyMonitor::SafetyMonitor(const SphereModel & model, const Params & p)
: model_(model), p_(p)
{
  auto nonneg = [](double v) {return std::isfinite(v) && v >= 0.0;};
  requireRange(nonneg(p.stop_time_s), "stop_time_s");
  requireRange(nonneg(p.reaction_time_s), "reaction_time_s");
  requireRange(nonneg(p.human_speed_mps), "human_speed_mps");
  requireRange(nonneg(p.static_margin_m), "static_margin_m");
  requireRange(nonneg(p.person_intrusion_m), "person_intrusion_m");
  requireRange(nonneg(p.person_uncertainty_m), "person_uncertainty_m");
  requireRange(nonneg(p.self_margin_m), "self_margin_m");
  requireRange(nonneg(p.static_slow_band_m), "static_slow_band_m");
  requireRange(nonneg(p.person_slow_band_m), "person_slow_band_m");
  requireRange(nonneg(p.self_slow_band_m), "self_slow_band_m");
  requireRange(nonneg(p.stale_timeout_s), "stale_timeout_s");
  requireRange(nonneg(p.min_clear_time_s), "min_clear_time_s");
  requireRange(p.sweep_samples >= 0, "sweep_samples");
  requireRange(nonneg(p.sweep_radius_inflation_m), "sweep_radius_inflation_m");
}

Result SafetyMonitor::compute(
  const std::vector<Eigen::Isometry3d> & T, const std::vector<Vec3> & vel_in,
  const std::vector<const DistanceSource *> & sources, double data_stamp, double now) const
{
  // --- input validation, fail-safe ---------------------------------------------------------
  if (!validTime(now) || !validTime(data_stamp)) {return failSafe(Reason::kInvalidInput);}
  if (data_stamp - now > kFutureStampEps_s) {return failSafe(Reason::kInvalidInput);}
  if (now - data_stamp > p_.stale_timeout_s) {return failSafe(Reason::kStaleData);}
  std::vector<double> age(sources.size(), 0.0);  // data age of each source, for extrapolation
  for (size_t k = 0; k < sources.size(); ++k) {
    if (sources[k] == nullptr) {return failSafe(Reason::kInvalidInput);}
    const double st = sources[k]->stamp();
    if (std::isinf(st) && st > 0.0) {continue;}  // static source, never stale
    if (!validTime(st)) {return failSafe(Reason::kInvalidInput);}
    if (st - now > kFutureStampEps_s) {return failSafe(Reason::kInvalidInput);}
    if (now - st > p_.stale_timeout_s) {return failSafe(Reason::kStaleData);}
    age[k] = std::max(0.0, now - st);
  }
  if (static_cast<int>(T.size()) != model_.linkCount()) {return failSafe(Reason::kInvalidInput);}
  for (const auto & t : T) {
    if (!t.matrix().allFinite()) {return failSafe(Reason::kInvalidInput);}
  }
  std::vector<WorldSphere> sp;
  if (!model_.worldSpheres(T, sp)) {return failSafe(Reason::kInvalidInput);}
  std::vector<Vec3> vel = vel_in;
  if (vel.empty()) {vel.assign(sp.size(), Vec3::Zero());}
  if (vel.size() != sp.size()) {return failSafe(Reason::kInvalidInput);}
  for (const auto & v : vel) {
    if (!v.allFinite()) {return failSafe(Reason::kInvalidInput);}
  }

  const int n = std::max(1, p_.sweep_samples);
  const double dt = p_.stop_time_s / n;
  Worst w;

  // --- obstacles --------------------------------------------------------------------------
  for (size_t si = 0; si < sources.size(); ++si) {
    const auto * src = sources[si];
    const bool person = src->kind() == SourceKind::kPerson;
    const double margin = person ? p_.person_intrusion_m + p_.person_uncertainty_m :
      p_.static_margin_m;
    const double band = person ? p_.person_slow_band_m : p_.static_slow_band_m;
    const Reason reason = person ? Reason::kPerson : Reason::kEnvironment;
    for (size_t i = 0; i < sp.size(); ++i) {
      if (src->isWorkSurface() && !person && model_.isContactExempt(sp[i].link)) {continue;}
      const double r = sp[i].radius + p_.sweep_radius_inflation_m;
      for (int k = 0; k <= n; ++k) {
        const double t = k * dt;
        const Vec3 c = sp[i].center + vel[i] * t;
        const DistanceQuery q = src->query(c, t + age[si]);
        if (!q.valid) {continue;}
        if (std::isnan(q.distance) || !q.gradient.allFinite() || !q.velocity.allFinite()) {
          return failSafe(Reason::kInvalidInput);
        }
        const double d = q.distance - r;
        const double toward = std::max(0.0, -vel[i].dot(q.gradient));
        const double v_h = person ? std::max(p_.human_speed_mps, q.velocity.norm()) : 0.0;
        const double d_stop = v_h * (p_.stop_time_s + p_.reaction_time_s) + toward * p_.reaction_time_s + margin;
        consider(w, scaleOf(d, d_stop, band), d, static_cast<int>(i), reason);
      }
    }
  }

  // --- self collision between non-allowed link pairs ---------------------------------------
  std::vector<std::vector<int>> by_link(model_.linkCount());
  for (size_t i = 0; i < sp.size(); ++i) {by_link[sp[i].link].push_back(static_cast<int>(i));}
  for (const auto & pr : model_.selfPairs()) {
    for (int i : by_link[pr.first]) {
      for (int j : by_link[pr.second]) {
        const double r = sp[i].radius + sp[j].radius + 2.0 * p_.sweep_radius_inflation_m;
        for (int k = 0; k <= n; ++k) {
          const double t = k * dt;
          const Vec3 ci = sp[i].center + vel[i] * t;
          const Vec3 cj = sp[j].center + vel[j] * t;
          const Vec3 dv = ci - cj;
          const double dist = dv.norm();
          const double d = dist - r;
          double closing = 0.0;
          if (dist > 1e-12) {closing = std::max(0.0, -(vel[i] - vel[j]).dot(dv / dist));}
          const double d_stop = closing * p_.reaction_time_s + p_.self_margin_m;
          consider(
            w, scaleOf(d, d_stop, p_.self_slow_band_m), d,
            i, Reason::kSelfCollision);
        }
      }
    }
  }

  Result out;
  out.speed_scale = w.scale;
  out.min_distance = w.min_distance;
  out.worst_sphere = w.sphere;
  out.worst_link = w.sphere >= 0 ? sp[w.sphere].link : -1;
  out.reason = w.reason;
  out.state = w.scale <= 0.0 ? State::kProtectiveStop : (w.scale < 1.0 ? State::kSlow : State::kOk);
  if (out.state == State::kOk) {out.reason = Reason::kNone;}
  return out;
}

Result SafetyMonitor::evaluate(
  const std::vector<Eigen::Isometry3d> & T, const std::vector<Vec3> & vel,
  const std::vector<const DistanceSource *> & sources, double data_stamp, double now)
{
  Result r = compute(T, vel, sources, data_stamp, now);
  const bool hazard = r.state == State::kProtectiveStop;
  if (hazard) {
    if (!latched_) {latched_reason_ = r.reason;}
    latched_ = true;
    clear_since_ = -1.0;
    reset_pending_ = false;
    return r;
  }
  if (!latched_) {return r;}

  // Latched, current input is not hazardous: wait for the clear time and an explicit reset.
  if (clear_since_ < 0.0) {clear_since_ = now;}
  if (reset_pending_ && now - clear_since_ >= p_.min_clear_time_s) {
    latched_ = false;
    reset_pending_ = false;
    clear_since_ = -1.0;
    latched_reason_ = Reason::kNone;
    return r;
  }
  r.speed_scale = 0.0;
  r.state = State::kProtectiveStop;
  r.reason = latched_reason_;
  return r;
}

void SafetyMonitor::reset()
{
  // Honoured only when the last cycle was clear (clear_since_ set); a reset during a hazard is
  // discarded, and evaluate() also drops a pending one on the next hazardous cycle.
  if (latched_ && clear_since_ >= 0.0) {reset_pending_ = true;}
}

}  // namespace safety_monitor
