#include "standoff_advisor/standoff_sweep.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "reachability/core/lift_bands.hpp"
#include "reachability/core/surface_frame.hpp"

namespace standoff_advisor
{

using reachability::heightRange;
using reachability::Map4D;

namespace
{
constexpr size_t kMaxTargets = 200000;
constexpr double kMaxWork = 4.0e9;   // distances x positions x targets x heights

std::string fmt(const char * f, double a = 0, double b = 0, double c = 0, double d = 0)
{
  char buf[400];
  std::snprintf(buf, sizeof(buf), f, a, b, c, d);
  return buf;
}

Eigen::Vector2d horizontal(const Vec3 & v) {return Eigen::Vector2d(v.x(), v.y());}

struct Geometry
{
  Eigen::Vector2d n{1.0, 0.0};   // unit horizontal normal into the free side
  Eigen::Vector2d u{0.0, 1.0};   // along the wall
  Eigen::Vector2d p{0.0, 0.0};   // wall point
};

Geometry geometryOf(const StandoffSweepRequest & r)
{
  Geometry g;
  g.n = horizontal(r.wall_normal).normalized();
  g.u = Eigen::Vector2d(-g.n.y(), g.n.x());
  g.p = horizontal(r.wall_point);
  return g;
}

const std::vector<Pose> & targetsOf(const StandoffSweepRequest & r, std::vector<Pose> & storage)
{
  if (!r.targets_world.empty() || !r.patch) {return r.targets_world;}
  storage = patchTargets(*r.patch, r.wall_normal);
  return storage;
}

struct Prepared
{
  std::vector<Pose> targets;
  std::vector<double> heights;
  std::vector<double> laterals;
};

Prepared prepare(const StandoffSweepRequest & r)
{
  Prepared p;
  std::vector<Pose> storage;
  p.targets = targetsOf(r, storage);
  p.heights = heightRange(r.lift_min, r.lift_max, r.lift_step);
  const Geometry g = geometryOf(r);
  double lo = std::numeric_limits<double>::infinity(), hi = -lo;
  for (const auto & t : p.targets) {
    const double s = (horizontal(t.translation()) - g.p).dot(g.u);
    lo = std::min(lo, s);
    hi = std::max(hi, s);
  }
  if (r.lateral_step > 0.0 && hi - lo > 1e-9) {
    for (double s = lo; s <= hi + 1e-9; s += r.lateral_step) {p.laterals.push_back(s);}
    if (p.laterals.back() < hi - 1e-9) {p.laterals.push_back(hi);}
  } else {
    p.laterals.push_back(0.5 * (lo + hi));
  }
  return p;
}

// Number of targets reachable at some height from an arm base at (bx, by).
size_t reachableCount(
  const Map4D & map, const StandoffSweepRequest & r, const Prepared & pr, double bx, double by)
{
  size_t n = 0;
  for (const auto & t : pr.targets) {
    for (const double h : pr.heights) {
      Pose in_base = t;
      in_base.translation() -= Vec3(bx, by, h + r.arm_mount_offset_z);
      if (map.isReachable(in_base)) {++n; break;}
    }
  }
  return n;
}

void quality(
  const Map4D & map, const StandoffSweepRequest & r, const Prepared & pr, double bx, double by,
  double * mean, double * min_q, size_t * reachable)
{
  double sum = 0.0, lo = 1.0;
  size_t n = 0;
  for (const auto & t : pr.targets) {
    double best = -1.0;
    for (const double h : pr.heights) {
      Pose in_base = t;
      in_base.translation() -= Vec3(bx, by, h + r.arm_mount_offset_z);
      const auto s = reachability::scorePose(map, in_base, r.margin);
      if (s.reachable) {best = std::max(best, s.score);}
    }
    if (best >= 0.0) {
      sum += best;
      lo = std::min(lo, best);
      ++n;
    }
  }
  *reachable = n;
  *mean = n > 0 ? sum / static_cast<double>(n) : 0.0;
  *min_q = n > 0 ? lo : 0.0;
}

StandoffRow evaluate(
  const Map4D & map, const StandoffSweepRequest & r, const Prepared & pr, double distance)
{
  StandoffRow row;
  row.distance = distance;
  const Geometry g = geometryOf(r);

  // Footprint (does not depend on the position along a straight wall).
  const Eigen::Vector2d arm0 = g.p + g.n * distance;
  if (r.base) {
    double c = 0.0;
    const double yaw = reachability::bestYaw(*r.base, arm0, g.p, g.n, &c);
    const reachability::Stand st{arm0, yaw};
    row.footprint_clearance = c;
    row.base_yaw = yaw;
    row.base_origin_distance = (reachability::baseCentre(*r.base, st) - g.p).dot(g.n);
    row.footprint_fits = c >= r.min_clearance - 1e-9;
  } else {
    row.footprint_clearance = distance;
    row.base_origin_distance = distance;
    row.footprint_fits = distance >= r.min_clearance - 1e-9;
  }

  // Position along the wall: most targets reachable; ties go to the better quality, then to the middle.
  std::vector<size_t> counts(pr.laterals.size());
  size_t best = 0;
  for (size_t i = 0; i < pr.laterals.size(); ++i) {
    const Eigen::Vector2d a = arm0 + g.u * pr.laterals[i];
    counts[i] = reachableCount(map, r, pr, a.x(), a.y());
    best = std::max(best, counts[i]);
  }
  row.reachable_count = best;
  row.reachable_fraction = static_cast<double>(best) / static_cast<double>(pr.targets.size());
  if (best == 0) {
    row.lateral_m = pr.laterals[pr.laterals.size() / 2];
    return row;
  }
  const double mid = 0.5 * (pr.laterals.front() + pr.laterals.back());
  double best_q = -1.0, best_off = 0.0;
  size_t best_i = 0;
  for (size_t i = 0; i < pr.laterals.size(); ++i) {
    if (counts[i] != best) {continue;}
    const Eigen::Vector2d a = arm0 + g.u * pr.laterals[i];
    double mean, lo;
    size_t n;
    quality(map, r, pr, a.x(), a.y(), &mean, &lo, &n);
    const double off = std::fabs(pr.laterals[i] - mid);
    if (mean > best_q + 1e-9 || (std::fabs(mean - best_q) <= 1e-9 && off < best_off)) {
      best_q = mean;
      best_off = off;
      best_i = i;
      row.mean_quality = mean;
      row.min_quality = lo;
    }
  }
  row.lateral_m = pr.laterals[best_i];

  const Eigen::Vector2d a = arm0 + g.u * row.lateral_m;
  const auto plan = reachability::planLiftBands(map, pr.targets, a.x(), a.y(), pr.heights, r.arm_mount_offset_z);
  row.lift_bands = plan.bands.size();
  if (!plan.bands.empty()) {
    row.lift_low = row.lift_high = plan.bands.front().lift_height;
    for (const auto & b : plan.bands) {
      row.lift_low = std::min(row.lift_low, b.lift_height);
      row.lift_high = std::max(row.lift_high, b.lift_height);
    }
  }
  return row;
}
}  // namespace

const char * toString(SweepStatus s)
{
  switch (s) {
    case SweepStatus::kOk: return "ok";
    case SweepStatus::kInvalidRequest: return "invalid_request";
    case SweepStatus::kNothingReachable: return "nothing_reachable";
    case SweepStatus::kNoFootprintFit: return "no_footprint_fit";
  }
  return "unknown";
}

std::vector<Pose> patchTargets(const WallPatch & patch, const Vec3 & wall_normal)
{
  std::vector<Pose> out;
  if (!patch.origin.allFinite() || !patch.u_axis.allFinite() || !patch.v_axis.allFinite() ||
    !wall_normal.allFinite() || !std::isfinite(patch.u0) || !std::isfinite(patch.u1) ||
    !std::isfinite(patch.v0) || !std::isfinite(patch.v1) || !std::isfinite(patch.resolution) ||
    patch.resolution <= 0.0 || patch.u1 < patch.u0 || patch.v1 < patch.v0 ||
    wall_normal.norm() < 1e-9 || patch.u_axis.norm() < 1e-9 || patch.v_axis.norm() < 1e-9)
  {
    return out;
  }
  const double nu = std::floor((patch.u1 - patch.u0) / patch.resolution + 1e-9) + 1.0;
  const double nv = std::floor((patch.v1 - patch.v0) / patch.resolution + 1e-9) + 1.0;
  if (!(nu * nv <= static_cast<double>(kMaxTargets))) {return out;}
  const Vec3 n = wall_normal.normalized();
  const Vec3 z = -n;
  Vec3 x = patch.v_axis.normalized() - patch.v_axis.normalized().dot(z) * z;
  if (x.norm() < 1e-6) {
    x = patch.u_axis.normalized() - patch.u_axis.normalized().dot(z) * z;
    if (x.norm() < 1e-6) {return out;}
  }
  x.normalize();
  const Vec3 y = z.cross(x);
  Pose T = Pose::Identity();
  T.linear().col(0) = x;
  T.linear().col(1) = y;
  T.linear().col(2) = z;
  const Vec3 ua = patch.u_axis.normalized(), va = patch.v_axis.normalized();
  const auto add = [&](double u, double v) {
      T.translation() = patch.origin + ua * u + va * v;
      out.push_back(T);
    };
  const int iu = static_cast<int>(nu), iv = static_cast<int>(nv);
  for (int i = 0; i < iu; ++i) {
    for (int j = 0; j < iv; ++j) {
      add(std::min(patch.u0 + i * patch.resolution, patch.u1), std::min(patch.v0 + j * patch.resolution, patch.v1));
    }
  }
  return out;
}

WallPatch patchFromPlane(
  const Vec3 & wall_point, const Vec3 & wall_normal, double u0, double u1, double v0, double v1,
  double resolution)
{
  WallPatch p;
  const auto f = reachability::surfaceFrame(wall_normal);
  p.origin = wall_point;
  if (f.ok) {
    p.u_axis = f.u;
    p.v_axis = f.v;
  }
  p.u0 = u0;
  p.u1 = u1;
  p.v0 = v0;
  p.v1 = v1;
  p.resolution = resolution;
  return p;
}

std::string sweepProblem(const Map4D & /*map*/, const StandoffSweepRequest & r)
{
  const double nums[] = {r.standoff_min, r.standoff_max, r.standoff_step, r.lift_min, r.lift_max, r.lift_step,
    r.arm_mount_offset_z, r.lateral_step, r.min_clearance, r.quality_min, r.fraction_tolerance,
    r.nav_tolerance_m, r.margin.min_score, r.margin.count_ref};
  for (const double v : nums) {
    if (!std::isfinite(v)) {return "a numeric parameter is not finite";}
  }
  if (!r.wall_point.allFinite() || !r.wall_normal.allFinite()) {return "the wall plane is not finite";}
  if (horizontal(r.wall_normal).norm() < 1e-6 || std::fabs(r.wall_normal.z()) > 0.05 * r.wall_normal.norm()) {
    return "the wall normal must be horizontal (a vertical wall); ceilings and floors are not supported";
  }
  if (r.standoff_step <= 0.0 || r.standoff_max < r.standoff_min || r.standoff_min < 0.0) {
    return "need 0 <= standoff_min <= standoff_max and standoff_step > 0";
  }
  if (r.standoff_max - r.standoff_min > 50.0 || (r.standoff_max - r.standoff_min) / r.standoff_step > 10000.0) {
    return "the standoff range has more than 10000 steps";
  }
  if (r.lift_step <= 0.0 || r.lift_max < r.lift_min) {return "need lift_min <= lift_max and lift_step > 0";}
  if (r.nav_tolerance_m < 0.0 || r.fraction_tolerance < 0.0 || r.min_clearance < 0.0) {
    return "nav_tolerance_m, fraction_tolerance and min_clearance must not be negative";
  }
  if (r.margin.count_ref < 1.0) {return "margin.count_ref must be at least 1";}
  for (const auto & t : r.targets_world) {
    if (!t.matrix().allFinite()) {return "a target pose is not finite";}
  }
  if (r.patch) {
    const auto & p = *r.patch;
    if (!(p.resolution > 0.0) || p.u1 < p.u0 || p.v1 < p.v0 || !p.origin.allFinite()) {
      return "the wall patch is empty, has a non-positive resolution or is not finite";
    }
  }
  std::vector<Pose> storage;
  const auto & targets = targetsOf(r, storage);
  if (targets.empty()) {return "no targets (give tool poses or a valid wall patch of at most 200000 poses)";}
  if (heightRange(r.lift_min, r.lift_max, r.lift_step).empty()) {return "the lift range gives no heights";}
  const Prepared pr = prepare(r);
  const double work = static_cast<double>(pr.targets.size()) * static_cast<double>(pr.heights.size()) *
    static_cast<double>(pr.laterals.size()) * ((r.standoff_max - r.standoff_min) / r.standoff_step + 1.0);
  if (work > kMaxWork) {return "the sweep is too large (targets x lift heights x positions x distances): coarsen a step";}
  return "";
}

StandoffRow evaluateStandoff(const Map4D & map, const StandoffSweepRequest & request, double distance)
{
  if (!sweepProblem(map, request).empty() || !std::isfinite(distance)) {return StandoffRow{};}
  return evaluate(map, request, prepare(request), distance);
}

StandoffSweepResult sweepStandoff(const Map4D & map, const StandoffSweepRequest & r)
{
  StandoffSweepResult res;
  res.problem = sweepProblem(map, r);
  if (!res.problem.empty()) {
    res.status = SweepStatus::kInvalidRequest;
    res.recommended.reason = "request refused: " + res.problem;
    return res;
  }
  const Prepared pr = prepare(r);
  res.target_count = pr.targets.size();
  const int n = static_cast<int>(std::floor((r.standoff_max - r.standoff_min) / r.standoff_step + 1e-9)) + 1;
  for (int i = 0; i < n; ++i) {
    res.rows.push_back(evaluate(map, r, pr, r.standoff_min + i * r.standoff_step));
  }
  auto & rows = res.rows;
  const int k = static_cast<int>(rows.size());

  // Robust fraction: the worst within +- nav tolerance (swept distances only).
  for (int i = 0; i < k; ++i) {
    double lo = rows[i].reachable_fraction;
    for (int j = 0; j < k; ++j) {
      if (std::fabs(rows[j].distance - rows[i].distance) <= r.nav_tolerance_m + 1e-9) {
        lo = std::min(lo, rows[j].reachable_fraction);
      }
    }
    rows[i].robust_fraction = lo;
  }

  double any_fraction = 0.0;
  for (const auto & row : rows) {any_fraction = std::max(any_fraction, row.reachable_fraction);}
  double max_fit = 0.0;
  bool any_fit = false;
  for (const auto & row : rows) {
    if (row.footprint_fits) {
      any_fit = true;
      max_fit = std::max(max_fit, row.reachable_fraction);
    }
  }
  res.max_fraction = max_fit;
  auto & rec = res.recommended;
  rec.nav_tolerance_m = r.nav_tolerance_m;
  if (any_fraction <= 0.0) {
    res.status = SweepStatus::kNothingReachable;
    rec.reason = fmt("no target is reachable at any distance from %.2f to %.2f m: check the map window, the lift "
        "range and the arm mount height", r.standoff_min, r.standoff_max);
    return res;
  }
  if (!any_fit) {
    res.status = SweepStatus::kNoFootprintFit;
    rec.reason = fmt("targets are reachable (up to %.0f%%) but the footprint never keeps %.2f m from the wall within "
        "%.2f..%.2f m: raise standoff_max or lower min_clearance", 100.0 * any_fraction, r.min_clearance,
        r.standoff_min, r.standoff_max);
    return res;
  }
  if (max_fit <= 0.0) {
    res.status = SweepStatus::kNothingReachable;
    rec.reason = "targets are reachable only at distances where the footprint does not fit the clearance";
    return res;
  }

  // Plateau: distances that fit, keep the fraction near its best and have good quality.
  const auto good = [&](const StandoffRow & row, bool use_quality) {
      return row.footprint_fits && row.reachable_fraction >= max_fit - r.fraction_tolerance - 1e-12 &&
             (!use_quality || row.mean_quality >= r.quality_min);
    };
  bool use_quality = false;
  for (const auto & row : rows) {use_quality = use_quality || good(row, true);}
  rec.quality_ok = use_quality;
  int best_a = -1, best_b = -1;
  double best_w = -1.0, best_q = -1.0;
  for (int i = 0; i < k;) {
    if (!good(rows[i], use_quality)) {++i; continue;}
    int j = i;
    double qsum = 0.0;
    while (j + 1 < k && good(rows[j + 1], use_quality)) {++j;}
    for (int m = i; m <= j; ++m) {qsum += rows[m].mean_quality;}
    const double w = rows[j].distance - rows[i].distance;
    const double q = qsum / (j - i + 1);
    if (w > best_w + 1e-9 || (std::fabs(w - best_w) <= 1e-9 && q > best_q)) {
      best_w = w;
      best_q = q;
      best_a = i;
      best_b = j;
    }
    i = j + 1;
  }
  const int erode = static_cast<int>(std::floor(r.nav_tolerance_m / r.standoff_step + 1e-9));
  int ra = best_a + erode, rb = best_b - erode;
  rec.robust = ra <= rb;
  if (!rec.robust) {ra = rb = -1;}
  rec.found = true;
  res.status = SweepStatus::kOk;
  rec.min_distance = rows[best_a].distance;
  rec.max_distance = rows[best_b].distance;
  rec.touches_range = best_a == 0 || best_b == k - 1;
  for (int i = best_a; i <= best_b; ++i) {rows[i].in_plateau = true;}
  const double centre = rec.robust ? 0.5 * (rows[ra].distance + rows[rb].distance) :
    0.5 * (rec.min_distance + rec.max_distance);
  if (rec.robust) {
    rec.robust_min = rows[ra].distance;
    rec.robust_max = rows[rb].distance;
  } else {
    rec.robust_min = rec.robust_max = centre;
  }
  rec.standoff_m = centre;

  // Numbers at the recommended distance (the nearest swept row for the lift and the footprint).
  int near = best_a;
  for (int i = best_a; i <= best_b; ++i) {
    if (std::fabs(rows[i].distance - centre) < std::fabs(rows[near].distance - centre)) {near = i;}
  }
  const StandoffRow at = evaluate(map, r, pr, centre);
  rec.reachable_fraction = at.reachable_fraction;
  rec.mean_quality = at.mean_quality;
  rec.lift_low = at.lift_low;
  rec.lift_high = at.lift_high;
  rec.base_origin_distance = at.base_origin_distance;
  rec.base_yaw = at.base_yaw;
  rec.lateral_m = at.lateral_m;
  rec.robust_fraction = rows[near].robust_fraction;
  // Lift stroke the whole plateau uses (every distance of it, so a navigation error stays inside).
  bool first = true;
  for (int i = best_a; i <= best_b; ++i) {
    if (rows[i].lift_bands == 0) {continue;}
    rec.lift_low = first ? rows[i].lift_low : std::min(rec.lift_low, rows[i].lift_low);
    rec.lift_high = first ? rows[i].lift_high : std::max(rec.lift_high, rows[i].lift_high);
    first = false;
  }

  // Reason.
  std::string why = fmt("centre of the widest plateau %.2f..%.2f m (reachable fraction >= %.0f%% of targets",
      rec.min_distance, rec.max_distance, 100.0 * (max_fit - r.fraction_tolerance));
  why += use_quality ? fmt(", mean quality >= %.2f)", r.quality_min) : ")";
  if (!use_quality) {
    why += fmt("; WARNING: no distance reached mean quality %.2f, the quality gate was dropped", r.quality_min);
  }
  if (rec.robust) {
    why += fmt("; shrunk by the navigation tolerance +-%.0f mm to %.2f..%.2f m", 1000.0 * r.nav_tolerance_m,
        rec.robust_min, rec.robust_max);
  } else if (best_a == best_b) {
    why += fmt("; single-point plateau: any navigation error loses reach (tolerance %.0f mm)", 1000.0 * r.nav_tolerance_m);
  } else {
    why += fmt("; the plateau (%.0f mm wide) is narrower than twice the navigation tolerance (%.0f mm): not robust",
        1000.0 * (rec.max_distance - rec.min_distance), 2000.0 * r.nav_tolerance_m);
  }
  if (rec.touches_range) {why += "; the plateau reaches the end of the swept range: extend it";}
  rec.reason = why;
  return res;
}

}  // namespace standoff_advisor
