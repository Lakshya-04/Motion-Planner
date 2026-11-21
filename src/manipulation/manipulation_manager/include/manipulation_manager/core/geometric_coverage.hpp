/**
 * @file geometric_coverage.hpp
 * @brief Covered share of the planned wall patch, from the tool path that was actually executed.
 * @details ROS-free, header-only. The column count in `mission_verify.hpp` says how many columns finished; this says how
 *          much of the surface the tool really passed over, which differs when a column ends early, drifts off its line
 *          or lifts off the wall.
 *
 *          Method (all in the wall plane, `GeometricCoverageConfig::plane_origin` / `wall_normal`):
 *            - The planned patch is the union, over the planned strokes, of the band `swath_m` wide around the stroke's
 *              polyline, on a grid of `cell_m` cells (a cell is in the patch when its centre is within swath/2 of a
 *              planned polyline).
 *            - The executed path is the tool samples (in time order, per column) that count as working: tool force at
 *              least `contact_force_n` (a negative value counts every sample, for a run without a force topic) and the
 *              tool no farther than `max_standoff_m` from the plane (0 = not checked). Consecutive working samples at most
 *              `max_gap_m` apart are joined into a polyline; a sample alone is a point. A gap breaks the line, so a lift-off
 *              or a missed stretch leaves a hole.
 *            - A patch cell is covered when its centre is within swath/2 of the executed path. The result is
 *              covered / planned, overall and per column (a column's own cells).
 *          This is the same idea as the scorecard of the coverage planner, written against the manager's own data (planned
 *          intent poses and sampled tool poses) and without its dependencies. It does not model overlap or thickness, and
 *          the swath width is a parameter, not read from the tool.
 *          Limits: the grid is coarsened (cell size doubled) when it would exceed `max_cells`; the plane is infinite, so a
 *          wall with a hole is measured as if solid, as the plan is.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace manipulation_manager
{

struct GeoPoint
{
  double x{0.0}, y{0.0}, z{0.0};
};

/// One planned stroke of a column (the contact intent's poses, as points).
struct PlannedStroke
{
  size_t column{0};
  std::vector<GeoPoint> points;
};

/// One tool sample (time order within a column).
struct ToolSample
{
  size_t column{0};
  GeoPoint p;
  double force_n{-1.0};   ///< tool force magnitude at the sample; negative = not read
};

struct GeometricCoverageConfig
{
  double swath_m{0.10};             ///< Width of the band the tool treats (centred on the path).
  double cell_m{0.01};              ///< Grid cell size.
  double contact_force_n{1.0};      ///< A sample works when force >= this; negative = every sample works.
  double max_gap_m{0.05};           ///< Working samples farther apart than this are not joined.
  double max_standoff_m{0.02};      ///< Samples farther than this from the plane do not work; 0 = not checked.
  GeoPoint plane_origin;            ///< A point on the wall plane.
  GeoPoint wall_normal{1.0, 0.0, 0.0};   ///< Plane normal (any length; normalised here).
  size_t max_cells{4000000};        ///< The grid is coarsened to stay within this.
  /// Optional groups of planned columns (e.g. the columns of one stand): group id -> column ids. Each group gets the
  /// coverage of the UNION of its columns' cells (columns may overlap). Unknown column ids are ignored.
  std::map<size_t, std::vector<size_t>> groups;
};

/// Coverage of one group of columns.
struct GeometricGroupResult
{
  size_t planned_cells{0};
  size_t covered_cells{0};
  double percent{0.0};   ///< 0 when the group has no planned cell.
};

struct GeometricCoverageResult
{
  bool available{false};            ///< False when there was no planned patch or the configuration was unusable.
  size_t planned_cells{0};
  size_t covered_cells{0};
  double percent{0.0};              ///< 100 * covered / planned.
  std::map<size_t, double> column_percent;   ///< Per planned column.
  std::map<size_t, GeometricGroupResult> group;   ///< Per `groups` entry of the config (empty without groups).
  size_t samples{0};                ///< Samples given.
  size_t working_samples{0};        ///< Samples that counted as working.
  size_t off_plane_samples{0};      ///< Samples refused for standoff.
  double cell_m{0.0};               ///< Cell size used (after any coarsening).
  std::string note;                 ///< Why it is unavailable, or a remark.
};

namespace geometric_coverage_detail
{

struct V2
{
  double a{0.0}, b{0.0};
};

struct Seg
{
  V2 p, q;
};

struct Basis
{
  std::array<double, 3> n, u, v;
  std::array<double, 3> o;
  bool ok{false};
};

inline double dot(const std::array<double, 3> & a, const std::array<double, 3> & b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline std::array<double, 3> cross(const std::array<double, 3> & a, const std::array<double, 3> & b)
{
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

inline Basis makeBasis(const GeometricCoverageConfig & c)
{
  Basis B;
  std::array<double, 3> n{c.wall_normal.x, c.wall_normal.y, c.wall_normal.z};
  const double len = std::sqrt(dot(n, n));
  if (!(len > 1e-9) || !std::isfinite(len)) {
    return B;
  }
  for (auto & x : n) {x /= len;}
  // tangent u: horizontal when the wall is vertical (reads as "along the wall"), else any perpendicular
  std::array<double, 3> up{0.0, 0.0, 1.0};
  if (std::fabs(dot(n, up)) > 0.99) {up = {1.0, 0.0, 0.0};}
  auto u = cross(up, n);
  const double ul = std::sqrt(dot(u, u));
  for (auto & x : u) {x /= ul;}
  B.n = n;
  B.u = u;
  B.v = cross(n, u);
  B.o = {c.plane_origin.x, c.plane_origin.y, c.plane_origin.z};
  B.ok = true;
  return B;
}

inline V2 project(const Basis & B, const GeoPoint & p)
{
  const std::array<double, 3> d{p.x - B.o[0], p.y - B.o[1], p.z - B.o[2]};
  return {dot(d, B.u), dot(d, B.v)};
}

inline double standoff(const Basis & B, const GeoPoint & p)
{
  const std::array<double, 3> d{p.x - B.o[0], p.y - B.o[1], p.z - B.o[2]};
  return std::fabs(dot(d, B.n));
}

inline double distToSeg(const V2 & x, const Seg & s)
{
  const double dx = s.q.a - s.p.a, dy = s.q.b - s.p.b;
  const double l2 = dx * dx + dy * dy;
  double t = 0.0;
  if (l2 > 1e-18) {
    t = ((x.a - s.p.a) * dx + (x.b - s.p.b) * dy) / l2;
    t = std::min(1.0, std::max(0.0, t));
  }
  const double ex = s.p.a + t * dx - x.a, ey = s.p.b + t * dy - x.b;
  return std::sqrt(ex * ex + ey * ey);
}

struct Grid
{
  double a0{0.0}, b0{0.0}, cell{0.01};
  long nx{0}, ny{0};
  size_t idx(long i, long j) const {return static_cast<size_t>(j) * static_cast<size_t>(nx) + static_cast<size_t>(i);}
};

/// Visit the cells whose centre is within r of segment s, calling f(index).
template<typename F>
inline void forCellsNear(const Grid & g, const Seg & s, double r, F && f)
{
  const double amin = std::min(s.p.a, s.q.a) - r, amax = std::max(s.p.a, s.q.a) + r;
  const double bmin = std::min(s.p.b, s.q.b) - r, bmax = std::max(s.p.b, s.q.b) + r;
  const long i0 = std::max(0L, static_cast<long>(std::floor((amin - g.a0) / g.cell - 0.5)));
  const long i1 = std::min(g.nx - 1, static_cast<long>(std::ceil((amax - g.a0) / g.cell - 0.5)));
  const long j0 = std::max(0L, static_cast<long>(std::floor((bmin - g.b0) / g.cell - 0.5)));
  const long j1 = std::min(g.ny - 1, static_cast<long>(std::ceil((bmax - g.b0) / g.cell - 0.5)));
  for (long j = j0; j <= j1; ++j) {
    for (long i = i0; i <= i1; ++i) {
      const V2 c{g.a0 + (static_cast<double>(i) + 0.5) * g.cell, g.b0 + (static_cast<double>(j) + 0.5) * g.cell};
      if (distToSeg(c, s) <= r) {
        f(g.idx(i, j));
      }
    }
  }
}

}  // namespace geometric_coverage_detail

/// Covered share of the planned patch. `samples` must be in time order within each column.
inline GeometricCoverageResult computeGeometricCoverage(
  const std::vector<PlannedStroke> & planned, const std::vector<ToolSample> & samples,
  const GeometricCoverageConfig & cfg)
{
  using namespace geometric_coverage_detail;
  GeometricCoverageResult r;
  r.samples = samples.size();
  if (!(cfg.swath_m > 0.0) || !(cfg.cell_m > 0.0) || !std::isfinite(cfg.swath_m) || !std::isfinite(cfg.cell_m)) {
    r.note = "swath_m and cell_m must be positive";
    return r;
  }
  const Basis B = makeBasis(cfg);
  if (!B.ok) {
    r.note = "wall_normal is zero or not finite";
    return r;
  }
  const double rad = 0.5 * cfg.swath_m;

  // planned segments per column (a single-point stroke is a degenerate segment: a disc)
  std::map<size_t, std::vector<Seg>> plan;
  for (const auto & st : planned) {
    auto & v = plan[st.column];
    std::vector<V2> pts;
    for (const auto & p : st.points) {
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {pts.push_back(project(B, p));}
    }
    if (pts.size() == 1) {v.push_back({pts[0], pts[0]});}
    for (size_t i = 1; i < pts.size(); ++i) {v.push_back({pts[i - 1], pts[i]});}
  }
  size_t plan_segs = 0;
  for (const auto & kv : plan) {plan_segs += kv.second.size();}
  if (plan_segs == 0) {
    r.note = "no planned stroke";
    return r;
  }

  // executed segments: working samples joined while consecutive samples of a column are within max_gap
  std::vector<Seg> exec;
  {
    std::map<size_t, std::pair<bool, V2>> last;   // column -> (have a previous working sample, it)
    for (const auto & s : samples) {
      const bool finite = std::isfinite(s.p.x) && std::isfinite(s.p.y) && std::isfinite(s.p.z);
      if (!finite) {
        last[s.column].first = false;
        continue;
      }
      bool works = cfg.contact_force_n < 0.0 || s.force_n >= cfg.contact_force_n;
      if (works && cfg.max_standoff_m > 0.0 && standoff(B, s.p) > cfg.max_standoff_m) {
        works = false;
        ++r.off_plane_samples;
      }
      auto & l = last[s.column];
      if (!works) {
        l.first = false;
        continue;
      }
      ++r.working_samples;
      const V2 q = project(B, s.p);
      if (l.first) {
        const double d = std::hypot(q.a - l.second.a, q.b - l.second.b);
        if (d <= cfg.max_gap_m) {
          exec.push_back({l.second, q});
        } else {
          exec.push_back({q, q});
        }
      } else {
        exec.push_back({q, q});
      }
      l = {true, q};
    }
  }

  // grid over the planned patch
  double amin = 1e300, amax = -1e300, bmin = 1e300, bmax = -1e300;
  for (const auto & kv : plan) {
    for (const auto & s : kv.second) {
      for (const V2 & p : {s.p, s.q}) {
        amin = std::min(amin, p.a);
        amax = std::max(amax, p.a);
        bmin = std::min(bmin, p.b);
        bmax = std::max(bmax, p.b);
      }
    }
  }
  Grid g;
  g.cell = cfg.cell_m;
  for (;;) {
    g.a0 = amin - rad - g.cell;
    g.b0 = bmin - rad - g.cell;
    g.nx = static_cast<long>(std::ceil((amax + rad + g.cell - g.a0) / g.cell)) + 1;
    g.ny = static_cast<long>(std::ceil((bmax + rad + g.cell - g.b0) / g.cell)) + 1;
    if (static_cast<double>(g.nx) * static_cast<double>(g.ny) <= static_cast<double>(cfg.max_cells)) {break;}
    g.cell *= 2.0;
    r.note = "grid coarsened to stay within max_cells";
  }
  r.cell_m = g.cell;
  const size_t n = static_cast<size_t>(g.nx) * static_cast<size_t>(g.ny);

  std::vector<std::uint8_t> planned_mask(n, 0), covered_mask(n, 0);
  std::map<size_t, std::vector<std::uint8_t>> column_mask;
  for (const auto & kv : plan) {
    auto & m = column_mask[kv.first];
    m.assign(n, 0);
    for (const auto & s : kv.second) {
      forCellsNear(g, s, rad, [&](size_t i) {m[i] = 1; planned_mask[i] = 1;});
    }
  }
  for (const auto & s : exec) {
    forCellsNear(g, s, rad, [&](size_t i) {covered_mask[i] = 1;});
  }
  for (size_t i = 0; i < n; ++i) {
    if (planned_mask[i]) {
      ++r.planned_cells;
      if (covered_mask[i]) {++r.covered_cells;}
    }
  }
  for (const auto & kv : column_mask) {
    size_t p = 0, c = 0;
    for (size_t i = 0; i < n; ++i) {
      if (kv.second[i]) {
        ++p;
        if (covered_mask[i]) {++c;}
      }
    }
    r.column_percent[kv.first] = p == 0 ? 0.0 : 100.0 * static_cast<double>(c) / static_cast<double>(p);
  }
  for (const auto & kv : cfg.groups) {
    std::vector<std::uint8_t> u(n, 0);
    for (const size_t col : kv.second) {
      const auto it = column_mask.find(col);
      if (it == column_mask.end()) {continue;}
      for (size_t i = 0; i < n; ++i) {
        if (it->second[i]) {u[i] = 1;}
      }
    }
    GeometricGroupResult gr;
    for (size_t i = 0; i < n; ++i) {
      if (u[i]) {
        ++gr.planned_cells;
        if (covered_mask[i]) {++gr.covered_cells;}
      }
    }
    gr.percent = gr.planned_cells == 0 ? 0.0 : 100.0 * static_cast<double>(gr.covered_cells) / static_cast<double>(gr.planned_cells);
    r.group[kv.first] = gr;
  }
  if (r.planned_cells == 0) {
    r.note = "the planned patch covers no grid cell (swath or cell size too small)";
    return r;
  }
  r.percent = 100.0 * static_cast<double>(r.covered_cells) / static_cast<double>(r.planned_cells);
  r.available = true;
  return r;
}

}  // namespace manipulation_manager
