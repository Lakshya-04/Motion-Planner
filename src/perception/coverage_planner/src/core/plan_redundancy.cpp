#include "coverage_planner/core/plan_redundancy.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace coverage_planner
{

namespace
{

Vec3 pos(const PoseStamped & p) {return {p.pose.position.x, p.pose.position.y, p.pose.position.z};}

double dist(const Vec3 & a, const Vec3 & b)
{
  return std::sqrt(
    (a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

bool hasTag(const MotionPlanSegment & s, const std::string & tag)
{
  return std::find(s.tags.begin(), s.tags.end(), tag) != s.tags.end();
}

double median(std::vector<double> v)
{
  if (v.empty()) {return 0.0;}
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}

}  // namespace

RedundancyReport analyzePlanRedundancy(
  const Path & path, const std::vector<MotionPlanSegment> & segments, const RedundancyOptions & options)
{
  RedundancyReport r;
  const auto & poses = path.poses;
  std::vector<const MotionPlanSegment *> strips;  // all cartesian passes in execution order
  for (const auto & s : segments) {
    if (s.start_index > s.end_index || s.end_index >= poses.size()) {
      throw std::invalid_argument("segment " + s.label + " indexes outside the path");
    }
    if (s.mode == "cartesian") {strips.push_back(&s);}
  }
  r.strip_passes = strips.size();
  if (strips.empty()) {return r;}

  bool any_second = false;
  for (const auto * s : strips) {any_second = any_second || hasTag(*s, "second_pass");}
  r.serpentine_expected = options.expect_serpentine < 0 ? !any_second : options.expect_serpentine != 0;

  // Lengths and the travel between consecutive passes.
  std::vector<double> travel;
  double zmin = 1e300, zmax = -1e300;
  Vec3 lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
  for (size_t k = 0; k < strips.size(); ++k) {
    const auto & s = *strips[k];
    for (uint32_t i = s.start_index; i <= s.end_index; ++i) {
      const Vec3 p = pos(poses[i]);
      if (i > s.start_index) {r.strip_length_m += dist(pos(poses[i - 1]), p);}
      zmin = std::min(zmin, p[2]);
      zmax = std::max(zmax, p[2]);
      for (int a = 0; a < 3; ++a) {lo[a] = std::min(lo[a], p[a]); hi[a] = std::max(hi[a], p[a]);}
    }
    if (k > 0) {
      double len = 0.0;
      for (uint32_t i = strips[k - 1]->end_index; i < s.start_index; ++i) {
        len += dist(pos(poses[i]), pos(poses[i + 1]));
      }
      if (s.start_index < strips[k - 1]->end_index) {  // out-of-order indices: chord only
        len = dist(pos(poses[strips[k - 1]->end_index]), pos(poses[s.start_index]));
      }
      travel.push_back(len);
    }
  }
  for (double t : travel) {r.connector_length_m += t;}

  // Pitch: given, else the median of the non-zero travels (neighbouring strips).
  std::vector<double> nz;
  for (double t : travel) {if (t > 1e-6) {nz.push_back(t);}}
  r.pitch_m = options.pitch_m > 0.0 ? options.pitch_m : median(nz);
  const double jump_limit = options.jump_factor * r.pitch_m;
  for (double t : travel) {
    r.max_jump_m = std::max(r.max_jump_m, t);
    if (r.pitch_m > 0.0 && t > jump_limit + 1e-9) {++r.jumps;}
  }

  // Direction alternation over first passes.
  std::vector<const MotionPlanSegment *> firsts;
  for (const auto * s : strips) {if (!hasTag(*s, "second_pass")) {firsts.push_back(s);}}
  r.strips = firsts.size();
  auto dir = [&](const MotionPlanSegment * s) {
      const Vec3 a = pos(poses[s->start_index]), b = pos(poses[s->end_index]);
      return Vec3{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    };
  for (size_t k = 1; k < firsts.size(); ++k) {
    const Vec3 d0 = dir(firsts[k - 1]), d1 = dir(firsts[k]);
    const double dot = d0[0] * d1[0] + d0[1] * d1[1] + d0[2] * d1[2];
    const bool opposite = dot < 0.0;
    if (opposite != r.serpentine_expected) {++r.direction_violations;}
  }
  r.alternation_ok = r.direction_violations == 0;

  // Passes per cell, on the two axes of the largest extent.
  int ax[3] = {0, 1, 2};
  std::sort(ax, ax + 3, [&](int a, int b) {return (hi[a] - lo[a]) > (hi[b] - lo[b]);});
  const double cell = options.cell_m > 0.0 ? options.cell_m : (r.pitch_m > 0.0 ? r.pitch_m / 2.0 : 0.05);
  std::map<std::pair<long, long>, std::set<size_t>> hits;
  // Each cartesian pass counts as one "sweep"; a pass and its own reverse is the planned pass count.
  size_t strip_id = 0;
  std::vector<size_t> strip_of_pass(strips.size());
  for (size_t k = 0; k < strips.size(); ++k) {
    if (!hasTag(*strips[k], "second_pass")) {++strip_id;}
    strip_of_pass[k] = strip_id;
  }
  int passes_per_strip = 1;
  {
    std::map<size_t, int> n;
    for (size_t k = 0; k < strips.size(); ++k) {passes_per_strip = std::max(passes_per_strip, ++n[strip_of_pass[k]]);}
  }
  r.expected_passes_per_cell = passes_per_strip;
  for (size_t k = 0; k < strips.size(); ++k) {
    const auto & s = *strips[k];
    for (uint32_t i = s.start_index; i < s.end_index || i == s.start_index; ++i) {
      const Vec3 a = pos(poses[i]);
      const Vec3 b = pos(poses[std::min<uint32_t>(i + 1, s.end_index)]);
      const double len = dist(a, b);
      const int n = std::max(1, static_cast<int>(std::ceil(len / (cell / 2.0))));
      for (int j = 0; j <= n; ++j) {
        const double t = static_cast<double>(j) / n;
        const double u = a[ax[0]] + t * (b[ax[0]] - a[ax[0]]);
        const double v = a[ax[1]] + t * (b[ax[1]] - a[ax[1]]);
        hits[{static_cast<long>(std::floor(u / cell)), static_cast<long>(std::floor(v / cell))}].insert(k);
      }
      if (i == s.end_index) {break;}
    }
  }
  r.cells = hits.size();
  for (const auto & [c, set] : hits) {
    // A strip's own there-and-back passes are planned; two different strips over one cell are a double sweep.
    r.max_passes_per_cell = std::max(r.max_passes_per_cell, static_cast<int>(set.size()));
    std::set<size_t> distinct;
    for (size_t k : set) {distinct.insert(strip_of_pass[k]);}
    if (distinct.size() > 1) {++r.cells_swept_more_than_expected;}  // two different strips over one cell
  }
  r.double_sweep_fraction =
    r.cells ? static_cast<double>(r.cells_swept_more_than_expected) / static_cast<double>(r.cells) : 0.0;

  // End of the plan against the wall.
  r.bottom_z = std::isnan(options.wall_bottom_z) ? zmin : options.wall_bottom_z;
  r.top_z = std::isnan(options.wall_top_z) ? zmax : options.wall_top_z;
  r.final_end_z = poses[strips.back()->end_index].pose.position.z;
  const double tol = r.pitch_m > 0.0 ? r.pitch_m / 2.0 : 0.05;
  r.ends_at_bottom = r.final_end_z <= r.bottom_z + tol;
  r.ends_at_top = r.final_end_z >= r.top_z - tol;
  return r;
}

ExpandedCoverage expandedCoverageFromJson(const nlohmann::json & doc)
{
  const auto & node = doc.contains("final_lifted_and_expanded") ? doc.at("final_lifted_and_expanded") : doc;
  ExpandedCoverage out;
  for (const auto & p : node.at("path").at("poses")) {
    PoseStamped ps;
    const auto & q = p.contains("position") ? p.at("position") : p.at("pose").at("position");
    ps.pose.position = {q.at("x").get<double>(), q.at("y").get<double>(), q.at("z").get<double>()};
    out.path.poses.push_back(ps);
  }
  if (node.contains("segments")) {
    for (const auto & sj : node.at("segments")) {
      MotionPlanSegment s;
      s.label = sj.value("label", "");
      s.mode = sj.value("mode", "");
      s.start_index = sj.value("start_index", 0U);
      s.end_index = sj.value("end_index", 0U);
      if (sj.contains("tags")) {s.tags = sj.at("tags").get<std::vector<std::string>>();}
      out.segments.push_back(std::move(s));
    }
  }
  return out;
}

nlohmann::json redundancyReportToJson(const RedundancyReport & r)
{
  return {
    {"strips", r.strips}, {"strip_passes", r.strip_passes}, {"strip_length_m", r.strip_length_m},
    {"connector_length_m", r.connector_length_m}, {"pitch_m", r.pitch_m}, {"jumps_over_2x_pitch", r.jumps},
    {"max_jump_m", r.max_jump_m}, {"cells", r.cells},
    {"cells_swept_more_than_expected", r.cells_swept_more_than_expected},
    {"double_sweep_fraction", r.double_sweep_fraction}, {"max_passes_per_cell", r.max_passes_per_cell},
    {"expected_passes_per_cell", r.expected_passes_per_cell}, {"serpentine_expected", r.serpentine_expected},
    {"direction_violations", r.direction_violations}, {"alternation_ok", r.alternation_ok},
    {"final_end_z", r.final_end_z}, {"bottom_z", r.bottom_z}, {"top_z", r.top_z},
    {"ends_at_bottom", r.ends_at_bottom}, {"ends_at_top", r.ends_at_top}};
}

std::string redundancyReportToTable(const RedundancyReport & r)
{
  std::ostringstream o;
  o << std::fixed << std::setprecision(3);
  auto row = [&](const std::string & k, const std::string & v) {o << std::left << std::setw(34) << k << v << "\n";};
  row("strips (passes)", std::to_string(r.strips) + " (" + std::to_string(r.strip_passes) + ")");
  row("strip length [m]", std::to_string(r.strip_length_m));
  row("connector/jump length [m]", std::to_string(r.connector_length_m));
  row("pitch [m]", std::to_string(r.pitch_m));
  row("jumps > 2x pitch", std::to_string(r.jumps) + " (max " + std::to_string(r.max_jump_m) + " m)");
  row("double-sweep cell fraction", std::to_string(r.double_sweep_fraction) + " (max " +
    std::to_string(r.max_passes_per_cell) + " per cell, expected " + std::to_string(r.expected_passes_per_cell) + ")");
  row(
    "direction alternation",
    std::string(r.alternation_ok ? "OK" : "VIOLATED") + " (" + std::to_string(r.direction_violations) +
    " violations, " + (r.serpentine_expected ? "serpentine" : "same-start") + " expected)");
  row(
    "final end z / bottom / top", std::to_string(r.final_end_z) + " / " + std::to_string(r.bottom_z) + " / " +
    std::to_string(r.top_z) + (r.ends_at_bottom ? "  ends at bottom" : r.ends_at_top ? "  ends at top" : "  ends mid-wall"));
  return o.str();
}

// ---------------------------------------------------------------------------------------------------------------

namespace
{

constexpr size_t kExactLimit = 8;

/// Executed cost: chords end(i) -> start(i+1) in the given order and directions.
double executedStripTravel(const std::vector<OrderStrip> & s)
{
  double t = 0.0;
  for (size_t i = 1; i < s.size(); ++i) {t += dist(s[i - 1].end, s[i].start);}
  return t;
}

/// Best open path through the strips with free order, free start and free direction per strip.
double bestStripTravel(const std::vector<OrderStrip> & s, bool & exact)
{
  const size_t n = s.size();
  if (n < 2) {return 0.0;}
  // Endpoint e of strip i: e = 0 means entered at start (leave at end), e = 1 entered at end (leave at start).
  auto leave = [&](size_t i, int e) {return e == 0 ? s[i].end : s[i].start;};
  auto enter = [&](size_t i, int e) {return e == 0 ? s[i].start : s[i].end;};
  if (n <= kExactLimit) {
    const size_t full = (1u << n) - 1;
    std::vector<std::array<std::array<double, 2>, kExactLimit>> dp(1u << n);
    for (auto & row : dp) {for (auto & a : row) {a = {1e300, 1e300};}}
    for (size_t i = 0; i < n; ++i) {dp[1u << i][i] = {0.0, 0.0};}
    for (size_t m = 1; m <= full; ++m) {
      for (size_t i = 0; i < n; ++i) {
        for (int e = 0; e < 2; ++e) {
          const double c = dp[m][i][e];
          if (c >= 1e299) {continue;}
          for (size_t j = 0; j < n; ++j) {
            if (m & (1u << j)) {continue;}
            for (int f = 0; f < 2; ++f) {
              auto & slot = dp[m | (1u << j)][j][f];
              slot = std::min(slot, c + dist(leave(i, e), enter(j, f)));
            }
          }
        }
      }
    }
    double best = 1e300;
    for (size_t i = 0; i < n; ++i) {for (int e = 0; e < 2; ++e) {best = std::min(best, dp[full][i][e]);}}
    return best;
  }
  exact = false;
  double best = 1e300;
  for (size_t st = 0; st < n; ++st) {
    for (int e0 = 0; e0 < 2; ++e0) {
      std::vector<bool> used(n, false);
      used[st] = true;
      Vec3 at = leave(st, e0);
      double t = 0.0;
      for (size_t k = 1; k < n; ++k) {
        double bd = 1e300;
        size_t bj = 0;
        int bf = 0;
        for (size_t j = 0; j < n; ++j) {
          if (used[j]) {continue;}
          for (int f = 0; f < 2; ++f) {
            const double d = dist(at, enter(j, f));
            if (d < bd) {bd = d; bj = j; bf = f;}
          }
        }
        used[bj] = true;
        t += bd;
        at = leave(bj, bf);
      }
      best = std::min(best, t);
    }
  }
  return best;
}

}  // namespace

TravelOrderReport checkTravelOrder(const std::vector<StandOrderEntry> & stands)
{
  TravelOrderReport r;
  for (const auto & st : stands) {
    r.strip_travel_executed_m += executedStripTravel(st.strips);
    r.strip_travel_optimal_m += bestStripTravel(st.strips, r.exact);
  }
  for (size_t i = 1; i < stands.size(); ++i) {
    r.stand_travel_executed_m += dist(stands[i - 1].base, stands[i].base);
  }
  // Best order of the stands: free start, exact by permutation (a handful of stands).
  if (stands.size() > 1) {
    std::vector<size_t> idx(stands.size());
    for (size_t i = 0; i < idx.size(); ++i) {idx[i] = i;}
    double best = 1e300;
    if (stands.size() <= kExactLimit) {
      do {
        double t = 0.0;
        for (size_t i = 1; i < idx.size(); ++i) {t += dist(stands[idx[i - 1]].base, stands[idx[i]].base);}
        best = std::min(best, t);
      } while (std::next_permutation(idx.begin(), idx.end()));
    } else {
      r.exact = false;
      for (size_t st = 0; st < stands.size(); ++st) {
        std::vector<bool> used(stands.size(), false);
        used[st] = true;
        size_t at = st;
        double t = 0.0;
        for (size_t k = 1; k < stands.size(); ++k) {
          double bd = 1e300;
          size_t bj = 0;
          for (size_t j = 0; j < stands.size(); ++j) {
            if (!used[j] && dist(stands[at].base, stands[j].base) < bd) {
              bd = dist(stands[at].base, stands[j].base);
              bj = j;
            }
          }
          used[bj] = true;
          t += bd;
          at = bj;
        }
        best = std::min(best, t);
      }
    }
    r.stand_travel_optimal_m = best;
  }
  r.total_executed_m = r.strip_travel_executed_m + r.stand_travel_executed_m;
  r.total_optimal_m = r.strip_travel_optimal_m + r.stand_travel_optimal_m;
  r.excess_m = std::max(0.0, r.total_executed_m - r.total_optimal_m);
  r.excess_fraction = r.total_optimal_m > 1e-9 ? r.excess_m / r.total_optimal_m : 0.0;
  return r;
}

nlohmann::json travelOrderReportToJson(const TravelOrderReport & r)
{
  return {
    {"strip_travel_executed_m", r.strip_travel_executed_m}, {"strip_travel_optimal_m", r.strip_travel_optimal_m},
    {"stand_travel_executed_m", r.stand_travel_executed_m}, {"stand_travel_optimal_m", r.stand_travel_optimal_m},
    {"total_executed_m", r.total_executed_m}, {"total_optimal_m", r.total_optimal_m}, {"excess_m", r.excess_m},
    {"excess_fraction", r.excess_fraction}, {"exact", r.exact}};
}

}  // namespace coverage_planner
