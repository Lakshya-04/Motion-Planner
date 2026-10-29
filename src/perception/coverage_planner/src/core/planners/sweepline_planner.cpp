#include "coverage_planner/core/sweepline_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <queue>

namespace coverage_planner
{
namespace
{

struct Strip
{
  int p;   // slice position along the primary axis (grid cells)
  int s0;  // entry along the secondary axis
  int s1;  // exit along the secondary axis
};

struct Chain
{
  std::vector<Strip> strips;
  int last_slice{-1};  // index into the slice list of the chain's last strip
  bool used{false};
};

struct Span
{
  int lo;
  int hi;
  bool taken{false};
};

bool pointInPolygon(double x, double y, const std::vector<Point32> & poly)
{
  bool inside = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const double xi = poly[i].x, yi = poly[i].y, xj = poly[j].x, yj = poly[j].y;
    if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
  }
  return inside;
}

// 4-connected components of the free mask, largest first.
std::vector<std::vector<int>> components(const std::vector<uint8_t> & free_mask, int w, int h)
{
  std::vector<uint8_t> seen(free_mask.size(), 0);
  std::vector<std::vector<int>> out;
  for (int start = 0; start < w * h; ++start) {
    if (!free_mask[start] || seen[start]) {
      continue;
    }
    std::vector<int> comp;
    std::queue<int> q;
    q.push(start);
    seen[start] = 1;
    while (!q.empty()) {
      const int cur = q.front();
      q.pop();
      comp.push_back(cur);
      const int x = cur % w, y = cur / w;
      const int nx[4] = {x - 1, x + 1, x, x};
      const int ny[4] = {y, y, y - 1, y + 1};
      for (int k = 0; k < 4; ++k) {
        if (nx[k] < 0 || nx[k] >= w || ny[k] < 0 || ny[k] >= h) {
          continue;
        }
        const int n = ny[k] * w + nx[k];
        if (free_mask[n] && !seen[n]) {
          seen[n] = 1;
          q.push(n);
        }
      }
    }
    out.push_back(std::move(comp));
  }
  std::sort(out.begin(), out.end(), [](const auto & a, const auto & b) {return a.size() > b.size();});
  return out;
}

MotionPlanSegment makeSegment(
  const std::string & label, uint32_t start, uint32_t end, const std::string & frame_id,
  const std::string & mode, const std::string & planner, bool contact, bool retraction,
  const std::vector<std::string> & tags)
{
  MotionPlanSegment s;
  s.label = label;
  s.start_index = start;
  s.end_index = end;
  s.render_start_index = start;
  s.render_end_index = end;
  s.frame_id = frame_id;
  s.mode = mode;
  s.planner = planner;
  s.requires_contact = contact;
  s.requires_retraction = retraction;
  s.priority = 0.0F;
  s.tags = tags;
  return s;
}

}  // namespace

CoveragePlannerResult SweeplinePlanner::generatePath(
  const OccupancyGrid & map, const CoveragePlannerParams & params)
{
  CoveragePlannerResult result{};
  result.success = false;
  result.effective_step_used = params.step;
  const auto t0 = std::chrono::steady_clock::now();

  if (map.data.empty() || map.info.width == 0 || map.info.height == 0) {
    result.message = "Invalid or empty map";
    return result;
  }
  if (map.info.resolution <= 0.0 || params.step <= 0.0 || params.swath <= 0.0) {
    result.message = "resolution, swath and step must be positive";
    return result;
  }

  const int w = static_cast<int>(map.info.width);
  const int h = static_cast<int>(map.info.height);
  const double res = map.info.resolution;

  // Free mask after inflation, optional region clipping and small-island removal.
  std::vector<int8_t> grid = map.data;
  if (params.obstacle_inflation_radius > 0.0) {
    inflateObstacles(grid, w, h, res, params.obstacle_inflation_radius, params.allow_unknown);
  }
  std::vector<uint8_t> free_mask(grid.size(), 0);
  for (size_t i = 0; i < grid.size(); ++i) {
    free_mask[i] = isCellFree(grid[i], params.allow_unknown) ? 1 : 0;
  }
  if (params.use_region && params.region.polygon.points.size() >= 3) {
    const std::string rf = params.region.header.frame_id;
    if (rf.empty() || rf == map.header.frame_id) {
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          if (!free_mask[y * w + x]) {
            continue;
          }
          const PoseStamped c = gridToWorld(x, y, map, nullptr, "identity");
          if (!pointInPolygon(c.pose.position.x, c.pose.position.y, params.region.polygon.points)) {
            free_mask[y * w + x] = 0;
          }
        }
      }
    } else {
      logWarn("Region frame '%s' != map frame '%s'; ignoring region clipping", rf.c_str(),
        map.header.frame_id.c_str());
    }
  }
  auto comps = components(free_mask, w, h);
  std::fill(free_mask.begin(), free_mask.end(), 0);
  int kept = 0;
  for (size_t c = 0; c < comps.size(); ++c) {
    if (static_cast<int>(comps[c].size()) < params.min_component_cells) {
      continue;
    }
    if (!params.plan_all_components && kept >= 1) {
      break;
    }
    for (int idx : comps[c]) {
      free_mask[idx] = 1;
    }
    ++kept;
  }
  result.free_component_count = kept;
  if (kept == 0) {
    result.message = "No free space found after clipping/filtering";
    return result;
  }

  // Primary axis = the one slices step along; secondary = along each pass.
  const bool vertical = determineWorldAwareSweepDirection(params.sweep_direction, map);
  const int p_size = vertical ? w : h;
  const int s_size = vertical ? h : w;
  // Build the plan in a frame mirrored toward the requested corner, so a new chain always
  // starts at that corner (reversing a finished chain cannot always reach it).
  const bool right = params.start_corner.find("right") != std::string::npos;
  const bool top = params.start_corner.find("top") != std::string::npos;
  const bool flip_p = vertical ? right : top;
  const bool flip_s = vertical ? top : right;
  auto realP = [&](int p) {return flip_p ? p_size - 1 - p : p;};
  auto realS = [&](int s) {return flip_s ? s_size - 1 - s : s;};
  auto isFree = [&](int p, int s) {
      const int rp = realP(p), rs = realS(s);
      return vertical ? free_mask[static_cast<size_t>(rs) * w + rp] != 0 :
             free_mask[static_cast<size_t>(rp) * w + rs] != 0;
    };
  auto toXY = [&](int p, int s) {
      const int rp = realP(p), rs = realS(s);
      return vertical ? std::make_pair(rp, rs) : std::make_pair(rs, rp);
    };

  int p_min = p_size, p_max = -1;
  for (int p = 0; p < p_size; ++p) {
    for (int s = 0; s < s_size; ++s) {
      if (isFree(p, s)) {
        p_min = std::min(p_min, p);
        p_max = std::max(p_max, p);
        break;
      }
    }
  }

  // Slice positions: first inset by half a swath, then every stepover, with the last pass
  // flush against the far edge when ensure_coverage_within_boundary is set.
  const int half = static_cast<int>(std::lround(0.5 * params.swath / res));
  const int step = std::max(1, static_cast<int>(std::lround(params.step / res)));
  std::vector<int> slices;
  if (p_max - p_min < 2 * half) {
    slices.push_back((p_min + p_max) / 2);
  } else {
    for (int p = p_min + half; p <= p_max - half; p += step) {
      slices.push_back(p);
    }
    const int last_edge = p_max - half;
    if (params.ensure_coverage_within_boundary) {
      if (slices.back() < last_edge) {
        slices.push_back(last_edge);
      }
    } else if (slices.back() + step <= p_max) {
      slices.push_back(std::min(slices.back() + step, p_max));
    }
  }

  // Free spans per slice, chained across consecutive slices.
  auto lineFree = [&](int p0, int s0, int p1, int s1) {
      const int n = std::max(std::abs(p1 - p0), std::abs(s1 - s0));
      for (int i = 0; i <= n; ++i) {
        const double t = n == 0 ? 0.0 : static_cast<double>(i) / n;
        if (!isFree(
            static_cast<int>(std::lround(p0 + t * (p1 - p0))),
            static_cast<int>(std::lround(s0 + t * (s1 - s0)))))
        {
          return false;
        }
      }
      return true;
    };

  std::vector<Chain> chains;
  std::vector<size_t> open;  // chains whose last strip is on the previous slice
  for (size_t si = 0; si < slices.size(); ++si) {
    const int p = slices[si];
    std::vector<Span> spans;
    for (int s = 0; s < s_size;) {
      if (!isFree(p, s)) {
        ++s;
        continue;
      }
      int e = s;
      while (e + 1 < s_size && isFree(p, e + 1)) {
        ++e;
      }
      spans.push_back({s, e, false});
      s = e + 1;
    }

    struct Candidate
    {
      double length;
      size_t chain;
      size_t span;
      bool enter_low;
    };
    std::vector<Candidate> candidates;
    for (size_t ci : open) {
      const Strip & last = chains[ci].strips.back();
      for (size_t k = 0; k < spans.size(); ++k) {
        for (bool low : {true, false}) {
          const int entry = low ? spans[k].lo : spans[k].hi;
          const double len = std::hypot(p - last.p, entry - last.s1);
          if (len <= 2.0 * step && lineFree(last.p, last.s1, p, entry)) {
            candidates.push_back({len, ci, k, low});
          }
        }
      }
    }
    std::sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
      return a.length < b.length;
    });
    std::vector<size_t> next_open;
    for (const auto & c : candidates) {
      if (chains[c.chain].used || spans[c.span].taken) {
        continue;
      }
      chains[c.chain].used = true;
      spans[c.span].taken = true;
      const int s0 = c.enter_low ? spans[c.span].lo : spans[c.span].hi;
      const int s1 = c.enter_low ? spans[c.span].hi : spans[c.span].lo;
      chains[c.chain].strips.push_back({p, s0, s1});
      chains[c.chain].last_slice = static_cast<int>(si);
      next_open.push_back(c.chain);
    }
    for (size_t ci : open) {
      chains[ci].used = false;
    }
    for (auto & sp : spans) {
      if (sp.taken) {
        continue;
      }
      Chain chain;
      chain.strips.push_back({p, sp.lo, sp.hi});  // new chains start low -> high
      chain.last_slice = static_cast<int>(si);
      chains.push_back(std::move(chain));
      next_open.push_back(chains.size() - 1);
    }
    open = std::move(next_open);
  }

  // Order chains by nearest start from the requested corner, reversing a chain when its far
  // end is the closer one.
  auto cornerXY = [&]() {return std::make_pair(right ? w - 1 : 0, top ? h - 1 : 0);};
  auto chainEnds = [&](const Chain & c, bool reversed) {
      const Strip & first = reversed ? c.strips.back() : c.strips.front();
      const Strip & last = reversed ? c.strips.front() : c.strips.back();
      const auto a = toXY(first.p, reversed ? first.s1 : first.s0);
      const auto b = toXY(last.p, reversed ? last.s0 : last.s1);
      return std::make_pair(a, b);
    };
  std::vector<std::pair<size_t, bool>> order;
  std::vector<uint8_t> done(chains.size(), 0);
  auto here = cornerXY();
  for (size_t n = 0; n < chains.size(); ++n) {
    double best = 1e18;
    size_t best_i = 0;
    bool best_rev = false;
    for (size_t i = 0; i < chains.size(); ++i) {
      if (done[i]) {
        continue;
      }
      for (bool rev : {false, true}) {
        const auto ends = chainEnds(chains[i], rev);
        const double d = std::hypot(ends.first.first - here.first, ends.first.second - here.second);
        if (d < best) {
          best = d;
          best_i = i;
          best_rev = rev;
        }
      }
    }
    done[best_i] = 1;
    order.emplace_back(best_i, best_rev);
    here = chainEnds(chains[best_i], best_rev).second;
  }

  // Emit waypoints and segments.
  result.path.header = map.header;
  const std::string axis_tag = vertical ? "v" : "h";
  int chain_number = 0;
  for (const auto & [ci, reversed] : order) {
    const Chain & chain = chains[ci];
    std::vector<Strip> strips = chain.strips;
    if (reversed) {
      std::reverse(strips.begin(), strips.end());
      for (auto & st : strips) {
        std::swap(st.s0, st.s1);
      }
    }
    for (size_t k = 0; k < strips.size(); ++k) {
      const auto a = toXY(strips[k].p, strips[k].s0);
      const auto b = toXY(strips[k].p, strips[k].s1);
      const PoseStamped start_pose = gridToWorld(
        a.first, a.second, map, result.path.poses.empty() ? nullptr : &result.path.poses.back(),
        params.pose_mode);
      const PoseStamped end_pose = gridToWorld(b.first, b.second, map, &start_pose, params.pose_mode);
      const uint32_t start_index = static_cast<uint32_t>(result.path.poses.size());
      result.path.poses.push_back(start_pose);
      result.path.poses.push_back(end_pose);
      std::vector<std::string> tags = {"strip", "sweepline", "chain_" + std::to_string(chain_number)};
      if (k == 0) {
        tags.push_back("chain_start");
      }
      result.segments.push_back(makeSegment(
        "strip_" + axis_tag + "_" + std::to_string(result.segments.size()), start_index,
        start_index + 1, map.header.frame_id, "cartesian", "cartesian", params.requires_contact,
        false, tags));
      if (params.add_strip_connectors && k + 1 < strips.size()) {
        result.segments.push_back(makeSegment(
          "connector_" + std::to_string(result.segments.size()), start_index + 1, start_index + 2,
          map.header.frame_id, "fsp", "ompl", false, true, {"connector"}));
      }
      ++result.num_strips;
    }
    ++chain_number;
  }
  result.num_obstacle_splits = std::max(0, chain_number - 1);

  result.planning_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  calculatePathStats(result.path, result.total_distance, result.num_waypoints);
  result.coverage_area = params.swath * result.total_distance;
  result.success = !result.path.poses.empty();
  result.message = result.success ? "Coverage path generated successfully" :
    "Failed to generate coverage path";
  return result;
}

}  // namespace coverage_planner
