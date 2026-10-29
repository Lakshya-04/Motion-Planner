#include "coverage_planner/core/coverage_patterns.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

#include "coverage_planner/core/wall_basis.hpp"

namespace coverage_planner
{

WallPattern parseWallPattern(const std::string & name)
{
  if (name == "vertical") {return WallPattern::kVertical;}
  if (name == "horizontal") {return WallPattern::kHorizontal;}
  if (name == "zigzag") {return WallPattern::kZigzag;}
  throw std::invalid_argument(
          "coverage pattern must be 'vertical', 'horizontal' or 'zigzag', got '" + name + "'");
}

const char * toString(WallPattern pattern)
{
  switch (pattern) {
    case WallPattern::kVertical: return "vertical";
    case WallPattern::kHorizontal: return "horizontal";
    case WallPattern::kZigzag: return "zigzag";
  }
  return "vertical";
}

std::vector<PanelInfo> splitIntoPanels(
  double u_min, double u_max, double width, double overlap, bool start_left)
{
  std::vector<PanelInfo> panels;
  const double length = u_max - u_min;
  if (length <= 0.0 || width <= 0.0) {
    return panels;
  }
  overlap = std::clamp(overlap, 0.0, 0.5 * width);
  int n = 1;
  if (length > width) {
    n = static_cast<int>(std::ceil((length - overlap) / (width - overlap)));
  }
  // Even split: n panels of equal width w with (n - 1) overlaps cover the length exactly.
  const double w = n == 1 ? length : (length + (n - 1) * overlap) / n;
  for (int k = 0; k < n; ++k) {
    PanelInfo p;
    p.u_min = u_min + k * (w - overlap);
    p.u_max = p.u_min + w;
    p.stand_u = 0.5 * (p.u_min + p.u_max);
    panels.push_back(p);
  }
  if (!start_left) {
    std::reverse(panels.begin(), panels.end());
  }
  for (size_t k = 0; k < panels.size(); ++k) {
    panels[k].index = static_cast<int>(k);
  }
  return panels;
}

namespace
{

struct Strip
{
  PoseStamped a;
  PoseStamped b;
  std::vector<std::string> tags;
};

struct Uv
{
  double u;
  double v;
};

Uv uvOf(const PoseStamped & p, const WallBasis & basis)
{
  const Eigen::Vector3d rel(p.pose.position.x, p.pose.position.y, p.pose.position.z);
  const Eigen::Vector3d d = rel - basis.origin;
  return {d.dot(basis.basis_u), d.dot(basis.basis_v)};
}

double dist(const PoseStamped & a, const PoseStamped & b)
{
  return std::hypot(
    std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y),
    a.pose.position.z - b.pose.position.z);
}

std::vector<Strip> stripsOf(const Path & path, const std::vector<MotionPlanSegment> & segments)
{
  std::vector<Strip> out;
  for (const auto & s : segments) {
    if (s.label.rfind("strip", 0) == 0) {
      out.push_back({path.poses[s.start_index], path.poses[s.end_index], s.tags});
    }
  }
  return out;
}

MotionPlanSegment makeSeg(
  const std::string & label, uint32_t start, uint32_t end, const std::string & frame,
  const std::string & mode, const std::string & planner, bool contact, bool retract,
  std::vector<std::string> tags)
{
  MotionPlanSegment s;
  s.label = label;
  s.start_index = start;
  s.end_index = end;
  s.render_start_index = start;
  s.render_end_index = end;
  s.frame_id = frame;
  s.mode = mode;
  s.planner = planner;
  s.requires_contact = contact;
  s.requires_retraction = retract;
  s.tags = std::move(tags);
  return s;
}

// Rebuilds a part's path and segments from strips. Consecutive strips are joined by a connector
// segment when `connectors` is set (always for the raster pattern: the swing back is long).
void rebuild(
  PatternPart & part, const std::vector<Strip> & strips, const std::string & frame, bool contact,
  bool connectors)
{
  part.path.poses.clear();
  part.segments.clear();
  part.path.header.frame_id = frame;
  for (size_t k = 0; k < strips.size(); ++k) {
    const uint32_t start = static_cast<uint32_t>(part.path.poses.size());
    part.path.poses.push_back(strips[k].a);
    part.path.poses.push_back(strips[k].b);
    part.segments.push_back(makeSeg(
      "strip_" + std::to_string(part.segments.size()), start, start + 1, frame, "cartesian",
      "cartesian", contact, false, strips[k].tags));
    if (connectors && k + 1 < strips.size()) {
      part.segments.push_back(makeSeg(
        "connector_" + std::to_string(part.segments.size()), start + 1, start + 2, frame, "fsp",
        "ompl", false, true, {"connector"}));
    }
  }
}

// Plans one masked region with the chosen grid planner. Returns false when nothing was planned.
bool planRegion(
  const OccupancyGrid & grid, const CoveragePlannerParams & params, const std::string & planner_name,
  CoveragePlannerResult & out)
{
  auto planner = CoveragePlannerFactory::createPlanner(planner_name);
  if (!planner) {
    out.message = "unknown planner '" + planner_name + "'";
    return false;
  }
  out = planner->generatePath(grid, params);
  return out.success && !out.path.poses.empty();
}

}  // namespace

PatternPlan planWallPattern(
  const WallPlane & wall, const OccupancyGrid & grid, const ToolCoverageProfile & tool,
  const PatternOptions & options, const CoveragePlannerParams & base_params)
{
  PatternPlan plan;
  const double res = grid.info.resolution;
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  const WallBasis basis = decomposeWallBasis(wall);
  const std::string frame = grid.header.frame_id;

  auto maskedCopy = [&](auto blocked) {
      OccupancyGrid g = grid;
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          if (blocked(wall.u_min + (x + 0.5) * res, wall.v_min + (y + 0.5) * res)) {
            g.data[static_cast<size_t>(y) * w + x] = 100;
          }
        }
      }
      return g;
    };

  const double v_floor = wall.v_min + options.floor_clearance_m;

  // -------------------------------------------------------------- vertical
  if (options.pattern == WallPattern::kVertical) {
    double band_top = v_floor;
    if (options.enable_bottom_pass && options.bottom_band_height_m > 0.0) {
      band_top = wall.v_min + options.bottom_band_height_m;
      // Bottom pass: horizontal rows over the band, with the tool turned like the panels.
      const double swath = options.panel_swath_m > 0.0 ? options.panel_swath_m : tool.coverage.swath_m;
      CoveragePlannerParams p = base_params;
      p.sweep_direction = "horizontal";
      p.swath = swath;
      p.step = swath * (1.0 - tool.coverage.overlap_pct / 100.0);
      const OccupancyGrid band = maskedCopy([&](double, double v) {return v < v_floor || v >= band_top;});
      CoveragePlannerResult r;
      if (planRegion(band, p, options.planner, r)) {
        PatternPart part;
        part.name = "bottom_pass";
        part.path = r.path;
        part.segments = r.segments;
        part.tool = tool;
        part.tool.coverage.sweep_direction = "horizontal";
        part.tool.coverage.swath_m = swath;
        part.tool.task.tool_frame_yaw_deg += options.panel_tool_turn_deg;
        plan.parts.push_back(std::move(part));
      }
    }
    CoveragePlannerParams p = base_params;
    p.sweep_direction = "vertical";
    const OccupancyGrid strokes = maskedCopy([&](double, double v) {return v < band_top;});
    CoveragePlannerResult r;
    if (!planRegion(strokes, p, options.planner, r)) {
      plan.message = r.message.empty() ? "vertical strokes: nothing to cover" : r.message;
      return plan;
    }
    PatternPart part;
    part.name = "strokes";
    part.path = r.path;
    part.segments = r.segments;
    part.tool = tool;
    part.tool.coverage.sweep_direction = "vertical";
    plan.parts.push_back(std::move(part));
    plan.success = true;
    plan.message = "ok";
    return plan;
  }

  // -------------------------------------------------------------- panels
  const bool raster = options.pattern == WallPattern::kZigzag;
  const double u0 = wall.u_min + options.panel_end_margin_m;
  const double u1 = wall.u_max - options.panel_end_margin_m;
  if (u1 - u0 <= 0.0 || wall.v_max - v_floor <= 0.0) {
    plan.message = "nothing is left of the wall once the floor and the ends are kept clear";
    return plan;
  }
  plan.panels = splitIntoPanels(u0, u1, options.panel_width_m, options.panel_overlap_m, options.start_left);

  const double swath = options.panel_swath_m > 0.0 ? options.panel_swath_m : tool.coverage.swath_m;
  ToolCoverageProfile panel_tool = tool;
  panel_tool.coverage.sweep_direction = "horizontal";
  panel_tool.coverage.swath_m = swath;
  panel_tool.task.tool_frame_yaw_deg += options.panel_tool_turn_deg;

  CoveragePlannerParams p = base_params;
  p.sweep_direction = "horizontal";
  p.swath = swath;
  p.step = swath * (1.0 - tool.coverage.overlap_pct / 100.0);

  const PoseStamped * last_end = nullptr;
  PoseStamped last_end_storage;
  for (const auto & panel : plan.panels) {
    const OccupancyGrid region = maskedCopy([&](double u, double v) {
        return u < panel.u_min || u > panel.u_max || v < v_floor;
      });
    CoveragePlannerResult r;
    if (!planRegion(region, p, options.planner, r)) {
      continue;  // a panel with no free cells (all obstacle) contributes nothing
    }
    std::vector<Strip> strips = stripsOf(r.path, r.segments);
    if (strips.empty()) {
      continue;
    }
    const std::string panel_tag = "panel_" + std::to_string(panel.index);
    for (auto & s : strips) {
      s.tags.push_back(panel_tag);
    }

    // Candidate row orders for this panel; keep the one whose first point is nearest the
    // previous panel's last point (so the base steps on where the arm already is).
    auto reversedStrips = [](std::vector<Strip> v) {
        std::reverse(v.begin(), v.end());
        for (auto & s : v) {std::swap(s.a, s.b);}
        return v;
      };
    std::vector<Strip> chosen;
    if (!raster) {
      chosen = strips;  // serpentine as planned: each row ends where the next begins
      if (options.panel_alternate && last_end != nullptr &&
        dist(*last_end, reversedStrips(strips).front().a) < dist(*last_end, strips.front().a))
      {
        chosen = reversedStrips(strips);
      }
    } else {
      // Raster: every row the same way (+u when the wall is entered from the left, else -u),
      // rows ordered bottom-up or top-down, whichever starts nearer the previous end.
      const bool ltr = options.start_left;
      auto oriented = [&](const Strip & s) {
          Strip o = s;
          const bool a_left = uvOf(s.a, basis).u <= uvOf(s.b, basis).u;
          if (a_left != ltr) {std::swap(o.a, o.b);}
          return o;
        };
      std::vector<Strip> rows;
      for (const auto & s : strips) {rows.push_back(oriented(s));}
      auto rowKey = [&](const Strip & s) {return std::lround(uvOf(s.a, basis).v / res);};
      auto sortRows = [&](bool bottom_up) {
          std::vector<Strip> v = rows;
          std::stable_sort(v.begin(), v.end(), [&](const Strip & x, const Strip & y) {
            const auto kx = rowKey(x), ky = rowKey(y);
            if (kx != ky) {return bottom_up ? kx < ky : kx > ky;}
            const double ux = uvOf(x.a, basis).u, uy = uvOf(y.a, basis).u;
            return ltr ? ux < uy : ux > uy;
          });
          return v;
        };
      const auto up = sortRows(true);
      const auto down = sortRows(false);
      chosen = up;
      if (options.panel_alternate && last_end != nullptr &&
        dist(*last_end, down.front().a) < dist(*last_end, up.front().a))
      {
        chosen = down;
      }
    }

    PatternPart part;
    part.name = "panel_" + std::to_string(panel.index);
    part.panel = panel.index;
    part.tool = panel_tool;
    rebuild(part, chosen, frame, base_params.requires_contact, /*connectors=*/raster || p.add_strip_connectors);
    last_end_storage = part.path.poses.back();
    last_end = &last_end_storage;
    plan.parts.push_back(std::move(part));
  }

  if (plan.parts.empty()) {
    plan.message = "no panel had free space to cover";
    return plan;
  }
  plan.success = true;
  plan.message = "ok";
  return plan;
}

}  // namespace coverage_planner
