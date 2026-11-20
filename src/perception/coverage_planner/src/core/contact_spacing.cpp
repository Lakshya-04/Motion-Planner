#include "coverage_planner/core/contact_spacing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <stdexcept>

namespace coverage_planner
{

std::vector<double> contactStripOffsets(
  double lo, double hi, double spacing_m, double edge_zone_m, double edge_scale,
  const std::vector<double> & edges)
{
  if (!(hi >= lo) || !(spacing_m > 0.0) || !(edge_zone_m >= 0.0) ||
    !(edge_scale > 0.0 && edge_scale <= 1.0))
  {
    throw std::invalid_argument("contactStripOffsets: invalid range, spacing, zone or scale");
  }
  const auto in_zone = [&](double x) {
      if (edge_zone_m <= 0.0 || edge_scale >= 1.0) {return false;}
      if (x - lo < edge_zone_m || hi - x < edge_zone_m) {return true;}
      return std::any_of(
        edges.begin(), edges.end(), [&](double e) {return std::abs(x - e) < edge_zone_m;});
    };
  std::vector<double> out{lo};
  double x = lo;
  while (x < hi) {
    x += spacing_m * (in_zone(x) ? edge_scale : 1.0);
    if (x >= hi - 1e-9) {
      x = hi;
    }
    out.push_back(x);
  }
  return out;
}

std::vector<int> stripCellPositions(
  int lo, int hi, bool ascending, int step_cells, int zone_cells, double edge_scale,
  const std::vector<int> & edge_cells)
{
  if (step_cells < 1 || !(edge_scale > 0.0 && edge_scale <= 1.0)) {
    throw std::invalid_argument("stripCellPositions: invalid step or scale");
  }
  if (hi < lo) {return {};}
  const bool adaptive = zone_cells > 0 && edge_scale < 1.0;
  const int fine = std::max(1, static_cast<int>(step_cells * edge_scale));
  const auto in_zone = [&](int x) {
      if (!adaptive) {return false;}
      if (x - lo < zone_cells || hi - x < zone_cells) {return true;}
      return std::any_of(
        edge_cells.begin(), edge_cells.end(), [&](int e) {return std::abs(x - e) < zone_cells;});
    };
  std::vector<int> out;
  if (ascending) {
    for (int x = lo; x <= hi;) {
      out.push_back(x);
      x += in_zone(x) ? fine : step_cells;
    }
  } else {
    for (int x = hi; x >= lo;) {
      out.push_back(x);
      x -= in_zone(x) ? fine : step_cells;
    }
  }
  return out;
}

std::vector<int> obstacleEdgeCells(
  int a_lo, int a_hi, int b_lo, int b_hi, const std::function<bool(int, int)> & blocked)
{
  if (a_hi < a_lo || b_hi < b_lo) {return {};}
  const int na = a_hi - a_lo + 1, nb = b_hi - b_lo + 1;
  std::vector<uint8_t> seen(static_cast<size_t>(na) * nb, 0);
  const auto at = [&](int a, int b) {return static_cast<size_t>(a - a_lo) * nb + (b - b_lo);};
  std::set<int> edges;
  std::vector<std::pair<int, int>> stack;
  for (int a = a_lo; a <= a_hi; ++a) {
    for (int b = b_lo; b <= b_hi; ++b) {
      if (seen[at(a, b)] || !blocked(a, b)) {continue;}
      int lo = a, hi = a;
      seen[at(a, b)] = 1;
      stack.assign(1, {a, b});
      while (!stack.empty()) {
        const auto [ca, cb] = stack.back();
        stack.pop_back();
        lo = std::min(lo, ca);
        hi = std::max(hi, ca);
        const int da[4] = {1, -1, 0, 0}, db[4] = {0, 0, 1, -1};
        for (int k = 0; k < 4; ++k) {
          const int na2 = ca + da[k], nb2 = cb + db[k];
          if (na2 < a_lo || na2 > a_hi || nb2 < b_lo || nb2 > b_hi) {continue;}
          if (seen[at(na2, nb2)] || !blocked(na2, nb2)) {continue;}
          seen[at(na2, nb2)] = 1;
          stack.push_back({na2, nb2});
        }
      }
      edges.insert(lo);
      edges.insert(hi);
    }
  }
  return {edges.begin(), edges.end()};
}

std::pair<double, double> resolveEdgeSpacing(
  double request_zone_m, double request_scale, double node_zone_m, double node_scale)
{
  const double zone = request_zone_m > 0.0 ? request_zone_m : node_zone_m;
  double scale = (request_scale > 0.0 && request_scale < 1.0) ? request_scale : node_scale;
  if (scale > 1.0) {scale = 1.0;}
  return {zone, scale};
}

std::vector<double> contactStripOffsets(
  const ToolCoverageProfile & tool, double lo, double hi, const std::vector<double> & edges)
{
  return contactStripOffsets(
    lo, hi, toolLineSpacing(tool), tool.coverage.edge_zone_m, tool.coverage.edge_spacing_scale,
    edges);
}

}  // namespace coverage_planner
