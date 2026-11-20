#include "coverage_planner/core/contact_spacing.hpp"

#include <algorithm>
#include <cmath>
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
  int lo, int hi, bool ascending, int step_cells, int zone_cells, double edge_scale)
{
  if (step_cells < 1 || !(edge_scale > 0.0 && edge_scale <= 1.0)) {
    throw std::invalid_argument("stripCellPositions: invalid step or scale");
  }
  if (hi < lo) {return {};}
  const bool adaptive = zone_cells > 0 && edge_scale < 1.0;
  const int fine = std::max(1, static_cast<int>(step_cells * edge_scale));
  std::vector<int> out;
  if (ascending) {
    for (int x = lo; x <= hi;) {
      out.push_back(x);
      x += (adaptive && (x - lo < zone_cells || hi - x < zone_cells)) ? fine : step_cells;
    }
  } else {
    for (int x = hi; x >= lo;) {
      out.push_back(x);
      x -= (adaptive && (x - lo < zone_cells || hi - x < zone_cells)) ? fine : step_cells;
    }
  }
  return out;
}

std::vector<double> contactStripOffsets(
  const ToolCoverageProfile & tool, double lo, double hi, const std::vector<double> & edges)
{
  return contactStripOffsets(
    lo, hi, toolLineSpacing(tool), tool.coverage.edge_zone_m, tool.coverage.edge_spacing_scale,
    edges);
}

}  // namespace coverage_planner
