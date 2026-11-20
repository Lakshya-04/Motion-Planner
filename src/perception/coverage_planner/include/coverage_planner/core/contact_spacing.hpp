/**
 * @file contact_spacing.hpp
 * @brief Strip positions for a contact tool with optional narrower spacing near edges.
 * @details Uniform spacing (toolLineSpacing) leaves the last strip before a boundary or obstacle
 *          arbitrarily overlapped or, with a pad that narrows near an edge, gapped. With
 *          `coverage.edge_zone_m > 0` and `edge_spacing_scale < 1`, strips that start within the
 *          zone of the range ends or of an obstacle edge advance by spacing * scale instead, so the
 *          pad overlaps more where the contact is least reliable. Spacing never exceeds the
 *          uniform value, so a gap-free uniform plan stays gap-free. Limit: spacing varies across
 *          strips (perpendicular to travel), not along a single stroke; strokes stay parallel.
 *          `stripCellPositions` is the same rule on a cell grid; the boustrophedon and sweepline
 *          planners use it when `CoveragePlannerParams::edge_zone_m` and `edge_spacing_scale` are
 *          set. Only the range ends open a zone there (obstacle edges are not passed in).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <vector>

#include "coverage_planner/core/tool_coverage_profile.hpp"

namespace coverage_planner
{

/// Strip centre coordinates (perpendicular to travel) over [lo, hi], first at lo, last at hi.
/// @param spacing_m uniform spacing, > 0
/// @param edge_zone_m width of the edge zone (0 = uniform)
/// @param edge_scale spacing factor in the zone, in (0, 1]
/// @param edges extra edge positions (obstacle borders) that also open a zone
/// @throws std::invalid_argument for hi < lo, spacing <= 0, scale outside (0, 1], zone < 0.
std::vector<double> contactStripOffsets(
  double lo, double hi, double spacing_m, double edge_zone_m, double edge_scale,
  const std::vector<double> & edges = {});

/// Strip positions in grid cells over [lo, hi] (lo <= hi), visited from the low end when `ascending`
/// and from the high end otherwise. With `zone_cells` <= 0 or `edge_scale` >= 1 the result is exactly
/// the uniform loop `lo, lo+step, ... <= hi` (mirrored when descending), no last strip is forced at the
/// far end. Otherwise a strip within `zone_cells` of either end advances by
/// max(1, floor(step_cells * edge_scale)) cells.
/// Empty for hi < lo.
/// @throws std::invalid_argument for step_cells < 1 or scale outside (0, 1].
std::vector<int> stripCellPositions(
  int lo, int hi, bool ascending, int step_cells, int zone_cells, double edge_scale);

/// Same, taking spacing and zone from the tool profile (toolLineSpacing, coverage.edge_*).
std::vector<double> contactStripOffsets(
  const ToolCoverageProfile & tool, double lo, double hi, const std::vector<double> & edges = {});

}  // namespace coverage_planner
