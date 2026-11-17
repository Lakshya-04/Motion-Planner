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
 *          The boustrophedon/sweepline planners still take one uniform step; this function is the
 *          building block for non-uniform strip placement and for scoring it.
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

/// Same, taking spacing and zone from the tool profile (toolLineSpacing, coverage.edge_*).
std::vector<double> contactStripOffsets(
  const ToolCoverageProfile & tool, double lo, double hi, const std::vector<double> & edges = {});

}  // namespace coverage_planner
