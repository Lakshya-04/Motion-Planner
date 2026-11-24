/**
 * @file plan_redundancy.hpp
 * @brief Redundancy report of an expanded coverage plan, and a strip/stand order check.
 * @details ROS-free. Answers "does the plan move for nothing?":
 *          - strip length vs connector/jump length (free travel between strips),
 *          - jumps longer than twice the strip pitch,
 *          - how many strips sweep each cell of the wall (double-sweep fraction),
 *          - whether consecutive strips alternate direction (serpentine) or, for a tool that makes a
 *            there-and-back pass over each strip, all start at the same end,
 *          - where the last strip ends (z) against the wall's bottom and top.
 *          Separately, `checkTravelOrder` compares the executed order of strips (per stand) and of
 *          stands with the optimal order and reports the excess free-space travel.
 *          Input is `expandCoveragePasses` output or `final_lifted_and_expanded` of coverage_output.json.
 *          Cells are taken on the two axes of the path with the largest extent (the wall plane), so the
 *          report is independent of the wall's orientation; "z" in the end check is world z.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "coverage_planner/core/coverage_pass_expander.hpp"

namespace coverage_planner
{

struct RedundancyOptions
{
  double pitch_m{0.0};          ///< Strip pitch; 0 = estimated (median distance between neighbouring strips).
  double jump_factor{2.0};      ///< A connector longer than jump_factor * pitch is a jump.
  double cell_m{0.0};           ///< Cell size for the pass count; 0 = pitch / 2.
  double wall_bottom_z{std::numeric_limits<double>::quiet_NaN()};  ///< NaN = lowest strip point.
  double wall_top_z{std::numeric_limits<double>::quiet_NaN()};     ///< NaN = highest strip point.
  /// -1 = auto: serpentine when no segment carries the "second_pass" tag, else every pass-0 strip must start at
  /// the same end as the previous one. 1 = force serpentine, 0 = force same-start.
  int expect_serpentine{-1};
};

struct RedundancyReport
{
  size_t strips{0};              ///< Strips counted (first passes only).
  size_t strip_passes{0};        ///< All cartesian passes, a double-pass tool counts two per strip.
  double strip_length_m{0.0};    ///< Sum over all cartesian passes.
  double connector_length_m{0.0};///< Path travelled between consecutive cartesian passes (connectors + jumps).
  double pitch_m{0.0};           ///< As given or estimated.
  size_t jumps{0};               ///< Connectors longer than jump_factor * pitch.
  double max_jump_m{0.0};
  size_t cells{0};               ///< Cells touched by a strip.
  size_t cells_swept_more_than_expected{0};
  double double_sweep_fraction{0.0};  ///< Of the touched cells.
  int max_passes_per_cell{0};
  int expected_passes_per_cell{1};
  bool serpentine_expected{true};
  size_t direction_violations{0};
  bool alternation_ok{true};
  double final_end_z{0.0};
  double bottom_z{0.0};
  double top_z{0.0};
  bool ends_at_bottom{false};    ///< Final end within pitch/2 of the bottom.
  bool ends_at_top{false};
};

/// @throws std::invalid_argument if a segment indexes outside the path.
RedundancyReport analyzePlanRedundancy(
  const Path & path, const std::vector<MotionPlanSegment> & segments, const RedundancyOptions & options = {});

/// `doc` is a coverage_output.json (or just its `final_lifted_and_expanded` object).
/// @throws std::runtime_error / nlohmann::json::exception on a missing field.
ExpandedCoverage expandedCoverageFromJson(const nlohmann::json & doc);

nlohmann::json redundancyReportToJson(const RedundancyReport & r);
std::string redundancyReportToTable(const RedundancyReport & r);

// ---------------------------------------------------------------------------------------------------------------
// Travel order check
// ---------------------------------------------------------------------------------------------------------------

using Vec3 = std::array<double, 3>;

struct OrderStrip
{
  Vec3 start{};   ///< TCP at the start of the strip as executed.
  Vec3 end{};
};

struct StandOrderEntry
{
  Vec3 base{};                      ///< Stand base position.
  std::vector<OrderStrip> strips;   ///< The strips (columns) done from this stand, in the executed order.
};

struct TravelOrderReport
{
  double strip_travel_executed_m{0.0};  ///< TCP chords between consecutive strips of a stand, summed over stands.
  double strip_travel_optimal_m{0.0};   ///< Same with the best order and best direction of each strip.
  double stand_travel_executed_m{0.0};  ///< Base distance between consecutive stands.
  double stand_travel_optimal_m{0.0};   ///< Best visiting order (free start).
  double total_executed_m{0.0};
  double total_optimal_m{0.0};
  double excess_m{0.0};
  double excess_fraction{0.0};          ///< excess / optimal (0 when optimal is 0).
  bool exact{true};                     ///< False when a stand had more than 8 strips (nearest neighbour used).
};

/// Free-space travel of the executed order vs the optimum. Per stand: exact search over the order and the direction
/// of each strip for up to 8 strips, nearest neighbour (best over all starts) beyond. Stands: exact for up to 8.
/// The strips of a stand keep the stand's own travel; the stand-to-stand move is the base distance.
TravelOrderReport checkTravelOrder(const std::vector<StandOrderEntry> & stands);

nlohmann::json travelOrderReportToJson(const TravelOrderReport & r);

}  // namespace coverage_planner
