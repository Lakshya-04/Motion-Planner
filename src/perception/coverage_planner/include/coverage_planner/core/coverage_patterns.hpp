// Wall-level coverage patterns. A pattern decides how a whole wall is covered; the grid planner
// underneath (sweepline or boustrophedon) only plans one region at a time.
//
//   vertical    vertical strokes over the whole wall, optionally preceded by a bottom pass of
//               horizontal rows for the strip the vertical strokes cannot reach.
//   horizontal  the wall split into panels, each reachable from one base stand, covered with
//               horizontal rows; a row ends where the next begins (serpentine). Panels
//               alternate their row order so the arm is already at the far end when the base
//               steps along the wall.
//   zigzag      the same panels, but every row runs the same way and the tool swings back
//               diagonally to the start of the next row.
//
// Pure core: no ROS, no PCL.
#pragma once

#include <string>
#include <vector>

#include "coverage_planner/core/coverage_planner_base.hpp"
#include "coverage_planner/core/tool_coverage_profile.hpp"
#include "coverage_planner/core/types.hpp"

namespace coverage_planner
{

enum class WallPattern
{
  kVertical,
  kHorizontal,
  kZigzag,
};

/// "vertical" | "horizontal" | "zigzag". Throws std::invalid_argument otherwise.
WallPattern parseWallPattern(const std::string & name);
const char * toString(WallPattern pattern);

struct PatternOptions
{
  WallPattern pattern{WallPattern::kVertical};

  // Panels (horizontal, zigzag).
  double panel_width_m{0.65};     ///< Sideways reach of one base stand.
  double panel_overlap_m{0.03};   ///< How far neighbouring panels reach into each other.
  double panel_swath_m{0.0};      ///< Row swath; 0 = the tool's swath.
  double panel_end_margin_m{0.0}; ///< Stay this far clear of both wall ends (the base cannot stand nearer).
  double panel_tool_turn_deg{90.0};  ///< Added to the tool yaw for horizontal rows (e.g. a mop turned vertical).
  bool panel_alternate{true};     ///< Let every panel start at whichever end is nearer the previous one.
  bool start_left{true};          ///< First panel at the low-u end of the wall.

  /// Lowest row's centre line stays this far above the wall's bottom edge (what the arm can
  /// reach). Applies to the panels and to the vertical pattern's bottom pass.
  double floor_clearance_m{0.0};

  // Vertical pattern: a bottom pass of horizontal rows below `bottom_band_height_m`, done first.
  bool enable_bottom_pass{false};
  double bottom_band_height_m{0.0};

  /// Underlying grid planner: "sweepline" or "boustrophedon".
  std::string planner{"sweepline"};
};

/// One panel = one base stand along the wall.
struct PanelInfo
{
  int index{0};
  double u_min{0.0};
  double u_max{0.0};
  double stand_u{0.0};  ///< Where the base should stand along the wall (panel centre).
};

/// A piece of the plan that is lifted with its own tool profile (panels use a turned tool).
struct PatternPart
{
  std::string name;  ///< "strokes", "bottom_pass", "panel_<k>"
  Path path;
  std::vector<MotionPlanSegment> segments;
  ToolCoverageProfile tool;
  int panel{-1};
};

struct PatternPlan
{
  bool success{false};
  std::string message;
  std::vector<PatternPart> parts;  ///< In execution order.
  std::vector<PanelInfo> panels;   ///< Empty for the vertical pattern.
};

/// Splits [u_min, u_max] into panels at most `width` wide, neighbours overlapping by `overlap`,
/// as evenly as possible. Ordered from the low-u end when `start_left`, else from the high end.
std::vector<PanelInfo> splitIntoPanels(
  double u_min, double u_max, double width, double overlap, bool start_left);

/// Plans a whole wall. `grid` is the wall's rasterized coverage grid (obstacles and unobserved
/// cells already set; cell (x, y) is at u = wall.u_min + (x + 0.5) * resolution, likewise v).
PatternPlan planWallPattern(
  const WallPlane & wall, const OccupancyGrid & grid, const ToolCoverageProfile & tool,
  const PatternOptions & options, const CoveragePlannerParams & base_params);

}  // namespace coverage_planner
