/**
 * @file tool_coverage_profile.hpp
 * @brief Plain, data-driven description of the coverage/orientation-relevant facets of a tool,
 *        independent of ROS and of any sibling * package.
 * @details Only the fields wall_pose_lift.cpp/wall_rasterizer.cpp/coverage_pass_expander.cpp
 *          actually read are kept -- force/admittance/control-layer fields are execution-layer
 *          concerns, out of scope for coverage-path generation.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include "coverage_planner/core/types.hpp"
#include <string>
#include <vector>

namespace coverage_planner
{

/**
 * @struct PassVariant
 * @brief One entry in a tool's `passes` list -- a delta from the tool's own base task-facet
 *        values, not a full duplicate profile (mop = two identical-standoff passes, reversed
 *        direction; a hypothetical multi-standoff sprayer strategy = several passes at different
 *        standoffs).
 */
struct PassVariant
{
  double standoff_delta_m{0.0};   ///< Added to task.standoff_m for this pass only.
  bool reverse_direction{false};  ///< Traverse this pass in the opposite direction of the strip.
};

/**
 * @struct ToolTask
 * @brief Orientation/standoff semantics, consumed by wall_pose_lift.cpp.
 */
struct ToolTask
{
  bool contact{false};     ///< True for tools physically touching the wall.
  double standoff_m{0.0};  ///< Stand-off along the normal; 0.0 for contact tools.

  /// "constant": TCP Z always along -normal. "tilt_at_edges": may tilt at a physical wall
  /// corner/edge for reach -- a geometric trigger, non-contact tools only.
  std::string tool_normal_policy{"constant"};
  double max_edge_tilt_deg{0.0};  ///< Max tilt for the geometric tilt_at_edges policy.

  /// Rotates the commanded tool frame's X/Y axes about the tool's own Z (contact-normal, into
  /// the wall) axis, in degrees. 0.0 (default): X aligned to the wall's basis_u, Y to basis_v.
  double tool_frame_yaw_deg{0.0};
};

/**
 * @struct ToolCoverage
 * @brief Coverage-planning modifiers, consumed by the coverage core.
 */
struct ToolCoverage
{
  double swath_m{0.0};      ///< Effective tool footprint width (fixed-swath tools).
  /// Non-contact spray tools: cone half-angle in degrees. > 0 means the swath is derived from the
  /// stand-off (see toolSwath) and swath_m is not used. 0 = fixed swath_m.
  double spray_half_angle_deg{0.0};
  double effective_width_fraction{1.0};  ///< Usable fraction of the cone width, in (0, 1].
  double overlap_pct{0.0};  ///< step_m = swath_m * (1 - overlap_pct / 100).
  std::vector<PassVariant> passes{
    PassVariant{}};                         ///< Per-strip pass list; single default = one pass.
  std::string sweep_direction{"vertical"};  ///< "vertical" | "horizontal".
  bool enable_boundary_pass{false};         ///< Perimeter contour pass for edges.
  double boundary_offset_m{0.0};

  /// "uniform": inflate obstacle regions equally in u and v (contact tools -- must clear on every
  /// side). "primary_axis": inflate only along the sweep axis (non-contact -- the fan is long
  /// perpendicular to travel, so passes shift away along the sweep direction only).
  std::string obstacle_inflation{"uniform"};
  double obstacle_inflation_radius_m{0.0};      ///< margin = swath_m / 2 by convention, stored
                                                ///< explicitly so a profile author can override it.
  bool ensure_coverage_within_boundary{false};  ///< Force a final pass exactly at the region edge.
  bool uniform_direction{false};  ///< All passes traverse the same direction (sprayer bias).
};

/**
 * @struct ToolCoverageProfile
 * @brief One tool's coverage/orientation-relevant behavior -- the subset of a full ToolProfile
 *        the coverage planner actually needs.
 */
struct ToolCoverageProfile
{
  std::string name;
  ToolTask task;
  ToolCoverage coverage;
};

/// Swath the planner must use for this tool: the stand-off dependent cone model for a non-contact
/// tool with spray_half_angle_deg > 0, else coverage.swath_m.
/// @throws std::invalid_argument if the cone parameters or the stand-off are out of range.
double toolSwath(const ToolCoverageProfile & tool);

/// Distance between adjacent strips: toolSwath * (1 - overlap_pct / 100).
double toolLineSpacing(const ToolCoverageProfile & tool);

}  // namespace coverage_planner
