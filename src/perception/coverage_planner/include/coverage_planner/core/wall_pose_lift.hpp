#ifndef COVERAGE_PLANNER__WALL_POSE_LIFT_HPP_
#define COVERAGE_PLANNER__WALL_POSE_LIFT_HPP_


#include "coverage_planner/core/types.hpp"
#include "coverage_planner/core/tool_coverage_profile.hpp"

namespace coverage_planner
{


/**
 * @brief Lifts a flat (identity-orientation) 2D coverage path -- already positioned in 3D world
 *        space via the wall basis grid origin -- into final oriented, standoff-offset poses.
 * @details Reads the WallPlane basis/normal + active ToolProfile.task to compute, per pose:
 *          - Orientation: TCP Z axis anti-parallel to `wall.normal_outward`; X/Y tangent axes
 *            built from `wall.basis_u`/`basis_v` so the tool doesn't twist along travel.
 *          - Standoff offset: shifted `standoff_m` along +normal_outward (0 for contact tools).
 *          - Geometric edge tilt (`tool_normal_policy == "tilt_at_edges"`, non-contact tools
 *            only -- contact tools are hard-excluded from this branch regardless of policy
 *            string): within `standoff_m` of a wall boundary, the Z axis tilts toward it by up to
 *            `max_edge_tilt_deg`, scaling linearly to full tilt at the boundary.
 * @param flat_path Positions only (world frame), identity orientation, one wall's coverage path.
 * @param wall The wall this path was planned on.
 * @param tool_task The active tool's task facet. `standoff_m` is the effective standoff for this
 *        call (base + any per-pass delta).
 */
Path liftCoveragePathPose(
  const Path & flat_path, const WallPlane & wall,
  const coverage_planner::ToolTask & tool_task);

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__WALL_POSE_LIFT_HPP_
