/**
 * @file tool_profile.hpp
 * @brief Plain, data-driven description of one physical end-effector tool, independent of ROS.
 * @details The four-facet shape (geometry / task / coverage / control) matches
 *          the `tool_profiles.yaml` schema field for field. Adapters
 *          (ros_adapters) load this from YAML at the boundary -- this type never depends on
 *          ROS or YAML itself. One tool difference, one struct: the coverage core, manipulation
 *          manager, and force layer all read the same `ToolProfile` instead of forking per tool.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <array>
#include <string>
#include <vector>

namespace core_types
{

/**
 * @struct ToolGeometry
 * @brief Mesh + flange-to-TCP transform, consumed by URDF/xacro + MoveIt collision.
 */
struct ToolGeometry
{
  std::string mesh;                              ///< Mesh filename (relative to the tool package).
  std::array<double, 3> mesh_origin_xyz{0, 0, 0}; ///< Mesh origin translation.
  std::array<double, 3> mesh_origin_rpy{0, 0, 0}; ///< Mesh origin rotation (roll, pitch, yaw).
  std::array<double, 3> tcp_xyz{0, 0, 0};         ///< Flange -> link_tcp translation.
  std::array<double, 3> tcp_rpy{0, 0, 0};         ///< Flange -> link_tcp rotation.
  /// Links allowed to contact the wall collision object (ACM exclusion), empty for non-contact tools.
  std::vector<std::string> contact_allowed_links;
};

/**
 * @struct PassVariant
 * @brief One entry in a tool's `passes` list -- a delta from the tool's own base task-facet
 *        values, not a full duplicate profile. Replaces a single `passes_per_strip` scalar so
 *        "this tool needs N passes, each varying standoff/direction" is a config change, not a
 *        new code path (e.g. mop = two identical-standoff passes, reversed direction; a
 *        multi-standoff sprayer strategy = several passes at different standoffs).
 */
struct PassVariant
{
  double standoff_delta_m{0.0};   ///< Added to task.standoff_m for this pass only.
  bool reverse_direction{false};  ///< Traverse this pass in the opposite direction of the strip.
};

/**
 * @struct ToolTask
 * @brief Contact/force/speed/orientation semantics, consumed by the manipulation manager, force
 *        layer, and segment generation.
 */
struct ToolTask
{
  bool contact{false};                    ///< True for tools physically touching the wall.
  double target_force_n{0.0};             ///< Normal force target along tool +Z, contact tools only.
  double force_band_n{0.0};               ///< Settle/convergence band around target_force_n.
  double force_zero_threshold_n{0.0};     ///< Unload-converged threshold.
  double standoff_m{0.0};                 ///< Stand-off along the normal; 0.0 for contact tools.
  double standoff_tolerance_m{0.0};       ///< Allowed standoff tracking error, non-contact tools.
  std::vector<double> approach_standoff_m; ///< Approach waypoint standoffs, empty if no approach cycle.
  double approach_speed_mps{0.0};         ///< Speed during the approach cycle.
  double ee_speed_mps{0.0};               ///< Required constant EE speed along the surface.

  /// "constant": TCP Z always along -normal. "tilt_at_edges": may tilt at a physical wall
  /// corner/edge for reach -- a geometric trigger, non-contact tools only.
  std::string tool_normal_policy{"constant"};
  double max_edge_tilt_deg{0.0};          ///< Max tilt for the geometric tilt_at_edges policy.

  /// Rotates the commanded tool frame's X/Y axes about the tool's own Z (contact-normal, into
  /// the wall) axis, in degrees. 0.0 (default): X aligned to the wall's basis_u, Y to basis_v --
  /// unchanged behavior. Use this to change which physical axis of an asymmetric tool (e.g. a
  /// mop pad longer one way than the other) tracks the wall's "up the wall" direction vs. "across
  /// it" -- NOT tcp_xyz/tcp_rpy in geometry (those are a static URDF mounting offset for FK/
  /// collision, orthogonal to this: they never affect what orientation wall_pose_lift.cpp
  /// computes and commands link_tcp to reach).
  double tool_frame_yaw_deg{0.0};

  /// Reachability-triggered tilt fallback (distinct from tilt_at_edges above -- this one fires
  /// when the ideal perpendicular pose is kinematically unreachable, not at a geometric edge).
  /// 0.0 (default) disables it. MUST stay 0.0 whenever `contact` is true -- tilting a tool meant
  /// to be in contact with the wall changes the force/contact-normal semantics, not just reach;
  /// consumers must gate on `!contact` explicitly rather than trusting this field alone
  /// (a tilted tool in contact has different force/contact-normal semantics).
  double reachability_tilt_deg_max{0.0};

  double settle_timeout_s{0.0};           ///< FSM watchdog for force settle.
  double overload_abort_n{0.0};           ///< Instantaneous force-abort threshold.
  double tangential_torque_abort_nm{0.0}; ///< Tangential-torque-abort threshold.
  double unexpected_contact_abort_n{0.0}; ///< Non-contact tools: |F| above this = collision abort.

  std::string tool_command{"none"};       ///< "none" | "pump" | "valve".
  double tool_off_lead_s{0.0};            ///< Close the valve this many seconds before strip end.
  double tool_on_at_speed_ratio{0.0};      ///< Pump ON once EE speed reaches this fraction of nominal.
};

/**
 * @struct ToolCoverage
 * @brief Coverage-planning modifiers, consumed by the coverage core.
 */
struct ToolCoverage
{
  double swath_m{0.0};                   ///< Effective tool footprint width. Ignored by the coverage
                                         ///< planner for a non-contact tool with spray_half_angle_deg > 0.
  /// Non-contact spray tools: cone half-angle in degrees. When > 0 the planner derives the swath
  /// from the stand-off (sprayerEffectiveSwath) instead of using swath_m. 0 = fixed swath_m.
  double spray_half_angle_deg{0.0};
  /// Usable fraction of the geometric cone width (Gaussian-like deposition), in (0, 1].
  double effective_width_fraction{1.0};
  double overlap_pct{0.0};               ///< step_m = swath_m * (1 - overlap_pct / 100).
  std::vector<PassVariant> passes{PassVariant{}}; ///< Per-strip pass list; single default = one pass.
  std::string sweep_direction{"vertical"}; ///< "vertical" | "horizontal".
  bool enable_boundary_pass{false};       ///< Perimeter contour pass for edges.
  double boundary_offset_m{0.0};

  /// "uniform": inflate obstacle regions equally in u and v (contact tools -- must clear on every
  /// side). "primary_axis": inflate only along the sweep axis (non-contact -- the fan is long
  /// perpendicular to travel, so passes shift away along the sweep direction only).
  std::string obstacle_inflation{"uniform"};
  double obstacle_inflation_radius_m{0.0}; ///< margin = swath_m / 2 by convention (the earlier prototype's
                                           ///< proven formula), stored explicitly rather than
                                           ///< recomputed so a profile author can override it.
  bool ensure_coverage_within_boundary{false}; ///< Force a final pass exactly at the region edge.
  bool uniform_direction{false};          ///< All passes traverse the same direction (sprayer bias).
};

/**
 * @struct ToolAdmittance
 * @brief Admittance-chain parameters, only meaningful when ToolControl::controller ==
 *        "admittance_chain".
 */
struct ToolAdmittance
{
  std::array<bool, 6> selected_axes{false, false, false, false, false, false}; ///< x,y,z,rx,ry,rz.
  std::array<double, 3> mass{0, 0, 0};
  std::array<double, 3> damping{0, 0, 0};
  std::array<double, 3> stiffness{0, 0, 0};
  double wrench_reference_z_n{0.0}; ///< Must equal ToolTask::target_force_n (loader-checked).
  double trim_ki{0.0};
  double trim_integral_max_n{0.0};
  double contact_threshold_n{0.0};
  double max_offset_m{0.0};
};

/**
 * @struct ToolFtHygiene
 * @brief Force-torque sensor conditioning, shared by every tool regardless of controller.
 */
struct ToolFtHygiene
{
  double lpf_cutoff_hz{0.0};
  double deadband_n{0.0};
  double clamp_n{0.0};
  bool tare_per_goal{true};
};

/**
 * @struct ToolControl
 * @brief Force/control-layer profile.
 */
struct ToolControl
{
  std::string controller{"jtc_only"}; ///< "admittance_chain" | "jtc_only".
  ToolAdmittance admittance;
  ToolFtHygiene ft;
};

/**
 * @struct ToolProfile
 * @brief One physical tool's complete, data-driven behavior across all four consumers.
 */
struct ToolProfile
{
  std::string name;
  ToolGeometry geometry;
  ToolTask task;
  ToolCoverage coverage;
  ToolControl control;
};

}  // namespace core_types
