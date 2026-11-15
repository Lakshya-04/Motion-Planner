/**
 * @file tool_profile_loading.hpp
 * @brief Converts a full core_types::ToolProfile to the coverage subset and applies overrides.
 * @details ROS-free. The coverage nodes read `tool_profiles_file` / `tool_name`, load the profile
 *          (coverage_planner/ros/tool_profile_params.hpp), then fill a ToolOverrides from whichever
 *          explicit parameters (swath_m, overlap_pct, standoff_m, ...) the user actually set.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <optional>
#include <string>

#include "core_types/tool_profile.hpp"
#include "coverage_planner/core/tool_coverage_profile.hpp"

namespace coverage_planner
{

/// Explicit values that win over the profile. An unset optional leaves the profile value alone.
struct ToolOverrides
{
  std::optional<bool> contact;
  std::optional<double> standoff_m;
  std::optional<std::string> tool_normal_policy;
  std::optional<double> max_edge_tilt_deg;
  std::optional<double> tool_frame_yaw_deg;
  /// An explicit swath also switches off the spray-cone derivation (the user chose the width).
  std::optional<double> swath_m;
  std::optional<double> overlap_pct;
  std::optional<std::string> sweep_direction;
  std::optional<bool> enable_boundary_pass;
  std::optional<double> boundary_offset_m;
  std::optional<std::string> obstacle_inflation;
  std::optional<double> obstacle_inflation_radius_m;
  /// true forces the two-pass (forward, then reversed) strategy; false/unset keeps the profile's.
  std::optional<bool> double_pass;
};

/// Keeps the coverage-relevant facets of a full ToolProfile.
ToolCoverageProfile toCoverageProfile(const core_types::ToolProfile & profile);

/// Applies the overrides. When no obstacle inflation radius is set (profile or override) it falls
/// back to half the resolved swath.
void applyToolOverrides(ToolCoverageProfile & tool, const ToolOverrides & overrides);

}  // namespace coverage_planner
