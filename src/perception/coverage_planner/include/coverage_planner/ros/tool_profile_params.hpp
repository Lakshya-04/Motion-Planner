/**
 * @file tool_profile_params.hpp
 * @brief Node parameters that select a tool profile and override single fields of it.
 * @details Declares `tool_profiles_file` (default: the file this package ships), `tool_name`
 *          (default: `contact_wiper`) and one optional override parameter per planning field.
 *          An override parameter has no default: it only takes effect when it is set (launch
 *          argument, params file or `--ros-args -p`). Shared by wall_coverage_demo_node and
 *          wall_coverage_service_node.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <optional>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_adapters/tool_profile_yaml_loader.hpp"

#include "coverage_planner/core/tool_coverage_profile.hpp"
#include "coverage_planner/core/tool_profile_loading.hpp"

namespace coverage_planner
{

/// Path of the tool_profiles.yaml installed with this package.
inline std::string defaultToolProfilesFile()
{
  return ament_index_cpp::get_package_share_directory("coverage_planner") +
         "/config/tool_profiles.yaml";
}

/// Declares an override parameter that has no value until the user sets one.
inline void declareOptionalParameter(rclcpp::Node & node, const std::string & name)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  node.declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
}

inline std::optional<double> optionalDouble(rclcpp::Node & node, const std::string & name)
{
  const auto p = node.get_parameter(name);
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {return p.as_double();}
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(p.as_int());
  }
  return std::nullopt;
}

inline std::optional<bool> optionalBool(rclcpp::Node & node, const std::string & name)
{
  const auto p = node.get_parameter(name);
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {return p.as_bool();}
  return std::nullopt;
}

inline std::optional<std::string> optionalString(rclcpp::Node & node, const std::string & name)
{
  const auto p = node.get_parameter(name);
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING && !p.as_string().empty()) {
    return p.as_string();
  }
  return std::nullopt;
}

/**
 * @brief Declares the tool parameters on `node`, loads the selected profile and applies overrides.
 * @throws std::runtime_error if the profiles file or the named tool is missing or invalid.
 */
inline ToolCoverageProfile loadToolCoverageProfileFromParams(rclcpp::Node & node)
{
  std::string file = node.declare_parameter<std::string>("tool_profiles_file", "");
  if (file.empty()) {file = defaultToolProfilesFile();}
  const std::string name = node.declare_parameter<std::string>("tool_name", "contact_wiper");

  for (const char * param :
    {"contact", "standoff_m", "tool_normal_policy", "max_edge_tilt_deg", "tool_frame_yaw_deg",
      "swath_m", "overlap_pct", "sweep_direction", "enable_boundary_pass", "boundary_offset_m",
      "obstacle_inflation", "obstacle_inflation_radius_m", "edge_zone_m", "edge_spacing_scale",
      "double_pass"})
  {
    declareOptionalParameter(node, param);
  }

  ToolOverrides o;
  o.contact = optionalBool(node, "contact");
  o.standoff_m = optionalDouble(node, "standoff_m");
  o.tool_normal_policy = optionalString(node, "tool_normal_policy");
  o.max_edge_tilt_deg = optionalDouble(node, "max_edge_tilt_deg");
  o.tool_frame_yaw_deg = optionalDouble(node, "tool_frame_yaw_deg");
  o.swath_m = optionalDouble(node, "swath_m");
  o.overlap_pct = optionalDouble(node, "overlap_pct");
  o.sweep_direction = optionalString(node, "sweep_direction");
  o.enable_boundary_pass = optionalBool(node, "enable_boundary_pass");
  o.boundary_offset_m = optionalDouble(node, "boundary_offset_m");
  o.obstacle_inflation = optionalString(node, "obstacle_inflation");
  o.obstacle_inflation_radius_m = optionalDouble(node, "obstacle_inflation_radius_m");
  o.edge_zone_m = optionalDouble(node, "edge_zone_m");
  o.edge_spacing_scale = optionalDouble(node, "edge_spacing_scale");
  o.double_pass = optionalBool(node, "double_pass");

  ToolCoverageProfile tool = toCoverageProfile(ros_adapters::loadToolProfile(file, name));
  applyToolOverrides(tool, o);
  RCLCPP_INFO(
    node.get_logger(), "Tool profile '%s' from %s (contact=%s, standoff=%.3f m, swath=%.3f m)",
    name.c_str(), file.c_str(), tool.task.contact ? "true" : "false", tool.task.standoff_m,
    toolSwath(tool));
  return tool;
}

}  // namespace coverage_planner
