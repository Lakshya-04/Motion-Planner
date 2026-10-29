/**
 * @file tool_profile_yaml_loader.hpp
 * @brief Loads core_types::ToolProfile from tool_profiles.yaml.
 * @details Schema matches reference_docs/13_cleaning_modes_spec.md §2.1 exactly (top-level
 *          `schema_version` + `tools: {<name>: {geometry, task, coverage, control}}`). Enforces
 *          three of doc 13's four loader invariants purely from the YAML's own internal
 *          consistency (the fourth -- TCP transform matching the *running* URDF -- needs live
 *          robot_description context this loader doesn't have, and is left to the caller to
 *          cross-check at the ROS boundary), plus one new invariant this project added: a
 *          reachability tilt fallback is never permitted on a contact tool.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "core_types/tool_profile.hpp"

namespace ros_adapters
{

namespace detail
{

inline std::array<double, 3> readXyz(const YAML::Node & node, const std::string & field_name)
{
  const auto values = node.as<std::vector<double>>();
  if (values.size() != 3) {
    throw std::runtime_error(field_name + " must have exactly 3 elements");
  }
  return {values[0], values[1], values[2]};
}

inline core_types::ToolGeometry loadGeometry(const YAML::Node & node)
{
  core_types::ToolGeometry geometry;
  geometry.mesh = node["mesh"] ? node["mesh"].as<std::string>() : "";
  if (node["mesh_origin_xyz"]) {
    geometry.mesh_origin_xyz = readXyz(node["mesh_origin_xyz"], "geometry.mesh_origin_xyz");
  }
  if (node["mesh_origin_rpy"]) {
    geometry.mesh_origin_rpy = readXyz(node["mesh_origin_rpy"], "geometry.mesh_origin_rpy");
  }
  if (node["tcp_xyz"]) {
    geometry.tcp_xyz = readXyz(node["tcp_xyz"], "geometry.tcp_xyz");
  }
  if (node["tcp_rpy"]) {
    geometry.tcp_rpy = readXyz(node["tcp_rpy"], "geometry.tcp_rpy");
  }
  if (node["contact_allowed_links"]) {
    geometry.contact_allowed_links = node["contact_allowed_links"].as<std::vector<std::string>>();
  }
  return geometry;
}

inline core_types::ToolTask loadTask(const YAML::Node & node)
{
  core_types::ToolTask task;
  if (node["contact"]) {task.contact = node["contact"].as<bool>();}
  if (node["target_force_n"]) {task.target_force_n = node["target_force_n"].as<double>();}
  if (node["force_band_n"]) {task.force_band_n = node["force_band_n"].as<double>();}
  if (node["force_zero_threshold_n"]) {
    task.force_zero_threshold_n = node["force_zero_threshold_n"].as<double>();
  }
  if (node["standoff_m"]) {task.standoff_m = node["standoff_m"].as<double>();}
  if (node["standoff_tolerance_m"]) {
    task.standoff_tolerance_m = node["standoff_tolerance_m"].as<double>();
  }
  if (node["approach_standoff_m"]) {
    task.approach_standoff_m = node["approach_standoff_m"].as<std::vector<double>>();
  }
  if (node["approach_speed_mps"]) {
    task.approach_speed_mps = node["approach_speed_mps"].as<double>();
  }
  if (node["ee_speed_mps"]) {task.ee_speed_mps = node["ee_speed_mps"].as<double>();}
  if (node["tool_normal_policy"]) {
    task.tool_normal_policy = node["tool_normal_policy"].as<std::string>();
  }
  if (node["max_edge_tilt_deg"]) {task.max_edge_tilt_deg = node["max_edge_tilt_deg"].as<double>();}
  if (node["tool_frame_yaw_deg"]) {
    task.tool_frame_yaw_deg = node["tool_frame_yaw_deg"].as<double>();
  }
  if (node["reachability_tilt_deg_max"]) {
    task.reachability_tilt_deg_max = node["reachability_tilt_deg_max"].as<double>();
  }
  if (node["settle_timeout_s"]) {task.settle_timeout_s = node["settle_timeout_s"].as<double>();}
  if (node["overload_abort_n"]) {task.overload_abort_n = node["overload_abort_n"].as<double>();}
  if (node["tangential_torque_abort_nm"]) {
    task.tangential_torque_abort_nm = node["tangential_torque_abort_nm"].as<double>();
  }
  if (node["unexpected_contact_abort_n"]) {
    task.unexpected_contact_abort_n = node["unexpected_contact_abort_n"].as<double>();
  }
  if (node["tool_command"]) {task.tool_command = node["tool_command"].as<std::string>();}
  if (node["tool_off_lead_s"]) {task.tool_off_lead_s = node["tool_off_lead_s"].as<double>();}
  if (node["tool_on_at_speed_ratio"]) {
    task.tool_on_at_speed_ratio = node["tool_on_at_speed_ratio"].as<double>();
  }
  return task;
}

inline std::vector<core_types::PassVariant> loadPasses(const YAML::Node & node)
{
  std::vector<core_types::PassVariant> passes;
  for (const auto & pass_node : node) {
    core_types::PassVariant pass;
    if (pass_node["standoff_delta_m"]) {
      pass.standoff_delta_m = pass_node["standoff_delta_m"].as<double>();
    }
    if (pass_node["reverse_direction"]) {
      pass.reverse_direction = pass_node["reverse_direction"].as<bool>();
    }
    passes.push_back(pass);
  }
  if (passes.empty()) {
    passes.push_back(core_types::PassVariant{});
  }
  return passes;
}

inline core_types::ToolCoverage loadCoverage(const YAML::Node & node)
{
  core_types::ToolCoverage coverage;
  if (node["swath_m"]) {coverage.swath_m = node["swath_m"].as<double>();}
  if (node["overlap_pct"]) {coverage.overlap_pct = node["overlap_pct"].as<double>();}
  if (node["passes"]) {
    coverage.passes = loadPasses(node["passes"]);
  }
  if (node["sweep_direction"]) {
    coverage.sweep_direction = node["sweep_direction"].as<std::string>();
  }
  if (node["enable_boundary_pass"]) {
    coverage.enable_boundary_pass = node["enable_boundary_pass"].as<bool>();
  }
  if (node["boundary_offset_m"]) {
    coverage.boundary_offset_m = node["boundary_offset_m"].as<double>();
  }
  if (node["obstacle_inflation"]) {
    coverage.obstacle_inflation = node["obstacle_inflation"].as<std::string>();
  }
  if (node["obstacle_inflation_radius_m"]) {
    coverage.obstacle_inflation_radius_m = node["obstacle_inflation_radius_m"].as<double>();
  }
  if (node["ensure_coverage_within_boundary"]) {
    coverage.ensure_coverage_within_boundary = node["ensure_coverage_within_boundary"].as<bool>();
  }
  if (node["uniform_direction"]) {
    coverage.uniform_direction = node["uniform_direction"].as<bool>();
  }
  return coverage;
}

inline core_types::ToolControl loadControl(const YAML::Node & node)
{
  core_types::ToolControl control;
  if (node["controller"]) {control.controller = node["controller"].as<std::string>();}
  if (node["admittance"]) {
    const YAML::Node admittance_node = node["admittance"];
    core_types::ToolAdmittance & admittance = control.admittance;
    if (admittance_node["selected_axes"]) {
      const auto axes = admittance_node["selected_axes"].as<std::vector<bool>>();
      if (axes.size() != 6) {
        throw std::runtime_error("control.admittance.selected_axes must have exactly 6 elements");
      }
      for (size_t i = 0; i < 6; ++i) {
        admittance.selected_axes[i] = axes[i];
      }
    }
    if (admittance_node["mass"]) {
      admittance.mass = readXyz(admittance_node["mass"], "admittance.mass");
    }
    if (admittance_node["damping"]) {
      admittance.damping = readXyz(admittance_node["damping"], "admittance.damping");
    }
    if (admittance_node["stiffness"]) {
      admittance.stiffness = readXyz(admittance_node["stiffness"], "admittance.stiffness");
    }
    if (admittance_node["wrench_reference_z_n"]) {
      admittance.wrench_reference_z_n = admittance_node["wrench_reference_z_n"].as<double>();
    }
    if (admittance_node["trim_ki"]) {admittance.trim_ki = admittance_node["trim_ki"].as<double>();}
    if (admittance_node["trim_integral_max_n"]) {
      admittance.trim_integral_max_n = admittance_node["trim_integral_max_n"].as<double>();
    }
    if (admittance_node["contact_threshold_n"]) {
      admittance.contact_threshold_n = admittance_node["contact_threshold_n"].as<double>();
    }
    if (admittance_node["max_offset_m"]) {
      admittance.max_offset_m = admittance_node["max_offset_m"].as<double>();
    }
  }
  if (node["ft"]) {
    const YAML::Node ft_node = node["ft"];
    core_types::ToolFtHygiene & ft = control.ft;
    if (ft_node["lpf_cutoff_hz"]) {ft.lpf_cutoff_hz = ft_node["lpf_cutoff_hz"].as<double>();}
    if (ft_node["deadband_n"]) {ft.deadband_n = ft_node["deadband_n"].as<double>();}
    if (ft_node["clamp_n"]) {ft.clamp_n = ft_node["clamp_n"].as<double>();}
    if (ft_node["tare_per_goal"]) {ft.tare_per_goal = ft_node["tare_per_goal"].as<bool>();}
  }
  return control;
}

/**
 * @brief Validates the internal-consistency subset of doc 13's four loader invariants -- the
 *        TCP-vs-running-URDF check needs live robot_description context this function doesn't
 *        have, and is the caller's responsibility at the ROS boundary.
 * @throws std::runtime_error on any violated invariant, naming the tool and the specific rule.
 */
inline void validateToolProfile(const core_types::ToolProfile & profile)
{
  const auto fail = [&profile](const std::string & rule) {
      throw std::runtime_error("tool_profiles.yaml: tool '" + profile.name + "' violates: " + rule);
    };

  const bool is_admittance = profile.control.controller == "admittance_chain";
  if (profile.task.contact != is_admittance) {
    fail(
      "task.contact == true must imply control.controller == \"admittance_chain\" (and vice versa)");
  }
  if (is_admittance &&
    profile.control.admittance.wrench_reference_z_n != profile.task.target_force_n)
  {
    fail("control.admittance.wrench_reference_z_n must equal task.target_force_n");
  }
  if (!profile.task.contact) {
    if (profile.task.standoff_m <= 0.0) {
      fail("task.contact == false requires task.standoff_m > 0");
    }
    if (!profile.geometry.contact_allowed_links.empty()) {
      fail("task.contact == false requires geometry.contact_allowed_links to be empty");
    }
  }
  // This project's own addition, not in doc 13: a reachability tilt fallback changes the tool's
  // final orientation, which is unsafe for a tool meant to be in contact with the wall -- gate
  // explicitly here rather than trusting a profile author to leave the default alone.
  if (profile.task.contact && profile.task.reachability_tilt_deg_max > 0.0) {
    fail("task.reachability_tilt_deg_max must be 0 whenever task.contact == true");
  }
}

}  // namespace detail

/**
 * @brief Loads one named tool's profile from a tool_profiles.yaml file.
 * @param[in] yaml_path Path to a schema_version: 1 tool_profiles.yaml (reference_docs/13 §2.1).
 * @param[in] tool_name Key under the top-level `tools:` map (e.g. "sprayer", "mop", "contact_soap").
 * @return The fully-populated, invariant-checked ToolProfile.
 * @throws std::runtime_error if the file, the `tools:` section, or the named tool is missing, or
 *         any loader invariant is violated.
 */
inline core_types::ToolProfile loadToolProfile(
  const std::string & yaml_path, const std::string & tool_name)
{
  const YAML::Node config = YAML::LoadFile(yaml_path);
  if (!config["tools"]) {
    throw std::runtime_error("tool_profiles.yaml: no 'tools' section in " + yaml_path);
  }
  const YAML::Node tools = config["tools"];
  if (!tools[tool_name]) {
    throw std::runtime_error(
            "tool_profiles.yaml: no tool named '" + tool_name + "' in " + yaml_path);
  }
  const YAML::Node tool_node = tools[tool_name];

  core_types::ToolProfile profile;
  profile.name = tool_name;
  if (tool_node["geometry"]) {profile.geometry = detail::loadGeometry(tool_node["geometry"]);}
  if (tool_node["task"]) {profile.task = detail::loadTask(tool_node["task"]);}
  if (tool_node["coverage"]) {profile.coverage = detail::loadCoverage(tool_node["coverage"]);}
  if (tool_node["control"]) {profile.control = detail::loadControl(tool_node["control"]);}

  detail::validateToolProfile(profile);
  return profile;
}

}  // namespace ros_adapters
