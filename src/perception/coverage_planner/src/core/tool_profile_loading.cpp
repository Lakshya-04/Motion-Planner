#include "coverage_planner/core/tool_profile_loading.hpp"

namespace coverage_planner
{

ToolCoverageProfile toCoverageProfile(const core_types::ToolProfile & profile)
{
  ToolCoverageProfile out;
  out.name = profile.name;
  out.task.contact = profile.task.contact;
  out.task.standoff_m = profile.task.standoff_m;
  out.task.target_force_n = profile.task.target_force_n;
  out.task.tool_normal_policy = profile.task.tool_normal_policy;
  out.task.max_edge_tilt_deg = profile.task.max_edge_tilt_deg;
  out.task.tool_frame_yaw_deg = profile.task.tool_frame_yaw_deg;
  out.task.tool_tilt_deg = profile.geometry.tilt_deg;
  out.task.tool_tilt_axis = profile.geometry.tilt_axis;

  const auto & c = profile.coverage;
  out.coverage.swath_m = c.swath_m;
  out.coverage.spray_half_angle_deg = c.spray_half_angle_deg;
  out.coverage.effective_width_fraction = c.effective_width_fraction;
  out.coverage.overlap_pct = c.overlap_pct;
  out.coverage.sweep_direction = c.sweep_direction;
  out.coverage.enable_boundary_pass = c.enable_boundary_pass;
  out.coverage.boundary_offset_m = c.boundary_offset_m;
  out.coverage.obstacle_inflation = c.obstacle_inflation;
  out.coverage.obstacle_inflation_radius_m = c.obstacle_inflation_radius_m;
  out.coverage.ensure_coverage_within_boundary = c.ensure_coverage_within_boundary;
  out.coverage.uniform_direction = c.uniform_direction;
  out.coverage.contact_area = c.contact_area;
  out.coverage.edge_zone_m = c.edge_zone_m;
  out.coverage.edge_spacing_scale = c.edge_spacing_scale;
  out.coverage.passes.clear();
  for (const auto & p : c.passes) {
    out.coverage.passes.push_back(PassVariant{p.standoff_delta_m, p.reverse_direction});
  }
  if (out.coverage.passes.empty()) {
    out.coverage.passes.push_back(PassVariant{});
  }
  return out;
}

void applyToolOverrides(ToolCoverageProfile & tool, const ToolOverrides & o)
{
  if (o.contact) {tool.task.contact = *o.contact;}
  if (o.standoff_m) {tool.task.standoff_m = *o.standoff_m;}
  if (o.tool_normal_policy) {tool.task.tool_normal_policy = *o.tool_normal_policy;}
  if (o.max_edge_tilt_deg) {tool.task.max_edge_tilt_deg = *o.max_edge_tilt_deg;}
  if (o.tool_frame_yaw_deg) {tool.task.tool_frame_yaw_deg = *o.tool_frame_yaw_deg;}
  if (o.swath_m) {
    tool.coverage.swath_m = *o.swath_m;
    tool.coverage.spray_half_angle_deg = 0.0;
    tool.coverage.contact_area.pad_width_m = 0.0;  // an explicit swath wins over the model
  }
  if (o.overlap_pct) {tool.coverage.overlap_pct = *o.overlap_pct;}
  if (o.sweep_direction) {tool.coverage.sweep_direction = *o.sweep_direction;}
  if (o.enable_boundary_pass) {tool.coverage.enable_boundary_pass = *o.enable_boundary_pass;}
  if (o.boundary_offset_m) {tool.coverage.boundary_offset_m = *o.boundary_offset_m;}
  if (o.obstacle_inflation) {tool.coverage.obstacle_inflation = *o.obstacle_inflation;}
  if (o.obstacle_inflation_radius_m) {
    tool.coverage.obstacle_inflation_radius_m = *o.obstacle_inflation_radius_m;
  }
  if (o.edge_zone_m) {tool.coverage.edge_zone_m = *o.edge_zone_m;}
  if (o.edge_spacing_scale) {tool.coverage.edge_spacing_scale = *o.edge_spacing_scale;}
  if (o.double_pass && *o.double_pass) {
    tool.coverage.passes = {PassVariant{}, PassVariant{0.0, true}};
  }
  if (tool.coverage.obstacle_inflation_radius_m <= 0.0) {
    tool.coverage.obstacle_inflation_radius_m = toolSwath(tool) / 2.0;
  }
}

}  // namespace coverage_planner
