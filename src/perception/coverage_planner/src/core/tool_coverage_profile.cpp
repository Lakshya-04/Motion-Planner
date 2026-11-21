#include "coverage_planner/core/tool_coverage_profile.hpp"

#include <cmath>

#include "core_types/sprayer_effective_swath.hpp"
#include "core_types/tool_footprint.hpp"

namespace coverage_planner
{

double tiltDirectionFromStrokeDeg(const ToolCoverageProfile & tool)
{
  const auto & a = tool.task.tool_tilt_axis;
  const double axis_deg = std::atan2(a[1], a[0]) * 180.0 / M_PI + tool.task.tool_frame_yaw_deg;
  const double stroke_deg = tool.coverage.sweep_direction == "horizontal" ? 0.0 : 90.0;
  return axis_deg + 90.0 - stroke_deg;
}

double toolSwath(const ToolCoverageProfile & tool)
{
  const double tilt = std::fabs(tool.task.tool_tilt_deg);
  if (!tool.task.contact && tool.coverage.spray_half_angle_deg > 0.0) {
    core_types::SpraySwathModel model;
    model.half_angle_deg = tool.coverage.spray_half_angle_deg;
    model.effective_width_fraction = tool.coverage.effective_width_fraction;
    const double w = core_types::sprayerEffectiveSwath(tool.task.standoff_m, model);
    return tilt == 0.0 ? w :
           core_types::tiltedSwathAcrossStroke(w, 0.0, tilt, tiltDirectionFromStrokeDeg(tool));
  }
  if (tool.task.contact && tool.coverage.contact_area.enabled()) {
    const double w =
      core_types::contactEffectiveWidth(tool.coverage.contact_area, tool.task.target_force_n);
    return tilt == 0.0 ? w :
           core_types::tiltedSwathAcrossStroke(
      w, tool.coverage.contact_area.pad_length_m, tilt, tiltDirectionFromStrokeDeg(tool));
  }
  return tool.coverage.swath_m;
}

double toolLineSpacing(const ToolCoverageProfile & tool)
{
  return toolSwath(tool) * (1.0 - tool.coverage.overlap_pct / 100.0);
}

}  // namespace coverage_planner
