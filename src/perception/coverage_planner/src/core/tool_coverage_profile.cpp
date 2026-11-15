#include "coverage_planner/core/tool_coverage_profile.hpp"

#include "core_types/sprayer_effective_swath.hpp"

namespace coverage_planner
{

double toolSwath(const ToolCoverageProfile & tool)
{
  if (!tool.task.contact && tool.coverage.spray_half_angle_deg > 0.0) {
    core_types::SpraySwathModel model;
    model.half_angle_deg = tool.coverage.spray_half_angle_deg;
    model.effective_width_fraction = tool.coverage.effective_width_fraction;
    return core_types::sprayerEffectiveSwath(tool.task.standoff_m, model);
  }
  return tool.coverage.swath_m;
}

double toolLineSpacing(const ToolCoverageProfile & tool)
{
  return toolSwath(tool) * (1.0 - tool.coverage.overlap_pct / 100.0);
}

}  // namespace coverage_planner
