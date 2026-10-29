#ifndef COVERAGE_PLANNER__SWEEPLINE_PLANNER_HPP_
#define COVERAGE_PLANNER__SWEEPLINE_PLANNER_HPP_

#include <string>

#include "coverage_planner/core/coverage_planner_base.hpp"
#include "coverage_planner/core/types.hpp"

namespace coverage_planner
{

/**
 * @brief Sweepline coverage planner.
 * @details Slices the free space with parallel lines spaced by the stepover, finds the free
 *          spans on each slice (obstacles split a slice into several spans, concave obstacles
 *          included), and chains spans of consecutive slices into serpentine passes:
 *          a span joins a chain when its nearer endpoint can be reached from the chain's last
 *          point by a connector that crosses no obstacle and is at most two stepovers long.
 *          Spans that cannot join start a new chain, so an obstacle yields separate serpentine
 *          blocks around it, each internally continuous.
 *
 *          Design after the sweepline planner used in an earlier production stack (polygon
 *          based there); this version works on the same occupancy grid as BoustrophedonPlanner,
 *          so both are interchangeable behind CoveragePlannerBase.
 *
 *          Honors: swath, step, sweep_direction, start_corner, obstacle_inflation_radius,
 *          allow_unknown, region clipping, plan_all_components, min_component_cells,
 *          add_strip_connectors, requires_contact, ensure_coverage_within_boundary.
 */
class SweeplinePlanner : public CoveragePlannerBase
{
public:
  CoveragePlannerResult generatePath(
    const OccupancyGrid & map, const CoveragePlannerParams & params) override;

  std::string getName() const override {return "sweepline";}
};

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__SWEEPLINE_PLANNER_HPP_
