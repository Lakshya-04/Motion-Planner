/**
 * @file ik_then_plan_planner.hpp
 * @brief Planner decorator that turns a pose goal into a joint goal with an IkPort, for planners
 *        that only take joint goals (VAMP).
 * @details A joint goal goes straight to the inner planner. A pose goal is solved by the IkPort
 *          from the start state, then from up to `max_ik_seeds` more seeds (home pose, then
 *          perturbed copies of the start from a fixed RNG seed); the first solution inside the joint
 *          limits and, with a CollisionPort, collision-free is the goal the inner planner gets.
 *          Failures: ik_failed, ik_goal_in_collision, unsupported_constraint, invalid_request,
 *          ik_timeout, cancelled, collision_check_failed; the inner planner's own result is passed
 *          through with its code unchanged. Messages start with "ik stage:" or "plan stage:".
 *          ROS-free. Not registered as a plugin yet, see docs/backend_selection.md.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "motion_planner/core/planners/collision_port.hpp"
#include "motion_planner/core/planners/ik_port.hpp"
#include "motion_planner/core/planners/planner.hpp"

namespace motion_planner
{

struct IkThenPlanConfig
{
  std::string end_effector_link;       ///< Passed to the IkPort and CollisionPort.
  int max_ik_seeds{4};                 ///< Seeds tried besides the start state; 0 = start state only.
  double perturbation{0.3};            ///< Half-width of the uniform seed noise [rad].
  uint32_t rng_seed{1};                ///< Fixed, so a query gives the same seeds every time.
  std::vector<double> home_pose;       ///< Optional; the first extra seed.
  std::vector<double> lower_limits;    ///< Optional; with upper_limits, solutions outside are rejected.
  std::vector<double> upper_limits;
  /// Goal tolerance for pose goals; negative = none. Setting one needs `ik_honors_tolerance`,
  /// otherwise a pose goal fails with unsupported_constraint instead of being solved looser.
  double position_tolerance_m{-1.0};
  double orientation_tolerance_rad{-1.0};
  bool ik_honors_tolerance{false};     ///< True only if the IkPort acts on IkRequest's tolerance fields.
  double time_budget_s{0.0};           ///< Whole-call budget, checked between steps; 0 = none.
  std::function<bool()> cancel;        ///< Polled between steps; true stops with "cancelled".
  std::function<double()> now_s;       ///< Clock in seconds; empty = steady_clock (tests inject one).
};

class IkThenPlanPlanner : public Planner
{
public:
  /// `collision` may be null (no goal-state check).
  IkThenPlanPlanner(
    std::unique_ptr<Planner> inner, std::shared_ptr<IkPort> ik, std::shared_ptr<CollisionPort> collision,
    IkThenPlanConfig config);

  std::string name() const override;
  PlanningResult plan(const PlanningRequest & request) override;
  IkResult ik(const IkRequest & request) override;
  CollisionResult collision(const CollisionRequest & request) override;

  /// The seeds a pose goal would try from `start`, in order (first is `start` itself).
  std::vector<std::vector<double>> seedsFor(const std::vector<double> & start) const;

private:
  std::unique_ptr<Planner> inner_;
  std::shared_ptr<IkPort> ik_;
  std::shared_ptr<CollisionPort> collision_;
  IkThenPlanConfig config_;
};

}  // namespace motion_planner
