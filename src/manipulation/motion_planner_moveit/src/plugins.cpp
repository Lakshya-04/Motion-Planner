// pluginlib registration of the MoveIt backend: Cartesian planner, OMPL planner, IK, collision and
// scene. All of them share one MoveItPlanningBackend through the PluginContext ("moveit_backend"),
// created by whichever plugin is initialized first.
#include <memory>
#include <sstream>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "motion_planner/core/plugins.hpp"
#include "motion_planner_moveit/cartesian_planner.hpp"
#include "motion_planner_moveit/moveit_planning_backend.hpp"
#include "motion_planner_moveit/moveit_scene_adapter.hpp"
#include "motion_planner_moveit/ompl_planner.hpp"

namespace motion_planner
{
namespace
{

std::shared_ptr<MoveItPlanningBackend> sharedBackend(PluginContext & context)
{
  return context.getOrCreate<MoveItPlanningBackend>(
    "moveit_backend", [] {return std::make_shared<MoveItPlanningBackend>();});
}

/// Opt-in: a joint-goal free-space segment (stow, approach by joint goal) also gets the Cartesian-first attempt.
void configure(CartesianPlanner & planner, const PluginParams & params)
{
  planner.setJointGoalMode(
    paramOr(params, "cartesian_joint_goal", false), paramOr(params, "cartesian_joint_goal_tolerance", 0.05));
}

/// Free-space fast path of the OMPL planner (all params optional, off by default).
void configure(OmplPlanner & planner, const PluginParams & params)
{
  DirectJointPathConfig config;
  config.revolute_step = paramOr(params, "direct_revolute_step", config.revolute_step);
  config.prismatic_step = paramOr(params, "direct_prismatic_step", config.prismatic_step);
  planner.setDirectFirst(
    paramOr(params, "free_space_direct_first", false), config,
    paramOr(params, "direct_default_acceleration", 2.0));

  FreeSpaceOptimizeConfig optimize;
  optimize.enabled = paramOr(params, "free_space_optimize", false);
  optimize.time_budget_s = paramOr(params, "free_space_time_budget_s", optimize.time_budget_s);
  optimize.num_candidates =
    static_cast<int>(paramOr(params, "free_space_num_candidates", static_cast<double>(optimize.num_candidates)));
  optimize.objective = parseObjective(paramOr(params, "free_space_objective", std::string("length")));
  const std::string planners = paramOr(params, "free_space_planners", std::string());
  if (!planners.empty()) {
    optimize.planners.clear();
    std::stringstream stream(planners);
    for (std::string item; std::getline(stream, item, ',');) {
      if (!item.empty()) {
        optimize.planners.push_back(item);
      }
    }
  }
  optimize.shortcut.shortcut_iterations =
    static_cast<std::size_t>(paramOr(params, "free_space_shortcut_iterations", 300.0));
  optimize.shortcut.smoothing_rounds =
    static_cast<std::size_t>(paramOr(params, "free_space_smoothing_rounds", 4.0));
  optimize.cartesian_candidate = paramOr(params, "free_space_cartesian_candidate", true);
  optimize.cartesian_eef_step = paramOr(params, "free_space_cartesian_eef_step", optimize.cartesian_eef_step);
  optimize.cartesian_goal_tolerance =
    paramOr(params, "free_space_cartesian_goal_tolerance", optimize.cartesian_goal_tolerance);
  optimize.cartesian_revolute_jump =
    paramOr(params, "free_space_cartesian_revolute_jump", optimize.cartesian_revolute_jump);
  optimize.cartesian_prismatic_jump =
    paramOr(params, "free_space_cartesian_prismatic_jump", optimize.cartesian_prismatic_jump);
  planner.setFreeSpaceOptimize(optimize, paramOr(params, "direct_default_acceleration", 2.0));
}

/// Wraps a MoveIt planner (built on the shared backend in initialize()) as a PlannerPlugin.
template<typename PlannerT>
class MoveItPlannerPlugin : public PlannerPlugin
{
public:
  PluginStatus initialize(PluginContext & context, const PluginParams & params) override
  {
    planner_ = std::make_unique<PlannerT>(sharedBackend(context));
    configure(*planner_, params);
    return PluginStatus::success();
  }
  std::string name() const override {return planner_ ? planner_->name() : "uninitialized";}
  PlanningResult plan(const PlanningRequest & request) override {return planner_->plan(request);}
  IkResult ik(const IkRequest & request) override {return planner_->ik(request);}
  CollisionResult collision(const CollisionRequest & request) override
  {
    return planner_->collision(request);
  }

private:
  std::unique_ptr<PlannerT> planner_;
};

class MoveItIkPlugin : public IkPlugin
{
public:
  PluginStatus initialize(PluginContext & context, const PluginParams & params) override
  {
    backend_ = sharedBackend(context);
    if (params.count("ik_extra_seeds") != 0) {
      backend_->setIkSeeding(
        static_cast<int>(paramOr(params, "ik_extra_seeds", 0.0)),
        paramOr(params, "ik_seed_perturbation", 0.3));
    }
    return PluginStatus::success();
  }
  IkResult ik(const IkRequest & request) override {return backend_->ik(request);}

private:
  std::shared_ptr<MoveItPlanningBackend> backend_;
};

class MoveItCollisionPlugin : public CollisionPlugin
{
public:
  PluginStatus initialize(PluginContext & context, const PluginParams &) override
  {
    backend_ = sharedBackend(context);
    return PluginStatus::success();
  }
  CollisionResult collision(const CollisionRequest & request) override
  {
    return backend_->collision(request);
  }

private:
  std::shared_ptr<MoveItPlanningBackend> backend_;
};

class MoveItScenePlugin : public ScenePlugin
{
public:
  PluginStatus initialize(PluginContext &, const PluginParams & params) override
  {
    adapter_ = std::make_unique<MoveItSceneAdapter>(paramOr(params, "octomap_resolution", 0.05));
    return PluginStatus::success();
  }
  bool updateWorldGeometry(const WorldGeometryUpdate & update, std::string & error) override
  {
    return adapter_->updateWorldGeometry(update, error);
  }
  bool clearWorldGeometry(std::string & error) override {return adapter_->clearWorldGeometry(error);}

private:
  std::unique_ptr<MoveItSceneAdapter> adapter_;
};

using MoveItCartesianPlugin = MoveItPlannerPlugin<CartesianPlanner>;
using MoveItOmplPlugin = MoveItPlannerPlugin<OmplPlanner>;

}  // namespace
}  // namespace motion_planner

PLUGINLIB_EXPORT_CLASS(motion_planner::MoveItCartesianPlugin, motion_planner::PlannerPlugin)
PLUGINLIB_EXPORT_CLASS(motion_planner::MoveItOmplPlugin, motion_planner::PlannerPlugin)
PLUGINLIB_EXPORT_CLASS(motion_planner::MoveItIkPlugin, motion_planner::IkPlugin)
PLUGINLIB_EXPORT_CLASS(motion_planner::MoveItCollisionPlugin, motion_planner::CollisionPlugin)
PLUGINLIB_EXPORT_CLASS(motion_planner::MoveItScenePlugin, motion_planner::ScenePlugin)
