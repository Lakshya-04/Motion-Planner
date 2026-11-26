#include "motion_planner/core/planners/ik_then_plan_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <utility>

namespace motion_planner
{

namespace
{

PlanningResult failure(const std::string & code, const std::string & message, FailureStage stage)
{
  PlanningResult r;
  r.success = false;
  r.status = "failed";
  r.failure_reason_code = code;
  r.failure_stage = stage;
  r.message = "ik stage: " + message;
  return r;
}

bool finite(const std::vector<double> & v)
{
  return std::all_of(v.begin(), v.end(), [](double x) {return std::isfinite(x);});
}

}  // namespace

IkThenPlanPlanner::IkThenPlanPlanner(
  std::unique_ptr<Planner> inner, std::shared_ptr<IkPort> ik, std::shared_ptr<CollisionPort> collision,
  IkThenPlanConfig config)
: inner_(std::move(inner)), ik_(std::move(ik)), collision_(std::move(collision)), config_(std::move(config))
{
  config_.perturbation = std::isfinite(config_.perturbation) ? std::fabs(config_.perturbation) : 0.0;
  config_.max_ik_seeds = std::max(0, config_.max_ik_seeds);
}

std::string IkThenPlanPlanner::name() const {return "ik_then_" + inner_->name();}

IkResult IkThenPlanPlanner::ik(const IkRequest & request) {return ik_->ik(request);}

CollisionResult IkThenPlanPlanner::collision(const CollisionRequest & request)
{
  return collision_ ? collision_->collision(request) : inner_->collision(request);
}

std::vector<std::vector<double>> IkThenPlanPlanner::seedsFor(const std::vector<double> & start) const
{
  std::vector<std::vector<double>> seeds{start};
  std::mt19937 rng(config_.rng_seed);
  std::uniform_real_distribution<double> noise(-config_.perturbation, config_.perturbation);
  const bool have_limits = !start.empty() && config_.lower_limits.size() >= start.size() &&
    config_.upper_limits.size() >= start.size();
  bool home_used = config_.home_pose.empty();
  while (static_cast<int>(seeds.size()) <= config_.max_ik_seeds) {
    if (!home_used) {
      home_used = true;
      seeds.push_back(config_.home_pose);
      continue;
    }
    std::vector<double> s = start.empty() ? config_.home_pose : start;
    for (size_t i = 0; i < s.size(); ++i) {
      double q = s[i] + noise(rng);
      if (have_limits) {q = std::min(std::max(q, config_.lower_limits[i]), config_.upper_limits[i]);}
      s[i] = q;
    }
    seeds.push_back(std::move(s));
  }
  return seeds;
}

PlanningResult IkThenPlanPlanner::plan(const PlanningRequest & request)
{
  if (!request.target_joint_state.empty() || request.target_poses.empty()) {
    return inner_->plan(request);
  }

  const auto t0 = std::chrono::steady_clock::now();
  const auto now = [this, t0]() {
      if (config_.now_s) {return config_.now_s();}
      return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    };
  const double started = now();
  const auto elapsed_ms = [&]() {return static_cast<uint64_t>(std::max(0.0, now() - started) * 1000.0);};
  const auto stop = [&](PlanningResult & out) {
      if (config_.cancel && config_.cancel()) {
        out = failure("cancelled", "cancelled before a goal was found", FailureStage::kPrecheck);
        return true;
      }
      if (config_.time_budget_s > 0.0 && now() - started >= config_.time_budget_s) {
        out = failure("ik_timeout", "time budget used up before a goal was found", FailureStage::kPrecheck);
        return true;
      }
      return false;
    };

  const bool tolerance_set = config_.position_tolerance_m >= 0.0 || config_.orientation_tolerance_rad >= 0.0;
  if (tolerance_set && !config_.ik_honors_tolerance) {
    return failure(
      "unsupported_constraint",
      "a pose tolerance is configured but the IK port does not take one; not solving it looser",
      FailureStage::kPrecheck);
  }
  if (request.target_poses.size() != 1) {
    return failure(
      "invalid_request", "a joint-goal planner takes one pose goal, got " +
      std::to_string(request.target_poses.size()), FailureStage::kPrecheck);
  }
  const auto & p = request.target_poses.front();
  const std::vector<double> pose{p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw};
  const double qnorm = std::sqrt(p.qx * p.qx + p.qy * p.qy + p.qz * p.qz + p.qw * p.qw);
  if (!finite(pose) || qnorm < 1e-6) {
    return failure("invalid_request", "target pose is not finite or has a zero quaternion", FailureStage::kPrecheck);
  }

  const size_t n = request.start_joint_state.size();
  const bool have_limits = n > 0 && config_.lower_limits.size() >= n && config_.upper_limits.size() >= n;
  auto in_limits = [&](const std::vector<double> & q) {
      if (n > 0 && q.size() != n) {return false;}
      if (!finite(q)) {return false;}
      if (!have_limits) {return true;}
      for (size_t i = 0; i < n; ++i) {
        if (q[i] < config_.lower_limits[i] - 1e-9 || q[i] > config_.upper_limits[i] + 1e-9) {return false;}
      }
      return true;
    };

  std::string first_ik_error;
  bool any_ik_solution = false;
  bool any_in_collision = false;
  std::vector<double> goal;
  for (const auto & seed : seedsFor(request.start_joint_state)) {
    PlanningResult stopped;
    if (stop(stopped)) {
      stopped.planning_time_ms = elapsed_ms();
      return stopped;
    }
    IkRequest ik_req;
    ik_req.frame_id = request.frame_id;
    ik_req.planning_group = request.planning_group;
    ik_req.end_effector_link = config_.end_effector_link;
    ik_req.target_pose = pose;
    ik_req.joint_seed = seed;
    ik_req.position_tolerance = config_.position_tolerance_m;
    ik_req.orientation_tolerance = config_.orientation_tolerance_rad;
    const IkResult sol = ik_->ik(ik_req);
    if (!sol.success || sol.joint_solution.empty()) {
      if (first_ik_error.empty()) {first_ik_error = sol.failure_reason_code + ": " + sol.message;}
      continue;
    }
    any_ik_solution = true;
    if (!in_limits(sol.joint_solution)) {continue;}
    if (collision_) {
      CollisionRequest c;
      c.frame_id = request.frame_id;
      c.planning_group = request.planning_group;
      c.end_effector_link = config_.end_effector_link;
      c.joint_values = sol.joint_solution;
      const CollisionResult cr = collision_->collision(c);
      if (!cr.success) {
        auto r = failure(
          "collision_check_failed", "could not check the IK goal: " + cr.message, FailureStage::kPrecheck);
        r.planning_time_ms = elapsed_ms();
        return r;
      }
      if (!cr.collision_free) {
        any_in_collision = true;
        continue;
      }
    }
    goal = sol.joint_solution;
    break;
  }

  if (goal.empty()) {
    PlanningResult r;
    if (any_in_collision) {
      r = failure("ik_goal_in_collision", "every IK solution within limits is in collision", FailureStage::kPrecheck);
    } else if (any_ik_solution) {
      r = failure("ik_failed", "every IK solution is outside the joint limits", FailureStage::kPrecheck);
    } else {
      r = failure("ik_failed", "no IK solution (" + first_ik_error + ")", FailureStage::kPrecheck);
    }
    r.planning_time_ms = elapsed_ms();
    return r;
  }

  PlanningResult stopped;
  if (stop(stopped)) {
    stopped.planning_time_ms = elapsed_ms();
    return stopped;
  }

  PlanningRequest joint_request = request;
  joint_request.target_poses.clear();
  joint_request.target_joint_state = goal;
  const uint64_t ik_ms = elapsed_ms();
  PlanningResult result = inner_->plan(joint_request);
  result.planning_time_ms += ik_ms;
  if (!result.success) {result.message = "plan stage: " + result.message;}
  return result;
}

}  // namespace motion_planner
