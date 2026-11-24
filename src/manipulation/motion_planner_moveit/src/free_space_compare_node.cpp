// Free-space transition comparison, MoveIt side (docs/free_space_comparison.md).
//
// Runs where move_group and the planning scene (the wall, the stands) are up, e.g. next to a paused sim. For every
// transition of a JSON file it plans with each requested mode and writes one JSON line per (transition, mode):
// success, route, planning time, trajectory duration after the time parameterisation, the joint positions of the
// plan, the TCP path length. Joint path length and all ratios are computed offline by
// docker/tools/free_space_compare.py (host-testable), so this node stays thin.
//
//   ros2 run motion_planner_moveit free_space_compare_node --ros-args \
//     -p transitions_file:=/abs/transitions.json -p out_file:=/abs/moveit_results.jsonl \
//     [-p modes:=ompl_default,ompl_optimal,ompl_optimal_cart,direct,cartesian,cartesian_joint] \
//     [-p planning_group:=manipulator] [-p time_budget_s:=10.0] [-p num_candidates:=5] [-p repeats:=1]
//
//   fk mode (TCP length of trajectories made elsewhere, e.g. by cuRobo):
//     -p fk_in:=/abs/curobo_results.jsonl -p out_file:=/abs/curobo_results_fk.jsonl
//
// Modes: ompl_default (the unchanged OMPL call), ompl_optimal (optimal mode, straight joint line + optimal planners,
// no Cartesian candidate), ompl_optimal_cart (the same plus the Cartesian line as a candidate), direct (the straight
// joint line, OMPL when it is blocked), straight_joint (baseline only: the unchecked straight joint line), cartesian (Cartesian line to the goal POSE, continuity IK, any branch),
// cartesian_joint (Cartesian line to the goal's tool pose, accepted only on the goal's own joint branch).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"
#include "rclcpp/rclcpp.hpp"

#include "motion_planner/core/path_quality.hpp"
#include "motion_planner_moveit/cartesian_planner.hpp"
#include "motion_planner_moveit/moveit_planning_backend.hpp"
#include "motion_planner_moveit/moveit_time_parameterizer.hpp"
#include "motion_planner_moveit/ompl_planner.hpp"

using nlohmann::json;
using namespace motion_planner;

namespace
{

std::vector<std::string> split(const std::string & s)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  for (std::string item; std::getline(ss, item, ',');) {
    if (!item.empty()) {
      out.push_back(item);
    }
  }
  return out;
}

struct Context
{
  std::shared_ptr<MoveItPlanningBackend> backend;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group;
  const moveit::core::JointModelGroup * jmg{nullptr};
  std::string tip;
};

core_types::Pose toCorePose(const Eigen::Isometry3d & t)
{
  const Eigen::Quaterniond q(t.linear());
  core_types::Pose p;
  p.x = t.translation().x();
  p.y = t.translation().y();
  p.z = t.translation().z();
  p.qx = q.x();
  p.qy = q.y();
  p.qz = q.z();
  p.qw = q.w();
  return p;
}

Eigen::Isometry3d fk(const Context & c, const std::vector<double> & q)
{
  moveit::core::RobotState st(c.group->getRobotModel());
  st.setToDefaultValues();
  st.setJointGroupPositions(c.jmg, q);
  st.update();
  return st.getGlobalLinkTransform(c.tip);
}

double tcpLength(const Context & c, const std::vector<std::vector<double>> & path)
{
  double len = 0.0;
  Eigen::Vector3d prev;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const Eigen::Vector3d p = fk(c, path[i]).translation();
    if (i > 0) {
      len += (p - prev).norm();
    }
    prev = p;
  }
  return len;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("free_space_compare");
  const auto get_s = [&](const std::string & n, const std::string & d) {
      node->declare_parameter<std::string>(n, d);
      return node->get_parameter(n).as_string();
    };
  const auto get_d = [&](const std::string & n, double d) {
      node->declare_parameter<double>(n, d);
      return node->get_parameter(n).as_double();
    };
  const std::string transitions_file = get_s("transitions_file", "");
  const std::string fk_in = get_s("fk_in", "");
  const std::string out_file = get_s("out_file", "");
  const std::string group_name = get_s("planning_group", "manipulator");
  const auto modes = split(get_s(
      "modes", "straight_joint,ompl_default,ompl_optimal,ompl_optimal_cart,direct,cartesian,cartesian_joint"));
  const double budget = get_d("time_budget_s", 10.0);
  const int candidates = static_cast<int>(get_d("num_candidates", 5.0));
  const int repeats = std::max(1, static_cast<int>(get_d("repeats", 1.0)));
  if (out_file.empty() || (transitions_file.empty() && fk_in.empty())) {
    std::fprintf(stderr, "need out_file and transitions_file (or fk_in)\n");
    return 2;
  }

  Context ctx;
  ctx.backend = std::make_shared<MoveItPlanningBackend>();
  std::string error;
  if (!ctx.backend->ensureMoveGroup(group_name, error)) {
    std::fprintf(stderr, "move group '%s' not available: %s\n", group_name.c_str(), error.c_str());
    return 2;
  }
  ctx.group = ctx.backend->moveGroup();
  ctx.jmg = ctx.group->getRobotModel()->getJointModelGroup(group_name);
  ctx.tip = ctx.group->getEndEffectorLink();
  std::ofstream out(out_file);

  // Header: joint names and ranges, so the offline weights (1 / range) match the planner's.
  json header = {{"type", "header"}, {"group", group_name}, {"tip", ctx.tip},
    {"joint_names", ctx.jmg->getActiveJointModelNames()}};
  std::vector<double> ranges;
  for (const auto * j : ctx.jmg->getActiveJointModels()) {
    const auto & b = j->getVariableBounds(j->getName());
    ranges.push_back(b.position_bounded_ ? b.max_position_ - b.min_position_ : 0.0);
  }
  header["ranges"] = ranges;
  out << header.dump() << "\n";

  if (!fk_in.empty()) {
    std::ifstream in(fk_in);
    for (std::string line; std::getline(in, line);) {
      if (line.empty()) {
        continue;
      }
      json rec = json::parse(line, nullptr, false);
      if (rec.is_discarded() || rec.value("type", "") != "run") {
        continue;
      }
      if (rec.value("success", false) && rec.contains("positions")) {
        rec["tcp_length"] = tcpLength(ctx, rec["positions"].get<std::vector<std::vector<double>>>());
      }
      out << rec.dump() << "\n";
    }
    rclcpp::shutdown();
    return 0;
  }

  // One planner per mode, all on the one shared backend (the same ground truth as in the manager).
  const auto make_ompl = [&](bool optimize, bool cartesian, bool direct) {
      auto p = std::make_unique<OmplPlanner>(ctx.backend);
      DirectJointPathConfig dc;
      p->setDirectFirst(direct, dc, 2.0);
      FreeSpaceOptimizeConfig oc;
      oc.enabled = optimize;
      oc.time_budget_s = budget;
      oc.num_candidates = candidates;
      oc.cartesian_candidate = cartesian;
      p->setFreeSpaceOptimize(oc, 2.0);
      return p;
    };
  auto ompl_default = make_ompl(false, false, false);
  auto ompl_optimal = make_ompl(true, false, false);
  auto ompl_optimal_cart = make_ompl(true, true, false);
  auto ompl_direct = make_ompl(false, false, true);
  CartesianPlanner cart(ctx.backend);
  CartesianPlanner cart_joint(ctx.backend);
  cart_joint.setJointGoalMode(true, 0.05);

  std::ifstream tin(transitions_file);
  const json doc = json::parse(tin, nullptr, false);
  if (doc.is_discarded() || !doc.contains("transitions")) {
    std::fprintf(stderr, "cannot parse %s\n", transitions_file.c_str());
    return 2;
  }
  MoveItTimeParameterizer retimer(ctx.group->getRobotModel(), group_name);
  const auto limits = retimer.modelLimits(2.0, 0.0);

  for (const auto & t : doc["transitions"]) {
    const std::string name = t.value("name", "t");
    const std::vector<double> start = t["start"].get<std::vector<double>>();
    std::vector<double> goal_q;
    if (t.contains("goal_q")) {
      goal_q = t["goal_q"].get<std::vector<double>>();
    }
    core_types::Pose goal_pose;
    bool have_pose = false;
    if (t.contains("goal_pose")) {
      const auto & g = t["goal_pose"];
      goal_pose.x = g.value("x", 0.0);
      goal_pose.y = g.value("y", 0.0);
      goal_pose.z = g.value("z", 0.0);
      goal_pose.qx = g.value("qx", 0.0);
      goal_pose.qy = g.value("qy", 0.0);
      goal_pose.qz = g.value("qz", 0.0);
      goal_pose.qw = g.value("qw", 1.0);
      have_pose = true;
    }
    if (goal_q.empty() && have_pose) {  // IK from the start state, the seed the planners use too
      moveit::core::RobotState st(ctx.group->getRobotModel());
      st.setToDefaultValues();
      st.setJointGroupPositions(ctx.jmg, start);
      geometry_msgs::msg::Pose gp;
      gp.position.x = goal_pose.x;
      gp.position.y = goal_pose.y;
      gp.position.z = goal_pose.z;
      gp.orientation.x = goal_pose.qx;
      gp.orientation.y = goal_pose.qy;
      gp.orientation.z = goal_pose.qz;
      gp.orientation.w = goal_pose.qw;
      if (st.setFromIK(ctx.jmg, gp, 0.2)) {
        st.copyJointGroupPositions(ctx.jmg, goal_q);
      }
    }
    if (goal_q.empty()) {
      json rec = {{"type", "transition_skipped"}, {"transition", name}, {"reason", "no goal joint state (IK failed)"}};
      out << rec.dump() << "\n";
      continue;
    }
    if (!have_pose) {
      goal_pose = toCorePose(fk(ctx, goal_q));
    }
    json resolved = {{"type", "transition"}, {"transition", name}, {"class", t.value("class", "")},
      {"start", start}, {"goal_q", goal_q},
      {"goal_pose", {{"x", goal_pose.x}, {"y", goal_pose.y}, {"z", goal_pose.z}, {"qx", goal_pose.qx},
        {"qy", goal_pose.qy}, {"qz", goal_pose.qz}, {"qw", goal_pose.qw}}},
      {"tcp_start", {fk(ctx, start).translation().x(), fk(ctx, start).translation().y(),
        fk(ctx, start).translation().z()}},
      {"tcp_goal", {goal_pose.x, goal_pose.y, goal_pose.z}}};
    out << resolved.dump() << "\n";

    for (const auto & mode : modes) {
      for (int rep = 0; rep < repeats; ++rep) {
        if (mode == "straight_joint") {
          // Baseline, not a plan: the straight joint line, NOT collision checked (checked:false), 0.02 rad/m steps.
          core_types::Trajectory line;
          line.joint_names = ctx.jmg->getActiveJointModelNames();
          double span = 0.0;
          for (std::size_t j = 0; j < start.size(); ++j) {
            span = std::max(span, std::fabs(goal_q[j] - start[j]));
          }
          const int steps = std::max(2, static_cast<int>(std::ceil(span / 0.02)));
          for (int i = 0; i <= steps; ++i) {
            core_types::TrajectoryPoint pt;
            for (std::size_t j = 0; j < start.size(); ++j) {
              pt.positions.push_back(start[j] + (goal_q[j] - start[j]) * i / steps);
            }
            line.points.push_back(std::move(pt));
          }
          std::string msg;
          retimer.retime(line, SegmentRole::kTransition, limits, 0.0, msg);
          std::vector<std::vector<double>> pos;
          for (const auto & pt : line.points) {
            pos.push_back(pt.positions);
          }
          json rec = {{"type", "run"}, {"transition", name}, {"mode", mode}, {"rep", rep}, {"success", true},
            {"checked", false}, {"planning_s", 0.0}, {"route", "baseline"}, {"reason", "none"},
            {"duration_s", line.points.back().time_from_start_sec}, {"points", pos.size()}, {"positions", pos},
            {"tcp_length", tcpLength(ctx, pos)}};
          out << rec.dump() << "\n";
          continue;
        }
        PlanningRequest req;
        req.planning_group = group_name;
        req.start_joint_state = start;
        req.min_path_fraction = 0.99;
        req.mode = (mode.rfind("cartesian", 0) == 0) ? "cartesian" : "fsp";
        Planner * planner = nullptr;
        OmplPlanner * ompl = nullptr;
        if (mode == "ompl_default") {ompl = ompl_default.get();}
        else if (mode == "ompl_optimal") {ompl = ompl_optimal.get();}
        else if (mode == "ompl_optimal_cart") {ompl = ompl_optimal_cart.get();}
        else if (mode == "direct") {ompl = ompl_direct.get();}
        if (ompl != nullptr) {
          planner = ompl;
          req.target_joint_state = goal_q;
        } else if (mode == "cartesian") {
          planner = &cart;
          req.target_poses = {goal_pose};
        } else if (mode == "cartesian_joint") {
          planner = &cart_joint;
          req.target_joint_state = goal_q;
        } else {
          continue;
        }
        const auto t0 = std::chrono::steady_clock::now();
        PlanningResult r = planner->plan(req);
        const double wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        json rec = {{"type", "run"}, {"transition", name}, {"mode", mode}, {"rep", rep}, {"success", r.success},
          {"planning_s", wall_s}, {"reason", r.failure_reason_code}, {"path_fraction", r.path_fraction},
          {"route", ompl != nullptr ? ompl->lastRoute() : mode}};
        if (r.success && r.trajectory.points.size() >= 2) {
          if (!(r.trajectory.points.back().time_from_start_sec > 0.0)) {  // not timed yet: the same TOTG the manager uses
            std::string msg;
            retimer.retime(r.trajectory, SegmentRole::kTransition, limits, 0.0, msg);
          }
          std::vector<std::vector<double>> pos;
          for (const auto & p : r.trajectory.points) {
            pos.push_back(p.positions);
          }
          rec["duration_s"] = r.trajectory.points.back().time_from_start_sec;
          rec["points"] = pos.size();
          rec["positions"] = pos;
          rec["tcp_length"] = tcpLength(ctx, pos);
        }
        out << rec.dump() << "\n";
        out.flush();
      }
    }
  }
  rclcpp::shutdown();
  return 0;
}
