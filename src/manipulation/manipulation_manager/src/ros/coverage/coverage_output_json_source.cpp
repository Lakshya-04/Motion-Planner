#include "manipulation_manager/ros/coverage/coverage_output_json_source.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "core_types/pose.hpp"
#include "manipulation_manager/core/coverage_plan_intents.hpp"

namespace manipulation_manager
{

CoverageOutputJsonSource::CoverageOutputJsonSource(
  std::string json_path, std::string frame_id, double z_offset, double z_step_m,
  double z_min_m, double z_max_m, double target_eef_speed, WallDirection wall_direction)
: json_path_(std::move(json_path)),
  frame_id_(std::move(frame_id)),
  z_offset_(z_offset),
  z_step_m_(z_step_m),
  z_min_m_(z_min_m),
  z_max_m_(z_max_m),
  target_eef_speed_(target_eef_speed),
  wall_dir_(wall_direction)
{
}

std::string CoverageOutputJsonSource::getFrameId()
{
  return frame_id_;
}

std::vector<CoverageSegmentIntent> CoverageOutputJsonSource::getCoveragePlan()
{
  std::ifstream file(json_path_);
  if (!file.is_open()) {
    throw std::runtime_error("CoverageOutputJsonSource: could not open " + json_path_);
  }
  nlohmann::json doc;
  file >> doc;
  const auto & plan_node = doc.at("final_lifted_and_expanded");
  RawCoveragePlan plan;
  for (const auto & p : plan_node.at("path").at("poses")) {
    core_types::Pose pose;
    pose.x = p.at("position").at("x").get<double>();
    pose.y = p.at("position").at("y").get<double>();
    pose.z = p.at("position").at("z").get<double>();
    pose.qx = p.at("orientation").at("x").get<double>();
    pose.qy = p.at("orientation").at("y").get<double>();
    pose.qz = p.at("orientation").at("z").get<double>();
    pose.qw = p.at("orientation").at("w").get<double>();
    plan.poses.push_back(pose);
  }
  if (plan_node.contains("segments")) {
    for (const auto & sj : plan_node.at("segments")) {
      plan.segments.push_back({sj.value("start_index", 0U), sj.value("end_index", 0U)});
    }
  }
  CoverageIntentOptions opt;
  opt.z_offset = z_offset_;
  opt.z_step_m = z_step_m_;
  opt.z_min_m = z_min_m_;
  opt.z_max_m = z_max_m_;
  opt.target_eef_speed = target_eef_speed_;
  opt.wall_dir = wall_dir_;
  opt.follow_plan_direction = follow_plan_direction_;
  return buildCoverageIntents(plan, opt);
}

}  // namespace manipulation_manager
