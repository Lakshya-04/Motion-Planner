#include "manipulation_manager/ros/coverage/coverage_output_json_source.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "core_types/pose.hpp"

namespace manipulation_manager
{

namespace
{

struct RawPoint
{
  double x, y, z, qx, qy, qz, qw;
};

}  // namespace

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
  const auto & poses = doc.at("final_lifted_and_expanded").at("path").at("poses");

  // Group raw points by their position along the wall's horizontal tangent (direction x up; x for
  // the +y wall)
  const auto & wd = wall_dir_.d;
  const double tx = wd[1], ty = -wd[0];
  // Group raw points by x (rounded to kill float noise between two points on the same
  // column) -- the boustrophedon path visits each column as a top/bottom pair, sorted here
  // left-to-right (ascending x) so a caller sweeping the wall naturally starts at the left.
  std::map<long, std::vector<RawPoint>> columns_by_x_key;
  constexpr double kXQuantum = 1e-3;
  for (const auto & p : poses) {
    RawPoint pt;
    pt.x = p.at("position").at("x").get<double>();
    pt.y = p.at("position").at("y").get<double>();
    pt.z = p.at("position").at("z").get<double>() + z_offset_;
    pt.qx = p.at("orientation").at("x").get<double>();
    pt.qy = p.at("orientation").at("y").get<double>();
    pt.qz = p.at("orientation").at("z").get<double>();
    pt.qw = p.at("orientation").at("w").get<double>();
    const long key = static_cast<long>(std::round((pt.x * tx + pt.y * ty) / kXQuantum));
    columns_by_x_key[key].push_back(pt);
  }

  std::vector<CoverageSegmentIntent> segments;
  segments.reserve(columns_by_x_key.size());
  size_t column_index = 0;
  for (auto & [key, column] : columns_by_x_key) {
    std::sort(column.begin(), column.end(), [](const RawPoint & a, const RawPoint & b) {
        return a.z < b.z;
      });
    const double z_min = std::max(z_min_m_, column.front().z);
    const double z_max = std::min(z_max_m_, column.back().z);

    CoverageSegmentIntent intent;
    intent.label = "wall_column_" + std::to_string(column_index);
    intent.mode_hint = "cartesian";
    intent.requires_contact = true;
    intent.requires_retraction = true;
    intent.target_eef_speed = target_eef_speed_;

    if (z_max > z_min) {
      const auto & rep = column.front();
      for (double z = z_min; z <= z_max + 1e-9; z += z_step_m_) {
        core_types::Pose pose;
        pose.x = rep.x;
        pose.y = rep.y;
        pose.z = z;
        pose.qx = rep.qx;
        pose.qy = rep.qy;
        pose.qz = rep.qz;
        pose.qw = rep.qw;
        intent.poses.push_back(pose);
      }
    }

    if (!intent.poses.empty()) {
      segments.push_back(std::move(intent));
    }
    ++column_index;
  }
  return segments;
}

}  // namespace manipulation_manager
