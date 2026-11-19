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
  int segment{-1};   // index into the plan's `segments`, -1 if the plan has none
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
  // The plan's segments say which points belong to one stroke: an obstacle splits a column into several strokes
  // whose ends are far apart, and the manager must stroke each of them, not the whole column through the obstacle.
  std::vector<int> segment_of_pose(poses.size(), -1);
  const auto & plan_node = doc.at("final_lifted_and_expanded");
  if (plan_node.contains("segments")) {
    int seg = 0;
    for (const auto & sj : plan_node.at("segments")) {
      const size_t a = sj.value("start_index", 0U), b = sj.value("end_index", 0U);
      for (size_t i = a; i <= b && i < segment_of_pose.size(); ++i) {segment_of_pose[i] = seg;}
      ++seg;
    }
  }
  std::map<long, std::vector<RawPoint>> columns_by_x_key;
  constexpr double kXQuantum = 1e-3;
  // Half the length of the short stroke given to a one-cell gap. A free cell lies wholly outside the inflated
  // obstacle, so a stroke within half a cell of the cell centre cannot enter the keep-out clearance. 0.01 m is
  // half the 0.02 m grid resolution of the demo scenes; a coarser grid is safe, a finer one needs this lowered.
  constexpr double kPointStrokeHalfM = 0.01;
  size_t pose_index = 0;
  for (const auto & p : poses) {
    RawPoint pt;
    pt.segment = segment_of_pose[pose_index++];
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
    // z span of every stroke in the column (passes over the same stretch share one span); without segment
    // information the whole column is one span
    std::vector<std::pair<double, double>> spans;
    if (column.front().segment < 0) {
      spans.push_back({column.front().z, column.back().z});
    } else {
      std::map<int, std::pair<double, double>> by_segment;
      for (const auto & pt : column) {
        auto it = by_segment.find(pt.segment);
        if (it == by_segment.end()) {by_segment[pt.segment] = {pt.z, pt.z};} else {
          it->second.first = std::min(it->second.first, pt.z);
          it->second.second = std::max(it->second.second, pt.z);
        }
      }
      for (const auto & [seg, span] : by_segment) {
        bool seen = false;
        for (const auto & e : spans) {
          if (std::fabs(e.first - span.first) < 1e-3 && std::fabs(e.second - span.second) < 1e-3) {seen = true;}
        }
        if (!seen) {spans.push_back(span);}
      }
      std::sort(spans.begin(), spans.end());
    }
    const auto & rep = column.front();
    for (const auto & span : spans) {
      double z_min = std::max(z_min_m_, span.first);
      double z_max = std::min(z_max_m_, span.second);
      // The planner's gap strokes beside a small obstacle can be a single grid cell (first == last): a stroke of
      // zero length would be dropped and the strip next to the obstacle never wiped. Give it a short stroke
      // around the point instead, kept inside the free cell (see kPointStrokeHalfM).
      if (span.second - span.first < 1e-6 && z_max >= z_min) {
        z_min = std::max(z_min_m_, span.first - kPointStrokeHalfM);
        z_max = std::min(z_max_m_, span.first + kPointStrokeHalfM);
      }

      CoverageSegmentIntent intent;
      intent.label = "wall_column_" + std::to_string(column_index);
      intent.mode_hint = "cartesian";
      intent.requires_contact = true;
      intent.requires_retraction = true;
      intent.target_eef_speed = target_eef_speed_;

      if (z_max > z_min) {
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
  }
  return segments;
}

}  // namespace manipulation_manager
