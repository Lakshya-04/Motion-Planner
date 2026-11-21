#include "manipulation_manager/adapters/reachability_column_port.hpp"

#include <cmath>
#include <utility>

#include <Eigen/Geometry>

namespace manipulation_manager
{

namespace
{

reachability::Pose toEigen(const core_types::Pose & p)
{
  Eigen::Quaterniond q(p.qw, p.qx, p.qy, p.qz);
  if (q.norm() < 1e-12) {
    q = Eigen::Quaterniond::Identity();
  }
  q.normalize();
  reachability::Pose out = reachability::Pose::Identity();
  out.linear() = q.toRotationMatrix();
  out.translation() = reachability::Vec3(p.x, p.y, p.z);
  return out;
}

const char * reasonOf(reachability::PoseFailure f)
{
  using reachability::PoseFailure;
  switch (f) {
    case PoseFailure::kOutOfReach: return "out_of_reach";
    case PoseFailure::kLowMargin: return "low_margin";
    case PoseFailure::kRefused: return "refused";
    case PoseFailure::kSingular: return "singular";
    case PoseFailure::kJointLimit: return "joint_limit";
    case PoseFailure::kNone: break;
  }
  return "";
}

const char * poseOf(reachability::PoseRole r)
{
  using reachability::PoseRole;
  switch (r) {
    case PoseRole::kApproach: return "approach";
    case PoseRole::kContact: return "contact";
    case PoseRole::kStroke: return "stroke";
    case PoseRole::kRetract: return "retract";
  }
  return "";
}

}  // namespace

std::vector<double> liftHeights(double lift_min, double lift_max, double step)
{
  if (!(lift_max > lift_min) || !(step > 0.0)) {
    return {lift_min};
  }
  const size_t n = static_cast<size_t>(std::ceil((lift_max - lift_min) / step - 1e-9));
  std::vector<double> h;
  h.reserve(n + 1);
  for (size_t i = 0; i < n; ++i) {
    h.push_back(lift_min + static_cast<double>(i) * step);
  }
  h.push_back(lift_max);
  return h;
}

ReachabilityColumnPort::ReachabilityColumnPort(
  std::shared_ptr<const reachability::Map4D> map, ReachabilityColumnConfig config, StandProvider stand)
: map_(std::move(map)), config_(std::move(config)), stand_(std::move(stand))
{
  if (config_.lift_heights.empty()) {
    config_.lift_heights = {0.0};
  }
}

ColumnReachVerdict ReachabilityColumnPort::checkColumn(
  const ColumnReachInput & column, const Eigen::Vector2d & base_xy) const
{
  ColumnReachVerdict verdict;
  verdict.status = ColumnReachStatus::kUnreachable;
  if (!map_ || column.strokes.empty()) {
    verdict.limiting_pose = "stroke";
    verdict.limiting_reason = "out_of_reach";
    return verdict;
  }

  std::vector<reachability::Pose> strokes;
  strokes.reserve(column.strokes.size());
  for (const auto & p : column.strokes) {
    strokes.push_back(toEigen(p));
  }
  const auto seq = reachability::buildColumnSequence(strokes, config_.sequence);
  const auto feas = reachability::evaluateColumn(
    *map_, seq, base_xy, config_.lift_heights, config_.arm_mount_offset_z, config_.margin);
  const auto & best = feas.best;

  switch (feas.status) {
    case reachability::ColumnStatus::kWhole: verdict.status = ColumnReachStatus::kWhole; break;
    case reachability::ColumnStatus::kSplit: verdict.status = ColumnReachStatus::kSplit; break;
    case reachability::ColumnStatus::kUnreachable: verdict.status = ColumnReachStatus::kUnreachable; break;
  }

  // One flag per input stroke, from the stroke poses of the sequence.
  verdict.stroke_usable.assign(column.strokes.size(), false);
  for (const auto & pc : best.poses) {
    if (pc.role == reachability::PoseRole::kStroke && pc.stroke < verdict.stroke_usable.size()) {
      verdict.stroke_usable[pc.stroke] = pc.usable;
    }
  }

  verdict.min_score = best.min_score;
  if (verdict.status != ColumnReachStatus::kWhole && best.limiting && *best.limiting < best.poses.size()) {
    const auto & lim = best.poses[*best.limiting];
    verdict.limiting_pose = poseOf(lim.role);
    verdict.limiting_reason = reasonOf(lim.failure);
    if (lim.role == reachability::PoseRole::kStroke && lim.stroke < column.strokes.size()) {
      verdict.limiting_stroke = lim.stroke;
    }
  }
  return verdict;
}

std::vector<ColumnReachVerdict> ReachabilityColumnPort::check(const std::vector<ColumnReachInput> & columns)
{
  const auto base = stand_ ? stand_() : std::nullopt;
  if (!base || !map_) {
    return {};
  }
  std::vector<ColumnReachVerdict> out;
  out.reserve(columns.size());
  for (const auto & c : columns) {
    out.push_back(checkColumn(c, *base));
  }
  return out;
}

}  // namespace manipulation_manager
