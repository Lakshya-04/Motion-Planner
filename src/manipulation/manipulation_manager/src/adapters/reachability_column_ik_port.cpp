#include "manipulation_manager/adapters/reachability_column_ik_port.hpp"

#include <algorithm>
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

core_types::Pose toCore(const reachability::Pose & p)
{
  const Eigen::Quaterniond q(p.linear());
  core_types::Pose out;
  out.x = p.translation().x();
  out.y = p.translation().y();
  out.z = p.translation().z();
  out.qx = q.x();
  out.qy = q.y();
  out.qz = q.z();
  out.qw = q.w();
  return out;
}

const char * roleOf(reachability::PoseRole r)
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

ReachabilityColumnIkPort::ReachabilityColumnIkPort(ReachabilityColumnIkConfig config, IkFn ik)
: config_(std::move(config)), ik_(std::move(ik))
{
}

ColumnIkVerdict ReachabilityColumnIkPort::check(
  const std::vector<core_types::Pose> & strokes, const std::array<double, 3> & free_side_normal,
  const std::vector<double> & seed)
{
  ColumnIkVerdict v;
  if (strokes.empty() || seed.empty() || !ik_) {
    return v;
  }
  reachability::SequenceParams sp = config_.sequence;
  sp.wall_normal = reachability::Vec3(free_side_normal[0], free_side_normal[1], free_side_normal[2]);
  std::vector<reachability::Pose> world;
  world.reserve(strokes.size());
  for (const auto & s : strokes) {
    world.push_back(toEigen(s));
  }
  const auto seq = reachability::buildColumnSequence(world, sp);

  reachability::ColumnIkChainParams cp = config_.chain;
  cp.joint_kinds.assign(seed.size(), reachability::JointKind::kRevolute);
  for (const size_t j : config_.prismatic_joints) {
    if (j < seed.size()) {cp.joint_kinds[j] = reachability::JointKind::kPrismatic;}
  }
  for (const size_t j : config_.continuous_joints) {
    if (j < seed.size()) {cp.joint_kinds[j] = reachability::JointKind::kContinuous;}
  }
  cp.initial_seed = Eigen::Map<const Eigen::VectorXd>(seed.data(), static_cast<Eigen::Index>(seed.size()));
  const size_t n = seed.size();
  const reachability::ColumnIkChain chain(
    cp, [this, n](const reachability::Pose & target, const reachability::JointVec & sd)
    -> std::optional<reachability::JointVec> {
      const std::vector<double> s(sd.data(), sd.data() + sd.size());
      const auto q = ik_(toCore(target), s);
      if (!q || q->size() != n) {
        return std::nullopt;
      }
      return Eigen::Map<const Eigen::VectorXd>(q->data(), static_cast<Eigen::Index>(q->size()));
    });
  const auto r = chain.solve(seq);

  v.solved = r.complete;
  v.continuous = r.continuous();
  v.split_count = r.split_count;
  v.branch_changes = r.branch_changes;
  v.max_continuity_ratio = r.max_continuity_ratio;
  v.lift_travel_m = r.lift_travel_m;
  if (r.failure == reachability::PoseFailure::kBranchBreak) {
    v.failure = "branch_break";
  } else if (r.failure == reachability::PoseFailure::kRefused) {
    v.failure = "refused";
  }
  if (r.failure_pose) {
    v.failure_pose = *r.failure_pose;
    if (*r.failure_pose < seq.roles.size()) {
      v.failure_role = roleOf(seq.roles[*r.failure_pose]);
    }
  }
  return v;
}

}  // namespace manipulation_manager
