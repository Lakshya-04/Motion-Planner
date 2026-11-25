/**
 * @file null_space_costs.hpp
 * @brief Pluggable secondary objectives for the differential-IK null space.
 * @details A cost supplies the ascent direction of a scalar H(q) to maximise (the negative gradient
 *          of a penalty), in joint units. The IK step projects it, scaled by a gain, with the TRUE
 *          (undamped) projector of the task Jacobian: components along directions whose singular
 *          value is above DiffIkParams::rank_tolerance are removed, so the null-space part moves
 *          the tool by at most rank_tolerance times its size, also while the task damping is
 *          active. Directions below the tolerance are rank-deficient (free by definition) and keep
 *          their share. Joints whose range is not a real limit (not positive, or 2*pi or more;
 *          continuous joints are passed in with an empty range) are skipped by the limit-based
 *          costs. The result is a fixed-capacity vector so the call does not allocate.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <Eigen/Dense>

#include <memory>
#include <vector>

namespace diff_ik_core
{

constexpr int kMaxJoints = 8;   ///< capacity of the allocation-free vectors
/// Joint-sized vector with inline storage for up to kMaxJoints entries (no heap).
using JointVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxJoints, 1>;
using JointRef = Eigen::Ref<const Eigen::VectorXd>;   ///< read-only view of any joint vector

class NullSpaceCost
{
public:
  virtual ~NullSpaceCost() = default;
  /// Ascent direction of H at q for joints with position limits [lower, upper]; same size as q.
  virtual JointVector ascent(
    const JointRef & q, const JointRef & lower, const JointRef & upper) const = 0;
};

/// H = -1/2 sum_i w_i ((q_i - mid_i) / range_i)^2: pulls limited joints towards mid-range.
class JointCenteringCost : public NullSpaceCost
{
public:
  explicit JointCenteringCost(Eigen::VectorXd weights = Eigen::VectorXd());
  JointVector ascent(
    const JointRef & q, const JointRef & lower, const JointRef & upper) const override;
private:
  Eigen::VectorXd weights_;   ///< empty = all 1
};

/// Zero in the middle of the range, pushing back linearly inside the outer `zone` share of the
/// range at each end (zone in (0, 0.5]); weighted per joint.
class LimitAvoidanceCost : public NullSpaceCost
{
public:
  explicit LimitAvoidanceCost(double zone = 0.2, Eigen::VectorXd weights = Eigen::VectorXd());
  JointVector ascent(
    const JointRef & q, const JointRef & lower, const JointRef & upper) const override;
private:
  double zone_;
  Eigen::VectorXd weights_;
};

/// Keeps one designated (prismatic) joint, e.g. a lift, away from its ends: zero inside the
/// central `onset` share of its half-range, quadratic push-back outside it.
class JointEndBiasCost : public NullSpaceCost
{
public:
  JointEndBiasCost(int joint_index, double onset = 0.5);
  JointVector ascent(
    const JointRef & q, const JointRef & lower, const JointRef & upper) const override;
private:
  int index_;
  double onset_;
};

struct NullSpaceTerm
{
  std::shared_ptr<const NullSpaceCost> cost;
  double gain = 1.0;
};

/// Sum of gain * ascent over the terms (zeros of size q.size() when empty; q.size() <= kMaxJoints).
JointVector combinedAscent(
  const std::vector<NullSpaceTerm> & terms, const JointRef & q,
  const JointRef & lower, const JointRef & upper);

}  // namespace diff_ik_core
