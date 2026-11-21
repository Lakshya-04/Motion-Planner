#include "diff_ik_core/null_space_costs.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "diff_ik_core/jacobian_quality.hpp"

namespace diff_ik_core
{

namespace
{
double weightAt(const Eigen::VectorXd & w, Eigen::Index i)
{
  return i < w.size() ? w(i) : 1.0;
}
}  // namespace

JointCenteringCost::JointCenteringCost(Eigen::VectorXd weights)
: weights_(std::move(weights)) {}

Eigen::VectorXd JointCenteringCost::ascent(
  const Eigen::VectorXd & q, const Eigen::VectorXd & lower, const Eigen::VectorXd & upper) const
{
  Eigen::VectorXd g = Eigen::VectorXd::Zero(q.size());
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    if (i >= lower.size() || i >= upper.size() || !isLimitedJoint(lower(i), upper(i))) {continue;}
    const double range = upper(i) - lower(i);
    g(i) = -weightAt(weights_, i) * (q(i) - 0.5 * (lower(i) + upper(i))) / (range * range);
  }
  return g;
}

LimitAvoidanceCost::LimitAvoidanceCost(double zone, Eigen::VectorXd weights)
: zone_(std::clamp(zone, 1e-6, 0.5)), weights_(std::move(weights)) {}

Eigen::VectorXd LimitAvoidanceCost::ascent(
  const Eigen::VectorXd & q, const Eigen::VectorXd & lower, const Eigen::VectorXd & upper) const
{
  Eigen::VectorXd g = Eigen::VectorXd::Zero(q.size());
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    if (i >= lower.size() || i >= upper.size() || !isLimitedJoint(lower(i), upper(i))) {continue;}
    const double range = upper(i) - lower(i);
    const double x = (q(i) - lower(i)) / range;   // 0 at the lower limit, 1 at the upper
    double push = 0.0;
    if (x < zone_) {
      push = (zone_ - std::max(x, 0.0)) / zone_ + (x < 0.0 ? -x / zone_ : 0.0);
    } else if (x > 1.0 - zone_) {
      push = -((std::min(x, 1.0) - (1.0 - zone_)) / zone_ + (x > 1.0 ? (x - 1.0) / zone_ : 0.0));
    }
    g(i) = weightAt(weights_, i) * push / range;
  }
  return g;
}

JointEndBiasCost::JointEndBiasCost(int joint_index, double onset)
: index_(joint_index), onset_(std::clamp(onset, 0.0, 0.999)) {}

Eigen::VectorXd JointEndBiasCost::ascent(
  const Eigen::VectorXd & q, const Eigen::VectorXd & lower, const Eigen::VectorXd & upper) const
{
  Eigen::VectorXd g = Eigen::VectorXd::Zero(q.size());
  if (index_ < 0 || index_ >= q.size() || index_ >= lower.size() || index_ >= upper.size()) {
    return g;
  }
  const double range = upper(index_) - lower(index_);
  if (!(range > 0.0)) {return g;}
  const double half = 0.5 * range;
  const double d = (q(index_) - 0.5 * (lower(index_) + upper(index_))) / half;   // -1 .. 1
  const double excess = std::max(0.0, std::abs(d) - onset_) / (1.0 - onset_);
  g(index_) = -(d < 0.0 ? -1.0 : 1.0) * excess * excess / half;
  return g;
}

Eigen::VectorXd combinedAscent(
  const std::vector<NullSpaceTerm> & terms, const Eigen::VectorXd & q,
  const Eigen::VectorXd & lower, const Eigen::VectorXd & upper)
{
  Eigen::VectorXd sum = Eigen::VectorXd::Zero(q.size());
  for (const auto & t : terms) {
    if (!t.cost) {continue;}
    const Eigen::VectorXd a = t.cost->ascent(q, lower, upper);
    if (a.size() == q.size()) {sum += t.gain * a;}
  }
  return sum;
}

}  // namespace diff_ik_core
