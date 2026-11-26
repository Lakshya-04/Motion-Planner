#include "diff_ik_core/jacobian_quality.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace diff_ik_core
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRankTol = 1e-9;
}  // namespace

namespace
{
template<typename M>
JacobianQuality qualityOf(const M & J)
{
  JacobianQuality out;
  if (J.rows() == 0 || J.cols() == 0) {return out;}
  Eigen::JacobiSVD<M> svd(J);
  const auto & s = svd.singularValues();
  out.sigma_max = s.maxCoeff();
  if (J.rows() > J.cols()) {
    out.sigma_min = 0.0;
    out.manipulability = 0.0;
  } else {
    out.sigma_min = s.minCoeff();
    out.manipulability = s.prod();
  }
  out.condition = out.sigma_min > 0.0 ? out.sigma_max / out.sigma_min :
    std::numeric_limits<double>::infinity();
  return out;
}
}  // namespace

JacobianQuality jacobianQuality(const Eigen::MatrixXd & J) {return qualityOf(J);}

JacobianQuality jacobianQuality(const JacobianRows & J) {return qualityOf(J);}

double minSingularValue(const Eigen::MatrixXd & J)
{
  return jacobianQuality(J).sigma_min;
}

double directionalGain(const Eigen::MatrixXd & J, const Eigen::VectorXd & direction)
{
  if (J.rows() == 0 || J.cols() == 0 || direction.size() != J.rows()) {return 0.0;}
  const double n = direction.norm();
  if (!(n > 0.0)) {return 0.0;}
  const Eigen::VectorXd u = direction / n;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU);
  const Eigen::VectorXd s = svd.singularValues();
  const Eigen::VectorXd c = svd.matrixU().transpose() * u;
  const double s_ref = s.size() > 0 ? s.maxCoeff() : 0.0;
  double acc = 0.0;
  for (Eigen::Index i = 0; i < c.size(); ++i) {
    const double si = i < s.size() ? s(i) : 0.0;
    if (si <= kRankTol * std::max(1.0, s_ref)) {
      if (std::abs(c(i)) > 1e-9) {return 0.0;}   // component outside the range of J
      continue;
    }
    acc += (c(i) / si) * (c(i) / si);
  }
  return acc > 0.0 ? 1.0 / std::sqrt(acc) : 0.0;
}

bool isLimitedJoint(double lower, double upper)
{
  const double range = upper - lower;
  return range > 0.0 && range < 2.0 * kPi - 1e-9;
}

double jointLimitMargin(
  const std::vector<double> & q, const std::vector<double> & lower,
  const std::vector<double> & upper)
{
  if (q.size() != lower.size() || q.size() != upper.size()) {return -1.0;}
  double m = 0.5;
  for (size_t j = 0; j < q.size(); ++j) {
    if (!isLimitedJoint(lower[j], upper[j])) {continue;}
    const double range = upper[j] - lower[j];
    m = std::min(m, std::max(0.0, std::min(q[j] - lower[j], upper[j] - q[j])) / range);
  }
  return m;
}

double jointLimitMargin(const JointRef & q, const JointRef & lower, const JointRef & upper)
{
  if (q.size() != lower.size() || q.size() != upper.size()) {return -1.0;}
  double m = 0.5;
  for (Eigen::Index j = 0; j < q.size(); ++j) {
    if (!isLimitedJoint(lower(j), upper(j))) {continue;}
    const double range = upper(j) - lower(j);
    m = std::min(m, std::max(0.0, std::min(q(j) - lower(j), upper(j) - q(j))) / range);
  }
  return m;
}

}  // namespace diff_ik_core
