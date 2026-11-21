#include "diff_ik_core/diff_ik.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "diff_ik_core/jacobian_quality.hpp"

namespace diff_ik_core
{

namespace
{
constexpr double kRankTol = 1e-9;

bool hasLimit(const Eigen::VectorXd & v, Eigen::Index i)
{
  return i < v.size() && std::isfinite(v(i)) && v(i) > 0.0;
}

double limitOr(const Eigen::VectorXd & v, Eigen::Index i)
{
  return hasLimit(v, i) ? v(i) : std::numeric_limits<double>::infinity();
}
}  // namespace

DiffIkResult computeStep(
  const Eigen::MatrixXd & J, const Vector6d & twist, const AxisMask & mask,
  const Eigen::VectorXd & q, const DiffIkLimits & limits, const DiffIkParams & params)
{
  DiffIkResult res;
  const Eigen::Index n = J.cols();
  res.qd = Eigen::VectorXd::Zero(n);
  res.saturation.assign(static_cast<size_t>(n), JointSaturation::None);
  if (J.rows() != 6 || q.size() != n || limits.q_min.size() != n || limits.q_max.size() != n ||
    (params.joint_weights.size() != 0 && params.joint_weights.size() != n))
  {
    return res;
  }
  for (Eigen::Index i = 0; i < params.joint_weights.size(); ++i) {
    if (!(params.joint_weights(i) > 0.0)) {return res;}
  }

  // Row selection for the masked task.
  std::vector<int> rows;
  for (int a = 0; a < 6; ++a) {
    if (mask[static_cast<size_t>(a)]) {rows.push_back(a);}
  }
  const Eigen::Index m = static_cast<Eigen::Index>(rows.size());
  Eigen::MatrixXd Js(m, n);
  Eigen::VectorXd vs(m);
  for (Eigen::Index r = 0; r < m; ++r) {
    Js.row(r) = J.row(rows[static_cast<size_t>(r)]);
    vs(r) = twist(rows[static_cast<size_t>(r)]);
  }

  // Weighted coordinates: Jw = Js W^-1/2, qd = W^-1/2 qd_w.
  Eigen::VectorXd winv = Eigen::VectorXd::Ones(n);
  for (Eigen::Index i = 0; i < params.joint_weights.size(); ++i) {
    winv(i) = 1.0 / std::sqrt(params.joint_weights(i));
  }
  const Eigen::MatrixXd Jw = Js * winv.asDiagonal();

  Eigen::VectorXd qd_task = Eigen::VectorXd::Zero(n);
  Eigen::MatrixXd V = Eigen::MatrixXd::Zero(n, 0);
  Eigen::VectorXd f;   // damped projection share per singular direction
  res.sigma_min = std::numeric_limits<double>::infinity();
  if (m > 0) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jw, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd s = svd.singularValues();
    res.sigma_min = (n < m) ? 0.0 : s.minCoeff();
    double lambda2 = 0.0;
    if (res.sigma_min < params.sigma_threshold && params.sigma_threshold > 0.0) {
      const double ratio = res.sigma_min / params.sigma_threshold;
      lambda2 = (1.0 - ratio * ratio) * params.lambda_max * params.lambda_max;
    }
    res.damping = std::sqrt(lambda2);
    V = svd.matrixV();
    f = Eigen::VectorXd::Zero(s.size());
    Eigen::VectorXd qd_w = Eigen::VectorXd::Zero(n);
    for (Eigen::Index i = 0; i < s.size(); ++i) {
      const double den = s(i) * s(i) + lambda2;
      if (s(i) < kRankTol || den < 1e-18) {continue;}
      f(i) = s(i) * s(i) / den;
      qd_w += (s(i) / den) * svd.matrixU().col(i).dot(vs) * V.col(i);
    }
    qd_task = winv.asDiagonal() * qd_w;
  }

  // Null-space motion: (I - J_W^+ J) qd0 in weighted coordinates.
  Eigen::VectorXd qd_null = Eigen::VectorXd::Zero(n);
  if (!params.null_space.empty()) {
    const Eigen::VectorXd qd0 = combinedAscent(params.null_space, q, limits.q_min, limits.q_max);
    Eigen::VectorXd nw(n);
    for (Eigen::Index i = 0; i < n; ++i) {nw(i) = qd0(i) / winv(i);}
    for (Eigen::Index i = 0; i < f.size(); ++i) {nw -= f(i) * V.col(i).dot(nw) * V.col(i);}
    qd_null = winv.asDiagonal() * nw;
  }

  // Uniform task scaling at the velocity limits.
  double s_task = 1.0;
  for (Eigen::Index i = 0; i < n; ++i) {
    if (hasLimit(limits.qd_max, i) && std::abs(qd_task(i)) > limits.qd_max(i)) {
      s_task = std::min(s_task, limits.qd_max(i) / std::abs(qd_task(i)));
    }
  }
  res.scale = s_task;
  qd_task *= s_task;

  // Largest share of the null-space motion that still fits under the velocity limits.
  double t = 1.0;
  for (Eigen::Index i = 0; i < n; ++i) {
    const double b = qd_null(i);
    if (!hasLimit(limits.qd_max, i) || std::abs(b) < 1e-15) {continue;}
    const double a = qd_task(i);
    const double room = limits.qd_max(i) - (b > 0.0 ? a : -a);
    t = std::min(t, std::max(0.0, room) / std::abs(b));
  }
  res.null_scale = t;
  res.qd = qd_task + t * qd_null;

  // Hard position-limit zones.
  for (Eigen::Index i = 0; i < n; ++i) {
    if (!isLimitedJoint(limits.q_min(i), limits.q_max(i))) {continue;}
    const double margin = std::max(0.0, limits.position_margin);
    const bool into_lo = q(i) <= limits.q_min(i) + margin && res.qd(i) < 0.0;
    const bool into_hi = q(i) >= limits.q_max(i) - margin && res.qd(i) > 0.0;
    if (into_lo || into_hi) {
      res.qd(i) = 0.0;
      res.saturation[static_cast<size_t>(i)] = JointSaturation::PositionLimit;
      ++res.position_limit_hits;
    }
  }
  for (Eigen::Index i = 0; i < n; ++i) {
    auto & sat = res.saturation[static_cast<size_t>(i)];
    if (sat == JointSaturation::None && hasLimit(limits.qd_max, i) &&
      std::abs(res.qd(i)) >= 0.999 * limits.qd_max(i))
    {
      sat = JointSaturation::Velocity;
      ++res.velocity_limit_hits;
    }
  }
  res.achieved_twist = J * res.qd;
  res.valid = true;
  return res;
}

IntegrateResult integrate(
  const Eigen::VectorXd & q, const Eigen::VectorXd & qd_prev, const Eigen::VectorXd & qd_cmd,
  double dt, const DiffIkLimits & limits)
{
  IntegrateResult out;
  const Eigen::Index n = q.size();
  out.q = q;
  out.qd = Eigen::VectorXd::Zero(n);
  if (qd_prev.size() != n || qd_cmd.size() != n || !(dt > 0.0)) {return out;}
  for (Eigen::Index i = 0; i < n; ++i) {
    double dv = qd_cmd(i) - qd_prev(i);
    const double amax = limitOr(limits.qdd_max, i) * dt;
    dv = std::clamp(dv, -amax, amax);
    double v = qd_prev(i) + dv;
    const double vmax = limitOr(limits.qd_max, i);
    v = std::clamp(v, -vmax, vmax);
    double qn = q(i) + v * dt;   // semi-implicit: new velocity
    if (i < limits.q_min.size() && i < limits.q_max.size() &&
      isLimitedJoint(limits.q_min(i), limits.q_max(i)))
    {
      if (qn < limits.q_min(i)) {qn = limits.q_min(i); v = 0.0;}
      if (qn > limits.q_max(i)) {qn = limits.q_max(i); v = 0.0;}
    }
    out.q(i) = qn;
    out.qd(i) = v;
  }
  return out;
}

}  // namespace diff_ik_core
