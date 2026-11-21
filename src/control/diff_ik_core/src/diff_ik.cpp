#include "diff_ik_core/diff_ik.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "diff_ik_core/jacobian_quality.hpp"

namespace diff_ik_core
{

namespace
{
constexpr double kRankTol = 1e-9;   // guard for the damped inverse only

using RowMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, 6, kMaxJoints>;
using RowVector = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 6, 1>;

/// A finite velocity limit (0 included); +infinity = unlimited.
bool hasLimit(const Eigen::VectorXd & v, Eigen::Index i) {return std::isfinite(v(i));}

bool isContinuous(const DiffIkLimits & l, Eigen::Index i)
{
  if (!l.continuous.empty()) {return l.continuous[static_cast<size_t>(i)];}
  return !isLimitedJoint(l.q_min(i), l.q_max(i));
}

bool nonNegOrInf(double x) {return !std::isnan(x) && x >= 0.0;}

InputStatus checkLimits(const DiffIkLimits & l, Eigen::Index n, bool need_qdd)
{
  if (l.q_min.size() != n || l.q_max.size() != n || l.qd_max.size() != n ||
    (need_qdd && l.qdd_max.size() != n) ||
    (!l.continuous.empty() && static_cast<Eigen::Index>(l.continuous.size()) != n))
  {
    return InputStatus::BadSize;
  }
  if (!std::isfinite(l.position_margin) || l.position_margin < 0.0) {return InputStatus::BadValue;}
  for (Eigen::Index i = 0; i < n; ++i) {
    if (std::isnan(l.q_min(i)) || std::isnan(l.q_max(i)) || l.q_min(i) > l.q_max(i)) {
      return InputStatus::BadValue;
    }
    if (!isContinuous(l, i) && !(std::isfinite(l.q_min(i)) && std::isfinite(l.q_max(i)))) {
      return InputStatus::BadValue;
    }
    if (!nonNegOrInf(l.qd_max(i)) || (need_qdd && !nonNegOrInf(l.qdd_max(i)))) {
      return InputStatus::BadValue;
    }
  }
  return InputStatus::Ok;
}

InputStatus checkStep(
  const Eigen::Ref<const Eigen::MatrixXd> & J, const Vector6d & twist, const JointRef & q,
  const DiffIkLimits & limits, const DiffIkParams & params)
{
  const Eigen::Index n = J.cols();
  if (n < 1 || n > kMaxJoints || J.rows() != 6 || q.size() != n) {return InputStatus::BadSize;}
  if (params.joint_weights.size() != 0 && params.joint_weights.size() != n) {
    return InputStatus::BadSize;
  }
  const InputStatus ls = checkLimits(limits, n, false);
  if (ls != InputStatus::Ok) {return ls;}
  if (!J.allFinite() || !twist.allFinite() || !q.allFinite()) {return InputStatus::NonFinite;}
  if (!std::isfinite(params.sigma_threshold) || !std::isfinite(params.lambda_max) ||
    !std::isfinite(params.rank_tolerance) || !params.row_scale.allFinite() ||
    (params.joint_weights.size() != 0 && !params.joint_weights.allFinite()))
  {
    return InputStatus::NonFinite;
  }
  if (params.sigma_threshold < 0.0 || params.lambda_max < 0.0 || params.rank_tolerance < 0.0) {
    return InputStatus::BadValue;
  }
  for (int a = 0; a < 6; ++a) {
    if (!(params.row_scale(a) > 0.0)) {return InputStatus::BadValue;}
  }
  for (Eigen::Index i = 0; i < params.joint_weights.size(); ++i) {
    if (!(params.joint_weights(i) > 0.0)) {return InputStatus::BadValue;}
  }
  for (const auto & t : params.null_space) {
    if (!std::isfinite(t.gain)) {return InputStatus::NonFinite;}
  }
  return InputStatus::Ok;
}

DiffIkResult invalid(Eigen::Index n, InputStatus why)
{
  DiffIkResult r;
  r.status = why;
  r.qd = (n >= 1 && n <= kMaxJoints) ? JointVector::Zero(n).eval() : JointVector();
  return r;
}
}  // namespace

DiffIkResult computeStep(
  const Eigen::Ref<const Eigen::MatrixXd> & J, const Vector6d & twist, const AxisMask & mask,
  const JointRef & q, const DiffIkLimits & limits, const DiffIkParams & params)
{
  const Eigen::Index n = J.cols();
  const InputStatus st = checkStep(J, twist, q, limits, params);
  if (st != InputStatus::Ok) {return invalid(n, st);}

  DiffIkResult res;
  res.qd = JointVector::Zero(n);

  // Row selection for the masked task.
  std::array<int, 6> rows = {};
  int m = 0;
  for (int a = 0; a < 6; ++a) {
    if (mask[static_cast<size_t>(a)]) {rows[static_cast<size_t>(m++)] = a;}
  }

  // Locked joints (velocity limit 0) leave the solve; weights and row scales go into Jw.
  JointVector winv = JointVector::Ones(n);
  std::array<bool, kMaxJoints> locked = {};
  for (Eigen::Index i = 0; i < n; ++i) {
    if (params.joint_weights.size() != 0) {winv(i) = 1.0 / std::sqrt(params.joint_weights(i));}
    locked[static_cast<size_t>(i)] = hasLimit(limits.qd_max, i) && limits.qd_max(i) == 0.0;
  }
  RowMatrix Jw(m, n);
  RowVector vs(m);
  for (int r = 0; r < m; ++r) {
    const int a = rows[static_cast<size_t>(r)];
    const double sc = params.row_scale(a);
    vs(r) = sc * twist(a);
    for (Eigen::Index j = 0; j < n; ++j) {
      Jw(r, j) = locked[static_cast<size_t>(j)] ? 0.0 : sc * J(a, j) * winv(j);
    }
  }

  JointVector qd_task = JointVector::Zero(n);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, kMaxJoints, kMaxJoints> V;
  Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxJoints, 1> s;
  res.sigma_min = std::numeric_limits<double>::infinity();
  if (m > 0) {
    Eigen::JacobiSVD<RowMatrix> svd(Jw, Eigen::ComputeThinU | Eigen::ComputeThinV);
    s = svd.singularValues();
    V = svd.matrixV();
    res.sigma_min = (n < m) ? 0.0 : s.minCoeff();
    double lambda2 = 0.0;
    if (res.sigma_min < params.sigma_threshold && params.sigma_threshold > 0.0) {
      const double ratio = res.sigma_min / params.sigma_threshold;
      lambda2 = (1.0 - ratio * ratio) * params.lambda_max * params.lambda_max;
    }
    res.damping = std::sqrt(lambda2);
    JointVector qd_w = JointVector::Zero(n);
    for (Eigen::Index i = 0; i < s.size(); ++i) {
      const double den = s(i) * s(i) + lambda2;
      if (s(i) < kRankTol || den < 1e-18) {continue;}
      qd_w += (s(i) / den) * svd.matrixU().col(i).dot(vs) * V.col(i);
    }
    qd_task = winv.asDiagonal() * qd_w;
  }
  for (Eigen::Index i = 0; i < n; ++i) {
    if (locked[static_cast<size_t>(i)]) {qd_task(i) = 0.0;}
  }

  // Null-space motion: N qd0 with the TRUE projector of the undamped Jacobian, in weighted
  // coordinates (remove the components along singular directions above the rank tolerance).
  JointVector qd_null = JointVector::Zero(n);
  if (!params.null_space.empty()) {
    JointVector lo = JointVector::Zero(n), hi = JointVector::Zero(n);
    for (Eigen::Index i = 0; i < n; ++i) {
      if (!isContinuous(limits, i)) {lo(i) = limits.q_min(i); hi(i) = limits.q_max(i);}
    }
    const JointVector qd0 = combinedAscent(params.null_space, q, lo, hi);
    if (!qd0.allFinite()) {return invalid(n, InputStatus::NonFinite);}
    JointVector nw(n);
    for (Eigen::Index i = 0; i < n; ++i) {
      nw(i) = locked[static_cast<size_t>(i)] ? 0.0 : qd0(i) / winv(i);
    }
    for (Eigen::Index i = 0; i < s.size(); ++i) {
      if (s(i) > params.rank_tolerance) {nw -= V.col(i).dot(nw) * V.col(i);}
    }
    qd_null = winv.asDiagonal() * nw;
    for (Eigen::Index i = 0; i < n; ++i) {
      if (locked[static_cast<size_t>(i)]) {qd_null(i) = 0.0;}
    }
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
  const double margin = limits.position_margin;
  for (Eigen::Index i = 0; i < n; ++i) {
    if (isContinuous(limits, i)) {continue;}
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
    if (sat == JointSaturation::None && hasLimit(limits.qd_max, i) && limits.qd_max(i) > 0.0 &&
      std::abs(res.qd(i)) >= 0.999 * limits.qd_max(i))
    {
      sat = JointSaturation::Velocity;
      ++res.velocity_limit_hits;
    }
  }
  for (int a = 0; a < 6; ++a) {
    double acc = 0.0;
    for (Eigen::Index j = 0; j < n; ++j) {acc += J(a, j) * res.qd(j);}
    res.achieved_twist(a) = acc;
  }
  if (!res.qd.allFinite() || !res.achieved_twist.allFinite()) {
    return invalid(n, InputStatus::NonFinite);
  }
  res.valid = true;
  res.status = InputStatus::Ok;
  return res;
}

IntegrateResult integrate(
  const JointRef & q, const JointRef & qd_prev, const JointRef & qd_cmd,
  double dt, const DiffIkLimits & limits)
{
  IntegrateResult out;
  const Eigen::Index n = q.size();
  if (n < 1 || n > kMaxJoints) {return out;}
  out.q = q;
  out.qd = JointVector::Zero(n);
  if (qd_prev.size() != n || qd_cmd.size() != n || !std::isfinite(dt) || !(dt > 0.0) ||
    !q.allFinite() || !qd_prev.allFinite() || !qd_cmd.allFinite() ||
    checkLimits(limits, n, true) != InputStatus::Ok)
  {
    return out;
  }
  for (Eigen::Index i = 0; i < n; ++i) {
    double dv = qd_cmd(i) - qd_prev(i);
    const double amax = limits.qdd_max(i) * dt;
    dv = std::clamp(dv, -amax, amax);
    double v = qd_prev(i) + dv;
    const double vmax = limits.qd_max(i);
    v = std::clamp(v, -vmax, vmax);
    double qn = q(i) + v * dt;   // semi-implicit: new velocity
    if (!isContinuous(limits, i)) {
      const double lo = limits.q_min(i), hi = limits.q_max(i);
      if (qn < lo) {
        if (q(i) < lo) {
          if (qn < q(i)) {qn = q(i); v = 0.0;}   // further out: cancelled; inwards is allowed
        } else {
          qn = lo; v = 0.0;
        }
      } else if (qn > hi) {
        if (q(i) > hi) {
          if (qn > q(i)) {qn = q(i); v = 0.0;}
        } else {
          qn = hi; v = 0.0;
        }
      }
    }
    if (!std::isfinite(qn)) {return IntegrateResult{false, JointVector(q), JointVector::Zero(n)};}
    out.q(i) = qn;
    out.qd(i) = v;
  }
  out.valid = true;
  return out;
}

}  // namespace diff_ik_core
