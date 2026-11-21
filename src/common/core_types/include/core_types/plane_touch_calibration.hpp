/**
 * @file plane_touch_calibration.hpp
 * @brief Tool-tip offset from touches on one plane, and tool-tilt estimate from a measured normal.
 *        ROS-free, simulation-tested maths only.
 * @details Plane touch (the field alternative to a pivot fixture). The tool tip is pressed on a
 *          flat surface at N flange poses (R_i, p_i) in the base frame. The tip t (flange frame)
 *          and the plane (unit normal n, offset d) satisfy
 *
 *              n . (p_i + R_i t) + d = 0
 *
 *          With a known normal this is linear in (t, d); with an unknown normal it is solved by
 *          Gauss-Newton over (t, two normal angles, d), starting from a plane fitted to the tips
 *          of the initial guess. The normal equations are solved with the same Gauss-Jordan
 *          routine as `pivotCalibration` (`detail::solveAugmented`). Only the component of t
 *          along the sampled flange-frame normal directions R_i^T n is observable, so the
 *          orientation must vary about two axes: a set of poses is refused when the largest angle
 *          between the flange-frame normals R_i^T n is below `min_orientation_variation_deg`
 *          (default 20 degrees), or when the Jacobian condition number exceeds the limit.
 *
 *          Tilt fit. `fitToolTilt` takes touches with the commanded tool-axis direction and the
 *          plane normal measured by a detector (or fitted from the touches above) and returns
 *          the rotation that aligns the commanded axis with the inward normal. Roll about the
 *          tool axis does not move the axis and is NOT observable; it is reported as such. A
 *          tilt estimated from contact force alone is not offered: with friction the measured
 *          force direction is confounded with the tilt, and the pad needs a real-hardware check.
 *
 *          Limits: static geometry, no pad compliance, no kinematic error of the arm (the
 *          residual includes it), sign convention of the normal fixed below.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "core_types/pivot_calibration.hpp"
#include "core_types/pose.hpp"

namespace core_types
{

using Vec3 = std::array<double, 3>;

namespace detail
{

inline double dot3(const Vec3 & a, const Vec3 & b) {return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];}

inline Vec3 cross3(const Vec3 & a, const Vec3 & b)
{
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

inline double norm3(const Vec3 & a) {return std::sqrt(dot3(a, a));}

inline Vec3 scale3(const Vec3 & a, double s) {return {a[0] * s, a[1] * s, a[2] * s};}

inline Vec3 add3(const Vec3 & a, const Vec3 & b) {return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};}

inline Vec3 sub3(const Vec3 & a, const Vec3 & b) {return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};}

/// Row-major 3x3 matrix times a vector.
inline Vec3 mulMat(const std::array<double, 9> & r, const Vec3 & v)
{
  return {r[0] * v[0] + r[1] * v[1] + r[2] * v[2], r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
    r[6] * v[0] + r[7] * v[1] + r[8] * v[2]};
}

/// Transpose of a row-major 3x3 matrix times a vector.
inline Vec3 mulMatT(const std::array<double, 9> & r, const Vec3 & v)
{
  return {r[0] * v[0] + r[3] * v[1] + r[6] * v[2], r[1] * v[0] + r[4] * v[1] + r[7] * v[2],
    r[2] * v[0] + r[5] * v[1] + r[8] * v[2]};
}

inline Vec3 normalized3(const Vec3 & a)
{
  const double n = norm3(a);
  return n > 0.0 ? scale3(a, 1.0 / n) : a;
}

/// Eigenvalues (ascending) and eigenvectors (columns of v) of a symmetric NxN matrix, cyclic Jacobi.
template<int N>
inline void symmetricEigen(double (& a)[N][N], double (& w)[N], double (& v)[N][N])
{
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      v[i][j] = (i == j) ? 1.0 : 0.0;
    }
  }
  for (int sweep = 0; sweep < 60; ++sweep) {
    double off = 0.0;
    for (int p = 0; p < N; ++p) {
      for (int q = p + 1; q < N; ++q) {
        off += a[p][q] * a[p][q];
      }
    }
    if (off < 1e-30) {
      break;
    }
    for (int p = 0; p < N; ++p) {
      for (int q = p + 1; q < N; ++q) {
        if (std::fabs(a[p][q]) < 1e-300) {
          continue;
        }
        const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        const double t = (theta >= 0 ? 1.0 : -1.0) /
          (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
        const double c = 1.0 / std::sqrt(t * t + 1.0);
        const double s = t * c;
        for (int k = 0; k < N; ++k) {
          const double akp = a[k][p], akq = a[k][q];
          a[k][p] = c * akp - s * akq;
          a[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < N; ++k) {
          const double apk = a[p][k], aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
        for (int k = 0; k < N; ++k) {
          const double vkp = v[k][p], vkq = v[k][q];
          v[k][p] = c * vkp - s * vkq;
          v[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }
  int idx[N];
  for (int i = 0; i < N; ++i) {
    idx[i] = i;
    w[i] = a[i][i];
  }
  std::sort(idx, idx + N, [&w](int x, int y) {return w[x] < w[y];});
  double w2[N], v2[N][N];
  for (int i = 0; i < N; ++i) {
    w2[i] = w[idx[i]];
    for (int k = 0; k < N; ++k) {
      v2[k][i] = v[k][idx[i]];
    }
  }
  for (int i = 0; i < N; ++i) {
    w[i] = w2[i];
    for (int k = 0; k < N; ++k) {
      v[k][i] = v2[k][i];
    }
  }
}

/// Rotation vector (axis * angle) of the minimal rotation taking unit `from` to unit `to`.
/// Returns false when they are (nearly) opposite.
inline bool minimalRotation(const Vec3 & from, const Vec3 & to, Vec3 & rotvec)
{
  const Vec3 c = cross3(from, to);
  const double s = norm3(c);
  const double d = dot3(from, to);
  if (s < 1e-12 && d < 0.0) {
    return false;
  }
  if (s < 1e-15) {
    rotvec = {0, 0, 0};
    return true;
  }
  rotvec = scale3(c, std::atan2(s, d) / s);
  return true;
}

}  // namespace detail

/// Options of planeTouchCalibration().
struct PlaneTouchOptions
{
  /// Known plane normal in the base frame (any length > 0). Unset = estimate it.
  std::optional<Vec3> known_normal;
  Vec3 initial_tcp_xyz{0, 0, 0};          ///< Starting tip offset (CAD value); the solve is not sensitive.
  double min_orientation_variation_deg{20.0};  ///< Refuse smaller flange-frame normal variation.
  double max_condition_number{1e4};       ///< Refuse an ill-conditioned Jacobian (sqrt of normal-matrix ratio).
  std::size_t min_touches{0};             ///< 0 = 4 with a known normal, 6 with an unknown one.
};

/// Result of planeTouchCalibration().
struct PlaneTouchResult
{
  bool ok{false};
  std::string error;                      ///< Reason when !ok.
  Vec3 tcp_xyz{0, 0, 0};                  ///< Tip in the flange frame.
  Vec3 normal{0, 0, 1};                   ///< Unit plane normal, base frame; points away from the
                                          ///< surface when the flange +Z points toward it.
  double plane_offset_m{0.0};             ///< d in n . x + d = 0 (base frame).
  double rms_m{0.0};                      ///< RMS distance of the tips from the plane.
  double max_residual_m{0.0};             ///< Worst single touch.
  double condition_number{0.0};           ///< Jacobian condition number at the solution.
  double orientation_variation_deg{0.0};  ///< Largest angle between flange-frame normals R_i^T n.
  std::size_t touches_used{0};            ///< Touches with forces_ok.
  int iterations{0};
};

/**
 * @brief Least-squares tip offset and plane from flange poses at touches on one plane.
 * @param flange_poses Flange pose in the base frame at each touch.
 * @param forces_ok Per touch: true when the contact force confirmed the touch. Empty = all true.
 *        Touches with false are dropped; a mismatching size is an error.
 */
inline PlaneTouchResult planeTouchCalibration(
  const std::vector<Pose> & flange_poses, const std::vector<bool> & forces_ok,
  const PlaneTouchOptions & opt = {})
{
  PlaneTouchResult out;
  if (!forces_ok.empty() && forces_ok.size() != flange_poses.size()) {
    out.error = "forces_ok must be empty or the same size as the poses";
    return out;
  }
  std::vector<Vec3> p;
  std::vector<std::array<double, 9>> rot;
  for (std::size_t i = 0; i < flange_poses.size(); ++i) {
    if (!forces_ok.empty() && !forces_ok[i]) {
      continue;
    }
    std::array<double, 9> r;
    const Pose & q = flange_poses[i];
    if (!std::isfinite(q.x + q.y + q.z + q.qx + q.qy + q.qz + q.qw)) {
      out.error = "pose " + std::to_string(i) + " is not finite";
      return out;
    }
    if (!detail::quatToMatrix(q, r)) {
      out.error = "pose " + std::to_string(i) + " has a zero quaternion";
      return out;
    }
    p.push_back({q.x, q.y, q.z});
    rot.push_back(r);
  }
  const std::size_t n = p.size();
  out.touches_used = n;
  const bool known = opt.known_normal.has_value();
  const std::size_t need = std::max<std::size_t>(opt.min_touches, known ? 4 : 6);
  if (n < need) {
    out.error = "need at least " + std::to_string(need) + " confirmed touches, got " +
      std::to_string(n);
    return out;
  }

  Vec3 t = opt.initial_tcp_xyz;
  Vec3 nrm{0, 0, 1};
  if (known) {
    const double len = detail::norm3(*opt.known_normal);
    if (!(len > 1e-9) || !std::isfinite(len)) {
      out.error = "known_normal must be a finite non-zero vector";
      return out;
    }
    nrm = detail::scale3(*opt.known_normal, 1.0 / len);
  } else {
    // Initial plane: smallest principal axis of the tips for the starting offset.
    Vec3 mean{0, 0, 0};
    std::vector<Vec3> tip(n);
    for (std::size_t i = 0; i < n; ++i) {
      tip[i] = detail::add3(p[i], detail::mulMat(rot[i], t));
      mean = detail::add3(mean, tip[i]);
    }
    mean = detail::scale3(mean, 1.0 / static_cast<double>(n));
    double cov[3][3] = {};
    for (std::size_t i = 0; i < n; ++i) {
      const Vec3 d = detail::sub3(tip[i], mean);
      for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
          cov[a][b] += d[a] * d[b];
        }
      }
    }
    double w[3], v[3][3];
    detail::symmetricEigen<3>(cov, w, v);
    nrm = {v[0][0], v[1][0], v[2][0]};
  }
  // Orient: the flange +Z points toward the surface, so the outward normal opposes it.
  Vec3 mean_z{0, 0, 0};
  for (std::size_t i = 0; i < n; ++i) {
    mean_z = detail::add3(mean_z, Vec3{rot[i][2], rot[i][5], rot[i][8]});
  }
  if (detail::dot3(nrm, mean_z) > 0.0) {
    nrm = detail::scale3(nrm, -1.0);
  }
  double d = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    d -= detail::dot3(nrm, detail::add3(p[i], detail::mulMat(rot[i], t)));
  }
  d /= static_cast<double>(n);

  double cond = 0.0;
  for (int it = 0; it < 100; ++it) {
    out.iterations = it + 1;
    // Tangent basis of the current normal.
    const Vec3 helper = std::fabs(nrm[0]) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    const Vec3 e1 = detail::normalized3(detail::cross3(nrm, helper));
    const Vec3 e2 = detail::cross3(nrm, e1);
    double jtj6[6][7] = {};
    double jtj4[4][5] = {};
    double jfull[6][6] = {};
    for (std::size_t i = 0; i < n; ++i) {
      const Vec3 tip = detail::add3(p[i], detail::mulMat(rot[i], t));
      const double r = detail::dot3(nrm, tip) + d;
      const Vec3 jt = detail::mulMatT(rot[i], nrm);
      double row[6] = {jt[0], jt[1], jt[2], 1.0, detail::dot3(e1, tip), detail::dot3(e2, tip)};
      if (known) {
        for (int a = 0; a < 4; ++a) {
          for (int c = 0; c < 4; ++c) {
            jtj4[a][c] += row[a] * row[c];
          }
          jtj4[a][4] -= row[a] * r;
        }
      } else {
        for (int a = 0; a < 6; ++a) {
          for (int c = 0; c < 6; ++c) {
            jtj6[a][c] += row[a] * row[c];
            jfull[a][c] += row[a] * row[c];
          }
          jtj6[a][6] -= row[a] * r;
        }
      }
    }
    double step[6] = {};
    if (known) {
      double sym[4][4];
      for (int a = 0; a < 4; ++a) {
        for (int c = 0; c < 4; ++c) {
          sym[a][c] = jtj4[a][c];
        }
      }
      double w[4], v[4][4];
      detail::symmetricEigen<4>(sym, w, v);
      cond = w[0] > 0.0 ? std::sqrt(w[3] / w[0]) : INFINITY;
      double x4[4];
      if (detail::solveAugmented<4>(jtj4, x4)) {
        for (int a = 0; a < 4; ++a) {
          step[a] = x4[a];
        }
      } else {
        out.condition_number = INFINITY;
        out.error = "degenerate touches: the tip offset is not observable from these poses";
        return out;
      }
    } else {
      double sym[6][6];
      for (int a = 0; a < 6; ++a) {
        for (int c = 0; c < 6; ++c) {
          sym[a][c] = jfull[a][c];
        }
      }
      double w[6], v[6][6];
      detail::symmetricEigen<6>(sym, w, v);
      cond = w[0] > 0.0 ? std::sqrt(w[5] / w[0]) : INFINITY;
      if (!detail::solveAugmented<6>(jtj6, step)) {
        out.condition_number = INFINITY;
        out.error = "degenerate touches: the tip offset is not observable from these poses";
        return out;
      }
    }
    t = detail::add3(t, Vec3{step[0], step[1], step[2]});
    d += step[3];
    double mag = std::fabs(step[0]) + std::fabs(step[1]) + std::fabs(step[2]) + std::fabs(step[3]);
    if (!known) {
      nrm = detail::normalized3(
        detail::add3(nrm, detail::add3(detail::scale3(e1, step[4]), detail::scale3(e2, step[5]))));
      mag += std::fabs(step[4]) + std::fabs(step[5]);
    }
    if (mag < 1e-12) {
      break;
    }
  }

  out.tcp_xyz = t;
  out.normal = nrm;
  out.plane_offset_m = d;
  out.condition_number = cond;
  double sum_sq = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const double r = detail::dot3(nrm, detail::add3(p[i], detail::mulMat(rot[i], t))) + d;
    sum_sq += r * r;
    out.max_residual_m = std::max(out.max_residual_m, std::fabs(r));
  }
  out.rms_m = std::sqrt(sum_sq / static_cast<double>(n));
  double best = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const Vec3 ai = detail::mulMatT(rot[i], nrm);
    for (std::size_t j = i + 1; j < n; ++j) {
      const Vec3 aj = detail::mulMatT(rot[j], nrm);
      const double c = std::clamp(detail::dot3(ai, aj), -1.0, 1.0);
      best = std::max(best, std::acos(c));
    }
  }
  out.orientation_variation_deg = best * 180.0 / M_PI;
  if (!std::isfinite(t[0] + t[1] + t[2] + d)) {
    out.error = "the solve diverged";
    return out;
  }
  if (out.orientation_variation_deg < opt.min_orientation_variation_deg) {
    out.error = "orientation variation " + std::to_string(out.orientation_variation_deg) +
      " deg is below the required " + std::to_string(opt.min_orientation_variation_deg) +
      " deg: reorient the tool about two axes between touches";
    return out;
  }
  if (!(cond <= opt.max_condition_number)) {
    out.error = "ill-conditioned touches (condition number " + std::to_string(cond) +
      "): vary the orientation about two non-parallel axes";
    return out;
  }
  out.ok = true;
  return out;
}

/// One touch for fitToolTilt().
struct ToolTiltTouch
{
  Vec3 commanded_axis{0, 0, 1};  ///< Commanded tool axis (into the work), base frame, > 0 length.
  Vec3 measured_normal{0, 0, 1}; ///< Measured outward surface normal, base frame, > 0 length.
  /// Flange (= tool) orientation at the touch. When set for every touch, the correction is
  /// averaged in the tool frame (a fixed mounting error); otherwise in the base frame
  /// (e.g. a wall-normal error).
  std::optional<Pose> tool_pose;
};

/// Result of fitToolTilt().
struct ToolTiltFit
{
  bool ok{false};
  std::string error;
  /// Correction rotation (rotation vector, rad) taking the commanded axis to -normal, with the
  /// component along the axis removed; in the tool frame when `in_tool_frame`, else the base frame.
  Vec3 correction_rotvec{0, 0, 0};
  bool in_tool_frame{false};
  double tilt_error_deg{0.0};      ///< Magnitude of the correction.
  double about_x_deg{0.0};         ///< Tool-frame component about X (valid when in_tool_frame).
  double about_y_deg{0.0};         ///< Tool-frame component about Y (valid when in_tool_frame).
  double spread_deg{0.0};          ///< RMS deviation of the per-touch corrections from the mean.
  bool roll_observable{false};     ///< Always false: rotation about the tool axis is unobservable.
  std::size_t touches_used{0};
};

/**
 * @brief Tilt correction from the commanded tool axes and the measured plane normals.
 * @details For each touch the minimal rotation aligning the commanded axis with -normal is
 *          computed; the mean over touches is the estimate (small-angle average of rotation
 *          vectors, fine to a few tens of degrees). Not a force-based estimate.
 */
inline ToolTiltFit fitToolTilt(const std::vector<ToolTiltTouch> & touches, std::size_t min_touches = 1)
{
  ToolTiltFit out;
  const std::size_t n = touches.size();
  if (n < std::max<std::size_t>(min_touches, 1)) {
    out.error = "need at least " + std::to_string(std::max<std::size_t>(min_touches, 1)) +
      " touches, got " + std::to_string(n);
    return out;
  }
  bool all_pose = true;
  for (const auto & tc : touches) {
    all_pose = all_pose && tc.tool_pose.has_value();
  }
  std::vector<Vec3> rv(n);
  Vec3 mean{0, 0, 0};
  for (std::size_t i = 0; i < n; ++i) {
    const double ca = detail::norm3(touches[i].commanded_axis);
    const double cn = detail::norm3(touches[i].measured_normal);
    if (!(ca > 1e-9 && cn > 1e-9 && std::isfinite(ca) && std::isfinite(cn))) {
      out.error = "touch " + std::to_string(i) + ": axis and normal must be finite and non-zero";
      return out;
    }
    const Vec3 c = detail::scale3(touches[i].commanded_axis, 1.0 / ca);
    const Vec3 g = detail::scale3(touches[i].measured_normal, -1.0 / cn);
    Vec3 r;
    if (!detail::minimalRotation(c, g, r)) {
      out.error = "touch " + std::to_string(i) + ": the commanded axis points away from the surface";
      return out;
    }
    if (all_pose) {
      std::array<double, 9> m;
      if (!detail::quatToMatrix(*touches[i].tool_pose, m)) {
        out.error = "touch " + std::to_string(i) + ": zero quaternion";
        return out;
      }
      r = detail::mulMatT(m, r);
      // Remove the roll about the tool axis (zero by construction up to noise in the pose).
      r[2] = 0.0;
    } else {
      r = detail::sub3(r, detail::scale3(c, detail::dot3(r, c)));
    }
    rv[i] = r;
    mean = detail::add3(mean, r);
  }
  mean = detail::scale3(mean, 1.0 / static_cast<double>(n));
  double ss = 0.0;
  for (const auto & r : rv) {
    const Vec3 d = detail::sub3(r, mean);
    ss += detail::dot3(d, d);
  }
  out.correction_rotvec = mean;
  out.in_tool_frame = all_pose;
  out.tilt_error_deg = detail::norm3(mean) * 180.0 / M_PI;
  if (all_pose) {
    out.about_x_deg = mean[0] * 180.0 / M_PI;
    out.about_y_deg = mean[1] * 180.0 / M_PI;
  }
  out.spread_deg = std::sqrt(ss / static_cast<double>(n)) * 180.0 / M_PI;
  out.touches_used = n;
  out.roll_observable = false;
  out.ok = true;
  return out;
}

}  // namespace core_types
