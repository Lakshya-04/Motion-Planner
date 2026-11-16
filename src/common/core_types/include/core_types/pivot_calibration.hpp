/**
 * @file pivot_calibration.hpp
 * @brief Tool-centre-point (TCP) calibration by pivoting, ROS-free.
 * @details Touch the tool tip to one fixed point (a cone, a dimple, a nail) and reorient the
 *          flange about it in several poses. For every flange pose (R_i, p_i) in the base frame
 *          the unknown tip offset t (in the flange frame) and the unknown fixed point q (in the
 *          base frame) satisfy
 *
 *              R_i t + p_i = q   <=>   [ R_i  -I ] [t; q] = -p_i
 *
 *          which is linear in (t, q): stack all poses and solve the 6-unknown least squares
 *          (the classic sphere fit of the flange origins). The residual of each pose is the
 *          distance the tip would be from q, so the RMS is a direct quality number.
 *
 *          This calibrates the position of the TCP only; the tool orientation (tcp_rpy) is not
 *          observable from a point contact and must be measured or taken from the CAD.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "core_types/pose.hpp"

namespace core_types
{

/// Result of pivotCalibration().
struct PivotCalibrationResult
{
  bool ok{false};
  std::string error;                         ///< Reason when !ok.
  std::array<double, 3> tcp_xyz{0, 0, 0};    ///< Tip in the flange frame (tool_profile tcp_xyz).
  std::array<double, 3> pivot_xyz{0, 0, 0};  ///< Fixed point in the base frame.
  double rms_m{0.0};                         ///< RMS distance of the tip from the pivot point.
  double max_residual_m{0.0};                ///< Worst single pose.
};

namespace detail
{

/// Rotation matrix (row-major) of a quaternion, normalised first. Returns false for a zero quaternion.
inline bool quatToMatrix(const Pose & p, std::array<double, 9> & r)
{
  const double n = std::sqrt(p.qx * p.qx + p.qy * p.qy + p.qz * p.qz + p.qw * p.qw);
  if (!(n > 1e-9)) {
    return false;
  }
  const double x = p.qx / n, y = p.qy / n, z = p.qz / n, w = p.qw / n;
  r = {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)};
  return true;
}

}  // namespace detail

/**
 * @brief Least-squares tool-tip offset from flange poses recorded while pivoting about one point.
 * @param flange_poses Flange (the frame tcp_xyz is relative to) pose in the base frame per sample.
 * @param min_poses Fewer poses than this are refused (3 is the mathematical minimum; use 6+).
 * @return ok=false with a reason when there are too few poses or the rotations are degenerate
 *         (all about one axis, which leaves the offset along that axis unobservable).
 */
inline PivotCalibrationResult pivotCalibration(
  const std::vector<Pose> & flange_poses, std::size_t min_poses = 4)
{
  PivotCalibrationResult out;
  const std::size_t n = flange_poses.size();
  const std::size_t need = std::max<std::size_t>(min_poses, 3);
  if (n < need) {
    out.error = "need at least " + std::to_string(need) + " poses, got " + std::to_string(n);
    return out;
  }
  std::vector<std::array<double, 9>> rot(n);
  for (std::size_t i = 0; i < n; ++i) {
    if (!detail::quatToMatrix(flange_poses[i], rot[i])) {
      out.error = "pose " + std::to_string(i) + " has a zero quaternion";
      return out;
    }
  }

  // Normal equations A^T A x = A^T b with A rows [R_i | -I], b = -p_i (3 rows per pose).
  double ata[6][7] = {};
  for (std::size_t i = 0; i < n; ++i) {
    const double p[3] = {flange_poses[i].x, flange_poses[i].y, flange_poses[i].z};
    for (int r = 0; r < 3; ++r) {
      double row[6] = {rot[i][3 * r], rot[i][3 * r + 1], rot[i][3 * r + 2], 0, 0, 0};
      row[3 + r] = -1.0;
      const double b = -p[r];
      for (int a = 0; a < 6; ++a) {
        for (int c = 0; c < 6; ++c) {
          ata[a][c] += row[a] * row[c];
        }
        ata[a][6] += row[a] * b;
      }
    }
  }
  // Gauss-Jordan with partial pivoting; a tiny pivot means a rank-deficient (degenerate) pose set.
  double scale = 0.0;
  for (int a = 0; a < 6; ++a) {
    scale = std::max(scale, std::fabs(ata[a][a]));
  }
  for (int col = 0; col < 6; ++col) {
    int best = col;
    for (int r = col + 1; r < 6; ++r) {
      if (std::fabs(ata[r][col]) > std::fabs(ata[best][col])) {
        best = r;
      }
    }
    if (std::fabs(ata[best][col]) < 1e-9 * scale) {
      out.error = "degenerate poses: the flange rotations do not span enough axes "
        "(reorient about at least two non-parallel axes)";
      return out;
    }
    for (int c = 0; c < 7; ++c) {
      std::swap(ata[col][c], ata[best][c]);
    }
    for (int r = 0; r < 6; ++r) {
      if (r == col) {
        continue;
      }
      const double f = ata[r][col] / ata[col][col];
      for (int c = col; c < 7; ++c) {
        ata[r][c] -= f * ata[col][c];
      }
    }
  }
  double x[6];
  for (int a = 0; a < 6; ++a) {
    x[a] = ata[a][6] / ata[a][a];
  }
  for (int k = 0; k < 3; ++k) {
    out.tcp_xyz[k] = x[k];
    out.pivot_xyz[k] = x[3 + k];
  }
  double sum_sq = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    double e2 = 0.0;
    const double p[3] = {flange_poses[i].x, flange_poses[i].y, flange_poses[i].z};
    for (int r = 0; r < 3; ++r) {
      const double tip =
        rot[i][3 * r] * x[0] + rot[i][3 * r + 1] * x[1] + rot[i][3 * r + 2] * x[2] + p[r];
      e2 += (tip - x[3 + r]) * (tip - x[3 + r]);
    }
    sum_sq += e2;
    out.max_residual_m = std::max(out.max_residual_m, std::sqrt(e2));
  }
  out.rms_m = std::sqrt(sum_sq / static_cast<double>(n));
  out.ok = true;
  return out;
}

}  // namespace core_types
