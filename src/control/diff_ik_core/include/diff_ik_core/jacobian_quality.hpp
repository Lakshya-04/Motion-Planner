/**
 * @file jacobian_quality.hpp
 * @brief Kinematic quality measures on a Jacobian (Eigen SVD, no ROS).
 * @details All measures take the Jacobian as given (any row count, typically 6 x N in the base
 *          frame, or a row-selected subset for a masked task). A Jacobian with fewer columns than
 *          rows cannot produce every task direction, so sigma_min is 0 for it.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <Eigen/Dense>

#include <vector>

namespace diff_ik_core
{

struct JacobianQuality
{
  double sigma_min = 0.0;        ///< smallest singular value (0 when rows > cols)
  double sigma_max = 0.0;        ///< largest singular value
  double condition = 0.0;        ///< sigma_max / sigma_min, +infinity at a singularity
  double manipulability = 0.0;   ///< Yoshikawa: product of the singular values = sqrt(det(J J^T)); 0 when rows > cols
};

/// All four measures from one SVD. An empty Jacobian gives all zeros.
JacobianQuality jacobianQuality(const Eigen::MatrixXd & J);

/// Smallest singular value (0 when the Jacobian has more rows than columns, or is empty).
double minSingularValue(const Eigen::MatrixXd & J);

/**
 * Velocity-ellipsoid gain along a task direction: the largest task speed along the unit vector
 * `direction` that a unit-norm joint velocity can produce, 1 / sqrt(u^T (J J^T)^-1 u). Unlike
 * sigma_min it measures only the direction the stroke actually moves in (a stroke tangent), so an
 * arm can be near a singularity in another direction and still have a good gain here. Returns 0
 * if the direction is outside the range of J (a direction the arm cannot move in) or the vector
 * has zero length. `direction` is normalised internally; its size must equal J.rows().
 */
double directionalGain(const Eigen::MatrixXd & J, const Eigen::VectorXd & direction);

/**
 * Smallest distance to a position limit as a share of that joint's range, in [0, 0.5] (0.5 =
 * every limited joint is centred, 0 = a joint is at or past a limit). Continuous joints are
 * ignored: a range that is not positive or is at least 2*pi. Same semantics as the reachability
 * package's jointLimitMargin, restated here so this core has no dependency on it. Returns -1 on
 * a size mismatch.
 */
double jointLimitMargin(
  const std::vector<double> & q, const std::vector<double> & lower,
  const std::vector<double> & upper);

/// True for a joint with a real position limit (positive range below 2*pi), false for continuous.
bool isLimitedJoint(double lower, double upper);

}  // namespace diff_ik_core
