/**
 * @file tool_footprint.hpp
 * @brief Footprint of a tilted flat tool on the surface, projected across the stroke direction.
 * @details A tool whose axis is tilted by theta from the surface normal, about an in-surface
 *          axis, touches with a footprint stretched by 1/cos(theta) along the tilt direction (the
 *          in-surface direction perpendicular to the tilt axis) and unchanged along the tilt axis.
 *          For a pad of extent `w` across the stroke and `l` along it, with the tilt direction at
 *          angle `phi` from the stroke direction, the stretched pad's extent across the stroke is
 *
 *              w (1 + k sin^2 phi) + l |k sin phi cos phi|,    k = 1 / cos(theta) - 1.
 *
 *          phi = 0 (tilt along the stroke) leaves the swath at `w`; phi = 90 deg (tilt across
 *          the stroke) widens it to `w / cos(theta)`. Used for the contact-area and the spray
 *          footprint models (spray: first-order approximation, the cone asymmetry and the
 *          slant range are ignored). Limit: geometric, rigid projection; a compliant pad
 *          tilted at speed also changes its pressure distribution.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cmath>
#include <stdexcept>

namespace core_types
{

/**
 * @brief Extent of a tilted footprint across the stroke.
 * @param width_across_m Footprint width across the stroke at zero tilt (>= 0).
 * @param length_along_m Footprint length along the stroke at zero tilt (>= 0; 0 if unknown).
 * @param tilt_deg Tool tilt, in [0, 90).
 * @param tilt_dir_from_stroke_deg Angle of the tilt direction from the stroke direction.
 * @throws std::invalid_argument for an input out of range or not finite.
 */
inline double tiltedSwathAcrossStroke(
  double width_across_m, double length_along_m, double tilt_deg, double tilt_dir_from_stroke_deg)
{
  if (!(width_across_m >= 0.0) || !(length_along_m >= 0.0) || !std::isfinite(width_across_m) ||
    !std::isfinite(length_along_m))
  {
    throw std::invalid_argument("tool footprint: width and length must be finite and >= 0");
  }
  if (!(tilt_deg >= 0.0 && tilt_deg < 90.0) || !std::isfinite(tilt_dir_from_stroke_deg)) {
    throw std::invalid_argument("tool footprint: tilt must be in [0, 90) and the direction finite");
  }
  if (tilt_deg == 0.0) {
    return width_across_m;
  }
  const double k = 1.0 / std::cos(tilt_deg * M_PI / 180.0) - 1.0;
  const double phi = tilt_dir_from_stroke_deg * M_PI / 180.0;
  const double s = std::sin(phi);
  const double c = std::cos(phi);
  return width_across_m * (1.0 + k * s * s) + length_along_m * std::fabs(k * s * c);
}

}  // namespace core_types
