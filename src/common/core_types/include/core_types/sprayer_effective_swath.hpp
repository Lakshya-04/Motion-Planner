/**
 * @file sprayer_effective_swath.hpp
 * @brief Parametric swath model for a non-contact spray tool, and line spacing from swath and overlap.
 * @details The spray is modelled as a cone with half-angle theta whose apex is at the nozzle. At a
 *          stand-off d the geometric footprint on a flat wall is a strip of width
 *          `2 d tan(theta)`. Real deposition is not uniform across that footprint (it peaks on
 *          the axis and falls off towards the edge, roughly Gaussian), so only a central fraction
 *          of the geometric width is usable as the swath that adjacent passes are spaced by:
 *
 *              w_geo(d)  = 2 d tan(theta)
 *              w_eff(d)  = effective_width_fraction * w_geo(d)
 *              spacing   = w_eff * (1 - overlap_pct / 100)
 *
 *          Limits: this is a width model only. It has no deposition thickness, overspray, nozzle
 *          pressure or wall-angle terms, and says nothing about valve timing. Both parameters are
 *          fitted by hand from a spray test; see docs/tools.md.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cmath>
#include <stdexcept>

namespace core_types
{

/// Parameters of the cone swath model.
struct SpraySwathModel
{
  double half_angle_deg{20.0};          ///< Cone half-angle theta, degrees, in (0, 90).
  double effective_width_fraction{0.8};  ///< Usable fraction of the geometric width, in (0, 1].
};

/**
 * @brief Geometric footprint width of the cone at a stand-off, `2 d tan(theta)`.
 * @param[in] standoff_m Stand-off from nozzle to wall along the normal, meters, > 0.
 * @param[in] half_angle_deg Cone half-angle in degrees, in (0, 90).
 * @throws std::invalid_argument if an input is out of range.
 */
inline double sprayGeometricWidth(double standoff_m, double half_angle_deg)
{
  if (!(standoff_m > 0.0)) {
    throw std::invalid_argument("spray swath: standoff_m must be > 0");
  }
  if (!(half_angle_deg > 0.0 && half_angle_deg < 90.0)) {
    throw std::invalid_argument("spray swath: half_angle_deg must be in (0, 90)");
  }
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  return 2.0 * standoff_m * std::tan(half_angle_deg * kDegToRad);
}

/**
 * @brief Effective swath width (m) at a stand-off: `fraction * 2 d tan(theta)`.
 * @throws std::invalid_argument if standoff or half angle is out of range, or the fraction is
 *         not in (0, 1].
 */
inline double sprayerEffectiveSwath(double standoff_m, const SpraySwathModel & model = {})
{
  if (!(model.effective_width_fraction > 0.0 && model.effective_width_fraction <= 1.0)) {
    throw std::invalid_argument("spray swath: effective_width_fraction must be in (0, 1]");
  }
  return model.effective_width_fraction * sprayGeometricWidth(standoff_m, model.half_angle_deg);
}

/**
 * @brief Distance between adjacent passes: `swath * (1 - overlap_pct / 100)`.
 * @param[in] swath_m Effective swath width, meters, > 0.
 * @param[in] overlap_pct Overlap of neighbouring swaths in percent, in [0, 100).
 * @throws std::invalid_argument if an input is out of range.
 */
inline double lineSpacingFromSwath(double swath_m, double overlap_pct)
{
  if (!(swath_m > 0.0)) {
    throw std::invalid_argument("line spacing: swath_m must be > 0");
  }
  if (!(overlap_pct >= 0.0 && overlap_pct < 100.0)) {
    throw std::invalid_argument("line spacing: overlap_pct must be in [0, 100)");
  }
  return swath_m * (1.0 - overlap_pct / 100.0);
}

}  // namespace core_types
