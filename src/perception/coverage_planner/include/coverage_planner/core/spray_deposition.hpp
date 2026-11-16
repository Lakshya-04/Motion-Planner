/**
 * @file spray_deposition.hpp
 * @brief Deposition footprint of a spray cone on a flat wall: how much lands at each distance from the
 *        path, per metre of travel with the valve open.
 * @details The cone of half-angle theta at stand-off d covers a strip of width `w(d) = 2 d tan(theta)`
 *          (core_types::sprayGeometricWidth). Across that strip the deposit is a Gaussian centred on the
 *          path, truncated at the strip edge and normalised so that its integral over the strip is 1:
 *
 *              g(x) = exp(-x^2 / (2 s^2)) / (s sqrt(2 pi) erf(h / (s sqrt 2))),   |x| <= h = w / 2
 *
 *          with `s = sigma_fraction * w`. The default `sigma_fraction = 0.25` puts the strip edge at two
 *          sigma (the deposit there is 13.5 % of the peak). A pass that travels a length L with the valve
 *          open deposits `g(x) * L` at perpendicular distance x: the unit is "relative thickness"
 *          (flow / speed folded to 1), which is all the uniformity (coefficient of variation) needs.
 *
 *          Limits: a stationary, symmetric, wall-normal cone. No overspray beyond the strip, no wall
 *          tilt, no speed dependence (a stroke is weighted by path length, i.e. constant speed), no
 *          flow transient at valve opening. The shape parameter is a starting point to fit to a real
 *          spray test.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <cmath>
#include <stdexcept>

#include "core_types/sprayer_effective_swath.hpp"

namespace coverage_planner
{

struct SprayFootprint
{
  double half_width_m{0.0};   ///< Strip half-width h = d tan(theta); nothing lands beyond it.
  double sigma_m{0.0};        ///< Gaussian sigma.

  /// Footprint of a cone at a stand-off. @throws std::invalid_argument for an input out of range
  /// (see core_types::sprayGeometricWidth) or a sigma_fraction <= 0.
  static SprayFootprint fromCone(double standoff_m, double half_angle_deg, double sigma_fraction = 0.25)
  {
    if (!(sigma_fraction > 0.0)) {
      throw std::invalid_argument("spray footprint: sigma_fraction must be > 0");
    }
    const double w = core_types::sprayGeometricWidth(standoff_m, half_angle_deg);
    return {0.5 * w, sigma_fraction * w};
  }

  /// Deposit density at perpendicular distance x from the path (per metre of x, per metre of travel);
  /// 0 beyond the strip. Integrates to 1 over [-h, h].
  double density(double x) const
  {
    const double ax = std::fabs(x);
    if (!(half_width_m > 0.0) || !(sigma_m > 0.0) || ax > half_width_m) {return 0.0;}
    const double norm = sigma_m * std::sqrt(2.0 * 3.14159265358979323846) *
      std::erf(half_width_m / (sigma_m * std::sqrt(2.0)));
    return std::exp(-0.5 * ax * ax / (sigma_m * sigma_m)) / norm;
  }
};

}  // namespace coverage_planner
