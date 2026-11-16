/**
 * @file contact_area.hpp
 * @brief Contact-area model of a pad tool (mop, wiper, sander): effective contact width as a
 *        function of normal force, surface tilt and surface curvature.
 * @details The pad is modelled as a flat compliant block of nominal width `pad_width_m` (measured
 *          at `reference_force_n`) and length `pad_length_m` (along travel):
 *
 *              w(F)   = pad_width * clamp(1 + width_gain_per_n * (F - F_ref), min_factor, max_factor)
 *              w_tilt = w(F) * cos(tilt)                    (a tilted pad touches a narrower strip)
 *              w_conv = min(w_tilt, 2 sqrt(2 d / kappa))    (convex surface, curvature kappa > 0,
 *                                                            indentation d = F / pad_stiffness_n_per_m)
 *
 *          The model is off (`enabled() == false`) while `pad_width_m == 0`, in which case a tool
 *          keeps using its fixed `swath_m`. Limits: static, quasi-flat model; a fitted gain, not a
 *          contact mechanics solver; no wear, no pad rotation.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace core_types
{

struct ContactAreaModel
{
  double pad_width_m{0.0};           ///< Width at reference_force_n, perpendicular to travel. 0 = model off.
  double pad_length_m{0.0};          ///< Length along travel (only used for the pressure estimate).
  double reference_force_n{0.0};     ///< Force at which pad_width_m was measured.
  double width_gain_per_n{0.0};      ///< Relative width change per N of force above the reference.
  double min_width_factor{0.5};      ///< Lower clamp of the force factor, in (0, 1].
  double max_width_factor{1.5};      ///< Upper clamp of the force factor, >= 1.
  double pad_stiffness_n_per_m{0.0}; ///< Normal pad stiffness; 0 disables the curvature cap.
  double nominal_tilt_deg{0.0};      ///< Planning tilt of the pad against the surface, [0, 90).
  double surface_curvature_per_m{0.0}; ///< Planning convex curvature 1/R of the surface, >= 0.

  bool enabled() const {return pad_width_m > 0.0;}
};

/// @throws std::invalid_argument on out-of-range parameters (only checked when enabled).
inline void validateContactArea(const ContactAreaModel & m)
{
  if (!m.enabled()) {
    return;
  }
  if (m.pad_length_m < 0.0 || m.reference_force_n < 0.0 || m.pad_stiffness_n_per_m < 0.0 ||
    m.surface_curvature_per_m < 0.0)
  {
    throw std::invalid_argument("contact_area: lengths, forces, stiffness, curvature must be >= 0");
  }
  if (!(m.min_width_factor > 0.0 && m.min_width_factor <= 1.0 && m.max_width_factor >= 1.0)) {
    throw std::invalid_argument("contact_area: need 0 < min_width_factor <= 1 <= max_width_factor");
  }
  if (!(m.nominal_tilt_deg >= 0.0 && m.nominal_tilt_deg < 90.0)) {
    throw std::invalid_argument("contact_area: nominal_tilt_deg must be in [0, 90)");
  }
}

/// Effective contact width (m) at normal force `force_n` (>= 0), tilt and curvature.
/// @throws std::invalid_argument if the model is off, invalid, or an input is out of range.
inline double contactEffectiveWidth(
  const ContactAreaModel & m, double force_n, double tilt_deg, double curvature_per_m)
{
  if (!m.enabled()) {
    throw std::invalid_argument("contact_area: model is off (pad_width_m == 0)");
  }
  validateContactArea(m);
  if (force_n < 0.0 || tilt_deg < 0.0 || tilt_deg >= 90.0 || curvature_per_m < 0.0) {
    throw std::invalid_argument("contact_area: force, tilt in [0,90), curvature must be >= 0");
  }
  const double factor = std::clamp(
    1.0 + m.width_gain_per_n * (force_n - m.reference_force_n), m.min_width_factor,
    m.max_width_factor);
  double w = m.pad_width_m * factor * std::cos(tilt_deg * M_PI / 180.0);
  if (curvature_per_m > 0.0 && m.pad_stiffness_n_per_m > 0.0) {
    const double indentation = force_n / m.pad_stiffness_n_per_m;
    w = std::min(w, 2.0 * std::sqrt(2.0 * indentation / curvature_per_m));
  }
  return w;
}

/// Width at the model's own nominal tilt and curvature.
inline double contactEffectiveWidth(const ContactAreaModel & m, double force_n)
{
  return contactEffectiveWidth(m, force_n, m.nominal_tilt_deg, m.surface_curvature_per_m);
}

/// Mean contact pressure (Pa) = force / (width * pad_length); 0 when pad_length_m == 0.
inline double contactPressurePa(const ContactAreaModel & m, double force_n)
{
  if (m.pad_length_m <= 0.0) {
    return 0.0;
  }
  return force_n / (contactEffectiveWidth(m, force_n) * m.pad_length_m);
}

}  // namespace core_types
