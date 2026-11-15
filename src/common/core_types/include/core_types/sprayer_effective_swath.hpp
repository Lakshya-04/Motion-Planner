/**
 * @file sprayer_effective_swath.hpp
 * @brief Software-first placeholder for the sprayer standoff <-> effective-swath relationship.
 * @details No sprayer hardware exists yet, so there's no way to reproduce the earlier prototype's empirical
 *          fan-width measurement (a fan-edge-peak mapper reading a captured spray pattern --
 *          genuinely needs the physical nozzle; the fit that prototype used was
 *          `sprayerStandoffDistance = 1.5441 * fan_width - 0.07059`). This function is the sole,
 *          isolated place that relationship lives -- `standoff_m` and `coverage.swath_m` stay
 *          independent, explicit fields in tool_profiles.yaml in the meantime (this function is
 *          validate-only, never silently authoritative), so swapping in a real regression once a
 *          nozzle exists is a one-function change, not a design change.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

namespace core_types
{

/// [TUNE] Placeholder linear-fit constants -- re-derive once a physical nozzle exists and a
/// fan-width mapper (analogous to the earlier prototype's) can measure a real spray pattern. Shaped after
/// the earlier prototype's fit (slope ~1.5441, intercept ~-0.07059 in the inverse direction,
/// standoff-from-fan-width) but these are placeholder round numbers, not a regression result.
constexpr double kSprayerSwathSlopePerMeterStandoff = 1.0;   ///< [TUNE]
constexpr double kSprayerSwathInterceptM = 0.0;              ///< [TUNE]

/**
 * @brief Placeholder estimate of effective spray-fan width at a given standoff distance.
 * @param[in] standoff_m Stand-off distance along the tool normal, in meters.
 * @return Effective swath (fan width) in meters, per the placeholder linear fit above.
 * @details Validate against `coverage.swath_m` in tool_profiles.yaml -- do not use this to
 *          silently override that field. It exists so a real regression can replace this one
 *          function's body without touching any caller.
 */
inline double sprayerEffectiveSwath(double standoff_m)
{
  return kSprayerSwathSlopePerMeterStandoff * standoff_m + kSprayerSwathInterceptM;
}

}  // namespace core_types
