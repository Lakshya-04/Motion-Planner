/**
 * @file safety_monitor.hpp
 * @brief Speed-and-separation monitor: speed scale, state, reason, worst sphere.
 * @details For every robot sphere i and every obstacle source s the monitor computes the surface
 *          distance d = sdf(c) - r over a sweep of the stopping horizon (the sphere advanced along
 *          its velocity at `sweep_samples + 1` times in [0, stop_time], radius inflated by
 *          `sweep_radius_inflation_m`; a moving source is advanced too). With the protective
 *          distance
 *
 *              d_stop = v_human * T_stop + v_toward * T_reaction + margin
 *              d_slow = d_stop + slow_band
 *              scale  = clamp((d - d_stop) / (d_slow - d_stop), 0, 1)
 *
 *          (form of ISO/TS 15066 speed-and-separation monitoring; v_human is 0 for static
 *          sources, max(human_speed, measured speed) for persons; v_toward is the sphere speed
 *          towards the obstacle; margin is the intrusion plus position uncertainty for persons and
 *          a smaller static margin otherwise) the overall scale is the minimum over all spheres,
 *          sources and sweep samples. Scale 0 is a protective stop. Self-collision uses the same
 *          form with the closing speed of each non-allowed sphere pair and the self margin.
 *
 *          Latching: a protective stop (including a fail-safe one) holds until the hazard is
 *          gone, has been gone for `min_clear_time_s`, AND `reset()` was called after the hazard
 *          cleared (a reset made while the hazard is present is discarded).
 *
 *          Fail-safe: a missing or negative or non-finite stamp, input older than
 *          `stale_timeout_s`, a stale or unstamped obstacle source, a NaN anywhere in the
 *          transforms, velocities or distances, or a size mismatch gives a protective stop with
 *          scale 0 and reason kStaleData or kInvalidInput.
 *
 *          Contact-exempt links are skipped against sources flagged as the work surface only.
 *
 *          NOT A CERTIFIED SAFETY FUNCTION: ordinary software on a general-purpose OS. It only
 *          slows the robot earlier; see the package README.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <Eigen/Dense>
#include <limits>
#include <vector>

#include "safety_monitor/distance_source.hpp"
#include "safety_monitor/sphere_model.hpp"

namespace safety_monitor
{

enum class State {kOk, kSlow, kProtectiveStop};
enum class Reason {kNone, kSelfCollision, kEnvironment, kPerson, kStaleData, kInvalidInput};

struct Params
{
  double stop_time_s{0.3};         ///< T_stop, the stopping horizon of the whole system
  double reaction_time_s{0.1};     ///< T_reaction
  double human_speed_mps{1.6};     ///< assumed person speed (ISO 13855 style)
  double static_margin_m{0.05};
  double person_intrusion_m{0.10};
  double person_uncertainty_m{0.10};
  double self_margin_m{0.02};
  double static_slow_band_m{0.30};
  double person_slow_band_m{0.60};  ///< wider than static: persons slow the robot earlier
  double self_slow_band_m{0.10};
  double stale_timeout_s{0.25};
  double min_clear_time_s{1.0};
  int sweep_samples{4};
  double sweep_radius_inflation_m{0.02};
};

struct Result
{
  double speed_scale{0.0};
  State state{State::kProtectiveStop};
  Reason reason{Reason::kInvalidInput};
  int worst_sphere{-1};  ///< index into SphereModel::worldSpheres ordering, -1 if none
  int worst_link{-1};
  double min_distance{std::numeric_limits<double>::infinity()};  ///< smallest surface distance seen
};

class SafetyMonitor
{
public:
  SafetyMonitor(const SphereModel & model, const Params & params)
  : model_(model), p_(params) {}

  /// One monitor cycle. `sphere_velocities` is in worldSpheres() order (empty = all at rest; see
  /// sphereVelocities() to build it from link twists). `data_stamp_s` is the time of the robot
  /// state the transforms came from, `now_s` the current time, both in seconds.
  Result evaluate(
    const std::vector<Eigen::Isometry3d> & link_transforms,
    const std::vector<Vec3> & sphere_velocities,
    const std::vector<const DistanceSource *> & sources, double data_stamp_s, double now_s);

  /// Requests release of a latched stop. Discarded unless the last evaluate() cycle was clear; once
  /// accepted it takes effect when the minimum clear time has elapsed, and is dropped again if the
  /// hazard returns first.
  void reset();

  bool latched() const {return latched_;}
  const Params & params() const {return p_;}

private:
  Result compute(
    const std::vector<Eigen::Isometry3d> & T, const std::vector<Vec3> & vel,
    const std::vector<const DistanceSource *> & sources, double data_stamp, double now) const;

  const SphereModel & model_;
  Params p_;
  bool latched_{false};
  bool reset_pending_{false};
  double clear_since_{-1.0};
  Reason latched_reason_{Reason::kNone};
};

}  // namespace safety_monitor
