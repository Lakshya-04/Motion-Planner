/**
 * @file bias_tracker.hpp
 * @brief Slow tracking of the F/T bias (zero and temperature drift) while the tool is free.
 * @details The sensor bias drifts with temperature and time. When the tool is verified free of contact
 *          and the arm is still, the compensated wrench (raw - sign-corrected payload gravity - current
 *          bias) is pure bias error, so it can be tracked. A scalar Kalman filter per axis (random-walk
 *          bias model) does it: process noise `q` lets the bias move slowly, measurement noise `r` is the
 *          sensor noise. Updates are gated:
 *            - the caller states `free_of_contact` and `still` (e.g. from the force FSM and joint velocity);
 *            - an innovation above `reject_threshold` (force N / torque N m) is not averaged in: a real
 *              contact the caller missed, or a payload change, must not be absorbed into the bias; a run of
 *              `max_consecutive_rejects` rejections raises `needs_recalibration` instead;
 *            - the total correction is capped at `max_total_correction` away from the bias it started from.
 *          Plain C++, no clock: the caller calls `update` at its own rate.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <array>
#include <cstddef>

#include "core_types/wrench.hpp"

namespace force_core
{

struct BiasTrackerConfig
{
  double process_noise_force{1e-8};     ///< Random-walk variance per update [N^2].
  double process_noise_torque{1e-10};   ///< [N^2 m^2].
  double measurement_noise_force{0.01}; ///< Sensor noise variance [N^2] (0.1 N sigma).
  double measurement_noise_torque{1e-4};///< [N^2 m^2].
  double initial_variance_force{1e-2};
  double initial_variance_torque{1e-4};
  double reject_threshold_force{1.0};   ///< Innovation above this is rejected [N].
  double reject_threshold_torque{0.1};  ///< [N m].
  double max_total_correction_force{2.0};   ///< Max |bias - initial bias| [N].
  double max_total_correction_torque{0.2};  ///< [N m].
  std::size_t max_consecutive_rejects{20};
};

enum class BiasUpdateStatus {kUpdated, kSkippedNotFree, kRejectedInnovation, kRejectedCap};

class BiasTracker
{
public:
  BiasTracker(const core_types::Wrench & initial_bias, const BiasTrackerConfig & config = {});

  /// `residual` = raw wrench minus the compensated payload gravity (sign applied) i.e. what the
  /// current bias estimate should equal at rest. Returns what happened to this sample.
  BiasUpdateStatus update(const core_types::Wrench & residual, bool free_of_contact, bool still);

  const core_types::Wrench & bias() const {return bias_;}
  std::size_t accepted() const {return accepted_;}
  std::size_t rejected() const {return rejected_;}
  /// Too many rejections in a row: the model is wrong (payload or sensor changed), recalibrate.
  bool needsRecalibration() const {return consecutive_rejects_ >= config_.max_consecutive_rejects;}

private:
  BiasTrackerConfig config_;
  core_types::Wrench bias_;
  core_types::Wrench initial_;
  std::array<double, 6> var_;
  std::size_t accepted_{0}, rejected_{0}, consecutive_rejects_{0};
};

}  // namespace force_core
