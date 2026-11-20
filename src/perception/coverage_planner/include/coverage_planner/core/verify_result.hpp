/**
 * @file verify_result.hpp
 * @brief One verify-result contract and per-mode pass/fail thresholds for a wall run.
 * @details ROS-free. `VerifyResult` is the single answer a run gives to whoever asked "did it
 *          work": passed, coverage percent, failed segment ids, report path. Version 1 comes from
 *          execution telemetry (`verifyFromTelemetry`, built on `scoreRun`); `source` leaves room
 *          for a version 2 filled from perception, which is not implemented.
 *
 *          `limitsForMode` gives the numeric thresholds per run mode. The numbers are starting
 *          points chosen from the tool profile's abort level and the roadmap, not values fitted to
 *          hardware; callers override any of them.
 *
 *          A segment is a time window of the recording (a wall column). It fails when the tool never
 *          worked in it, when the force peak inside it exceeds the mode's peak limit, or (contact
 *          tools) when the in-band fraction inside it is under the minimum. Coverage is a property
 *          of the whole run and is not attributed to segments.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "coverage_planner/core/run_scorecard.hpp"

namespace coverage_planner
{

enum class RunMode { Contact, NonContact, Spray };

/// Numeric pass/fail limits per mode.
///  - Contact: coverage 0.95, force peak 60 N (the tool profile's `overload_abort_n`), in-band 0.8.
///  - NonContact: coverage 0.95, any force above 5 N fails the run (the tool must not touch).
///  - Spray: NonContact plus a thickness CoV limit of 0.5.
/// `target_force_n` only matters for Contact and is used for the in-band checks by the caller.
ScorecardLimits limitsForMode(RunMode mode);

/// Parse "contact" / "non_contact" / "spray". @throws std::invalid_argument otherwise.
RunMode runModeFromString(const std::string & name);

/// A recorded time window that belongs to one segment (a wall column).
struct SegmentWindow
{
  uint32_t id{0};
  double t_start_s{0.0};
  double t_end_s{0.0};
};

struct VerifyResult
{
  bool passed{false};
  double coverage_percent{0.0};              ///< 0..100, geometric.
  std::vector<uint32_t> failed_segment_ids;  ///< Ascending, no duplicates.
  std::string report_path;                   ///< Where the full scorecard was written; may be empty.
  std::string source{"telemetry"};           ///< "telemetry" (v1); "perception" is not implemented.
  std::vector<std::string> failures;         ///< Run-level failures from the scorecard.
};

/// Score the recording and fold the result into the contract. A run with segments fails when
/// any segment fails, even if the run-level card passed.
/// @throws std::invalid_argument as `scoreRun`.
VerifyResult verifyFromTelemetry(
  const WallPlane & wall, const std::vector<ObstacleRegion> & obstacles,
  const std::vector<RunSample> & samples, const std::vector<SegmentWindow> & segments,
  const ScorecardOptions & options, const ScorecardLimits & limits,
  const std::string & report_path = "", RunScorecard * card_out = nullptr);

}  // namespace coverage_planner
