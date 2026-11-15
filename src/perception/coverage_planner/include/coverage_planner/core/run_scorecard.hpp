/**
 * @file run_scorecard.hpp
 * @brief Pass/fail scorecard of a wall run: coverage, contact force, time.
 * @details ROS-free. The input is what a run recorded: tool position (in the wall's frame), the
 *          measured force along the wall normal and whether the tool was working. Strokes are the
 *          engaged stretches (a new stroke starts after every disengaged sample), projected onto the
 *          wall's (u, v) plane and rasterised by `CoverageVerifier`.
 *
 *          A contact tool counts as engaged while the force is at least `engaged_force_n`; a
 *          non-contact tool while `tool_active` is set (valve open). Force statistics are taken over
 *          engaged samples of a contact tool only.
 *
 *          Limits: geometric coverage only (see coverage_verification.hpp), no collision count (the
 *          recorder must report those itself), and the pass criteria are the caller's numbers.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <string>
#include <vector>

#include "coverage_planner/core/coverage_verification.hpp"
#include "coverage_planner/core/types.hpp"

namespace coverage_planner
{

/// One recorded sample, position in the same frame as the wall's pose.
struct RunSample
{
  double t_s{0.0};
  double x{0.0}, y{0.0}, z{0.0};
  double force_n{0.0};      ///< Force along the wall normal (positive pressing on the wall).
  bool tool_active{false};  ///< Valve open / pump on (non-contact tools).
};

struct ScorecardOptions
{
  bool contact{true};
  double swath_m{0.2};
  double resolution_m{0.01};
  double engaged_force_n{1.0};   ///< Contact tool: pressing at least this hard counts as working.
  double target_force_n{0.0};    ///< Contact tool: the force the controller aims for.
  double force_band_n{0.0};      ///< Contact tool: |force - target| within this is "in band".
};

struct ScorecardLimits
{
  double min_covered_fraction{0.95};
  double max_force_rms_error_n{0.0};   ///< 0 = not checked.
  double max_force_peak_n{0.0};        ///< 0 = not checked; the abort threshold of the tool.
  double min_in_band_fraction{0.0};    ///< 0 = not checked.
};

struct RunScorecard
{
  VerificationReport coverage;
  size_t strokes{0};
  double duration_s{0.0};
  double engaged_s{0.0};
  double force_rms_error_n{0.0};    ///< RMS of (force - target) over engaged samples.
  double force_peak_n{0.0};         ///< Largest force over all samples.
  double in_band_fraction{0.0};     ///< Engaged samples within the force band.
  bool passed{false};
  std::vector<std::string> failures;
};

/// Score a run on one wall. `obstacles` are excluded from the target.
/// @throws std::invalid_argument for a non-positive swath/resolution or a degenerate wall.
RunScorecard scoreRun(
  const WallPlane & wall, const std::vector<ObstacleRegion> & obstacles,
  const std::vector<RunSample> & samples, const ScorecardOptions & options,
  const ScorecardLimits & limits = {});

}  // namespace coverage_planner
