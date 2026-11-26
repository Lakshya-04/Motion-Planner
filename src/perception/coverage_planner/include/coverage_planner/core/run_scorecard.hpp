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
 *          A non-contact run has no force limits. When `ScorecardOptions::spray` is set the engaged
 *          (valve open) strokes also feed the thickness raster and the card reports its uniformity;
 *          samples with the valve closed never count, for coverage or thickness.
 *
 *          Obstacles: they are excluded from the coverage target (grown by `keepout_margin_m`, the
 *          planner's own clearance, so the target is the reachable area). The card also counts
 *          obstacle contacts: an episode is a run of consecutive samples (any force) whose tool
 *          point is inside an obstacle's footprint (grown by `obstacle_margin_m`) and lower than
 *          its height plus `obstacle_contact_tol_m` above the wall. Only the tool point is
 *          checked, not the arm's links.
 *
 *          Limits: geometric coverage (see coverage_verification.hpp), obstacle contacts, and the
 *          pass criteria are the caller's numbers.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <optional>
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
  /// Angle in degrees between the tool axis (pointing at the work) and the wall's inward normal, 0 = square to
  /// the wall. Negative = not recorded (older CSVs), the tilt is then not checked.
  double tilt_deg{-1.0};
};

struct ScorecardOptions
{
  bool contact{true};
  double swath_m{0.2};
  double resolution_m{0.01};
  double engaged_force_n{1.0};   ///< Contact tool: pressing at least this hard counts as working.
  double target_force_n{0.0};    ///< Contact tool: the force the controller aims for.
  double force_band_n{0.0};      ///< Contact tool: |force - target| within this is "in band".
  /// Non-contact tool: the spray footprint; when set the card carries the thickness statistics.
  std::optional<SprayFootprint> spray;
  double keepout_margin_m{0.0};          ///< Obstacle footprints grown by this are left out of the coverage target.
  double obstacle_margin_m{0.0};         ///< Obstacle footprints grown by this for the contact check.
  double obstacle_contact_tol_m{0.003};  ///< Tool point within height + this above the wall counts as inside.
  /// A sample only counts as working when the tool point is within this distance of the wall plane (either side,
  /// so a pad pressed into the surface still counts) and, if the tilt was recorded, within `max_tilt_deg` of square.
  /// Without it a link brushing the wall, or a tilted tool, scores as coverage. 0 = not checked (old behaviour).
  /// Contact tools only: ignored when `contact` is false (a spray nozzle stands off the wall by design).
  double max_wall_gap_m{0.0};
  double max_tilt_deg{0.0};              ///< Allowed deviation of the tilt from `nominal_tilt_deg`; 0 = not checked.
  double nominal_tilt_deg{0.0};          ///< The tool's intended angle to the wall normal (0 for a flat pad, e.g. 30 for an angled tool).
  /// TCP speed statistics (contact tools only, reported, never a pass criterion): windows of contact where the force
  /// is above `tcp_speed_force_n`, speed taken over `tcp_speed_window_s` of travel (see `computeTcpSpeedStats`).
  double tcp_speed_force_n{3.0};
  double tcp_speed_window_s{0.5};
  double target_speed_mps{0.0};          ///< The commanded stroke speed; 0 = no deviation figure.
};

/// Speed of the tool point over the contact strokes only. Each value is |p(t+W) - p(t)| / W for a sample inside a
/// run of consecutive samples whose force is above `force_threshold_n`, with W the shortest span of at least
/// `window_s` that stays inside the run (the position CSV is rounded to 10 um, so a single 10 ms step would carry
/// ~10 % noise at 1 cm/s). Windows overlap (one per sample). A turn inside a window lowers that window's value. A gap of more than `window_s` between two samples ends a run.
struct TcpSpeedStats
{
  size_t windows{0};              ///< Number of speed values; 0 = no contact run was long enough, the rest is 0.
  double mean_mps{0.0};
  double median_mps{0.0};
  double std_mps{0.0};
  double p5_mps{0.0};
  double p95_mps{0.0};
  double target_mps{0.0};         ///< Echo of the target (0 = none given).
  /// (median - target) / target; only meaningful when `target_mps` > 0, else 0.
  double median_rel_dev{0.0};
};

TcpSpeedStats computeTcpSpeedStats(
  const std::vector<RunSample> & samples, double force_threshold_n = 3.0, double window_s = 0.5,
  double target_mps = 0.0);

struct ScorecardLimits
{
  double min_covered_fraction{0.95};
  double max_force_rms_error_n{0.0};   ///< 0 = not checked.
  double max_force_peak_n{0.0};        ///< 0 = not checked; the abort threshold of the tool.
  double min_in_band_fraction{0.0};    ///< 0 = not checked.
  double max_thickness_cov{0.0};       ///< Spray: 0 = not checked.
  int max_obstacle_contacts{-1};       ///< Negative = not checked; 0 = no contact allowed.
  /// Fail when more than this fraction of the samples that would have worked (force or valve) are off the wall or
  /// tilted. Negative = not checked. Needs `max_wall_gap_m` / `max_tilt_deg` in the options.
  double max_bad_pose_fraction{-1.0};
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
  size_t obstacle_contact_samples{0};   ///< Samples with the tool point inside an obstacle.
  size_t obstacle_contacts{0};          ///< Episodes (consecutive such samples count once).
  size_t bad_pose_samples{0};           ///< Would-be working samples refused: off the wall or tilted.
  size_t off_wall_samples{0};           ///< ...of which farther than `max_wall_gap_m` from the wall plane.
  size_t tilted_samples{0};             ///< ...of which tilted more than `max_tilt_deg`.
  double bad_pose_fraction{0.0};        ///< bad_pose_samples over (working + bad pose) samples.
  /// Tilt and gap statistics cover every would-be-working sample (force at least engaged, or valve open), also the
  /// ones the pose gate refused, so a run that fails the gate still reports how far off it was.
  double tilt_mean_deg{0.0};            ///< Over would-be-working samples that carry a tilt.
  double tilt_max_deg{0.0};             ///< Largest recorded tilt among them.
  double wall_gap_mean_m{0.0};          ///< Signed tool-point height over the wall (+ = in front), same samples.
  double wall_gap_max_abs_m{0.0};       ///< Largest |gap| among them.
  TcpSpeedStats tcp_speed;              ///< Contact tools only; all zero for a non-contact run.
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
