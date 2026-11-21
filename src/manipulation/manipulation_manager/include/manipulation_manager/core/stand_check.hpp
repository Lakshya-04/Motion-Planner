/**
 * @file stand_check.hpp
 * @brief Gate between "the base arrived at the stand" and "the arm moves": go, re-dock or abort.
 * @details The navigator reports how far from the planned stand the base stopped (position error, yaw
 *          error). This compares that with a tolerance derived from the reachability margin of the stand
 *          (the room the planner left around it: the stand's base-offset margin in metres) and decides
 *          go / re-dock / abort with a coded reason. After `max_redocks` re-docks the next miss aborts and
 *          the stand is to be marked failed. ROS-free and stateless: the caller counts the re-docks.
 *          The tolerance is clamp(margin_scale * reach_margin_m, min_tol_m, max_tol_m); with no margin
 *          (negative) it is `fallback_tol_m`. The yaw tolerance is a fixed config value (the margin is
 *          a length). Defaults are unmeasured estimates.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <string>

namespace manipulation_manager
{

struct StandCheckConfig
{
  double min_tol_m{0.02};        ///< Never demand more than this (the navigator's own noise floor).
  double max_tol_m{0.15};        ///< Never accept more than this (the old base_check tolerance).
  double margin_scale{0.5};      ///< Share of the reach margin the base may use up.
  double fallback_tol_m{0.15};   ///< Tolerance when the reach margin is unknown (negative).
  double yaw_tol_rad{0.1};
  int max_redocks{1};            ///< Re-docks allowed before a miss aborts (0 = abort on the first miss).
};

/// What the navigator reported on arrival.
struct StandAchieved
{
  double position_error_m{0.0};
  double yaw_error_rad{0.0};
};

enum class StandDecision
{
  kGo,       ///< Within tolerance: the arm may move.
  kRedock,   ///< Outside tolerance, re-docks left: drive to the stand again.
  kAbort,    ///< Give up on this stand (stand_failed is set).
};

enum class StandReason
{
  kWithinTolerance,
  kPositionOutOfTolerance,
  kYawOutOfTolerance,
  kPositionAndYawOutOfTolerance,
  kInvalidMeasurement,   ///< NaN, infinite or negative position error.
  kRedocksExhausted,     ///< Out of tolerance and no re-dock left.
};

const char * toString(StandReason r);

struct StandCheckResult
{
  StandDecision decision{StandDecision::kAbort};
  StandReason reason{StandReason::kInvalidMeasurement};
  double tol_m{0.0};
  double tol_yaw_rad{0.0};
  bool stand_failed{false};   ///< True on abort: mark the stand failed, run none of its columns.
  std::string message;
};

class StandCheck
{
public:
  explicit StandCheck(const StandCheckConfig & cfg = {});

  /// Position tolerance for a stand whose reach margin is `reach_margin_m` (negative = unknown).
  double toleranceM(double reach_margin_m) const;

  /// `redocks_done` = re-docks already made for this stand.
  StandCheckResult evaluate(
    const StandAchieved & achieved, double reach_margin_m, int redocks_done) const;

  const StandCheckConfig & config() const {return cfg_;}

private:
  StandCheckConfig cfg_;
};

}  // namespace manipulation_manager
