// Standoff advisor: how far from the wall should the base dock?
//
// reachability::chooseStation searches stand positions on an RM4D map but does not say how reach and
// pose quality change with the distance to the wall, nor how much the base may be off before the work
// is lost. sweepStandoff does: for every distance in a range it places the arm base that far from the wall
// plane (at the best position along the wall), counts the targets the map reaches over the lift range,
// scores them (reachability::scorePose), asks the lift-band planner how much lift stroke is needed and,
// with a base model, whether the footprint fits. Then it picks the robust optimum: the centre of the widest
// contiguous run ("plateau") of distances that keep the reachable fraction near its maximum with good
// quality, shrunk by the navigation tolerance so a base that stops `nav_tolerance_m` early or late still
// works.
//
// ROS-free (C++17 + Eigen + the reachability core). Honest limits are in docs/standoff.md: the map's
// resolution, no obstacles, no dynamics, one stand per distance.
#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "reachability/core/base_model.hpp"
#include "reachability/core/map4d.hpp"
#include "reachability/core/pose_quality.hpp"

namespace standoff_advisor
{

using reachability::Pose;
using reachability::Vec3;

/// A rectangular patch of the wall: `origin + u * u_axis + v * v_axis` for u in [u0, u1], v in [v0, v1].
struct WallPatch
{
  Vec3 origin{Vec3::Zero()};
  Vec3 u_axis{Vec3::UnitY()};
  Vec3 v_axis{Vec3::UnitZ()};
  double u0{0.0}, u1{0.0};
  double v0{0.0}, v1{0.0};
  double resolution{0.1};   ///< Target spacing along u and v (m).
};

/// Tool poses on a patch: positions on a grid (both ends included), tool z axis along -normal (the tool
/// points into the wall), tool x axis along the patch's v direction. Empty for a non-finite or degenerate
/// input, a non-positive resolution, or more than 200000 poses.
std::vector<Pose> patchTargets(const WallPatch & patch, const Vec3 & wall_normal);

/// A patch from the wall plane alone: origin = `wall_point`, u = Z x n (horizontal, along the wall), v = n x u
/// (up the wall). Ranges are relative to `wall_point` (the point's own height is v = 0).
WallPatch patchFromPlane(
  const Vec3 & wall_point, const Vec3 & wall_normal, double u0, double u1, double v0, double v1,
  double resolution);

struct StandoffSweepRequest
{
  // Work: explicit tool poses, or a patch (used when `targets_world` is empty).
  std::vector<Pose> targets_world;
  std::optional<WallPatch> patch;
  Vec3 wall_point{Vec3::Zero()};    ///< A point on the wall plane.
  Vec3 wall_normal{-Vec3::UnitX()}; ///< Normal pointing into the free (robot) side; must be (nearly) horizontal.

  // Sweep: arm base distance from the plane.
  double standoff_min{0.30};
  double standoff_max{1.60};
  double standoff_step{0.02};

  // Lift (stroke above the lift's zero) and the arm base height at lift zero.
  double lift_min{0.0};
  double lift_max{1.5};
  double lift_step{0.05};
  double arm_mount_offset_z{0.0};

  /// Positions along the wall tried for the arm base at each distance (0 = only the middle of the targets).
  double lateral_step{0.1};

  /// With a base model the clearance applies to the footprint, otherwise to the arm base point.
  std::optional<reachability::BaseModel> base;
  double min_clearance{0.30};

  // Quality: scorePose settings, and the gates of the plateau.
  reachability::MarginConfig margin;
  double quality_min{0.50};         ///< Mean pose score (of the reachable targets) a plateau distance needs.
  double fraction_tolerance{0.05};  ///< A plateau distance is within this of the best reachable fraction.
  double nav_tolerance_m{0.05};     ///< Distance error the dock must survive (the manager accepts 0.02 .. 0.15).
};

struct StandoffRow
{
  double distance{0.0};             ///< Arm base to wall plane (m).
  double reachable_fraction{0.0};   ///< Targets reachable at some lift height, at the best position along the wall.
  size_t reachable_count{0};
  double mean_quality{0.0};         ///< Mean scorePose of the reachable targets (best lift height each), 0 if none.
  double min_quality{0.0};
  double lateral_m{0.0};            ///< Arm base position along the wall (from wall_point along u) that gave it.
  size_t lift_bands{0};             ///< Bands the planner needs for the reachable targets.
  double lift_low{0.0};             ///< Lowest and highest band height: the lift stroke range this distance uses.
  double lift_high{0.0};
  bool footprint_fits{false};       ///< The footprint (or the arm base point) keeps `min_clearance` from the wall.
  double footprint_clearance{0.0};  ///< Nearest footprint point to the wall (arm base distance without a base model).
  double base_origin_distance{0.0}; ///< base_footprint origin to the wall (the arm base distance without a base model).
  double base_yaw{0.0};             ///< Yaw that gives the clearance (0 without a base model).
  double robust_fraction{0.0};      ///< Smallest reachable fraction within +- nav_tolerance_m (filled by sweepStandoff).
  bool in_plateau{false};           ///< Part of the chosen plateau.
};

enum class SweepStatus
{
  kOk,             ///< A recommendation was made.
  kInvalidRequest, ///< See `problem`.
  kNothingReachable,  ///< No distance reaches any target.
  kNoFootprintFit,    ///< Targets are reachable but the footprint never fits the clearance.
};

const char * toString(SweepStatus s);

struct Recommendation
{
  bool found{false};
  double standoff_m{0.0};          ///< Recommended arm base to wall distance.
  double min_distance{0.0};        ///< The plateau (raw): first and last distance.
  double max_distance{0.0};
  double robust_min{0.0};          ///< The plateau shrunk by the navigation tolerance.
  double robust_max{0.0};
  bool robust{false};              ///< The shrunk plateau is not empty: a navigation error of nav_tolerance_m keeps the fraction.
  bool quality_ok{true};           ///< False when no distance met `quality_min` and the quality gate was dropped.
  bool touches_range{false};       ///< The plateau reaches the end of the swept range: extend the range.
  double nav_tolerance_m{0.0};
  double reachable_fraction{0.0};  ///< At the recommended distance.
  double robust_fraction{0.0};
  double mean_quality{0.0};
  double lift_low{0.0};
  double lift_high{0.0};
  double base_origin_distance{0.0};  ///< base_footprint origin to the wall at the recommended distance.
  double base_yaw{0.0};
  double lateral_m{0.0};
  std::string reason;
};

struct StandoffSweepResult
{
  SweepStatus status{SweepStatus::kInvalidRequest};
  std::string problem;               ///< Why the request was refused.
  size_t target_count{0};
  double max_fraction{0.0};
  std::vector<StandoffRow> rows;
  Recommendation recommended;
};

/// Empty when the request is usable: finite numbers, a horizontal normal, a positive step and range, a
/// non-empty set of targets, and a bounded amount of work. Otherwise the reason (sweepStandoff refuses the
/// request with it instead of looping or producing NaN).
std::string sweepProblem(const reachability::Map4D & map, const StandoffSweepRequest & request);

/// One distance of the sweep (the same numbers a row holds, without the plateau fields).
StandoffRow evaluateStandoff(
  const reachability::Map4D & map, const StandoffSweepRequest & request, double distance);

StandoffSweepResult sweepStandoff(const reachability::Map4D & map, const StandoffSweepRequest & request);

}  // namespace standoff_advisor
