/**
 * @file contact_connector.hpp
 * @brief Whether the step from one strip to the next can be one sliding move in contact, and the log of how often
 *        it was. ROS-free.
 * @details Every column normally runs dock -> approach -> contact stroke -> retract, so between two adjacent strips of
 *          one stand the tool leaves the wall and comes back. With `contact_connectors` the stroke's end is joined to
 *          the next stroke's start by one short Cartesian move along the wall with the force loop still engaged
 *          (serpentine: finish a stroke at the top, step over, come back down). This header holds the rule for when
 *          that is allowed. It checks only geometry of the two planned poses (same stand, a step of at most
 *          `max_step_m`, both on one plane parallel to the wall, similar tool orientation); a clear path and a
 *          continuous IK chain are checked by the Cartesian planner and the column IK gate when the move is made, and
 *          any failure there falls back to the normal retract and approach.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <cmath>
#include <string>

#include "core_types/pose.hpp"
#include "manipulation_manager/core/wall_direction.hpp"

namespace manipulation_manager
{

struct ConnectorConfig
{
  double max_step_m{0.15};     ///< longest stroke-end to next-stroke-start distance (about 1.5 x the swath spacing)
  double plane_tol_m{0.02};    ///< largest offset of the step along the wall normal (same wall plane)
  double min_quat_dot{0.98};   ///< smallest |q1 . q2| of the two tool orientations (about 23 degrees)
};

struct ConnectorDecision
{
  bool eligible{false};
  double step_m{0.0};          ///< distance between the stroke end and the next stroke start
  double plane_offset_m{0.0};  ///< component of that step along the wall normal
  std::string reason;          ///< why not, or "ok"
};

/// `same_stand`: the next column belongs to the stand being run; `stroke_complete`: the stroke reached its last pose;
/// `arm_ok`: the arm is not stuck. The poses are the contact-depth waypoints (stroke end, next stroke start).
inline ConnectorDecision decideContactConnector(
  bool enabled, bool same_stand, bool stroke_complete, bool arm_ok, const core_types::Pose & stroke_end,
  const core_types::Pose & next_start, const WallDirection & wall, const ConnectorConfig & cfg = {})
{
  ConnectorDecision d;
  if (!enabled) {d.reason = "off"; return d;}
  if (!same_stand) {d.reason = "next column is on another stand"; return d;}
  if (!stroke_complete) {d.reason = "stroke did not reach its end"; return d;}
  if (!arm_ok) {d.reason = "arm stuck"; return d;}
  const double dx = next_start.x - stroke_end.x, dy = next_start.y - stroke_end.y, dz = next_start.z - stroke_end.z;
  d.step_m = std::sqrt(dx * dx + dy * dy + dz * dz);
  d.plane_offset_m = dx * wall.d[0] + dy * wall.d[1] + dz * wall.d[2];
  if (!std::isfinite(d.step_m) || !std::isfinite(d.plane_offset_m)) {d.reason = "non-finite pose"; return d;}
  if (d.step_m > cfg.max_step_m) {d.reason = "strips not adjacent"; return d;}
  if (std::fabs(d.plane_offset_m) > cfg.plane_tol_m) {d.reason = "not the same wall plane"; return d;}
  const double dot = std::fabs(
    stroke_end.qx * next_start.qx + stroke_end.qy * next_start.qy + stroke_end.qz * next_start.qz +
    stroke_end.qw * next_start.qw);
  if (!(dot >= cfg.min_quat_dot)) {d.reason = "tool orientation differs"; return d;}
  d.eligible = true;
  d.reason = "ok";
  return d;
}

/// The one waypoint of the connector: the next stroke's start, with the stroke end's offset along the wall normal
/// removed (the tool slides in the plane it is already pressed in) and the next stroke's orientation.
inline core_types::Pose connectorTarget(
  const core_types::Pose & stroke_end, const core_types::Pose & next_start, const WallDirection & wall)
{
  core_types::Pose p = next_start;
  const double off = (next_start.x - stroke_end.x) * wall.d[0] + (next_start.y - stroke_end.y) * wall.d[1] +
    (next_start.z - stroke_end.z) * wall.d[2];
  return wall.shifted(p, -off);
}

/// Count of connectors made and refused, for the run result.
class ConnectorLog
{
public:
  void used() {++used_;}
  void fellBack(const std::string & why)
  {
    ++fell_back_;
    last_reason_ = why;
  }
  int usedCount() const {return used_;}
  int fellBackCount() const {return fell_back_;}
  std::string summary() const
  {
    if (used_ == 0 && fell_back_ == 0) {return "";}
    std::string s = "contact connectors: " + std::to_string(used_) + " used, " + std::to_string(fell_back_) +
      " fell back to retract/approach";
    if (fell_back_ > 0) {s += " (last: " + last_reason_ + ")";}
    return s;
  }

private:
  int used_{0};
  int fell_back_{0};
  std::string last_reason_;
};

}  // namespace manipulation_manager
