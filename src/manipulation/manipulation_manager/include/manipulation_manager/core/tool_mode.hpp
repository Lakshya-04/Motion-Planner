/**
 * @file tool_mode.hpp
 * @brief What a wall column does for a contact tool and for a non-contact (spray) tool.
 * @details Pure logic, ROS-free. A contact tool docks, presses (admittance force mode, target force),
 *          strokes with the reference `contact_y_overshoot` into the wall, unloads and retracts. A
 *          non-contact tool docks, strokes at the stand-off the coverage plan already carries
 *          (`coverage_planner` lifts the poses `standoff_m` in front of the wall for a non-contact tool),
 *          never engages force mode, never presses, never accepts wall contact, and has its valve open
 *          for the stroke only.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

namespace manipulation_manager
{

/// The steps of one column for a tool kind.
struct ColumnSteps
{
  bool engage_force{true};       ///< Admittance on, force target published.
  bool press{true};              ///< Press at the first stroke pose and settle before the stroke.
  bool allow_wall_contact{true}; ///< The stroke segment may touch the wall (penetration allowance).
  bool valve{false};             ///< Open the valve for the stroke.
  double stroke_shift_m{0.0};    ///< Stroke poses are moved this far towards the wall (negative: away).
};

/**
 * @brief The steps for a tool.
 * @param[in] contact true for a contact tool.
 * @param[in] contact_overshoot_m Contact tool: how far the stroke reference goes into the wall.
 * @param[in] extra_standoff_m Non-contact tool: additional stand-off on top of the plan's, >= 0
 *            (a negative value is treated as 0: a spray tool never moves towards the wall).
 */
inline ColumnSteps columnSteps(bool contact, double contact_overshoot_m, double extra_standoff_m)
{
  ColumnSteps s;
  if (contact) {
    s.stroke_shift_m = contact_overshoot_m;
    return s;
  }
  s.engage_force = false;
  s.press = false;
  s.allow_wall_contact = false;
  s.valve = true;
  s.stroke_shift_m = -(extra_standoff_m > 0.0 ? extra_standoff_m : 0.0);
  return s;
}

/// Edge filter for the valve command: `update` says whether a message has to be sent. The valve starts
/// closed and `force` always sends (to make the closed state explicit at start-up and after a stroke).
class ValveGate
{
public:
  bool on() const {return on_;}

  /// @return true when `want` differs from the current state (the caller publishes it).
  bool update(bool want)
  {
    if (want == on_) {
      return false;
    }
    on_ = want;
    return true;
  }

private:
  bool on_{false};
};

}  // namespace manipulation_manager
