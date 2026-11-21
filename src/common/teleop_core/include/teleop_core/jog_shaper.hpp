/**
 * @file jog_shaper.hpp
 * @brief Turns a normalised 6D operator input (spacemouse, joystick) into a Cartesian twist command.
 * @details Pipeline per axis: clamp to [-1, 1], deadband (rescaled so the output still reaches 1),
 *          expo curve `(1 - e) x + e x^3`, per-axis enable mask (a masked axis outputs zero, for example the
 *          contact-normal axis during force-limited contact), scale to the maximum linear or angular speed.
 *          The result is rotated from the selected frame (tool, base or world) into the base frame, then
 *          slew-limited as a vector (linear and angular separately, so direction is kept). A watchdog forces
 *          an immediate zero twist when no valid input arrived for `watchdog_s`.
 *          Input is [x, y, z, rx, ry, rz]; the twist is [vx, vy, vz, wx, wy, wz]. NaN/inf axes count as zero
 *          and the sample is not valid (it does not refresh the watchdog). No clock: the caller passes time.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <array>

#include <Eigen/Dense>

namespace teleop_core
{

using Vector6d = Eigen::Matrix<double, 6, 1>;

enum class JogFrame {kBase = 0, kTool, kWorld};

struct JogShaperConfig
{
  std::array<double, 6> deadband{0.05, 0.05, 0.05, 0.05, 0.05, 0.05};  ///< In [0, 1).
  double expo{0.3};                  ///< 0 linear, 1 cubic.
  double max_linear_mps{0.10};
  double max_angular_rps{0.30};
  std::array<bool, 6> enabled{true, true, true, true, true, true};
  double max_linear_accel{0.5};      ///< Slew limit [m/s^2]; <= 0 disables.
  double max_angular_accel{1.5};     ///< Slew limit [rad/s^2]; <= 0 disables.
  double watchdog_s{0.25};
  JogFrame frame{JogFrame::kBase};
};

struct FrameRotations
{
  Eigen::Matrix3d tool_in_base{Eigen::Matrix3d::Identity()};   ///< Rotation tool -> base.
  Eigen::Matrix3d world_in_base{Eigen::Matrix3d::Identity()};  ///< Rotation world -> base.
};

struct JogOutput
{
  Vector6d twist{Vector6d::Zero()};  ///< In the base frame.
  bool watchdog_tripped{false};
  bool saturated{false};             ///< An input axis was clamped.
  bool invalid_input{false};         ///< The last input had a non-finite axis.
};

class JogShaper
{
public:
  explicit JogShaper(const JogShaperConfig & config = JogShaperConfig{});

  void setConfig(const JogShaperConfig & config);
  void setFrame(JogFrame frame) {cfg_.frame = frame;}
  void setEnabled(const std::array<bool, 6> & mask) {cfg_.enabled = mask;}

  /// Store the latest operator input at time `now_s`.
  void setInput(const Vector6d & input, double now_s);
  /// Compute the twist for this tick. `dt` is the time since the previous call.
  JogOutput update(double now_s, double dt, const FrameRotations & rot = FrameRotations{});
  void reset();

private:
  static double shapeAxis(double x, double deadband, double expo);

  JogShaperConfig cfg_;
  Vector6d input_{Vector6d::Zero()};
  bool have_input_{false};
  bool last_invalid_{false};
  double last_input_s_{0.0};
  Vector6d prev_{Vector6d::Zero()};
};

}  // namespace teleop_core
