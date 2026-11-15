/**
 * @file compliance_port.hpp
 * @brief Backend-agnostic seam for low-level contact compliance, independent of ROS.
 * @details Lives in core_types (unlike ExecutionPort, which lives in its sole consumer
 *          manipulation_manager) because its implementations live OUTSIDE that package --
 *          a ros2_control admittance-chain adapter and a UFACTORY ft_mode adapter both belong in
 *          ros_adapters/platform, neither of which may depend on manipulation_manager.
 *          The port abstracts over the candidate force-control backends (see docs/ft_sensing.md
 *          and docs/architecture.md).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "core_types/wrench.hpp"

namespace core_types
{

/**
 * @struct ComplianceConfig
 * @brief Everything a backend needs to enter compliant mode for one contact task.
 * @details Populated wholesale from the active ToolProfile's ToolControl block --
 *          backends never read a ToolProfile themselves, so swapping the backend can never
 *          re-interpret a profile field differently.
 */
struct ComplianceConfig
{
  std::string control_frame{"link_tcp"};  ///< Frame the selected axes and wrench are expressed in.
  std::array<bool, 6> selected_axes{};    ///< x,y,z,rx,ry,rz -- ToolAdmittance::selected_axes.
  std::array<double, 3> mass{};           ///< ToolAdmittance::mass.
  std::array<double, 3> damping{};        ///< ToolAdmittance::damping.
  std::array<double, 3> stiffness{};      ///< ToolAdmittance::stiffness.
  double target_force_n{0.0};             ///< Commanded normal force at engage time.
  double max_offset_m{0.0};               ///< ToolAdmittance::max_offset_m -- backend must clamp.
  double lpf_cutoff_hz{0.0};              ///< ToolFtHygiene::lpf_cutoff_hz.
  double deadband_n{0.0};                 ///< ToolFtHygiene::deadband_n.
  double clamp_n{0.0};                    ///< ToolFtHygiene::clamp_n.
};

/**
 * @struct ComplianceStatus
 * @brief One non-blocking sample of the compliance backend's state.
 * @details stamp_ns is the backend's own monotonic sample time, NOT the read time -- the force
 *          FSM derives every timeout and debounce from it and never reads a clock itself, which
 *          is what makes the FSM unit-testable with fabricated force traces and virtual time.
 */
struct ComplianceStatus
{
  bool engaged{false};             ///< True while the compliant stack is the active controller.
  bool wrench_valid{false};        ///< False if stale/NaN/untared. A false NEVER advances the FSM.
  Wrench wrench;                   ///< Filtered, in control_frame -- see Wrench's sign convention.
  double normal_offset_m{0.0};     ///< Compliance-produced displacement along the normal.
  bool offset_saturated{false};    ///< True at max_offset_m -- freezes any trim integrator.
  uint64_t stamp_ns{0};            ///< Monotonic sample time of this wrench.
};

/**
 * @class CompliancePort
 * @brief Backend-agnostic seam for low-level contact compliance. Carries NO policy.
 * @details Deliberately state+sensor only: no "move until force", no "wait until settled", no
 *          notion of a segment. Every decision that reads a ToolProfile threshold (converged?
 *          overloaded? timed out? unloaded?) lives in the force-stage FSM, so swapping the
 *          backend cannot silently change when the robot decides it is in contact.
 */
class CompliancePort
{
public:
  virtual ~CompliancePort() = default;

  /// Zero the F/T sensor. Caller guarantees the arm is at rest (ToolFtHygiene::tare_per_goal).
  virtual bool tare(std::string & error_message) = 0;

  /// Rigid -> compliant. This is the controller switch; it is only ever called out of contact.
  virtual bool engage(const ComplianceConfig & config, std::string & error_message) = 0;

  /// Change the commanded normal force WITHOUT a controller switch (used to unload to 0 N
  /// during RETRACT). Returning false here is a hard abort, never a silent no-op.
  virtual bool setTargetForce(double target_force_n, std::string & error_message) = 0;

  /// Compliant -> rigid. Caller guarantees |normalForceN| is below the unload threshold first.
  virtual bool disengage(std::string & error_message) = 0;

  /// Non-blocking sample. Must be cheap; called at the sequencer's tick rate (50-100 Hz).
  virtual ComplianceStatus status() = 0;

  /// Immediate, in-place halt holding the current pose -- the supervisory lever for a guard
  /// trip. NOT the safety function of last resort: the backend's own hardware force/collision
  /// limit is.
  virtual bool protectiveStop(std::string & error_message) = 0;
};

}  // namespace core_types
