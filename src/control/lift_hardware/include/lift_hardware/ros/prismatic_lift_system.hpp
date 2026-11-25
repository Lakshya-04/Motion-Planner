/**
 * @file prismatic_lift_system.hpp
 * @brief ros2_control system component for one prismatic lift joint, over a LiftTransport.
 * @details A thin shell around the ROS-free LiftSupervisor (core/lift_supervisor.hpp): `read()`
 *          polls the supervisor and exports the measured height and velocity, `write()` hands the
 *          position command over. All motion, limit, fault and reconnect logic lives in the core.
 *
 *          Interfaces (names come from the ros2_control description, nothing is fixed here):
 *          - the one `<joint>`: state `position` and `velocity`; command `position` (driven) and,
 *            when declared, `velocity` (exported so a description that lists it loads; ignored).
 *          - an optional `<gpio>`: any of the state interfaces `homed`, `phase` (0 disconnected,
 *            1 homing, 2 ready, 3 held), `fault_code` (the `Fault` enum value) and
 *            `stop_confirmed`, and the command interface `reset_fault` (a rise above 0.5 asks the
 *            supervisor to leave a hold and home again; it is cleared once handled).
 *
 *          `read()` and `write()` always return OK. Returning ERROR makes ros2_control deactivate
 *          the component, after which nothing would call `read()` again, so a lift that lost its
 *          link could never reconnect. Safety does not rest on that return value: a commanded
 *          height reaches the device only while the supervisor is Ready.
 *
 *          Hardware parameters: `transport` (`mock` default, or `device`), the LiftConfig keys,
 *          and the mock's `mock_stroke_m` and `mock_initial_height_m`; see docs/lift.md.
 */

#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "lift_hardware/core/lift_config.hpp"
#include "lift_hardware/core/lift_supervisor.hpp"

namespace lift_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class PrismaticLiftSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static rclcpp::Logger logger() { return rclcpp::get_logger("PrismaticLiftSystem"); }
  void publish_status(const Snapshot & snapshot);
  bool gpio_declares_state(const std::string & name) const;

  std::string joint_name_;
  std::string transport_name_;
  LiftConfig config_;
  std::unique_ptr<LiftSupervisor> supervisor_;

  // Values behind the exported interfaces (realtime thread only).
  double position_state_{0.0};
  double velocity_state_{0.0};
  double position_command_{std::numeric_limits<double>::quiet_NaN()};
  double velocity_command_{std::numeric_limits<double>::quiet_NaN()};
  double homed_state_{0.0};
  double phase_state_{0.0};
  double fault_state_{0.0};
  double stop_confirmed_state_{1.0};
  double reset_command_{0.0};
  bool reset_was_high_{false};
  bool has_velocity_command_{false};

  Phase last_logged_phase_{Phase::kDisconnected};
  Fault last_logged_fault_{Fault::kNone};
};

}  // namespace lift_hardware
