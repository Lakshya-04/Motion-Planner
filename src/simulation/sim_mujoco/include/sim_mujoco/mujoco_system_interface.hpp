/**
 * @file mujoco_system_interface.hpp
 * @brief ros2_control SystemInterface that steps a MuJoCo model (hardware:=mujoco).
 * @details Same joint interfaces as the jtac_mock hardware: position and velocity command; position,
 *          velocity and effort state. The tool force/torque is exported as the sensor named in the
 *          <ros2_control> block (`ft_sensor`, state interfaces force.x .. torque.z), which is the
 *          shape ft_bridge_hardware and the JTAC / admittance controllers read.
 *
 *          Hardware parameters (xacro: <param name="..."> inside <hardware>):
 *            mjcf        path of the MJCF (default: environment variable SIM_MUJOCO_MJCF)
 *            ft_site     MJCF site carrying the force/torque sensors (default "ft_site")
 *            ft_tare     true (default): subtract the wrench read at activation (tool weight)
 *            gravity_comp is a property of the MJCF (bodies carry gravcomp), not of this plugin.
 *
 *          Threading: read() and write() run on the controller manager thread; the model is stepped
 *          in write() by the controller period, so simulated time follows the update rate.
 */

#pragma once

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

#include "sim_mujoco/mujoco_sim.hpp"

namespace sim_mujoco
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MujocoSystemInterface : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::unique_ptr<MujocoSim> sim_;
  std::string sensor_name_;
  bool ft_tare_{true};

  std::vector<double> initial_position_;
  std::vector<double> state_position_;
  std::vector<double> state_velocity_;
  std::vector<double> state_effort_;
  std::vector<double> command_position_;
  std::vector<double> command_velocity_;
  std::vector<bool> has_effort_state_;
  std::vector<bool> has_velocity_command_;
  std::array<double, 6> ft_{};
};

}  // namespace sim_mujoco
