/**
 * @file ft_bridge_sim_system.hpp
 * @brief FtBridgeHardwareInterface under gz_ros2_control's plugin base class.
 * @details gz_ros2_control's resource manager loads every <ros2_control> block of the robot
 *          description through its own plugin loader (base class GazeboSimSystemInterface), so a
 *          plain SensorInterface such as FtBridgeHardwareInterface is refused ("class ... with base
 *          class type gz_ros2_control::GazeboSimSystemInterface does not exist") and its sensor
 *          interfaces never appear. This class is that base class with nothing simulated: it owns
 *          an FtBridgeHardwareInterface and forwards the lifecycle, read() and the state
 *          interfaces to it, so the bridge (topic in, setpoint bias, `~/target_wrench`) behaves
 *          exactly as on a real robot. Declare it as a `type="system"` block with one <sensor>.
 *          Built only when gz_ros2_control is installed.
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include <gz_ros2_control/gz_system_interface.hpp>

#include "ft_bridge_hardware/ft_bridge_hardware_interface.hpp"

namespace ft_bridge_hardware
{

class FtBridgeSimSystem : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  bool initSim(
    rclcpp::Node::SharedPtr & model_nh, std::map<std::string, sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info, sim::EntityComponentManager & ecm,
    unsigned int update_rate) override;

private:
  FtBridgeHardwareInterface bridge_;
};

}  // namespace ft_bridge_hardware
