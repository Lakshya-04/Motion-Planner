#include "ft_bridge_hardware/ft_bridge_sim_system.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace ft_bridge_hardware
{

CallbackReturn FtBridgeSimSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return bridge_.on_init(info);
}

CallbackReturn FtBridgeSimSystem::on_configure(const rclcpp_lifecycle::State & s)
{
  return bridge_.on_configure(s);
}

CallbackReturn FtBridgeSimSystem::on_cleanup(const rclcpp_lifecycle::State & s)
{
  return bridge_.on_cleanup(s);
}

CallbackReturn FtBridgeSimSystem::on_shutdown(const rclcpp_lifecycle::State & s)
{
  return bridge_.on_shutdown(s);
}

CallbackReturn FtBridgeSimSystem::on_activate(const rclcpp_lifecycle::State & s)
{
  return bridge_.on_activate(s);
}

CallbackReturn FtBridgeSimSystem::on_deactivate(const rclcpp_lifecycle::State & s)
{
  return bridge_.on_deactivate(s);
}

std::vector<hardware_interface::StateInterface> FtBridgeSimSystem::export_state_interfaces()
{
  return bridge_.export_state_interfaces();
}

std::vector<hardware_interface::CommandInterface> FtBridgeSimSystem::export_command_interfaces()
{
  return {};
}

hardware_interface::return_type FtBridgeSimSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  return bridge_.read(time, period);
}

hardware_interface::return_type FtBridgeSimSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

bool FtBridgeSimSystem::initSim(
  rclcpp::Node::SharedPtr &, std::map<std::string, sim::Entity> &,
  const hardware_interface::HardwareInfo &, sim::EntityComponentManager &, unsigned int)
{
  return true;  // nothing to attach to in the simulator
}

}  // namespace ft_bridge_hardware

PLUGINLIB_EXPORT_CLASS(
  ft_bridge_hardware::FtBridgeSimSystem, gz_ros2_control::GazeboSimSystemInterface)
