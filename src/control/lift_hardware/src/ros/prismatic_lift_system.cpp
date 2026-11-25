#include "lift_hardware/ros/prismatic_lift_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "lift_hardware/core/transport_factory.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace lift_hardware
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

CallbackReturn PrismaticLiftSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (info_.joints.size() != 1) {
    RCLCPP_FATAL(logger(), "Expected exactly 1 <joint>, got %zu.", info_.joints.size());
    return CallbackReturn::ERROR;
  }
  const auto & joint = info_.joints[0];
  joint_name_ = joint.name;

  const auto has_command = [&joint](const std::string & name) {
      return std::any_of(
        joint.command_interfaces.begin(), joint.command_interfaces.end(),
        [&name](const auto & c) {return c.name == name;});
    };
  if (!has_command("position")) {
    RCLCPP_FATAL(logger(), "Joint '%s' needs a position command interface.", joint_name_.c_str());
    return CallbackReturn::ERROR;
  }
  has_velocity_command_ = has_command("velocity");

  std::string error;
  config_ = parse_lift_config(info_.hardware_parameters, &error);
  if (!error.empty()) {
    RCLCPP_FATAL(logger(), "Bad hardware parameter: %s", error.c_str());
    return CallbackReturn::ERROR;
  }
  const auto transport = info_.hardware_parameters.find("transport");
  transport_name_ = transport == info_.hardware_parameters.end() ? "mock" : transport->second;
  // Fail now, not at configure, on a transport that cannot be built.
  if (make_transport(transport_name_, info_.hardware_parameters, &error) == nullptr) {
    RCLCPP_FATAL(logger(), "Transport: %s", error.c_str());
    return CallbackReturn::ERROR;
  }

  position_command_ = kNaN;
  velocity_command_ = kNaN;
  RCLCPP_INFO(
    logger(), "joint='%s' transport='%s' max_velocity=%.3f m/s max_acceleration=%.3f m/s^2",
    joint_name_.c_str(), transport_name_.c_str(), config_.max_velocity_mps,
    config_.max_acceleration_mps2);
  return CallbackReturn::SUCCESS;
}

CallbackReturn PrismaticLiftSystem::on_configure(const rclcpp_lifecycle::State &)
{
  std::string error;
  auto transport = make_transport(transport_name_, info_.hardware_parameters, &error);
  if (transport == nullptr) {
    RCLCPP_ERROR(logger(), "Transport: %s", error.c_str());
    return CallbackReturn::ERROR;
  }
  supervisor_ = std::make_unique<LiftSupervisor>(std::move(transport), config_);
  position_state_ = 0.0;
  velocity_state_ = 0.0;
  return CallbackReturn::SUCCESS;
}

CallbackReturn PrismaticLiftSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  supervisor_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PrismaticLiftSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (supervisor_ == nullptr) {
    return CallbackReturn::ERROR;
  }
  position_command_ = kNaN;  // nothing is commanded until a controller writes
  velocity_command_ = kNaN;
  reset_command_ = 0.0;
  reset_was_high_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn PrismaticLiftSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (supervisor_ != nullptr) {
    supervisor_->shutdown();  // stops the lift and closes the link
  }
  return CallbackReturn::SUCCESS;
}

bool PrismaticLiftSystem::gpio_declares_state(const std::string & name) const
{
  if (info_.gpios.empty()) {
    return false;
  }
  const auto & states = info_.gpios[0].state_interfaces;
  return std::any_of(
    states.begin(), states.end(), [&name](const auto & s) {return s.name == name;});
}

std::vector<hardware_interface::StateInterface> PrismaticLiftSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  out.emplace_back(joint_name_, "position", &position_state_);
  out.emplace_back(joint_name_, "velocity", &velocity_state_);
  if (!info_.gpios.empty()) {
    const std::string & gpio = info_.gpios[0].name;
    if (gpio_declares_state("homed")) {
      out.emplace_back(gpio, "homed", &homed_state_);
    }
    if (gpio_declares_state("phase")) {
      out.emplace_back(gpio, "phase", &phase_state_);
    }
    if (gpio_declares_state("fault_code")) {
      out.emplace_back(gpio, "fault_code", &fault_state_);
    }
    if (gpio_declares_state("stop_confirmed")) {
      out.emplace_back(gpio, "stop_confirmed", &stop_confirmed_state_);
    }
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> PrismaticLiftSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.emplace_back(joint_name_, "position", &position_command_);
  if (has_velocity_command_) {
    out.emplace_back(joint_name_, "velocity", &velocity_command_);
  }
  if (!info_.gpios.empty()) {
    const auto & commands = info_.gpios[0].command_interfaces;
    if (std::any_of(commands.begin(), commands.end(), [](const auto & c) {
          return c.name == "reset_fault";
        }))
    {
      out.emplace_back(info_.gpios[0].name, "reset_fault", &reset_command_);
    }
  }
  return out;
}

void PrismaticLiftSystem::publish_status(const Snapshot & s)
{
  position_state_ = s.height_m;
  velocity_state_ = s.velocity_mps;
  homed_state_ = s.homed ? 1.0 : 0.0;
  phase_state_ = static_cast<double>(static_cast<int>(s.phase));
  fault_state_ = static_cast<double>(static_cast<int>(s.fault));
  stop_confirmed_state_ = s.stop_confirmed ? 1.0 : 0.0;
}

hardware_interface::return_type PrismaticLiftSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (supervisor_ == nullptr) {
    return hardware_interface::return_type::OK;
  }
  supervisor_->poll(time.seconds());
  const Snapshot & s = supervisor_->snapshot();
  publish_status(s);

  if (s.phase != last_logged_phase_ || s.fault != last_logged_fault_) {
    RCLCPP_INFO(
      logger(), "lift '%s': %s (fault: %s)", joint_name_.c_str(), to_string(s.phase),
      to_string(s.fault));
    last_logged_phase_ = s.phase;
    last_logged_fault_ = s.fault;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PrismaticLiftSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (supervisor_ == nullptr) {
    return hardware_interface::return_type::OK;
  }
  const bool reset_high = reset_command_ > 0.5;
  if (reset_high && !reset_was_high_) {
    supervisor_->reset_fault(time.seconds());
  }
  reset_was_high_ = reset_high;
  reset_command_ = 0.0;

  supervisor_->command(time.seconds(), position_command_);  // NaN is ignored by the supervisor
  return hardware_interface::return_type::OK;
}

}  // namespace lift_hardware

PLUGINLIB_EXPORT_CLASS(lift_hardware::PrismaticLiftSystem, hardware_interface::SystemInterface)
