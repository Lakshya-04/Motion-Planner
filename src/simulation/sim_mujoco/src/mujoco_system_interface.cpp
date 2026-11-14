#include "sim_mujoco/mujoco_system_interface.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace sim_mujoco
{

namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("MujocoSystemInterface");

std::string param(const hardware_interface::HardwareInfo & info, const std::string & key,
  const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(key);
  return (it == info.hardware_parameters.end() || it->second.empty()) ? fallback : it->second;
}
}  // namespace

CallbackReturn MujocoSystemInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const char * env = std::getenv("SIM_MUJOCO_MJCF");
  const std::string mjcf = param(info_, "mjcf", env ? env : "");
  if (mjcf.empty()) {
    RCLCPP_FATAL(kLogger, "No MJCF: set the 'mjcf' hardware parameter or SIM_MUJOCO_MJCF.");
    return CallbackReturn::ERROR;
  }
  ft_tare_ = param(info_, "ft_tare", "false") == "true";
  ft_frame_id_ = param(info_, "ft_frame_id", "tool_mount_link");
  tcp_frame_id_ = param(info_, "tcp_frame_id", "tcp_link");

  if (info_.sensors.size() > 1) {
    RCLCPP_FATAL(kLogger, "At most one <sensor> (the tool F/T) is supported.");
    return CallbackReturn::ERROR;
  }
  sensor_name_ = info_.sensors.empty() ? "" : info_.sensors[0].name;

  std::vector<std::string> joints;
  for (const auto & joint : info_.joints) {
    joints.push_back(joint.name);
    double initial = 0.0;
    for (const auto & state : joint.state_interfaces) {
      if (state.name == "position" && !state.initial_value.empty()) {
        initial = std::stod(state.initial_value);
      }
    }
    initial_position_.push_back(initial);
  }
  try {
    sim_ = std::make_unique<MujocoSim>(mjcf, joints, param(info_, "ft_site", "ft_sensor"),
      param(info_, "tcp_site", "tcp_site"));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(kLogger, "%s", e.what());
    return CallbackReturn::ERROR;
  }
  const size_t n = joints.size();
  state_position_.assign(n, 0.0);
  state_velocity_.assign(n, 0.0);
  state_effort_.assign(n, 0.0);
  command_position_.assign(n, std::numeric_limits<double>::quiet_NaN());
  command_velocity_.assign(n, std::numeric_limits<double>::quiet_NaN());
  node_ = rclcpp::Node::make_shared("mujoco_system_" + info_.name);
  raw_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "sim_mujoco/ft_raw", rclcpp::SensorDataQoS());
  truth_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "sim_mujoco/ft_ground_truth", rclcpp::SensorDataQoS());
  RCLCPP_INFO(
    kLogger, "MuJoCo %s: '%s', %zu joints, timestep %.4f s, F/T sensor '%s' (tare %s)",
    mj_versionString(), mjcf.c_str(), n, sim_->timestep(), sensor_name_.c_str(),
    ft_tare_ ? "on" : "off");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & name = info_.joints[i].name;
    out.emplace_back(name, hardware_interface::HW_IF_POSITION, &state_position_[i]);
    out.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &state_velocity_[i]);
    out.emplace_back(name, hardware_interface::HW_IF_EFFORT, &state_effort_[i]);
  }
  if (!sensor_name_.empty()) {
    static const char * const names[6] = {"force.x", "force.y", "force.z",
      "torque.x", "torque.y", "torque.z"};
    for (size_t k = 0; k < 6; ++k) {
      out.emplace_back(sensor_name_, names[k], &ft_[k]);
    }
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> MujocoSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & name = info_.joints[i].name;
    out.emplace_back(name, hardware_interface::HW_IF_POSITION, &command_position_[i]);
    out.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &command_velocity_[i]);
  }
  return out;
}

CallbackReturn MujocoSystemInterface::on_activate(const rclcpp_lifecycle::State &)
{
  sim_->setJointPositions(initial_position_);
  sim_->advance(0.0);
  for (size_t i = 0; i < initial_position_.size(); ++i) {
    state_position_[i] = sim_->position(i);
    state_velocity_[i] = 0.0;
    state_effort_[i] = 0.0;
    // hold the start pose until a controller commands something else
    command_position_[i] = sim_->position(i);
    command_velocity_[i] = 0.0;
  }
  if (ft_tare_) {
    sim_->tareWrench();
  } else {
    sim_->clearTare();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MujocoSystemInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MujocoSystemInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < state_position_.size(); ++i) {
    state_position_[i] = sim_->position(i);
    state_velocity_[i] = sim_->velocity(i);
    state_effort_[i] = sim_->effort(i);
  }
  ft_ = sim_->wrench();
  const auto stamp = node_->now();
  auto fill = [&stamp](const std::string & frame, const Wrench & w) {
      geometry_msgs::msg::WrenchStamped m;
      m.header.stamp = stamp;
      m.header.frame_id = frame;
      m.wrench.force.x = w[0];
      m.wrench.force.y = w[1];
      m.wrench.force.z = w[2];
      m.wrench.torque.x = w[3];
      m.wrench.torque.y = w[4];
      m.wrench.torque.z = w[5];
      return m;
    };
  raw_pub_->publish(fill(ft_frame_id_, ft_));
  truth_pub_->publish(fill(tcp_frame_id_, sim_->groundTruthWrench()));
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystemInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  for (size_t i = 0; i < command_position_.size(); ++i) {
    sim_->setCommand(i, command_position_[i], command_velocity_[i]);
  }
  sim_->advance(period.seconds());
  return hardware_interface::return_type::OK;
}

}  // namespace sim_mujoco

PLUGINLIB_EXPORT_CLASS(sim_mujoco::MujocoSystemInterface, hardware_interface::SystemInterface)
