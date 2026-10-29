#include "ft_bridge_hardware/ft_bridge_hardware_interface.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace ft_bridge_hardware
{

FtBridgeHardwareInterface::~FtBridgeHardwareInterface()
{
  // Unconditional safety net -- see lift_hardware's identical rationale: std::thread's own
  // destructor calls std::terminate() if it is destroyed while still joinable, regardless of
  // which lifecycle path (or none, on an abnormal exit) preceded destruction.
  stopExecutor();
}

CallbackReturn FtBridgeHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (info_.sensors.size() != 1) {
    RCLCPP_FATAL(
      rclcpp::get_logger("FtBridgeHardwareInterface"),
      "Expected exactly 1 <sensor> block, got %zu.", info_.sensors.size());
    return CallbackReturn::ERROR;
  }
  sensor_name_ = info_.sensors[0].name;

  config_ = parseFtBridgeConfig(info_.hardware_parameters);
  RCLCPP_INFO(
    rclcpp::get_logger("FtBridgeHardwareInterface"),
    "sensor='%s' injection_mode=%s source='%s' injection='%s' setpoint='%s'",
    sensor_name_.c_str(), config_.injection_mode ? "true" : "false", config_.source_topic.c_str(),
    config_.injection_topic.c_str(), config_.setpoint_topic.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn FtBridgeHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Tear down any node/executor/thread left over from a prior on_configure() call before
  // starting new ones -- see lift_hardware's identical re-entry rationale.
  stopExecutor();

  node_ = rclcpp::Node::make_shared("ft_bridge_hardware_" + sensor_name_);

  const std::string & source =
    config_.injection_mode ? config_.injection_topic : config_.source_topic;
  wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
    source, rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {onWrenchMessage(msg);});

  setpoint_sub_ = node_->create_subscription<geometry_msgs::msg::Wrench>(
    config_.setpoint_topic, rclcpp::QoS(1).transient_local(),
    [this](const geometry_msgs::msg::Wrench::SharedPtr msg) {onSetpointMessage(msg);});

  return CallbackReturn::SUCCESS;
}

CallbackReturn FtBridgeHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stopExecutor();
  return CallbackReturn::SUCCESS;
}

CallbackReturn FtBridgeHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stopExecutor();
  return CallbackReturn::SUCCESS;
}

CallbackReturn FtBridgeHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() {executor_->spin();});

  read(rclcpp::Time(0, 0, RCL_STEADY_TIME), rclcpp::Duration(0, 0));
  return CallbackReturn::SUCCESS;
}

CallbackReturn FtBridgeHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stopExecutor();
  return CallbackReturn::SUCCESS;
}

void FtBridgeHardwareInterface::stopExecutor()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  executor_.reset();
}

void FtBridgeHardwareInterface::onWrenchMessage(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  raw_wrench_ = {
    msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
    msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z};
  has_received_wrench_ = true;
}

void FtBridgeHardwareInterface::onSetpointMessage(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  setpoint_wrench_ = {
    msg->force.x, msg->force.y, msg->force.z, msg->torque.x, msg->torque.y, msg->torque.z};
}

hardware_interface::return_type FtBridgeHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  WrenchArray raw;
  WrenchArray setpoint;
  {
    std::lock_guard<std::mutex> lock(wrench_mutex_);
    raw = raw_wrench_;
    setpoint = setpoint_wrench_;
  }
  // No message received yet (raw stays all-zero): the biased result is simply -setpoint, which
  // is the correct "hungry for the full target force" reading for a fresh admittance engage --
  // not an error condition worth an ERROR return (see this codebase's own lesson on ERROR
  // returns freezing ros2_control components: a bridge with no data yet should read as zero
  // force, not stop being read from at all).
  const auto biased = applyWrenchBias(raw, setpoint);
  force_x_ = biased[0];
  force_y_ = biased[1];
  force_z_ = biased[2];
  torque_x_ = biased[3];
  torque_y_ = biased[4];
  torque_z_ = biased[5];
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> FtBridgeHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(sensor_name_, "force.x", &force_x_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(sensor_name_, "force.y", &force_y_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(sensor_name_, "force.z", &force_z_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(sensor_name_, "torque.x", &torque_x_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(sensor_name_, "torque.y", &torque_y_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(sensor_name_, "torque.z", &torque_z_));
  return state_interfaces;
}

}  // namespace ft_bridge_hardware

PLUGINLIB_EXPORT_CLASS(
  ft_bridge_hardware::FtBridgeHardwareInterface, hardware_interface::SensorInterface)
