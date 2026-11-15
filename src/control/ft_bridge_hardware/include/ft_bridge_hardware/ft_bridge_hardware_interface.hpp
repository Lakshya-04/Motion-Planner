/**
 * @file ft_bridge_hardware_interface.hpp
 * @brief ros2_control sensor hardware component exporting a wall-clock F/T reading as the six
 *        state interfaces admittance_controller (semantic_components::ForceTorqueSensor) expects.
 * @details No vendor changes: the real UFACTORY arm driver already publishes
 *          geometry_msgs/WrenchStamped on /ufactory/uf_ftsensor_ext_states -- this component
 *          subscribes to that (or, in injection_mode, a settable test topic instead) and
 *          re-exposes it as ros2_control state interfaces, which is the one piece the vendored
 *          arm hardware component doesn't provide. The
 *          injection_mode is what lets the whole admittance loop be exercised in mock/
 *          ufactory_sim with scripted forces, at zero hardware risk.
 *
 *          Threading model, matching lift_hardware's own established pattern: read() runs on
 *          controller_manager's realtime thread and must never block. A dedicated background
 *          thread spins a SingleThreadedExecutor for this component's own node (the wrench and
 *          setpoint subscriptions) -- see on_configure()/on_cleanup(). The only state written by
 *          that background thread and read by the realtime thread is raw_wrench_/
 *          setpoint_wrench_/has_received_wrench_, guarded by wrench_mutex_ (a full mutex, not
 *          atomics: this bridge's update rate is bounded by the F/T topic's own ~100-200Hz, far
 *          below anything where a short lock would matter, and a mutex avoids the
 *          std::atomic<double>-array default-initialization subtlety a lock-free design would
 *          otherwise have to get right).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "ft_bridge_hardware/ft_bridge_config.hpp"
#include "ft_bridge_hardware/wrench_bias.hpp"

namespace ft_bridge_hardware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FtBridgeHardwareInterface : public hardware_interface::SensorInterface
{
public:
  ~FtBridgeHardwareInterface() override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

private:
  void stopExecutor();
  void onWrenchMessage(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void onSetpointMessage(const geometry_msgs::msg::Wrench::SharedPtr msg);

  FtBridgeConfig config_;
  std::string sensor_name_;

  // -- ros2_control-facing state doubles (realtime-thread owned; written only inside read()) --
  double force_x_{0.0};
  double force_y_{0.0};
  double force_z_{0.0};
  double torque_x_{0.0};
  double torque_y_{0.0};
  double torque_z_{0.0};

  // -- cross-thread handoff, guarded by wrench_mutex_ -- see class doc's threading model --
  std::mutex wrench_mutex_;
  WrenchArray raw_wrench_{};
  WrenchArray setpoint_wrench_{};
  bool has_received_wrench_{false};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr setpoint_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

}  // namespace ft_bridge_hardware
