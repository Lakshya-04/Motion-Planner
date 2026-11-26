#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "controller_interface/chainable_controller_interface.hpp"
#include "diff_ik_core/cartesian_tracker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace cartesian_tracking_controller
{

/// Value of diagnostics.data[0].
enum class Status : int
{
  Ok = 0,
  NoReference = 1,       ///< nothing received since activation
  StaleReference = 2,    ///< last pose older than reference_timeout
  InvalidReference = 3,  ///< NaN / inf or a zero quaternion
  KinematicsFailed = 4,
  TrackerInvalid = 5,    ///< CartesianTracker rejected its input
  BadJointState = 6,
};

// Wraps diff_ik_core::CartesianTracker as a chainable controller; topics and parameters are in the README.
class CartesianTrackingController : public controller_interface::ChainableControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

protected:
  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_reference_interfaces_list() override;
  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr size_t kRefSize = 13;   // position(3) quaternion xyzw(4) twist lin(3) ang(3)

  template<size_t N>
  struct Input
  {
    std::array<double, N> v{};
    uint64_t seq = 0;   // 0 = nothing received
  };

  void hold(Status status, const Eigen::VectorXd & q, bool q_valid);
  void publishDiagnostics(Status status, const diff_ik_core::TrackerOutput * out);
  void applyToolOffset(const rclcpp::Time & time);
  std::string checkChain(const std::string & urdf_xml) const;

  // parameters, read in on_configure
  std::vector<std::string> joints_;
  std::vector<std::string> command_types_;
  std::string base_, tip_;
  double reference_timeout_ = 0.5;
  double diagnostics_period_ = 0.1;
  bool plane_enabled_ = false;
  bool write_velocity_ = true;
  bool write_position_ = false;
  size_t n_ = 0;

  std::unique_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> ik_loader_;
  std::unique_ptr<kinematics_interface::KinematicsInterface> ik_;
  std::unique_ptr<diff_ik_core::CartesianTracker> tracker_;

  Eigen::VectorXd q_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> jac_;
  std::array<double, kRefSize> ref_{};

  // subscribers write, the control loop reads
  realtime_tools::RealtimeBuffer<Input<7>> pose_in_;
  realtime_tools::RealtimeBuffer<Input<6>> twist_in_;
  realtime_tools::RealtimeBuffer<Input<7>> plane_in_;   // normal(3) standoff_error axis(3)
  realtime_tools::RealtimeBuffer<Input<6>> tool_in_;
  uint64_t pose_seq_ = 0, twist_seq_ = 0, plane_seq_ = 0, tool_seq_ = 0;   // callbacks only
  uint64_t seen_pose_ = 0, seen_twist_ = 0, seen_plane_ = 0, seen_tool_ = 0;   // control loop only
  rclcpp::Time pose_rx_, twist_rx_, plane_rx_;
  enum class RefState { None, Stale, Fresh } ref_state_ = RefState::None;
  bool twist_fresh_ = false;
  bool have_pose_ = false, have_twist_ = false, have_plane_ = false;

  bool holding_ = false;
  Eigen::VectorXd hold_q_;
  double since_diag_ = 1e9;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr plane_sub_, tool_sub_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> diag_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rt_diag_;
};

}  // namespace cartesian_tracking_controller
