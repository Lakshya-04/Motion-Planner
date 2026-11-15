/**
 * @file jtac_controller_compliance_port.cpp
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include "compliant_trajectory_controller/jtac_controller_compliance_port.hpp"

#include <chrono>
#include <cmath>

namespace compliant_trajectory_controller
{

namespace
{
// Same rotational-axis defaults jtac_controller.yaml's own startup config and
// AdmittanceRuleCompliancePort both use -- ComplianceConfig only carries 3 translational values
// (ToolAdmittance's own schema), so a real 6-element admittance.mass/stiffness/damping_ratio
// parameter set must supply SOME rotational values every call; kept identical across all three
// so a config swap in one place doesn't silently diverge from the others.
constexpr double kRotationalMass = 0.5;
constexpr double kRotationalStiffness = 50.0;
constexpr double kRotationalDampingRatio = 1.0;

double dampingRatio(double raw_damping, double mass, double stiffness)
{
  const double denom = 2.0 * std::sqrt(mass * stiffness);
  return denom > 1e-9 ? raw_damping / denom : 0.0;
}
}  // namespace

JtacControllerCompliancePort::JtacControllerCompliancePort(Options options)
: options_(std::move(options))
{
  node_ = std::make_shared<rclcpp::Node>("jtac_controller_compliance_port");
  param_client_node_ =
    std::make_shared<rclcpp::Node>("jtac_controller_compliance_port_param_client");

  param_client_ = std::make_shared<rclcpp::SyncParametersClient>(
    param_client_node_, options_.controller_node_name);

  target_force_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
    "/" + options_.controller_node_name + "/target_force_n", rclcpp::SystemDefaultsQoS());
  max_offset_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
    "/" + options_.controller_node_name + "/max_offset_m", rclcpp::SystemDefaultsQoS());

  measured_wrench_subscriber_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/" + options_.controller_node_name + "/measured_wrench", rclcpp::SystemDefaultsQoS(),
    std::bind(&JtacControllerCompliancePort::onMeasuredWrench, this, std::placeholders::_1));

  trajectory_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    node_, "/" + options_.controller_node_name + "/follow_joint_trajectory");

  // Background executor thread for the subscription callback -- status() is called
  // synchronously by ForceStageSequencer's dwell loop, which spins nothing itself. Same
  // threading model as ft_bridge_hardware's own, and for the same reason (see that
  // component's own header doc).
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() {executor_->spin();});
}

JtacControllerCompliancePort::~JtacControllerCompliancePort()
{
  stopExecutor();
}

void JtacControllerCompliancePort::stopExecutor()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void JtacControllerCompliancePort::onMeasuredWrench(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  latest_wrench_ = *msg;
  has_received_wrench_ = true;
}

bool JtacControllerCompliancePort::tare(std::string & error_message)
{
  if (engaged_) {
    error_message = "tare() must be called out of contact, before engage() -- caller violated "
      "ToolFtHygiene::tare_per_goal's mission-level-only contract";
    return false;
  }
  // Documented no-op for this mock-only increment -- see this class's own header doc. Real
  // hardware zeroing needs a service call to the vendor driver, not attempted here.
  return true;
}

bool JtacControllerCompliancePort::setSelectedAxes(
  const std::array<bool, 6> & selected_axes, std::string & error_message)
{
  const std::vector<bool> selected_axes_vec(selected_axes.begin(), selected_axes.end());
  const auto results = param_client_->set_parameters(
    {rclcpp::Parameter("admittance.selected_axes", selected_axes_vec)});
  for (const auto & result : results) {
    if (!result.successful) {
      error_message = "set_parameters(admittance.selected_axes) failed: " + result.reason;
      return false;
    }
  }
  return true;
}

bool JtacControllerCompliancePort::engage(
  const core_types::ComplianceConfig & config, std::string & error_message)
{
  if (engaged_) {
    error_message = "already engaged -- engage() is the rigid->compliant switch, call once";
    return false;
  }

  if (!param_client_->wait_for_service(
      std::chrono::duration<double>(options_.parameter_set_timeout_s)))
  {
    error_message = "parameter service for '" + options_.controller_node_name +
      "' not available within timeout";
    return false;
  }

  const std::vector<double> mass6{
    config.mass[0], config.mass[1], config.mass[2], kRotationalMass, kRotationalMass,
    kRotationalMass};
  const std::vector<double> stiffness6{
    config.stiffness[0], config.stiffness[1], config.stiffness[2], kRotationalStiffness,
    kRotationalStiffness, kRotationalStiffness};
  const std::vector<double> damping_ratio6{
    dampingRatio(config.damping[0], config.mass[0], config.stiffness[0]),
    dampingRatio(config.damping[1], config.mass[1], config.stiffness[1]),
    dampingRatio(config.damping[2], config.mass[2], config.stiffness[2]),
    kRotationalDampingRatio, kRotationalDampingRatio, kRotationalDampingRatio};
  const std::vector<bool> selected_axes_vec(
    config.selected_axes.begin(), config.selected_axes.end());

  const auto results = param_client_->set_parameters(
    {rclcpp::Parameter("admittance.mass", mass6),
      rclcpp::Parameter("admittance.stiffness", stiffness6),
      rclcpp::Parameter("admittance.damping_ratio", damping_ratio6),
      rclcpp::Parameter("control.frame.id", config.control_frame),
      // selected_axes set LAST: everything above must land before compliance actually goes live
      // on any axis, since AdmittanceRule::apply_parameters_update() reads the whole Params
      // struct atomically on its NEXT update() cycle regardless of call order here -- ordering
      // this call last is a belt-and-suspenders precaution, not load-bearing correctness, given
      // set_parameters() itself is a single atomic call for all of them together.
      rclcpp::Parameter("admittance.selected_axes", selected_axes_vec)});

  for (const auto & result : results) {
    if (!result.successful) {
      error_message = "engage() set_parameters() failed: " + result.reason;
      return false;
    }
  }

  publishTargetForce(config.target_force_n);

  // See JtacController::clampCartesianOffset()'s own doc comment for why this exists: without
  // it, an APPROACH that never finds real contact can drive the compliant offset unbounded for
  // the whole settle_timeout_s dwell -- confirmed live to cause self-collision.
  std_msgs::msg::Float64 max_offset_msg;
  max_offset_msg.data = config.max_offset_m;
  max_offset_publisher_->publish(max_offset_msg);

  engaged_ = true;
  return true;
}

void JtacControllerCompliancePort::publishTargetForce(double target_force_n)
{
  // JtacController applies this bias itself, on a local copy fed only to AdmittanceRule::update()
  // -- see that class's own header doc for why (ft_bridge_hardware's setpoint topic would
  // corrupt the RAW wrench this port's own status() needs to report).
  std_msgs::msg::Float64 msg;
  msg.data = target_force_n;
  target_force_publisher_->publish(msg);
}

bool JtacControllerCompliancePort::setTargetForce(double target_force_n, std::string &)
{
  publishTargetForce(target_force_n);
  return true;
}

bool JtacControllerCompliancePort::disengage(std::string & error_message)
{
  if (!engaged_) {
    error_message = "not engaged";
    return false;
  }
  // All axes deselected -- matches core_types::ForceStage::kNone's own convention (see this
  // class's own header doc). Mass/stiffness/damping_ratio are left as whatever engage() set --
  // harmless, since a deselected axis's spring dynamics are masked regardless (the admittance-rule integration test's own
  // ForceOnADeselectedAxisProducesOnlyItsSelectedAxisProjection).
  if (!setSelectedAxes({false, false, false, false, false, false}, error_message)) {
    return false;
  }
  publishTargetForce(0.0);
  engaged_ = false;
  return true;
}

core_types::ComplianceStatus JtacControllerCompliancePort::status()
{
  core_types::ComplianceStatus status;
  status.engaged = engaged_;

  geometry_msgs::msg::WrenchStamped wrench_snapshot;
  bool has_wrench = false;
  {
    std::lock_guard<std::mutex> lock(wrench_mutex_);
    has_wrench = has_received_wrench_;
    wrench_snapshot = latest_wrench_;
  }

  if (!has_wrench) {
    status.wrench_valid = false;
    return status;
  }

  const rclcpp::Time sample_time(wrench_snapshot.header.stamp);
  const double age_s = (node_->now() - sample_time).seconds();
  status.wrench_valid = engaged_ && age_s >= 0.0 && age_s <= options_.wrench_staleness_timeout_s;
  status.wrench.force_x = wrench_snapshot.wrench.force.x;
  status.wrench.force_y = wrench_snapshot.wrench.force.y;
  status.wrench.force_z = wrench_snapshot.wrench.force.z;
  status.wrench.torque_x = wrench_snapshot.wrench.torque.x;
  status.wrench.torque_y = wrench_snapshot.wrench.torque.y;
  status.wrench.torque_z = wrench_snapshot.wrench.torque.z;
  status.stamp_ns = static_cast<uint64_t>(sample_time.nanoseconds());
  // normal_offset_m / offset_saturated: not sourced in this increment -- see this class's own
  // header doc on why (needs either a JtacController-side AdmittanceControllerState publisher or
  // a frame-rotated computation here).
  return status;
}

bool JtacControllerCompliancePort::protectiveStop(std::string & error_message)
{
  if (!engaged_) {
    error_message = "not engaged -- nothing to stop";
    return false;
  }
  // Unload the target force AND cancel every in-flight goal on the controller's own action
  // server -- fire-and-forget, matching this method's own "immediate" characterization. See this
  // class's own header doc for why this is not the safety function of last resort, and for what
  // JtacController itself does on cancellation (immediate freeze, no decel ramp).
  publishTargetForce(0.0);
  trajectory_client_->async_cancel_all_goals();
  return true;
}

}  // namespace compliant_trajectory_controller
