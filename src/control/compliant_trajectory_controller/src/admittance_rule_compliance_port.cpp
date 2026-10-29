/**
 * @file admittance_rule_compliance_port.cpp
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include "compliant_trajectory_controller/admittance_rule_compliance_port.hpp"

#include <chrono>
#include <cmath>

namespace compliant_trajectory_controller
{

namespace
{
// Rotational-axis gains have no ComplianceConfig field (ToolAdmittance only carries 3-element
// mass/damping/stiffness -- translation only); reused verbatim from jtac_compliance_demo_node.cpp,
// where this exact set was proven stable at 100 Hz against the same class of chain.
constexpr double kRotationalMass = 0.5;
constexpr double kRotationalStiffness = 50.0;
constexpr double kRotationalDampingRatio = 1.0;
constexpr double kJointDamping = 5.0;

double dampingRatio(double raw_damping, double mass, double stiffness)
{
  const double denom = 2.0 * std::sqrt(mass * stiffness);
  return denom > 1e-9 ? raw_damping / denom : 0.0;
}
}  // namespace

AdmittanceRuleCompliancePort::AdmittanceRuleCompliancePort(
  Options options, std::vector<double> initial_joint_positions)
: options_(std::move(options)), num_joints_(options_.joint_names.size())
{
  current_ = makePoint(initial_joint_positions);
  reference_ = makePoint(initial_joint_positions);
}

trajectory_msgs::msg::JointTrajectoryPoint AdmittanceRuleCompliancePort::makePoint(
  const std::vector<double> & positions) const
{
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.velocities.assign(num_joints_, 0.0);
  point.accelerations.assign(num_joints_, 0.0);
  return point;
}

uint64_t AdmittanceRuleCompliancePort::nowNs()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch())
    .count());
}

void AdmittanceRuleCompliancePort::buildRule(const core_types::ComplianceConfig & config)
{
  std::vector<double> mass6{
    config.mass[0], config.mass[1], config.mass[2], kRotationalMass, kRotationalMass,
    kRotationalMass};
  std::vector<double> stiffness6{
    config.stiffness[0], config.stiffness[1], config.stiffness[2], kRotationalStiffness,
    kRotationalStiffness, kRotationalStiffness};
  // ComplianceConfig carries raw damping (ToolAdmittance's field, per tool_profiles.yaml); the
  // controller wants damping_ratio -- see force_control_backend_admittance_vs_diffik.md's
  // tool_profiles.yaml landmine list for why re-deriving this, not passing it through, matters.
  std::vector<double> damping_ratio6{
    dampingRatio(config.damping[0], config.mass[0], config.stiffness[0]),
    dampingRatio(config.damping[1], config.mass[1], config.stiffness[1]),
    dampingRatio(config.damping[2], config.mass[2], config.stiffness[2]),
    kRotationalDampingRatio, kRotationalDampingRatio, kRotationalDampingRatio};
  std::vector<bool> selected_axes6(config.selected_axes.begin(), config.selected_axes.end());

  std::vector<rclcpp::Parameter> params{
    rclcpp::Parameter("robot_description", options_.robot_description),
    rclcpp::Parameter("joints", options_.joint_names),
    rclcpp::Parameter("command_interfaces", std::vector<std::string>{"position", "velocity"}),
    rclcpp::Parameter("state_interfaces", std::vector<std::string>{"position", "velocity"}),
    rclcpp::Parameter(
      "chainable_command_interfaces", std::vector<std::string>{"position", "velocity"}),
    rclcpp::Parameter(
      "kinematics.plugin_name", std::string("kinematics_interface_kdl/KinematicsInterfaceKDL")),
    rclcpp::Parameter("kinematics.plugin_package", std::string("kinematics_interface")),
    rclcpp::Parameter("kinematics.base", options_.kinematics_base),
    rclcpp::Parameter("kinematics.tip", options_.kinematics_tip),
    rclcpp::Parameter("kinematics.alpha", 0.0005),
    rclcpp::Parameter("ft_sensor.name", std::string("compliance_port_ft")),
    rclcpp::Parameter("ft_sensor.frame.id", config.control_frame),
    rclcpp::Parameter("ft_sensor.filter_coefficient", 1.0),
    // control.frame.id == control_frame (the tool tip, by ComplianceConfig's default), NOT
    // kinematics_base: core_types::Wrench's force_z/normalForceN() convention ("negative
    // while pressing along the tool's own +Z") is only correct if AdmittanceRule's Cartesian Z
    // axis literally IS the tool's own Z -- exactly matching wall_penetration_check.hpp's own
    // convention that a tool's local +Z is the outward wall normal. Setting this to a fixed base
    // frame instead (as the demo node and T4a's gtest both do, for their own simplicity) would
    // silently redefine "the normal" to mean "world-frame Z" -- correct for a ceiling tool by
    // coincidence, wrong for a wall.
    rclcpp::Parameter("control.frame.id", config.control_frame),
    rclcpp::Parameter("fixed_world_frame.frame.id", options_.kinematics_base),
    rclcpp::Parameter("gravity_compensation.frame.id", config.control_frame),
    rclcpp::Parameter("gravity_compensation.CoG.pos", std::vector<double>{0.0, 0.0, 0.0}),
    // Must stay 0.0: UFACTORY's ft_ext_force already removes payload/gravity -- see
    // force_control_backend_admittance_vs_diffik.md's tool_profiles.yaml landmine list.
    rclcpp::Parameter("gravity_compensation.CoG.force", 0.0),
    rclcpp::Parameter("admittance.selected_axes", selected_axes6),
    rclcpp::Parameter("admittance.mass", mass6),
    rclcpp::Parameter("admittance.stiffness", stiffness6),
    rclcpp::Parameter("admittance.damping_ratio", damping_ratio6),
    rclcpp::Parameter("admittance.joint_damping", kJointDamping),
    rclcpp::Parameter("enable_parameter_update_without_reactivation", false),
  };

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);
  // A fresh node per engage(): engage() IS the controller switch (CompliancePort's own doc),
  // so rebuilding internal state here is the honest boundary, not a workaround -- see this file's
  // header doc.
  rule_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "admittance_rule_compliance_port_" + std::to_string(nowNs()), node_options);
  param_listener_ = std::make_shared<admittance_controller::ParamListener>(
    rule_node_->get_node_parameters_interface(), rule_node_->get_logger());
  rule_ = std::make_unique<admittance_controller::AdmittanceRule>(param_listener_);
  if (rule_->configure(rule_node_, num_joints_, options_.robot_description) !=
    controller_interface::return_type::OK)
  {
    throw std::runtime_error("AdmittanceRule::configure() failed");
  }

  kinematics_loader_ =
    std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
    "kinematics_interface", "kinematics_interface::KinematicsInterface");
  kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
    kinematics_loader_->createUnmanagedInstance(
      "kinematics_interface_kdl/KinematicsInterfaceKDL"));
  // "kinematics": this node's own NodeOptions already declares "kinematics.tip" =
  // options_.kinematics_tip (see the parameter_overrides above), so the KDL plugin resolves the
  // same tip by reading it back through the default namespace.
  if (!kinematics_->initialize(
      options_.robot_description, rule_node_->get_node_parameters_interface(), "kinematics"))
  {
    throw std::runtime_error("kinematics initialize() failed");
  }

  Eigen::VectorXd q =
    Eigen::VectorXd::Map(reference_.positions.data(), static_cast<long>(num_joints_));
  kinematics_->calculate_link_transform(q, options_.kinematics_tip, base_to_tip_at_reference_);

  max_offset_m_ = config.max_offset_m;
  deadband_n_ = config.deadband_n;
  clamp_n_ = config.clamp_n;
  lpf_alpha_ = config.lpf_cutoff_hz > 1e-9 ?
    (options_.dt_s / (options_.dt_s + 1.0 / (2.0 * M_PI * config.lpf_cutoff_hz))) :
    1.0;
  filter_initialized_ = false;
}

bool AdmittanceRuleCompliancePort::tare(std::string & error_message)
{
  if (engaged_) {
    error_message = "tare() must be called out of contact, before engage() -- caller violated "
      "ToolFtHygiene::tare_per_goal's mission-level-only contract";
    return false;
  }
  filter_initialized_ = false;
  filtered_force_z_ = 0.0;
  return true;
}

bool AdmittanceRuleCompliancePort::engage(
  const core_types::ComplianceConfig & config, std::string & error_message)
{
  if (engaged_) {
    error_message = "already engaged -- engage() is the rigid->compliant switch, call once";
    return false;
  }
  try {
    buildRule(config);
  } catch (const std::exception & e) {
    error_message = e.what();
    return false;
  }
  target_force_n_ = config.target_force_n;
  engaged_ = true;
  return true;
}

bool AdmittanceRuleCompliancePort::setTargetForce(double target_force_n, std::string &)
{
  target_force_n_ = target_force_n;
  return true;
}

bool AdmittanceRuleCompliancePort::disengage(std::string & error_message)
{
  if (!engaged_) {
    error_message = "not engaged";
    return false;
  }
  // Caller (ForceStageSequencer, per the FSM's switch_safe_force_n bound) guarantees |F| is
  // already below the unload threshold -- this is where a real backend hands the joints back to
  // whatever rigid controller runs next; here that just means: stop tracking the spring.
  reference_ = current_;
  engaged_ = false;
  rule_.reset();
  kinematics_.reset();
  return true;
}

void AdmittanceRuleCompliancePort::tickOnce(const core_types::Wrench & measured_wrench)
{
  filtered_force_z_ = filter_initialized_ ?
    filtered_force_z_ + lpf_alpha_ * (measured_wrench.force_z - filtered_force_z_) :
    measured_wrench.force_z;
  filter_initialized_ = true;

  double clamped_force_z = filtered_force_z_;
  if (clamp_n_ > 1e-9) {
    clamped_force_z = std::max(-clamp_n_, std::min(clamp_n_, clamped_force_z));
  }

  // Bias so "drive to zero" (the installed AdmittanceRule's only mode) becomes "drive to
  // target_force_n_" -- see this file's header doc, option (a).
  const double target_force_z = -target_force_n_;
  double error_force_z = clamped_force_z - target_force_z;
  if (std::abs(error_force_z) < deadband_n_) {
    error_force_z = 0.0;
  }

  geometry_msgs::msg::Wrench wrench;
  wrench.force.z = error_force_z;
  wrench.torque.x = measured_wrench.torque_x;
  wrench.torque.y = measured_wrench.torque_y;
  wrench.torque.z = measured_wrench.torque_z;

  trajectory_msgs::msg::JointTrajectoryPoint desired = makePoint(reference_.positions);
  const auto result = rule_->update(
    current_, wrench, reference_, rclcpp::Duration::from_seconds(options_.dt_s), desired);
  if (result == controller_interface::return_type::OK) {
    current_.positions = desired.positions;
    current_.velocities = desired.velocities;
  }
  simulated_stamp_ns_ += static_cast<uint64_t>(options_.dt_s * 1e9);
}

double AdmittanceRuleCompliancePort::normalOffsetM() const
{
  Eigen::VectorXd q =
    Eigen::VectorXd::Map(current_.positions.data(), static_cast<long>(num_joints_));
  Eigen::Isometry3d base_to_tip_now;
  if (!kinematics_->calculate_link_transform(q, options_.kinematics_tip, base_to_tip_now)) {
    return 0.0;
  }
  const Eigen::Vector3d base_offset =
    base_to_tip_now.translation() - base_to_tip_at_reference_.translation();
  // Project into the tool frame captured at engage time so "normal" means the tool's own Z,
  // not the world Z -- correct for a horizontal (wall) as well as a vertical (ceiling) contact.
  const Eigen::Vector3d tool_offset = base_to_tip_at_reference_.rotation().transpose() *
    base_offset;
  return tool_offset.z();
}

core_types::ComplianceStatus AdmittanceRuleCompliancePort::status()
{
  core_types::ComplianceStatus status;
  status.engaged = engaged_;
  status.wrench_valid = engaged_;
  status.stamp_ns = simulated_stamp_ns_;
  if (!engaged_) {
    return status;
  }
  status.wrench.force_z = filtered_force_z_;
  status.normal_offset_m = normalOffsetM();
  status.offset_saturated = max_offset_m_ > 1e-9 &&
    std::abs(status.normal_offset_m) >= max_offset_m_ - 1e-4;
  return status;
}

bool AdmittanceRuleCompliancePort::protectiveStop(std::string & error_message)
{
  if (!engaged_) {
    error_message = "not engaged -- nothing to stop";
    return false;
  }
  // Immediate, in-place halt: freeze the reference at wherever the tool currently is, so the
  // spring has nothing left to pull toward -- not the hardware safety function of last resort
  // (CompliancePort's own doc), just the supervisory lever for a guard trip.
  reference_ = current_;
  target_force_n_ = 0.0;
  return true;
}

std::vector<double> AdmittanceRuleCompliancePort::simulateTrajectoryExecution(
  const std::vector<double> & target_joint_positions, double duration_s,
  const std::function<core_types::Wrench()> & measured_wrench_provider)
{
  const auto start_reference = reference_.positions;
  const int ticks = std::max(1, static_cast<int>(std::round(duration_s / options_.dt_s)));
  for (int tick = 1; tick <= ticks; ++tick) {
    const double fraction = static_cast<double>(tick) / static_cast<double>(ticks);
    for (size_t i = 0; i < num_joints_; ++i) {
      reference_.positions[i] =
        start_reference[i] + fraction * (target_joint_positions[i] - start_reference[i]);
    }
    tickOnce(measured_wrench_provider());
  }
  return current_.positions;
}

}  // namespace compliant_trajectory_controller
