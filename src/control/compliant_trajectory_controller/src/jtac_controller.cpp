/**
 * @file jtac_controller.cpp
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include "compliant_trajectory_controller/jtac_controller.hpp"

#include <algorithm>
#include <limits>
#include <memory>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace compliant_trajectory_controller
{

namespace
{
/// Reorders trajectory_msg's joint_names (and every point's positions/velocities/accelerations)
/// into local_joint_order -- matching joint_trajectory_controller's own sort_to_local_joint_order,
/// since Trajectory::sample() returns values positionally, in whatever order the message's own
/// joint_names list happened to carry.
void sortTrajectoryToLocalOrder(
  const std::vector<std::string> & local_joint_order,
  trajectory_msgs::msg::JointTrajectory & trajectory_msg)
{
  std::vector<size_t> mapping(local_joint_order.size());
  for (size_t i = 0; i < local_joint_order.size(); ++i) {
    const auto it = std::find(
      trajectory_msg.joint_names.begin(), trajectory_msg.joint_names.end(),
      local_joint_order[i]);
    mapping[i] = static_cast<size_t>(std::distance(trajectory_msg.joint_names.begin(), it));
  }
  trajectory_msg.joint_names = local_joint_order;
  for (auto & point : trajectory_msg.points) {
    auto reorder = [&mapping](std::vector<double> & values) {
        if (values.size() != mapping.size()) {
          return;
        }
        std::vector<double> reordered(values.size());
        for (size_t i = 0; i < mapping.size(); ++i) {
          reordered[i] = values[mapping[i]];
        }
        values = reordered;
      };
    reorder(point.positions);
    reorder(point.velocities);
    reorder(point.accelerations);
    reorder(point.effort);
  }
}
}  // namespace

Eigen::MatrixXd weightedPseudoInverse(
  const Eigen::Matrix<double, 3, Eigen::Dynamic> & j_lin, const Eigen::VectorXd & weights)
{
  const Eigen::MatrixXd w_inv = weights.cwiseInverse().asDiagonal();
  const Eigen::Matrix3d j_w_jt = j_lin * w_inv * j_lin.transpose();
  return w_inv * j_lin.transpose() * j_w_jt.inverse();
}

controller_interface::InterfaceConfiguration JtacController::command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    for (const auto & type : command_interface_types_) {
      config.names.push_back(joint + "/" + type);
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration JtacController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    for (const auto & type : state_interface_types_) {
      config.names.push_back(joint + "/" + type);
    }
  }
  if (has_ft_sensor_) {
    const auto ft_names = ft_sensor_->get_state_interface_names();
    config.names.insert(config.names.end(), ft_names.begin(), ft_names.end());
  }
  return config;
}

controller_interface::CallbackReturn JtacController::on_init()
{
  // Constructing ParamListener against get_node() declares admittance_controller's ENTIRE
  // parameter schema (joints, command_interfaces, state_interfaces, kinematics.*, ft_sensor.*,
  // admittance.*, control.frame.id, ...) on THIS controller's own node -- see this class's header
  // doc for why that is deliberately preferred over AdmittanceRuleCompliancePort's own throwaway-
  // node pattern for a controller meant to be actually deployed and addressed over ROS.
  try {
    param_listener_ = std::make_shared<admittance_controller::ParamListener>(
      get_node()->get_node_parameters_interface(), get_node()->get_logger());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_node()->get_logger(), "Exception in on_init(): %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_names_ = params_.joints;
  command_interface_types_ = params_.command_interfaces;
  state_interface_types_ = params_.state_interfaces;
  num_joints_ = joint_names_.size();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // per-joint redundancy weighting -- plain params on this controller's own node, NOT routed through admittance_controller's
  // own generated schema (ParamListener above) -- see this class's own header doc for why. Read
  // once here, not live-reconfigurable this increment (matching jtac_compliance_demo_node.cpp's
  // own scope).
  //
  // has_parameter() guard: the spawner's own --param-file (jtac_controller_test.launch.py, or
  // a MoveIt launch file's own JTAC --param-file) sets these directly in the YAML
  // handed to this node at construction, which auto-declares them BEFORE on_init() ever runs --
  // an unconditional declare_parameter() here then throws ParameterAlreadyDeclaredException (hit
  // live: SIGABRT on activation, the exact double-declare bug class also seen with
  // another node in this package).
  if (!get_node()->has_parameter("enable_redundancy_weighting")) {
    get_node()->declare_parameter("enable_redundancy_weighting", false);
  }
  if (!get_node()->has_parameter("redundancy_weights")) {
    get_node()->declare_parameter(
      "redundancy_weights", std::vector<double>{10.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  }
  enable_redundancy_weighting_ = get_node()->get_parameter("enable_redundancy_weighting").as_bool();
  redundancy_weights_ = get_node()->get_parameter("redundancy_weights").as_double_array();
  if (enable_redundancy_weighting_) {
    if (redundancy_weights_.size() != num_joints_) {
      RCLCPP_FATAL(
        get_node()->get_logger(), "redundancy_weights must have %zu elements, got %zu.",
        num_joints_, redundancy_weights_.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    for (double w : redundancy_weights_) {
      if (w <= 0.0) {
        RCLCPP_FATAL(
          get_node()->get_logger(), "redundancy_weights entries must be > 0 (got %f).", w);
        return controller_interface::CallbackReturn::ERROR;
      }
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JtacController::on_configure(
  const rclcpp_lifecycle::State &)
{
  admittance_rule_ = std::make_unique<admittance_controller::AdmittanceRule>(param_listener_);
  if (admittance_rule_->configure(get_node(), num_joints_, get_robot_description()) !=
    controller_interface::return_type::OK)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "AdmittanceRule::configure() failed.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Unconditional -- unlike redundancy_kinematics_ below, this safety clamp must work on
  // WHICHEVER update path is active. See clampCartesianOffset()'s own doc comment.
  offset_clamp_kinematics_loader_ = std::make_shared<
    pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
    "kinematics_interface", "kinematics_interface::KinematicsInterface");
  offset_clamp_kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
    offset_clamp_kinematics_loader_->createUnmanagedInstance(params_.kinematics.plugin_name));
  // "kinematics" (the default param_namespace, per kinematics_interface's own doc comment):
  // admittance_controller::Params already declares "kinematics.tip" = params_.kinematics.tip on
  // this same node (that's the whole struct this plugin populates), so the KDL plugin resolves
  // the exact same tip by reading it back through this namespace -- passing params_.kinematics.tip
  // directly here would be read as a NAMESPACE, not a tip link name, and fail to find "<tip
  // value>.tip".
  if (!offset_clamp_kinematics_->initialize(
      get_robot_description(), get_node()->get_node_parameters_interface(), "kinematics"))
  {
    RCLCPP_FATAL(get_node()->get_logger(), "offset_clamp_kinematics_ initialize() failed.");
    return controller_interface::CallbackReturn::ERROR;
  }
  max_offset_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "~/max_offset_m", rclcpp::SystemDefaultsQoS(),
    std::bind(&JtacController::onMaxOffset, this, std::placeholders::_1));

  if (enable_redundancy_weighting_) {
    // See this class's own header doc's redundancy weighting section for why this scope limit is asserted here
    // (once, loudly, at configure time) rather than silently assumed.
    if (params_.kinematics.tip != params_.control.frame.id ||
      params_.kinematics.tip != params_.ft_sensor.frame.id)
    {
      RCLCPP_FATAL(
        get_node()->get_logger(),
        "enable_redundancy_weighting requires kinematics.tip == control.frame.id == "
        "ft_sensor.frame.id (got tip='%s', control='%s', ft_sensor='%s').",
        params_.kinematics.tip.c_str(), params_.control.frame.id.c_str(),
        params_.ft_sensor.frame.id.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    redundancy_kinematics_loader_ = std::make_shared<
      pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
      "kinematics_interface", "kinematics_interface::KinematicsInterface");
    redundancy_kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
      redundancy_kinematics_loader_->createUnmanagedInstance(params_.kinematics.plugin_name));
    if (!redundancy_kinematics_->initialize(
        get_robot_description(), get_node()->get_node_parameters_interface(), "kinematics"))
    {
      RCLCPP_FATAL(get_node()->get_logger(), "T11 kinematics initialize() failed.");
      return controller_interface::CallbackReturn::ERROR;
    }
    weighted_joint_pos_offset_.assign(num_joints_, 0.0);
    weighted_joint_vel_offset_.assign(num_joints_, 0.0);
  }

  has_ft_sensor_ = !params_.ft_sensor.name.empty();
  if (has_ft_sensor_) {
    ft_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(params_.ft_sensor.name);
  }

  measured_wrench_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "~/measured_wrench", rclcpp::SystemDefaultsQoS());
  measured_wrench_rt_publisher_ =
    std::make_unique<WrenchStatePublisher>(measured_wrench_publisher_);

  target_force_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "~/target_force_n", rclcpp::SystemDefaultsQoS(),
    std::bind(&JtacController::onTargetForce, this, std::placeholders::_1));

  debug_offset_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/debug_offset", rclcpp::SystemDefaultsQoS());
  debug_offset_rt_publisher_ =
    std::make_unique<DebugOffsetPublisher>(debug_offset_publisher_);

  // AdmittanceRule::update() indexes .accelerations[i] on all three points unconditionally --
  // see jtac_compliance_demo_node.cpp's own doc comment on this exact segfault, hit once already
  // this session. Every point AdmittanceRule ever touches must carry all three fields pre-sized.
  state_current_.positions.resize(num_joints_);
  state_current_.velocities.assign(num_joints_, 0.0);
  state_current_.accelerations.assign(num_joints_, 0.0);
  state_reference_.positions.resize(num_joints_);
  state_reference_.velocities.assign(num_joints_, 0.0);
  state_reference_.accelerations.assign(num_joints_, 0.0);
  state_desired_.positions.resize(num_joints_);
  state_desired_.velocities.assign(num_joints_, 0.0);
  state_desired_.accelerations.assign(num_joints_, 0.0);

  // See enforceJointRateLimits()'s own doc comment for why this exists. Read directly via
  // get_parameter() (not admittance_controller::ParamListener's schema, which knows nothing about
  // joint_limits.*) -- once, here, matching the redundancy-weighting redundancy_weights_ precedent for a plain
  // param on this controller's own node. A generous-but-finite fallback (not admittance's own
  // max_offset_m_/dt, which would defeat the purpose) if a joint is missing from the loaded
  // joint_limits.yaml, so a caller who forgets to wire it gets safe-but-slow motion, not a crash.
  constexpr double kFallbackMaxVelocity = 1.0;      // rad/s or m/s -- generous, still finite.
  constexpr double kFallbackMaxAcceleration = 5.0;  // rad/s^2 or m/s^2.
  max_joint_velocity_.assign(num_joints_, kFallbackMaxVelocity);
  max_joint_acceleration_.assign(num_joints_, kFallbackMaxAcceleration);
  previous_command_velocity_.assign(num_joints_, 0.0);
  for (size_t i = 0; i < num_joints_; ++i) {
    const std::string prefix = "joint_limits." + joint_names_[i];
    if (!get_node()->has_parameter(prefix + ".max_velocity")) {
      get_node()->declare_parameter(prefix + ".max_velocity", kFallbackMaxVelocity);
    }
    if (!get_node()->has_parameter(prefix + ".max_acceleration")) {
      get_node()->declare_parameter(prefix + ".max_acceleration", kFallbackMaxAcceleration);
    }
    max_joint_velocity_[i] = get_node()->get_parameter(prefix + ".max_velocity").as_double();
    max_joint_acceleration_[i] =
      get_node()->get_parameter(prefix + ".max_acceleration").as_double();
    if (max_joint_velocity_[i] == kFallbackMaxVelocity) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "%s.max_velocity not configured -- using the generous-but-finite fallback (%.2f). Wire "
        "robot_description's own joint_limits.yaml onto this node's params for the real "
        "hardware limit.",
        prefix.c_str(), kFallbackMaxVelocity);
    }
  }

  command_guard_.reset();
  if (!get_node()->has_parameter("command_guard.enabled")) {
    get_node()->declare_parameter("command_guard.enabled", false);
  }
  if (get_node()->get_parameter("command_guard.enabled").as_bool()) {
    core_types::CommandGuardConfig guard_cfg;
    if (!get_node()->has_parameter("command_guard.debounce_cycles")) {
      get_node()->declare_parameter("command_guard.debounce_cycles", 3);
    }
    guard_cfg.debounce_cycles =
      static_cast<int>(get_node()->get_parameter("command_guard.debounce_cycles").as_int());
    constexpr double kInf = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < num_joints_; ++i) {
      const std::string prefix = "joint_limits." + joint_names_[i];
      if (!get_node()->has_parameter(prefix + ".min_position")) {
        get_node()->declare_parameter(prefix + ".min_position", -kInf);
      }
      if (!get_node()->has_parameter(prefix + ".max_position")) {
        get_node()->declare_parameter(prefix + ".max_position", kInf);
      }
      guard_cfg.lower.push_back(get_node()->get_parameter(prefix + ".min_position").as_double());
      guard_cfg.upper.push_back(get_node()->get_parameter(prefix + ".max_position").as_double());
    }
    command_guard_ = std::make_unique<core_types::CommandGuard>(guard_cfg);
    RCLCPP_INFO(get_node()->get_logger(), "command guard enabled");
  }

  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
    get_node(), std::string(get_node()->get_name()) + "/follow_joint_trajectory",
    std::bind(
      &JtacController::goalReceivedCallback, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&JtacController::goalCancelledCallback, this, std::placeholders::_1),
    std::bind(&JtacController::goalAcceptedCallback, this, std::placeholders::_1));

  // Non-RT timer draining RealtimeServerGoalHandle state transitions -- matches
  // joint_trajectory_controller's own goal_handle_timer_ pattern (see this class's header doc on
  // why this hand-rolled action-server plumbing mirrors JTC's, not reuses it).
  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
    [this]() {
      auto active_goal = *rt_active_goal_.readFromRT();
      if (active_goal) {
        active_goal->runNonRealtime();
      }
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JtacController::on_activate(const rclcpp_lifecycle::State &)
{
  if (command_guard_) {command_guard_->reset();}  // re-activation clears a latched fault
  joint_position_command_interface_.clear();
  joint_velocity_command_interface_.clear();
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();

  if (!controller_interface::get_ordered_interfaces(
      command_interfaces_, joint_names_, hardware_interface::HW_IF_POSITION,
      joint_position_command_interface_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing position command interfaces.");
    return controller_interface::CallbackReturn::ERROR;
  }
  controller_interface::get_ordered_interfaces(
    command_interfaces_, joint_names_, hardware_interface::HW_IF_VELOCITY,
    joint_velocity_command_interface_);

  if (!controller_interface::get_ordered_interfaces(
      state_interfaces_, joint_names_, hardware_interface::HW_IF_POSITION,
      joint_position_state_interface_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing position state interfaces.");
    return controller_interface::CallbackReturn::ERROR;
  }
  has_velocity_state_interface_ = controller_interface::get_ordered_interfaces(
    state_interfaces_, joint_names_, hardware_interface::HW_IF_VELOCITY,
    joint_velocity_state_interface_);

  if (has_ft_sensor_ && !ft_sensor_->assign_loaned_state_interfaces(state_interfaces_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to assign FT sensor state interfaces.");
    return controller_interface::CallbackReturn::ERROR;
  }

  readStateFromStateInterfaces(state_current_);
  // Hold at the current measured pose until the first goal arrives -- no trajectory yet, so
  // there is nothing for Trajectory::sample() to sample. Freeze state_reference_ here (once),
  // and mark it already-frozen via had_active_trajectory_last_cycle_ -- otherwise update()'s
  // own no-active-trajectory branch would see had_active_trajectory_last_cycle_'s in-class
  // default of false on this controller's very first cycle and skip freezing, leaving
  // state_reference_ at on_configure()'s all-zero placeholder instead of the real current pose.
  state_reference_ = state_current_;
  state_reference_.velocities.assign(num_joints_, 0.0);
  had_active_trajectory_last_cycle_ = true;
  std::fill(previous_command_velocity_.begin(), previous_command_velocity_.end(), 0.0);
  traj_external_point_ptr_.reset();
  current_active_traj_msg_.reset();
  rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  terminal_goal_handled_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JtacController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (has_ft_sensor_) {
    ft_sensor_->release_interfaces();
  }
  release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

void JtacController::readStateFromStateInterfaces(trajectory_msgs::msg::JointTrajectoryPoint & state)
{
  state.positions.resize(num_joints_);
  state.velocities.assign(num_joints_, 0.0);
  for (size_t i = 0; i < num_joints_; ++i) {
    state.positions[i] = joint_position_state_interface_[i].get().get_value();
  }
  if (has_velocity_state_interface_) {
    for (size_t i = 0; i < num_joints_; ++i) {
      state.velocities[i] = joint_velocity_state_interface_[i].get().get_value();
    }
  }
}

void JtacController::writeCommandsToCommandInterfaces(
  const trajectory_msgs::msg::JointTrajectoryPoint & command)
{
  for (size_t i = 0; i < num_joints_; ++i) {
    joint_position_command_interface_[i].get().set_value(command.positions[i]);
  }
  if (!joint_velocity_command_interface_.empty()) {
    for (size_t i = 0; i < num_joints_; ++i) {
      const double velocity = (i < command.velocities.size()) ? command.velocities[i] : 0.0;
      joint_velocity_command_interface_[i].get().set_value(velocity);
    }
  }
}

geometry_msgs::msg::Wrench JtacController::readMeasuredWrench()
{
  geometry_msgs::msg::Wrench wrench;
  if (has_ft_sensor_) {
    ft_sensor_->get_values_as_message(wrench);
  }
  return wrench;
}

void JtacController::onTargetForce(const std_msgs::msg::Float64::SharedPtr msg)
{
  target_force_n_.writeFromNonRT(msg->data);
}

void JtacController::onMaxOffset(const std_msgs::msg::Float64::SharedPtr msg)
{
  max_offset_m_.writeFromNonRT(msg->data);
}

void JtacController::clampCartesianOffset()
{
  const Eigen::VectorXd q_reference = Eigen::VectorXd::Map(
    state_reference_.positions.data(), static_cast<long>(state_reference_.positions.size()));
  const Eigen::VectorXd q_desired = Eigen::VectorXd::Map(
    state_desired_.positions.data(), static_cast<long>(state_desired_.positions.size()));

  Eigen::Isometry3d reference_tf, desired_tf;
  const bool success =
    offset_clamp_kinematics_->calculate_link_transform(
      q_reference, params_.kinematics.tip, reference_tf) &&
    offset_clamp_kinematics_->calculate_link_transform(
      q_desired, params_.kinematics.tip, desired_tf);
  if (!success) {
    // No safe way to know the real offset -- leave state_desired_ as-is rather than guess. This
    // matches updateWeighted()'s own precedent (a kinematics failure there freezes at state_
    // reference_ instead); this clamp's own failure mode is silent-pass-through, not a freeze,
    // since a hard freeze here could itself be a discontinuous jump if this is a transient glitch.
    return;
  }

  const double offset_norm = (desired_tf.translation() - reference_tf.translation()).norm();

  if (debug_offset_rt_publisher_->trylock()) {
    auto & msg = debug_offset_rt_publisher_->msg_;
    msg.data = {
      reference_tf.translation().x(), reference_tf.translation().y(),
      reference_tf.translation().z(), desired_tf.translation().x(),
      desired_tf.translation().y(), desired_tf.translation().z(), offset_norm};
    debug_offset_rt_publisher_->unlockAndPublish();
  }

  const double max_offset_m = *max_offset_m_.readFromRT();
  if (offset_norm <= max_offset_m || offset_norm < 1e-9) {
    return;
  }

  // ROOT CAUSE (found live, 2026-08-24, and fixed at the source -- see tool_profiles.yaml's own
  // admittance.damping comment): AdmittanceRule::calculate_admittance_rule() integrates its own
  // private joint_vel/joint_pos offset with plain explicit Euler, which is numerically unstable
  // whenever damping*dt/mass >= 2.0 -- a profile with damping tuned for "heavily overdamped"
  // physical behavior (e.g. a raw damping of 1000 with mass=2.0 at this controller's 100Hz/dt=0.01
  // update rate: damping*dt/mass=5.0) integrates as an ALTERNATING-SIGN, ROUGHLY QUADRUPLING
  // sequence every single cycle, from nothing more than floating-point noise -- no force or
  // contact required to trigger it, and independent of kinematics.alpha (rules out a Jacobian-
  // conditioning explanation) and of environment stiffness (rules out a mismatched-contact-model
  // explanation). With every shipped tool profile's damping now inside the explicit-Euler
  // stability bound, this branch should be unreachable in normal operation; it stays as a
  // defense-in-depth catch for a future profile that gets re-tuned back into instability (or any
  // other cause of the same symptom) rather than something this codebase relies on to run every
  // segment. A genuine runaway is unmistakable from a legitimate hold at the physical limit (e.g.
  // the wall is further away than the nominal trajectory assumed, which also legitimately touches
  // max_offset_m every cycle) by MAGNITUDE, not by merely touching the boundary -- explicit-Euler
  // divergence clears several multiples of max_offset_m within 2-3 cycles of first crossing it,
  // while a stable hold never exceeds max_offset_m by more than ordinary tracking noise. An
  // earlier version of this check reset on every cycle that merely touched max_offset_m, which
  // punished the legitimate case too and produced a clean ~0.75s sawtooth (0N -> 30N -> reset ->
  // 0N -> ...) that never held long enough to converge -- resetting only past kRunawayMultiplier
  // gives a real divergence no room to hide while leaving a legitimate clamp-and-hold alone.
  constexpr double kRunawayMultiplier = 5.0;
  if (!std::isfinite(offset_norm) || offset_norm > kRunawayMultiplier * max_offset_m) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "JtacController: Cartesian offset diverged (%.4gm, more than %.0fx max_offset_m=%.4fm) -- "
      "resetting the compliance integrator and holding at the reference this cycle.",
      offset_norm, kRunawayMultiplier, max_offset_m);
    admittance_rule_->reset(num_joints_);
    state_desired_.positions = state_reference_.positions;
    state_desired_.velocities = state_reference_.velocities;
    return;
  }
  const double scale = max_offset_m / offset_norm;
  for (size_t i = 0; i < num_joints_; ++i) {
    state_desired_.positions[i] =
      state_reference_.positions[i] + scale * (q_desired[static_cast<long>(i)] - q_reference[static_cast<long>(i)]);
  }
}

void JtacController::enforceJointRateLimits(const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  bool any_non_finite = false;
  for (size_t i = 0; i < num_joints_; ++i) {
    if (!std::isfinite(state_desired_.positions[i]) ||
      !std::isfinite(state_desired_.velocities[i]))
    {
      any_non_finite = true;
      break;
    }
  }
  if (any_non_finite) {
    // Last-resort catch, expected to be unreachable in normal operation now that every shipped
    // tool profile's damping is inside AdmittanceRule's own explicit-Euler stability bound (see
    // clampCartesianOffset()'s own comment for the root cause and the real fix). Reachable only
    // if something else entirely produces a non-finite state_desired_ (e.g. a future profile
    // re-tuned back into instability, or a kinematics failure clampCartesianOffset() didn't
    // catch). A real actuator has no way to obey a non-finite setpoint -- hold at the last
    // known-good (measured) position for every joint rather than pass NaN/Inf through to the
    // command interfaces, and reset the shared (not per-joint) admittance integrator state so the
    // NEXT cycle gets a genuine chance to recover rather than holding for the rest of the segment.
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "JtacController: admittance diverged (non-finite state_desired) -- holding this cycle and "
      "resetting the compliance integrator.");
    admittance_rule_->reset(num_joints_);
    state_desired_.positions = state_current_.positions;
    std::fill(state_desired_.velocities.begin(), state_desired_.velocities.end(), 0.0);
    std::fill(previous_command_velocity_.begin(), previous_command_velocity_.end(), 0.0);
    return;
  }

  for (size_t i = 0; i < num_joints_; ++i) {
    double velocity = std::clamp(
      state_desired_.velocities[i], -max_joint_velocity_[i], max_joint_velocity_[i]);
    const double max_delta_v = max_joint_acceleration_[i] * dt;
    velocity = std::clamp(
      velocity, previous_command_velocity_[i] - max_delta_v,
      previous_command_velocity_[i] + max_delta_v);

    const double max_delta_p = max_joint_velocity_[i] * dt;
    const double position = std::clamp(
      state_desired_.positions[i], state_current_.positions[i] - max_delta_p,
      state_current_.positions[i] + max_delta_p);

    state_desired_.positions[i] = position;
    state_desired_.velocities[i] = velocity;
    previous_command_velocity_[i] = velocity;
  }
}

bool JtacController::updateWeighted(
  const geometry_msgs::msg::Wrench & biased_wrench, const rclcpp::Duration & period)
{
  // Translation-only -- see this class's own header doc. A live engage() selecting a rotational
  // axis while this path is active would silently apply zero torque along it if not caught here;
  // freeze at the current measured position instead (a real, disclosed stop, not a silent one).
  if (params_.admittance.selected_axes[3] || params_.admittance.selected_axes[4] ||
    params_.admittance.selected_axes[5])
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "enable_redundancy_weighting is on but a rotational admittance.selected_axes entry is "
      "true -- this path is translation-only. Freezing at the current measured position.");
    state_desired_ = state_current_;
    return false;
  }

  // Refresh params_ from the SAME live-reconfigurable source AdmittanceRule's own
  // apply_parameters_update() reads -- bypassing admittance_rule_->update() entirely means
  // bypassing that internal refresh too, so this path must do it itself for engage()/disengage()
  // (JtacControllerCompliancePort) to keep working identically regardless of which path is active.
  if (params_.enable_parameter_update_without_reactivation) {
    params_ = param_listener_->get_params();
  }

  const Eigen::VectorXd q_current = Eigen::VectorXd::Map(
    state_current_.positions.data(), static_cast<long>(state_current_.positions.size()));
  const Eigen::VectorXd q_reference = Eigen::VectorXd::Map(
    state_reference_.positions.data(), static_cast<long>(state_reference_.positions.size()));

  Eigen::Isometry3d current_tf, reference_tf;
  bool success = redundancy_kinematics_->calculate_link_transform(
    q_current, params_.kinematics.tip, current_tf);
  success &= redundancy_kinematics_->calculate_link_transform(
    q_reference, params_.kinematics.tip, reference_tf);
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, num_joints_);
  success &= redundancy_kinematics_->calculate_jacobian(q_current, params_.kinematics.tip, jacobian);
  if (!success) {
    state_desired_ = state_reference_;
    return false;
  }
  const Eigen::Matrix<double, 3, Eigen::Dynamic> j_lin = jacobian.topRows<3>();

  // X = FK(measured) - FK(reference) -- the SAME Cartesian tracking-error convention
  // AdmittanceRule::update() itself uses (its own current_joint_pos is the measured state passed
  // in, not an internally-integrated one).
  const Eigen::Vector3d x = current_tf.translation() - reference_tf.translation();

  const Eigen::VectorXd joint_vel_offset = Eigen::VectorXd::Map(
    weighted_joint_vel_offset_.data(), static_cast<long>(weighted_joint_vel_offset_.size()));
  const Eigen::Vector3d x_dot = j_lin * joint_vel_offset;

  // K/D are diagonal in CONTROL frame and must be rotated into BASE frame before use against X
  // (computed in base frame) -- matching AdmittanceRule's own calculate_admittance_rule()
  // exactly, including its own mass-applied-directly-in-base-frame asymmetry (mass is NOT
  // rotated the same way, in the real implementation -- replicated here for fidelity, not
  // "fixed"). rot_base_control == current_tf.rotation() because control.frame.id ==
  // kinematics.tip is asserted at on_configure().
  const Eigen::Matrix3d rot_base_control = current_tf.rotation();
  const Eigen::Vector3d stiffness_control(
    params_.admittance.stiffness[0], params_.admittance.stiffness[1],
    params_.admittance.stiffness[2]);
  const Eigen::Vector3d mass(
    params_.admittance.mass[0], params_.admittance.mass[1], params_.admittance.mass[2]);
  Eigen::Vector3d raw_damping;
  for (int i = 0; i < 3; ++i) {
    raw_damping[i] = params_.admittance.damping_ratio[i] * 2.0 *
      std::sqrt(mass[i] * stiffness_control[i]);
  }
  const Eigen::Matrix3d k_base =
    rot_base_control * stiffness_control.asDiagonal() * rot_base_control.transpose();
  const Eigen::Matrix3d d_base =
    rot_base_control * raw_damping.asDiagonal() * rot_base_control.transpose();

  const Eigen::Vector3d selected(
    params_.admittance.selected_axes[0] ? 1.0 : 0.0,
    params_.admittance.selected_axes[1] ? 1.0 : 0.0,
    params_.admittance.selected_axes[2] ? 1.0 : 0.0);
  // selected_axes is defined in CONTROL frame (ft_sensor.frame.id == control.frame.id here, so
  // the raw sensor reading IS already in control frame -- no rotation needed before masking).
  // Mask FIRST, THEN rotate to base -- matching AdmittanceRule's own F_control.cwiseProduct
  // (selected_axes) BEFORE its own rotate-back-to-base step exactly (get the order backwards --
  // rotate then mask in base frame -- and a deselected control-frame axis would leak through
  // whenever rot_base_control isn't axis-aligned with base, since masking a rotated vector along
  // BASE axes is not the same operation as masking the original vector along CONTROL axes). Same
  // "deselected axes relax to zero" convention the admittance-rule integration test's own
  // ForceOnADeselectedAxisProducesOnlyItsSelectedAxisProjection proves.
  const Eigen::Vector3d f_sensor(biased_wrench.force.x, biased_wrench.force.y, biased_wrench.force.z);
  const Eigen::Vector3d f_base = rot_base_control * f_sensor.cwiseProduct(selected);

  const Eigen::Vector3d x_ddot = (f_base - d_base * x_dot - k_base * x).cwiseQuotient(mass);

  const Eigen::VectorXd weights = Eigen::VectorXd::Map(
    redundancy_weights_.data(), static_cast<long>(redundancy_weights_.size()));
  const Eigen::MatrixXd j_w_pinv = weightedPseudoInverse(j_lin, weights);

  Eigen::VectorXd joint_acc = j_w_pinv * x_ddot;
  joint_acc -= params_.admittance.joint_damping * joint_vel_offset;

  Eigen::VectorXd new_joint_vel_offset = joint_vel_offset + joint_acc * period.seconds();
  Eigen::VectorXd new_joint_pos_offset =
    Eigen::VectorXd::Map(
      weighted_joint_pos_offset_.data(), static_cast<long>(weighted_joint_pos_offset_.size())) +
    new_joint_vel_offset * period.seconds();

  // Same output convention as AdmittanceRule::update(): desired = reference + accumulated
  // joint-space offset -- NOT current + velocity, which only coincides with this when the
  // reference is constant (true in jtac_compliance_demo_node.cpp's wall_ceiling phases, NOT true
  // here, where state_reference_ is continuously resampled from a moving trajectory).
  for (size_t i = 0; i < num_joints_; ++i) {
    state_desired_.positions[i] = state_reference_.positions[i] + new_joint_pos_offset[i];
    state_desired_.velocities[i] = state_reference_.velocities[i] + new_joint_vel_offset[i];
    state_desired_.accelerations[i] = state_reference_.accelerations[i] + joint_acc[i];
    weighted_joint_pos_offset_[i] = new_joint_pos_offset[i];
    weighted_joint_vel_offset_[i] = new_joint_vel_offset[i];
  }
  return true;
}

controller_interface::return_type JtacController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  readStateFromStateInterfaces(state_current_);

  auto * new_external_msg = traj_msg_external_point_ptr_.readFromRT();
  if (new_external_msg && *new_external_msg && *new_external_msg != current_active_traj_msg_) {
    current_active_traj_msg_ = *new_external_msg;
    trajectory_start_time_ = (rclcpp::Time(current_active_traj_msg_->header.stamp).seconds() ==
      0.0) ? time : rclcpp::Time(current_active_traj_msg_->header.stamp);
    traj_external_point_ptr_ = std::make_shared<joint_trajectory_controller::Trajectory>(
      time, state_current_, current_active_traj_msg_);
  }

  RealtimeGoalHandlePtr active_goal = *rt_active_goal_.readFromRT();

  const bool has_active_trajectory = traj_external_point_ptr_ &&
    traj_external_point_ptr_->has_trajectory_msg() && !current_active_traj_msg_->points.empty();

  if (has_active_trajectory) {
    const rclcpp::Time trajectory_end_time = trajectory_start_time_ +
      rclcpp::Duration(current_active_traj_msg_->points.back().time_from_start);
    const rclcpp::Time sample_time = std::min(time, trajectory_end_time);

    joint_trajectory_controller::TrajectoryPointConstIter start_segment_itr;
    joint_trajectory_controller::TrajectoryPointConstIter end_segment_itr;
    const bool sampled = traj_external_point_ptr_->sample(
      sample_time, joint_trajectory_controller::interpolation_methods::DEFAULT_INTERPOLATION,
      state_reference_, start_segment_itr, end_segment_itr);
    if (!sampled) {
      state_reference_ = state_current_;
      state_reference_.velocities.assign(num_joints_, 0.0);
    }
    // Trajectory::sample() resets output_state fresh and fills in only what the trajectory's own
    // points carried (e.g. a positions+velocities-only goal leaves accelerations empty) --
    // AdmittanceRule::update() indexes .accelerations[i] on this point unconditionally.
    if (state_reference_.accelerations.size() != num_joints_) {
      state_reference_.accelerations.assign(num_joints_, 0.0);
    }
    if (state_reference_.velocities.size() != num_joints_) {
      state_reference_.velocities.assign(num_joints_, 0.0);
    }

    if (active_goal) {
      if (active_goal->gh_->is_canceling() && active_goal != terminal_goal_handled_) {
        state_reference_ = state_current_;
        state_reference_.velocities.assign(num_joints_, 0.0);
        auto result = std::make_shared<FollowJTrajAction::Result>();
        result->error_code = FollowJTrajAction::Result::SUCCESSFUL;
        active_goal->setCanceled(result);
        terminal_goal_handled_ = active_goal;
      } else if (time >= trajectory_end_time) {
        // Fire the terminal transition exactly once per goal -- see terminal_goal_handled_'s own
        // doc for why calling this unconditionally every cycle is NOT the safe no-op it looks
        // like (RealtimeServerGoalHandle resets its own guard after runNonRealtime() processes
        // it, so an unconditional call re-fires every single cycle forever).
        if (active_goal != terminal_goal_handled_) {
          auto result = std::make_shared<FollowJTrajAction::Result>();
          result->error_code = FollowJTrajAction::Result::SUCCESSFUL;
          active_goal->setSucceeded(result);
          terminal_goal_handled_ = active_goal;
        }
      } else {
        auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
        feedback->header.stamp = time;
        feedback->joint_names = joint_names_;
        feedback->desired = state_reference_;
        feedback->actual = state_current_;
        feedback->error.positions.resize(num_joints_);
        for (size_t i = 0; i < num_joints_; ++i) {
          feedback->error.positions[i] = state_reference_.positions[i] - state_current_.positions[i];
        }
        active_goal->setFeedback(feedback);
      }
    }
  } else if (had_active_trajectory_last_cycle_) {
    // Trajectory just ended (or was never present and this is the very first cycle) -- freeze
    // the reference ONCE, at whatever pose is current right now, and never rebase it again while
    // idle. Doing this every idle cycle (the original bug) re-derives state_reference_ from
    // state_current_ each 100Hz tick, but state_current_ already reflects the PREVIOUS cycle's
    // own admittance offset -- so a per-cycle reset permanently banks that offset into the next
    // cycle's reference, defeating clampCartesianOffset()'s max_offset_m bound over time (the
    // clamp only ever sees one cycle's worth of drift, never the cumulative total). Under any
    // sustained, unconverged force error -- e.g. APPROACH's constant target force before contact
    // is made -- this turns admittance's intended bounded F/K spring into an unbounded
    // integrator. Freezing once, on the cycle idle begins, restores the bounded behavior; on
    // every later idle cycle this branch is skipped and state_reference_ is left untouched.
    state_reference_ = state_current_;
    state_reference_.velocities.assign(num_joints_, 0.0);
  }
  had_active_trajectory_last_cycle_ = has_active_trajectory;

  const auto measured_wrench = readMeasuredWrench();

  if (has_ft_sensor_ && measured_wrench_rt_publisher_->trylock()) {
    measured_wrench_rt_publisher_->msg_.header.stamp = time;
    measured_wrench_rt_publisher_->msg_.header.frame_id = params_.ft_sensor.frame.id;
    measured_wrench_rt_publisher_->msg_.wrench = measured_wrench;
    measured_wrench_rt_publisher_->unlockAndPublish();
  }

  // Bias a LOCAL COPY only -- see this class's own header doc's CORRECTION note. AdmittanceRule
  // drives its measured wrench to zero; adding target_force_n_ here makes "zero" correspond to
  // "loaded at target_force_n_" instead, using the SAME sign convention core_types::Wrench
  // documents (force_z negative while pressing along the tool's own +Z).
  auto biased_wrench = measured_wrench;
  biased_wrench.force.z += *target_force_n_.readFromRT();

  state_desired_.positions = state_reference_.positions;
  state_desired_.velocities = state_reference_.velocities;
  if (enable_redundancy_weighting_) {
    updateWeighted(biased_wrench, period);
  } else {
    admittance_rule_->update(
      state_current_, biased_wrench, state_reference_, period, state_desired_);
  }

  // See clampCartesianOffset()'s own doc comment -- must run on EITHER path above, before the
  // result is ever written to the command interfaces.
  clampCartesianOffset();

  // See enforceJointRateLimits()'s own doc comment -- must run last, after clampCartesianOffset(),
  // so a finite-but-real-joint-limit-violating Cartesian clamp result still gets bounded to what
  // the joint can physically do in one period.
  enforceJointRateLimits(period);

  if (command_guard_) {
    const auto guarded = command_guard_->apply(state_desired_.positions, state_current_.positions);
    if (guarded.bad && !guarded.positions.empty()) {
      state_desired_.positions = guarded.positions;
      std::fill(state_desired_.velocities.begin(), state_desired_.velocities.end(), 0.0);
    }
    if (guarded.faulted) {
      // Hold, report, and fail: never report a dead or out-of-limit axis healthy.
      if (!guarded.positions.empty()) {
        writeCommandsToCommandInterfaces(state_desired_);
      }
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "command guard fault latched (%s): holding position", core_types::toString(guarded.code));
      return controller_interface::return_type::ERROR;
    }
    if (guarded.bad && guarded.positions.empty()) {
      return controller_interface::return_type::OK;  // nothing safe to write this cycle
    }
  }

  writeCommandsToCommandInterfaces(state_desired_);

  return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse JtacController::goalReceivedCallback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJTrajAction::Goal> goal)
{
  if (goal->trajectory.joint_names.empty() || goal->trajectory.points.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "Rejecting goal: empty trajectory.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  // Require an exact match of the joint set -- see this class's header doc's disclosed scope cut
  // (no stock-JTC-style partial-goal filling in this first increment).
  if (goal->trajectory.joint_names.size() != joint_names_.size()) {
    RCLCPP_WARN(get_node()->get_logger(), "Rejecting goal: joint count mismatch.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  for (const auto & name : goal->trajectory.joint_names) {
    if (std::find(joint_names_.begin(), joint_names_.end(), name) == joint_names_.end()) {
      RCLCPP_WARN(
        get_node()->get_logger(), "Rejecting goal: unknown joint '%s'.", name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JtacController::goalCancelledCallback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JtacController::goalAcceptedCallback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  auto trajectory_msg =
    std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal_handle->get_goal()->trajectory);
  sortTrajectoryToLocalOrder(joint_names_, *trajectory_msg);

  const auto goal_handle_ptr =
    std::make_shared<RealtimeGoalHandle>(goal_handle);
  goal_handle_ptr->execute();
  rt_active_goal_.writeFromNonRT(goal_handle_ptr);
  traj_msg_external_point_ptr_.writeFromNonRT(trajectory_msg);
}

}  // namespace compliant_trajectory_controller

PLUGINLIB_EXPORT_CLASS(
  compliant_trajectory_controller::JtacController, controller_interface::ControllerInterface)
