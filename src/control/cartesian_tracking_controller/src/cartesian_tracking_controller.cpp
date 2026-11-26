#include "cartesian_tracking_controller/cartesian_tracking_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "urdf/model.h"

namespace cartesian_tracking_controller
{
namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

const char * const kRefNames[13] = {
  "pose.position.x", "pose.position.y", "pose.position.z",
  "pose.orientation.x", "pose.orientation.y", "pose.orientation.z", "pose.orientation.w",
  "twist.linear.x", "twist.linear.y", "twist.linear.z",
  "twist.angular.x", "twist.angular.y", "twist.angular.z"};

const char * const kDiagLabels[6] = {
  "status", "position_error", "orientation_error", "sigma_min", "twist_scale",
  "joint_limit_margin"};

template<typename T>
void declareIfMissing(rclcpp_lifecycle::LifecycleNode & node, const std::string & name, const T & v)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter(name, v);
  }
}

template<size_t N>
bool allFinite(const std::array<double, N> & a)
{
  return std::all_of(a.begin(), a.end(), [](double x) {return std::isfinite(x);});
}
}  // namespace

controller_interface::InterfaceConfiguration
CartesianTrackingController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joints_) {
    for (const auto & t : command_types_) {
      config.names.push_back(j + "/" + t);
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianTrackingController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joints_) {
    config.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

controller_interface::CallbackReturn CartesianTrackingController::on_init()
{
  try {
    auto & n = *get_node();
    declareIfMissing(n, "joints", std::vector<std::string>{});
    declareIfMissing(n, "command_interfaces", std::vector<std::string>{"velocity"});
    declareIfMissing(
      n, "kinematics.plugin_name", std::string("kinematics_interface_kdl/KinematicsInterfaceKDL"));
    declareIfMissing(n, "kinematics.plugin_package", std::string("kinematics_interface"));
    declareIfMissing(n, "kinematics.base", std::string(""));
    declareIfMissing(n, "kinematics.tip", std::string(""));
    declareIfMissing(n, "kinematics.alpha", 0.0005);
    declareIfMissing(n, "tool.xyz", std::vector<double>{0.0, 0.0, 0.0});
    declareIfMissing(n, "tool.rpy", std::vector<double>{0.0, 0.0, 0.0});
    declareIfMissing(n, "max_linear_twist", 0.1);
    declareIfMissing(n, "max_angular_twist", 0.5);
    declareIfMissing(n, "feedback.kp_linear", 2.0);
    declareIfMissing(n, "feedback.kp_angular", 2.0);
    declareIfMissing(n, "feedback.max_linear", 0.05);
    declareIfMissing(n, "feedback.max_angular", 0.2);
    declareIfMissing(n, "plane.enabled", false);
    declareIfMissing(n, "plane.kp_distance", 2.0);
    declareIfMissing(n, "plane.kp_tilt", 2.0);
    declareIfMissing(n, "plane.max_linear", 0.02);
    declareIfMissing(n, "plane.max_angular", 0.1);
    declareIfMissing(n, "sigma_threshold", 0.05);
    declareIfMissing(n, "lambda_max", 0.1);
    declareIfMissing(n, "joint_weights", std::vector<double>{});
    declareIfMissing(n, "joint_acceleration_limits", std::vector<double>{});
    declareIfMissing(n, "position_margin", 0.0);
    declareIfMissing(n, "reference_timeout", 0.5);
    declareIfMissing(n, "diagnostics_rate", 10.0);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_node()->get_logger(), "on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

std::string CartesianTrackingController::checkChain(const std::string & urdf_xml) const
{
  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    return "robot_description does not parse";
  }
  auto link = model.getLink(tip_);
  if (!link || !model.getLink(base_)) {
    return "kinematics base or tip is not a link of the robot";
  }
  std::vector<std::string> chain;
  while (link->name != base_) {
    const auto joint = link->parent_joint;
    if (!joint) {
      return "kinematics tip is not below the base";
    }
    if (joint->type != urdf::Joint::FIXED) {
      chain.push_back(joint->name);
    }
    link = link->getParent();
  }
  std::reverse(chain.begin(), chain.end());
  if (chain != joints_) {
    return "joints do not match the movable joints from base to tip, in order";
  }
  return "";
}

controller_interface::CallbackReturn CartesianTrackingController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto & node = *get_node();
  const auto logger = node.get_logger();
  auto fail = [&](const std::string & msg) {
      RCLCPP_ERROR(logger, "%s", msg.c_str());
      return controller_interface::CallbackReturn::ERROR;
    };
  try {
    joints_ = node.get_parameter("joints").as_string_array();
    command_types_ = node.get_parameter("command_interfaces").as_string_array();
    base_ = node.get_parameter("kinematics.base").as_string();
    tip_ = node.get_parameter("kinematics.tip").as_string();
    n_ = joints_.size();
    if (n_ == 0 || n_ > static_cast<size_t>(diff_ik_core::kMaxJoints)) {
      return fail("'joints' must hold 1 to 8 names");
    }
    for (const auto & t : command_types_) {
      if (t != "velocity" && t != "position") {
        return fail("command_interfaces may only hold velocity and position");
      }
    }
    write_velocity_ =
      std::find(command_types_.begin(), command_types_.end(), "velocity") != command_types_.end();
    write_position_ =
      std::find(command_types_.begin(), command_types_.end(), "position") != command_types_.end();
    if (!write_velocity_) {
      return fail("command_interfaces must include velocity");
    }
    if (base_.empty() || tip_.empty()) {
      return fail("kinematics.base and kinematics.tip must be set");
    }
    if (get_update_rate() == 0) {
      return fail("update rate is 0, cannot derive dt");
    }
    if (get_robot_description().empty()) {
      return fail("robot_description is empty");
    }
    if (const auto err = checkChain(get_robot_description()); !err.empty()) {
      return fail(err);
    }

    urdf::Model model;
    model.initString(get_robot_description());
    diff_ik_core::TrackerParams tp;
    const int n = static_cast<int>(n_);
    tp.limits.q_min.resize(n);
    tp.limits.q_max.resize(n);
    tp.limits.qd_max.resize(n);
    tp.limits.qdd_max = Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity());
    tp.limits.continuous.assign(n_, false);
    for (int i = 0; i < n; ++i) {
      const auto joint = model.getJoint(joints_[i]);
      if (!joint->limits || joint->limits->velocity <= 0.0) {
        return fail("joint " + joints_[i] + " has no velocity limit in the URDF");
      }
      tp.limits.qd_max[i] = joint->limits->velocity;
      if (joint->type == urdf::Joint::CONTINUOUS) {
        tp.limits.continuous[i] = true;
        tp.limits.q_min[i] = -M_PI;
        tp.limits.q_max[i] = M_PI;
      } else {
        tp.limits.q_min[i] = joint->limits->lower;
        tp.limits.q_max[i] = joint->limits->upper;
      }
    }
    const auto qdd = node.get_parameter("joint_acceleration_limits").as_double_array();
    if (!qdd.empty()) {
      if (qdd.size() != n_) {
        return fail("joint_acceleration_limits must be empty or one value per joint");
      }
      tp.limits.qdd_max = Eigen::Map<const Eigen::VectorXd>(qdd.data(), n);
    }
    tp.limits.position_margin = node.get_parameter("position_margin").as_double();
    const auto w = node.get_parameter("joint_weights").as_double_array();
    if (!w.empty()) {
      if (w.size() != n_) {
        return fail("joint_weights must be empty or one value per joint");
      }
      tp.ik.joint_weights = Eigen::Map<const Eigen::VectorXd>(w.data(), n);
    }
    tp.ik.sigma_threshold = node.get_parameter("sigma_threshold").as_double();
    tp.ik.lambda_max = node.get_parameter("lambda_max").as_double();

    tp.feedback.kp_linear = node.get_parameter("feedback.kp_linear").as_double();
    tp.feedback.kp_angular = node.get_parameter("feedback.kp_angular").as_double();
    tp.feedback.max_linear = node.get_parameter("feedback.max_linear").as_double();
    tp.feedback.max_angular = node.get_parameter("feedback.max_angular").as_double();
    plane_enabled_ = node.get_parameter("plane.enabled").as_bool();
    tp.plane_enabled = plane_enabled_;
    tp.plane.kp_distance = node.get_parameter("plane.kp_distance").as_double();
    tp.plane.kp_tilt = node.get_parameter("plane.kp_tilt").as_double();
    tp.plane.max_linear = node.get_parameter("plane.max_linear").as_double();
    tp.plane.max_angular = node.get_parameter("plane.max_angular").as_double();
    tp.max_linear_twist = node.get_parameter("max_linear_twist").as_double();
    tp.max_angular_twist = node.get_parameter("max_angular_twist").as_double();
    tp.dt = 1.0 / static_cast<double>(get_update_rate());

    const auto xyz = node.get_parameter("tool.xyz").as_double_array();
    const auto rpy = node.get_parameter("tool.rpy").as_double_array();
    if (xyz.size() != 3 || rpy.size() != 3) {
      return fail("tool.xyz and tool.rpy need three values each");
    }
    tp.tool.xyz = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
    tp.tool.rpy = Eigen::Vector3d(rpy[0], rpy[1], rpy[2]);

    reference_timeout_ = node.get_parameter("reference_timeout").as_double();
    const double diag_rate = node.get_parameter("diagnostics_rate").as_double();
    diagnostics_period_ = diag_rate > 0.0 ? 1.0 / diag_rate : -1.0;

    ik_loader_ = std::make_unique<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
      node.get_parameter("kinematics.plugin_package").as_string(),
      "kinematics_interface::KinematicsInterface");
    ik_.reset(ik_loader_->createUnmanagedInstance(
        node.get_parameter("kinematics.plugin_name").as_string()));
    if (!ik_->initialize(
        get_robot_description(), node.get_node_parameters_interface(), "kinematics"))
    {
      return fail("kinematics plugin initialize() failed");
    }

    tracker_ = std::make_unique<diff_ik_core::CartesianTracker>(tp);
  } catch (const std::exception & e) {
    return fail(std::string("on_configure: ") + e.what());
  }

  q_ = Eigen::VectorXd::Zero(n_);
  hold_q_ = Eigen::VectorXd::Zero(n_);
  jac_.resize(6, n_);
  jac_.setZero();

  pose_in_.writeFromNonRT(Input<7>{});
  twist_in_.writeFromNonRT(Input<6>{});
  plane_in_.writeFromNonRT(Input<7>{});
  tool_in_.writeFromNonRT(Input<6>{});

  pose_sub_ = node.create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/reference_pose", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {
      const auto & p = m->pose;
      Input<7> in;
      in.v = {p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y,
        p.orientation.z, p.orientation.w};
      in.seq = ++pose_seq_;
      pose_in_.writeFromNonRT(in);
    });
  twist_sub_ = node.create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/reference_twist", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr m) {
      const auto & t = m->twist;
      Input<6> in;
      in.v = {t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z};
      in.seq = ++twist_seq_;
      twist_in_.writeFromNonRT(in);
    });
  plane_sub_ = node.create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/plane_measurement", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
      if (m->data.size() != 7) {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "plane_measurement needs 7 values [nx ny nz standoff_error ax ay az]");
        return;
      }
      Input<7> in;
      std::copy(m->data.begin(), m->data.end(), in.v.begin());
      in.seq = ++plane_seq_;
      plane_in_.writeFromNonRT(in);
    });
  tool_sub_ = node.create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/tool_offset", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
      if (m->data.size() != 6) {
        RCLCPP_WARN(get_node()->get_logger(), "tool_offset needs 6 values [x y z roll pitch yaw]");
        return;
      }
      Input<6> in;
      std::copy(m->data.begin(), m->data.end(), in.v.begin());
      if (!allFinite(in.v)) {
        RCLCPP_WARN(get_node()->get_logger(), "tool_offset is not finite, ignored");
        return;
      }
      in.seq = ++tool_seq_;
      tool_in_.writeFromNonRT(in);
    });

  diag_pub_ = node.create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/diagnostics", rclcpp::SystemDefaultsQoS());
  rt_diag_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
    diag_pub_);
  rt_diag_->msg_.data.assign(6, kNaN);
  rt_diag_->msg_.layout.dim.resize(6);
  for (size_t i = 0; i < 6; ++i) {
    rt_diag_->msg_.layout.dim[i].label = kDiagLabels[i];
    rt_diag_->msg_.layout.dim[i].size = 1;
    rt_diag_->msg_.layout.dim[i].stride = 1;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
CartesianTrackingController::on_export_reference_interfaces_list()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> out;
  for (const auto * name : kRefNames) {
    hardware_interface::InterfaceInfo info;
    info.name = name;
    info.data_type = "double";
    info.size = 1;
    info.enable_limits = false;
    out.push_back(
      std::make_shared<hardware_interface::CommandInterface>(
        hardware_interface::InterfaceDescription(get_node()->get_name(), info)));
  }
  return out;
}

bool CartesianTrackingController::on_set_chained_mode(bool)
{
  return true;
}

controller_interface::CallbackReturn CartesianTrackingController::on_activate(
  const rclcpp_lifecycle::State &)
{
  tracker_->reset();
  holding_ = false;
  since_diag_ = 1e9;
  ref_state_ = RefState::None;
  twist_fresh_ = false;
  ref_.fill(kNaN);
  // messages from before activation do not count
  seen_pose_ = pose_in_.readFromNonRT()->seq;
  seen_twist_ = twist_in_.readFromNonRT()->seq;
  seen_plane_ = plane_in_.readFromNonRT()->seq;
  seen_tool_ = tool_in_.readFromNonRT()->seq;
  have_pose_ = have_twist_ = have_plane_ = false;
  for (auto & c : command_interfaces_) {
    if (c.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
      (void)c.set_value(0.0);
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianTrackingController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (auto & c : command_interfaces_) {
    if (c.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
      (void)c.set_value(0.0);
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianTrackingController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  auto fresh = [&](bool have, const rclcpp::Time & rx) {
      return have && (reference_timeout_ <= 0.0 || (time - rx).seconds() <= reference_timeout_);
    };

  const auto * p = pose_in_.readFromRT();
  if (p->seq != seen_pose_) {
    seen_pose_ = p->seq;
    pose_rx_ = time;
    have_pose_ = true;
  }
  const auto * t = twist_in_.readFromRT();
  if (t->seq != seen_twist_) {
    seen_twist_ = t->seq;
    twist_rx_ = time;
    have_twist_ = true;
  }

  if (!have_pose_) {
    ref_state_ = RefState::None;
  } else if (!fresh(true, pose_rx_)) {
    ref_state_ = RefState::Stale;
  } else {
    ref_state_ = RefState::Fresh;
  }
  if (ref_state_ == RefState::Fresh) {
    std::copy(p->v.begin(), p->v.end(), ref_.begin());
  } else {
    std::fill(ref_.begin(), ref_.begin() + 7, kNaN);
  }
  // the twist is optional feedforward: none or stale means zero
  twist_fresh_ = fresh(have_twist_, twist_rx_);
  if (twist_fresh_) {
    std::copy(t->v.begin(), t->v.end(), ref_.begin() + 7);
  } else {
    std::fill(ref_.begin() + 7, ref_.end(), 0.0);
  }
  return controller_interface::return_type::OK;
}

void CartesianTrackingController::applyToolOffset(const rclcpp::Time &)
{
  const auto * t = tool_in_.readFromRT();
  if (t->seq == seen_tool_) {
    return;
  }
  seen_tool_ = t->seq;
  diff_ik_core::ToolOffset off;
  off.xyz = Eigen::Vector3d(t->v[0], t->v[1], t->v[2]);
  off.rpy = Eigen::Vector3d(t->v[3], t->v[4], t->v[5]);
  tracker_->setToolOffset(off);
}

void CartesianTrackingController::hold(Status status, const Eigen::VectorXd & q, bool q_valid)
{
  if (!holding_ && q_valid) {
    hold_q_ = q;
    holding_ = true;
  }
  const size_t nt = command_types_.size();
  for (size_t j = 0; j < n_; ++j) {
    for (size_t k = 0; k < nt; ++k) {
      if (command_types_[k] == hardware_interface::HW_IF_VELOCITY) {
        (void)command_interfaces_[j * nt + k].set_value(0.0);
      } else if (holding_) {
        (void)command_interfaces_[j * nt + k].set_value(hold_q_[j]);
      }
    }
  }
  publishDiagnostics(status, nullptr);
}

void CartesianTrackingController::publishDiagnostics(
  Status status, const diff_ik_core::TrackerOutput * out)
{
  if (diagnostics_period_ <= 0.0 || since_diag_ < diagnostics_period_ - 1e-9) {
    return;
  }
  if (!rt_diag_->trylock()) {
    return;
  }
  since_diag_ = 0.0;
  auto & d = rt_diag_->msg_.data;
  d[0] = static_cast<double>(static_cast<int>(status));
  d[1] = out ? out->position_error : kNaN;
  d[2] = out ? out->orientation_error : kNaN;
  d[3] = out ? out->sigma_min : kNaN;
  d[4] = out ? out->twist_scale : kNaN;
  d[5] = out ? out->joint_limit_margin : kNaN;
  rt_diag_->unlockAndPublish();
}

controller_interface::return_type CartesianTrackingController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  since_diag_ += period.seconds();
  applyToolOffset(time);

  bool q_ok = true;
  for (size_t j = 0; j < n_; ++j) {
    const auto v = state_interfaces_[j].get_optional();
    q_(j) = v ? *v : kNaN;
    q_ok = q_ok && std::isfinite(q_(j));
  }
  if (!q_ok) {
    hold(Status::BadJointState, q_, false);
    return controller_interface::return_type::OK;
  }

  if (is_in_chained_mode()) {
    for (size_t i = 0; i < kRefSize; ++i) {
      const auto v = ordered_exported_reference_interfaces_[i]->get_optional();
      ref_[i] = v ? *v : kNaN;
    }
    ref_state_ = RefState::Fresh;
  }

  if (ref_state_ != RefState::Fresh) {
    hold(ref_state_ == RefState::None ? Status::NoReference : Status::StaleReference, q_, true);
    return controller_interface::return_type::OK;
  }
  const double qn = std::sqrt(
    ref_[3] * ref_[3] + ref_[4] * ref_[4] + ref_[5] * ref_[5] + ref_[6] * ref_[6]);
  if (!std::all_of(ref_.begin(), ref_.end(), [](double x) {return std::isfinite(x);}) ||
    qn < 1e-6)
  {
    hold(Status::InvalidReference, q_, true);
    return controller_interface::return_type::OK;
  }

  Eigen::Isometry3d flange;
  if (!ik_->calculate_link_transform(q_, tip_, flange) ||
    !ik_->calculate_jacobian(q_, tip_, jac_))
  {
    hold(Status::KinematicsFailed, q_, true);
    return controller_interface::return_type::OK;
  }

  diff_ik_core::TrackerReference ref;
  ref.pose.linear() =
    Eigen::Quaterniond(ref_[6] / qn, ref_[3] / qn, ref_[4] / qn, ref_[5] / qn).toRotationMatrix();
  ref.pose.translation() = Eigen::Vector3d(ref_[0], ref_[1], ref_[2]);
  for (int i = 0; i < 6; ++i) {
    ref.twist[i] = ref_[7 + i];
  }

  std::optional<diff_ik_core::PlaneMeasurement> plane;
  if (plane_enabled_) {
    const auto * pl = plane_in_.readFromRT();
    if (pl->seq != seen_plane_) {
      seen_plane_ = pl->seq;
      plane_rx_ = time;
      have_plane_ = true;
    }
    const bool plane_fresh =
      have_plane_ && (reference_timeout_ <= 0.0 || (time - plane_rx_).seconds() <= reference_timeout_);
    if (plane_fresh && allFinite(pl->v)) {
      diff_ik_core::PlaneMeasurement m;
      m.normal = Eigen::Vector3d(pl->v[0], pl->v[1], pl->v[2]);
      m.standoff_error = pl->v[3];
      m.tool_axis = Eigen::Vector3d(pl->v[4], pl->v[5], pl->v[6]);
      plane = m;
    }
  }

  const auto out = tracker_->step(q_, jac_, flange, ref, plane);
  if (!out.ok()) {
    hold(Status::TrackerInvalid, q_, true);
    return controller_interface::return_type::OK;
  }

  holding_ = false;
  const size_t nt = command_types_.size();
  for (size_t j = 0; j < n_; ++j) {
    for (size_t k = 0; k < nt; ++k) {
      const bool vel = command_types_[k] == hardware_interface::HW_IF_VELOCITY;
      (void)command_interfaces_[j * nt + k].set_value(vel ? out.qdot[j] : out.q_next[j]);
    }
  }
  publishDiagnostics(Status::Ok, &out);
  return controller_interface::return_type::OK;
}

}  // namespace cartesian_tracking_controller

PLUGINLIB_EXPORT_CLASS(
  cartesian_tracking_controller::CartesianTrackingController,
  controller_interface::ChainableControllerInterface)
