#include "ft_conditioner/wrench_conditioner_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ft_conditioner
{
namespace
{
using force_core::Mat3;
using force_core::Vec3;

core_types::Wrench toCore(const geometry_msgs::msg::Wrench & w)
{
  core_types::Wrench c;
  c.force_x = w.force.x; c.force_y = w.force.y; c.force_z = w.force.z;
  c.torque_x = w.torque.x; c.torque_y = w.torque.y; c.torque_z = w.torque.z;
  return c;
}

Mat3 fromRpy(double roll, double pitch, double yaw)    // URDF convention: Rz(yaw) Ry(pitch) Rx(roll)
{
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  Mat3 r;
  r.m = {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr};
  return r;
}

Vec3 vec(const char * name, const std::vector<double> & v)
{
  if (v.size() != 3) {
    throw std::invalid_argument(
      std::string("parameter '") + name + "' needs 3 values, got " + std::to_string(v.size()));
  }
  return Vec3{v[0], v[1], v[2]};
}

Mat3 mul3(const Mat3 & a, const Mat3 & b)
{
  Mat3 r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 3; ++k) {sum += a.at(i, k) * b.at(k, j);}
      r.m[3 * i + j] = sum;
    }
  }
  return r;
}

Mat3 axisAngle(const Vec3 & axis, double angle)     // Rodrigues, axis need not be normalised
{
  const double n = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
  const double x = axis.x / n, y = axis.y / n, z = axis.z / n;
  const double c = std::cos(angle), s = std::sin(angle), t = 1.0 - c;
  Mat3 r;
  r.m = {t * x * x + c, t * x * y - s * z, t * x * z + s * y,
    t * x * y + s * z, t * y * y + c, t * y * z - s * x,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c};
  return r;
}

geometry_msgs::msg::WrenchStamped toMsg(const core_types::Wrench & w, const rclcpp::Time & stamp,
  const std::string & frame)
{
  geometry_msgs::msg::WrenchStamped out;
  out.header.stamp = stamp;
  out.header.frame_id = frame;
  out.wrench.force.x = w.force_x; out.wrench.force.y = w.force_y; out.wrench.force.z = w.force_z;
  out.wrench.torque.x = w.torque_x; out.wrench.torque.y = w.torque_y; out.wrench.torque.z = w.torque_z;
  return out;
}

Mat3 toMat(const geometry_msgs::msg::Quaternion & q)
{
  return force_core::matrixFromQuaternion(q.x, q.y, q.z, q.w);
}
}  // namespace

WrenchConditionerNode::WrenchConditionerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("wrench_conditioner", options)
{
  const auto in_topic = declare_parameter<std::string>("input_topic", "wrench_raw");
  const auto out_topic = declare_parameter<std::string>("output_topic", "wrench_conditioned");
  world_frame_ = declare_parameter<std::string>("world_frame", "base_footprint");
  tool_frame_ = declare_parameter<std::string>("tool_frame", "tcp_link");
  sensor_frame_param_ = declare_parameter<std::string>("sensor_frame", "");
  max_tf_age_s_ = declare_parameter<double>("max_tf_age", 0.1);
  capture_window_s_ = declare_parameter<double>("capture_window_s", 0.5);
  capture_max_age_s_ = declare_parameter<double>("capture_max_age_s", 0.25);
  capture_max_spread_n_ = declare_parameter<double>("capture_max_spread_n", 1.0);
  capture_max_rotation_rad_ = declare_parameter<double>("capture_max_rotation_rad", 0.02);
  capture_min_samples_ = static_cast<std::size_t>(
    std::max<int64_t>(1, declare_parameter<int64_t>("capture_min_samples", 5)));
  offset_from_tf_ = declare_parameter<bool>("tool_in_sensor_from_tf", true);
  const auto off_xyz = declare_parameter<std::vector<double>>("tool_in_sensor_xyz", {0.0, 0.0, 0.0});
  const auto off_rpy = declare_parameter<std::vector<double>>("tool_in_sensor_rpy", {0.0, 0.0, 0.0});
  config_.sign = declare_parameter<double>("sign", 1.0);
  config_.payload.mass_kg = declare_parameter<double>("payload_mass_kg", 0.0);
  config_.payload.cog = vec("payload_cog_xyz", declare_parameter<std::vector<double>>("payload_cog_xyz", {0.0, 0.0, 0.0}));
  const auto bias = declare_parameter<std::vector<double>>("bias", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  config_.gravity_world = vec("gravity_world", declare_parameter<std::vector<double>>("gravity_world", {0.0, 0.0, -9.80665}));
  if (bias.size() >= 6) {
    config_.bias.force_x = bias[0]; config_.bias.force_y = bias[1]; config_.bias.force_z = bias[2];
    config_.bias.torque_x = bias[3]; config_.bias.torque_y = bias[4]; config_.bias.torque_z = bias[5];
  } else {
    RCLCPP_WARN(get_logger(), "bias needs 6 values, got %zu: using zero", bias.size());
  }
  if (std::abs(config_.sign) != 1.0) {
    RCLCPP_WARN(get_logger(), "sign must be +1 or -1, got %g: using +1", config_.sign);
    config_.sign = 1.0;
  }
  offset_param_.p = vec("tool_in_sensor_xyz", off_xyz);
  offset_param_.R = off_rpy.size() >= 3 ? fromRpy(off_rpy[0], off_rpy[1], off_rpy[2]) : Mat3{};

  reference_bias_ = config_.bias;
  bias_tracking_ = declare_parameter<bool>("bias_tracking.enabled", false);
  bias_apply_ = declare_parameter<bool>("bias_tracking.apply", false);
  const auto cf_topic = declare_parameter<std::string>("bias_tracking.contact_free_topic", "contact_free");
  const auto js_topic = declare_parameter<std::string>("bias_tracking.joint_states_topic", "joint_states");
  still_velocity_rad_s_ = declare_parameter<double>("bias_tracking.still_velocity_rad_s", 0.01);
  contact_free_timeout_s_ = declare_parameter<double>("bias_tracking.contact_free_timeout_s", 1.0);
  joint_state_timeout_s_ = declare_parameter<double>("bias_tracking.joint_state_timeout_s", 0.5);
  bias_cfg_.process_noise_force = declare_parameter<double>("bias_tracking.process_noise_force", bias_cfg_.process_noise_force);
  bias_cfg_.process_noise_torque = declare_parameter<double>("bias_tracking.process_noise_torque", bias_cfg_.process_noise_torque);
  bias_cfg_.measurement_noise_force = declare_parameter<double>("bias_tracking.measurement_noise_force", bias_cfg_.measurement_noise_force);
  bias_cfg_.measurement_noise_torque = declare_parameter<double>("bias_tracking.measurement_noise_torque", bias_cfg_.measurement_noise_torque);
  bias_cfg_.reject_threshold_force = declare_parameter<double>("bias_tracking.reject_threshold_force", bias_cfg_.reject_threshold_force);
  bias_cfg_.reject_threshold_torque = declare_parameter<double>("bias_tracking.reject_threshold_torque", bias_cfg_.reject_threshold_torque);
  bias_cfg_.max_total_correction_force = declare_parameter<double>("bias_tracking.max_total_correction_force", bias_cfg_.max_total_correction_force);
  bias_cfg_.max_total_correction_torque = declare_parameter<double>("bias_tracking.max_total_correction_torque", bias_cfg_.max_total_correction_torque);
  bias_cfg_.max_consecutive_rejects = static_cast<std::size_t>(std::max<int64_t>(
      1, declare_parameter<int64_t>("bias_tracking.max_consecutive_rejects",
      static_cast<int64_t>(bias_cfg_.max_consecutive_rejects))));
  // Joint-space inputs of ~/plan_excitation (read when the service is called).
  declare_parameter<std::vector<double>>("excitation.q_start", std::vector<double>{});
  declare_parameter<std::vector<double>>("excitation.q_min", std::vector<double>{});
  declare_parameter<std::vector<double>>("excitation.q_max", std::vector<double>{});
  declare_parameter<std::vector<double>>("excitation.v_max", std::vector<double>{});
  declare_parameter<std::vector<double>>("excitation.a_max", std::vector<double>{});
  declare_parameter<std::vector<double>>("excitation.joint_axes", std::vector<double>{});
  declare_parameter<std::vector<double>>("excitation.home_rpy", std::vector<double>{0.0, 0.0, 0.0});
  declare_parameter<double>("excitation.margin_rad", 0.1);
  declare_parameter<double>("excitation.velocity_scale", 0.5);
  declare_parameter<int64_t>("excitation.num_poses", 8);
  declare_parameter<int64_t>("excitation.num_candidates", 400);
  declare_parameter<int64_t>("excitation.seed", 1);
  declare_parameter<double>("excitation.dwell_s", 1.5);
  declare_parameter<double>("excitation.max_condition", 20.0);
  if (bias_tracking_) {tracker_ = std::make_unique<force_core::BiasTracker>(reference_bias_, bias_cfg_);}

  param_cb_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) {return onParameters(p);});
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
  pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(out_topic, rclcpp::SensorDataQoS());
  sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
    in_topic, rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::WrenchStamped::SharedPtr m) {onRaw(m);});
  bias_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("~/tracked_bias", rclcpp::QoS(1).transient_local());
  recal_pub_ = create_publisher<std_msgs::msg::Bool>("~/recalibrate_needed", rclcpp::QoS(1).transient_local());
  contact_free_sub_ = create_subscription<std_msgs::msg::Bool>(
    cf_topic, rclcpp::QoS(10), [this](const std_msgs::msg::Bool::SharedPtr m) {
      std::lock_guard<std::mutex> lock(mutex_);
      contact_free_ = m->data;
      have_contact_free_ = true;
      contact_free_stamp_ = get_clock()->now();
    });
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    js_topic, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::JointState::SharedPtr m) {
      double peak = 0.0;
      for (const double v : m->velocity) {peak = std::max(peak, std::abs(v));}
      std::lock_guard<std::mutex> lock(mutex_);
      // A joint state without velocities cannot show the arm is still: treat as moving.
      max_joint_speed_ = m->velocity.empty() ? std::numeric_limits<double>::infinity() : peak;
      have_joint_state_ = true;
      joint_state_stamp_ = get_clock()->now();
    });
  using Trigger = std_srvs::srv::Trigger;
  using Req = std::shared_ptr<Trigger::Request>;
  using Res = std::shared_ptr<Trigger::Response>;
  capture_srv_ = create_service<Trigger>("~/capture_pose", [this](const Req q, Res r) {captureSample(q, r);});
  identify_srv_ = create_service<Trigger>("~/identify", [this](const Req q, Res r) {identify(q, r);});
  clear_srv_ = create_service<Trigger>("~/clear_samples", [this](const Req q, Res r) {clearSamples(q, r);});
  reset_bias_srv_ = create_service<Trigger>("~/reset_bias_tracker", [this](const Req q, Res r) {resetBiasTracker(q, r);});
  plan_srv_ = create_service<Trigger>("~/plan_excitation", [this](const Req q, Res r) {planExcitation(q, r);});
  if (bias_tracking_) {
    std_msgs::msg::Bool f;
    recal_pub_->publish(f);
    RCLCPP_INFO(get_logger(), "bias tracking on (%s): contact_free '%s', joint states '%s', still below %.3f rad/s",
      bias_apply_ ? "applied to the conditioner" : "published only", cf_topic.c_str(), js_topic.c_str(),
      still_velocity_rad_s_);
  }
  RCLCPP_INFO(get_logger(), "conditioning %s -> %s in frame '%s' (world '%s')", in_topic.c_str(),
    out_topic.c_str(), tool_frame_.c_str(), world_frame_.c_str());
}

rcl_interfaces::msg::SetParametersResult WrenchConditionerNode::onParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  auto fail = [&result](const std::string & why) {result.successful = false; result.reason = why;};
  force_core::ConditionerConfig next;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    next = config_;
  }
  bool reset_tracker = false;
  core_types::Wrench ref;
  bool apply = false, enabled = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ref = reference_bias_;
    apply = bias_apply_;
    enabled = bias_tracking_;
  }
  for (const auto & p : params) {
    const auto & n = p.get_name();
    if (n == "bias_tracking.enabled") {
      if (p.as_bool() != enabled) {reset_tracker = true;}
      enabled = p.as_bool();
      continue;
    }
    if (n == "bias_tracking.apply") {
      apply = p.as_bool();
      reset_tracker = true;
      continue;
    }
    if (n == "payload_mass_kg" || n == "payload_cog_xyz" || n == "gravity_world" || n == "sign" || n == "bias") {
      reset_tracker = true;       // the tracked residual depends on all of them
    }
    if (n == "payload_mass_kg") {
      next.payload.mass_kg = p.as_double();
    } else if (n == "sign") {
      if (std::abs(p.as_double()) != 1.0) {fail("sign must be +1 or -1"); return result;}
      next.sign = p.as_double();
    } else if (n == "payload_cog_xyz" || n == "gravity_world") {
      const auto v = p.as_double_array();
      if (v.size() != 3) {fail(n + " needs 3 values"); return result;}
      (n == "gravity_world" ? next.gravity_world : next.payload.cog) = Vec3{v[0], v[1], v[2]};
    } else if (n == "bias") {
      const auto v = p.as_double_array();
      if (v.size() != 6) {fail("bias needs 6 values"); return result;}
      next.bias.force_x = v[0]; next.bias.force_y = v[1]; next.bias.force_z = v[2];
      next.bias.torque_x = v[3]; next.bias.torque_y = v[4]; next.bias.torque_z = v[5];
      ref = next.bias;
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto offset = config_.tool_in_sensor;
  config_ = next;
  config_.tool_in_sensor = offset;
  reference_bias_ = ref;
  bias_apply_ = apply;
  bias_tracking_ = enabled;
  if (reset_tracker) {
    tracker_.reset();
    if (enabled) {tracker_ = std::make_unique<force_core::BiasTracker>(ref, bias_cfg_);}
    config_.bias = ref;      // restart from the reference bias (also when apply was just turned off)
    recal_state_ = false;
  }
  return result;
}

std::size_t WrenchConditionerNode::samples() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.size();
}

std::vector<force_core::CalibrationSample> WrenchConditionerNode::calibrationSamples() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_;
}

force_core::ConditionerConfig WrenchConditionerNode::config() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

bool WrenchConditionerNode::sensorOrientation(
  const std::string & sensor_frame, const rclcpp::Time & stamp, Mat3 & r_world_sensor)
{
  try {
    r_world_sensor = toMat(tf_buffer_->lookupTransform(world_frame_, sensor_frame, stamp).transform.rotation);
    return true;
  } catch (const tf2::TransformException &) {
    // The transform for this exact stamp is not there (yet): use the newest one, but only while it is
    // recent. A stalled TF publisher must not leave us compensating gravity with a frozen orientation.
    try {
      const auto t = tf_buffer_->lookupTransform(world_frame_, sensor_frame, tf2::TimePointZero);
      if (!tfFreshEnough(stamp.nanoseconds(), rclcpp::Time(t.header.stamp).nanoseconds(), max_tf_age_s_)) {
        ++stale_tf_;
        return false;
      }
      r_world_sensor = toMat(t.transform.rotation);
      ++stamp_fallbacks_;
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }
}

bool WrenchConditionerNode::toolOffset(const std::string & sensor_frame, force_core::FrameOffset & out)
{
  if (!offset_from_tf_) {
    out = offset_param_;
    return true;
  }
  const auto hit = offset_cache_.find(sensor_frame);
  if (hit != offset_cache_.end()) {
    out = hit->second;
    return true;
  }
  try {
    const auto t = tf_buffer_->lookupTransform(sensor_frame, tool_frame_, tf2::TimePointZero).transform;
    force_core::FrameOffset o;
    o.p = {t.translation.x, t.translation.y, t.translation.z};
    o.R = toMat(t.rotation);
    offset_cache_[sensor_frame] = o;      // the sensor is rigidly mounted to the tool: look it up once per frame
    out = o;
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

void WrenchConditionerNode::onRaw(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  const std::string sensor = sensor_frame_param_.empty() ? msg->header.frame_id : sensor_frame_param_;
  Mat3 r;
  force_core::FrameOffset off;
  if (!sensorOrientation(sensor, rclcpp::Time(msg->header.stamp), r) || !toolOffset(sensor, off)) {
    ++dropped_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "no usable TF for '%s' (world '%s', tool '%s'; a newest transform older than max_tf_age %.3f s "
      "counts as none): wrench dropped (%zu so far, %zu for a stale transform)", sensor.c_str(),
      world_frame_.c_str(), tool_frame_.c_str(), max_tf_age_s_, dropped_, stale_tf_);
    return;
  }
  force_core::ConditionerConfig cfg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.tool_in_sensor = off;
    cfg = config_;
    const rclcpp::Time now = get_clock()->now();
    recent_.push_back({now, r, toCore(msg->wrench)});
    while (!recent_.empty() && (now - recent_.front().received).seconds() > capture_window_s_) {
      recent_.pop_front();
    }
  }
  const core_types::Wrench w = force_core::WrenchConditioner(cfg).condition(toCore(msg->wrench), r);
  pub_->publish(toMsg(w, msg->header.stamp, tool_frame_));
  trackBias(toCore(msg->wrench), r, msg->header.stamp, sensor);
  ++published_;
}

void WrenchConditionerNode::trackBias(
  const core_types::Wrench & raw, const Mat3 & r, const rclcpp::Time & stamp, const std::string & sensor_frame)
{
  core_types::Wrench bias;
  bool recal = false, send_recal = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!bias_tracking_ || !tracker_) {return;}
    const rclcpp::Time now = get_clock()->now();
    const bool free = have_contact_free_ && contact_free_ &&
      (now - contact_free_stamp_).seconds() <= contact_free_timeout_s_;
    const bool still = have_joint_state_ && max_joint_speed_ < still_velocity_rad_s_ &&
      (now - joint_state_stamp_).seconds() <= joint_state_timeout_s_;
    // What the bias should equal at rest: the sign-applied reading minus the payload's weight.
    const core_types::Wrench g = force_core::gravityWrench(config_.payload, r, config_.gravity_world);
    core_types::Wrench residual;
    residual.force_x = config_.sign * raw.force_x - g.force_x;
    residual.force_y = config_.sign * raw.force_y - g.force_y;
    residual.force_z = config_.sign * raw.force_z - g.force_z;
    residual.torque_x = config_.sign * raw.torque_x - g.torque_x;
    residual.torque_y = config_.sign * raw.torque_y - g.torque_y;
    residual.torque_z = config_.sign * raw.torque_z - g.torque_z;
    tracker_->update(residual, free, still);
    bias = tracker_->bias();
    if (bias_apply_) {config_.bias = bias;}
    recal = tracker_->needsRecalibration();
    if (recal != recal_state_) {recal_state_ = recal; send_recal = true;}
  }
  bias_pub_->publish(toMsg(bias, stamp, sensor_frame));
  if (send_recal) {
    std_msgs::msg::Bool m;
    m.data = recal;
    recal_pub_->publish(m);
    if (recal) {
      RCLCPP_WARN(get_logger(), "bias tracker keeps rejecting samples: payload or sensor changed, re-run ~/identify");
    }
  }
}

core_types::Wrench WrenchConditionerNode::trackedBias() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return tracker_ ? tracker_->bias() : config_.bias;
}

std::size_t WrenchConditionerNode::biasAccepted() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return tracker_ ? tracker_->accepted() : 0;
}

std::size_t WrenchConditionerNode::biasRejected() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return tracker_ ? tracker_->rejected() : 0;
}

bool WrenchConditionerNode::recalibrationNeeded() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return tracker_ && tracker_->needsRecalibration();
}

void WrenchConditionerNode::resetBiasTracker(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  bool was_recal = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!bias_tracking_) {
      res->message = "bias tracking is off (bias_tracking.enabled:=true)";
      return;
    }
    tracker_ = std::make_unique<force_core::BiasTracker>(reference_bias_, bias_cfg_);
    config_.bias = reference_bias_;
    was_recal = recal_state_;
    recal_state_ = false;
  }
  if (was_recal) {
    std_msgs::msg::Bool m;
    recal_pub_->publish(m);
  }
  res->success = true;
  res->message = "tracker reset to the `bias` parameter";
}

void WrenchConditionerNode::planExcitation(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  const auto vecp = [this](const char * n) {return get_parameter(n).as_double_array();};
  force_core::ExcitationLimits lim;
  lim.q_min = vecp("excitation.q_min");
  lim.q_max = vecp("excitation.q_max");
  lim.v_max = vecp("excitation.v_max");
  lim.a_max = vecp("excitation.a_max");
  lim.margin_rad = get_parameter("excitation.margin_rad").as_double();
  lim.velocity_scale = get_parameter("excitation.velocity_scale").as_double();
  const auto q0 = vecp("excitation.q_start");
  const auto axes = vecp("excitation.joint_axes");
  const auto rpy = vecp("excitation.home_rpy");
  const std::size_t n = q0.size();
  if (n == 0 || lim.q_min.size() != n || lim.q_max.size() != n || lim.v_max.size() != n ||
    lim.a_max.size() != n || axes.size() != 3 * n || rpy.size() != 3)
  {
    res->message = "set excitation.q_start, q_min, q_max, v_max, a_max (n values each), joint_axes (3 n values, "
      "unit axes of the joints at q = 0 in the base frame) and home_rpy (3 values); got n=" + std::to_string(n);
    return;
  }
  std::vector<Vec3> ax;
  for (std::size_t i = 0; i < n; ++i) {
    const Vec3 a{axes[3 * i], axes[3 * i + 1], axes[3 * i + 2]};
    if (a.x * a.x + a.y * a.y + a.z * a.z < 1e-12) {res->message = "a zero joint axis in excitation.joint_axes"; return;}
    ax.push_back(a);
  }
  const Mat3 home = fromRpy(rpy[0], rpy[1], rpy[2]);
  // Rotation-only forward kinematics of a serial revolute chain (product of exponentials):
  // R_base_sensor(q) = exp(a1 q1) ... exp(an qn) R_home. Positions do not matter for gravity directions.
  const force_core::ForwardKinematics fk = [ax, home](const std::vector<double> & q) {
      Mat3 r;      // identity
      for (std::size_t i = 0; i < ax.size(); ++i) {r = mul3(r, axisAngle(ax[i], q[i]));}
      return mul3(r, home);
    };
  force_core::ExcitationOptions opt;
  opt.num_poses = static_cast<std::size_t>(std::max<int64_t>(1, get_parameter("excitation.num_poses").as_int()));
  opt.num_candidates = static_cast<std::size_t>(std::max<int64_t>(1, get_parameter("excitation.num_candidates").as_int()));
  opt.seed = static_cast<std::uint32_t>(get_parameter("excitation.seed").as_int());
  opt.dwell_s = get_parameter("excitation.dwell_s").as_double();
  opt.max_condition = get_parameter("excitation.max_condition").as_double();
  Vec3 g;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    g = config_.gravity_world;
  }
  // No self-collision proxy here (empty = always safe): the poses are inside the joint limits only.
  const auto plan = force_core::generateExcitationPoses(q0, lim, fk, force_core::SafetyProxy{}, g, opt);
  if (!plan.ok) {
    res->message = "plan failed: " + plan.message;
    return;
  }
  std::ostringstream o;
  o.precision(4);
  o << plan.waypoints.size() << " waypoints, " << plan.duration_s << " s, regressor condition number "
    << plan.condition_number << (plan.condition_number > opt.max_condition ? " (ABOVE max_condition, poses poorly spread)" : "")
    << ", smallest angle between gravity directions " << plan.min_pair_angle_rad << " rad. Joint limits only: no "
    << "self-collision or workspace check, review every pose before moving the arm. Nothing was moved.\n"
    << "waypoints:\n";
  for (const auto & wp : plan.waypoints) {
    o << "- {t_arrive_s: " << wp.t_arrive_s << ", t_leave_s: " << wp.t_leave_s << ", q: [";
    for (std::size_t i = 0; i < wp.q.size(); ++i) {o << (i ? ", " : "") << wp.q[i];}
    o << "]}\n";
  }
  res->success = true;
  res->message = o.str();
}

void WrenchConditionerNode::captureSample(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (recent_.empty()) {
    res->message = "no raw wrench with a usable TF received yet";
    return;
  }
  const double age = (get_clock()->now() - recent_.back().received).seconds();
  if (age > capture_max_age_s_) {
    char buf[200];
    std::snprintf(buf, sizeof(buf),
      "input is stale: newest raw wrench with a usable TF is %.2f s old (limit capture_max_age_s %.2f s)",
      age, capture_max_age_s_);
    res->message = buf;
    return;
  }
  if (recent_.size() < capture_min_samples_) {
    res->message = "only " + std::to_string(recent_.size()) + " raw wrenches in the last " +
      std::to_string(capture_window_s_) + " s, need capture_min_samples=" + std::to_string(capture_min_samples_);
    return;
  }
  core_types::Wrench mean;
  for (const auto & s : recent_) {
    mean.force_x += s.raw.force_x; mean.force_y += s.raw.force_y; mean.force_z += s.raw.force_z;
    mean.torque_x += s.raw.torque_x; mean.torque_y += s.raw.torque_y; mean.torque_z += s.raw.torque_z;
  }
  const double n = static_cast<double>(recent_.size());
  mean.force_x /= n; mean.force_y /= n; mean.force_z /= n;
  mean.torque_x /= n; mean.torque_y /= n; mean.torque_z /= n;
  double spread = 0.0;
  for (const auto & s : recent_) {
    spread = std::max(spread, std::hypot(s.raw.force_x - mean.force_x,
      std::hypot(s.raw.force_y - mean.force_y, s.raw.force_z - mean.force_z)));
  }
  const auto & a0 = recent_.front().r;
  const auto & a1 = recent_.back().r;
  double tr = 0.0;                 // trace of a0^T a1
  for (int i = 0; i < 9; ++i) {tr += a0.m[i] * a1.m[i];}
  const double drift = std::acos(std::clamp((tr - 1.0) / 2.0, -1.0, 1.0));
  if (spread > capture_max_spread_n_ || drift > capture_max_rotation_rad_) {
    char buf[200];
    std::snprintf(buf, sizeof(buf),
      "arm not still or in contact: force spread %.2f N (limit %.2f), orientation drift %.3f rad (limit %.3f)",
      spread, capture_max_spread_n_, drift, capture_max_rotation_rad_);
    res->message = buf;
    return;
  }
  samples_.push_back({a1, mean});
  res->success = true;
  res->message = "captured pose " + std::to_string(samples_.size()) + " (mean of " +
    std::to_string(recent_.size()) + " readings; arm still, no contact; need >= 4 with gravity pointing in " +
    "clearly different directions)";
}

void WrenchConditionerNode::clearSamples(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.clear();
  res->success = true;
  res->message = "cleared";
}

void WrenchConditionerNode::identify(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  force_core::CalibrationResult r;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    r = force_core::identifyPayload(samples_, config_.gravity_world, config_.sign);
    if (!r.ok) {
      res->message = r.message;
      return;
    }
  }
  // The parameter callback updates config_ (it takes the mutex itself, so not under the lock here).
  const auto set = set_parameters({
      rclcpp::Parameter("payload_mass_kg", r.payload.mass_kg),
      rclcpp::Parameter("payload_cog_xyz", std::vector<double>{r.payload.cog.x, r.payload.cog.y, r.payload.cog.z}),
      rclcpp::Parameter("bias", std::vector<double>{r.bias.force_x, r.bias.force_y, r.bias.force_z,
          r.bias.torque_x, r.bias.torque_y, r.bias.torque_z})});
  for (const auto & s : set) {
    if (!s.successful) {res->message = "fit ok but could not apply it: " + s.reason; return;}
  }
  char buf[820];
  std::snprintf(buf, sizeof(buf),
    "fit from %zu poses, residual %.4f N / %.5f N m, regressor condition number %.1f (below about 20 is well spread). Parameters now in use; to keep them:\n"
    "payload_mass_kg: %.5f\npayload_cog_xyz: [%.5f, %.5f, %.5f]\nbias: [%.5f, %.5f, %.5f, %.5f, %.5f, %.5f]\n"
    "The captured poses are kept: call ~/clear_samples before calibrating another tool.",
    r.samples, r.rms_force_n, r.rms_torque_nm, r.condition_number, r.payload.mass_kg, r.payload.cog.x, r.payload.cog.y,
    r.payload.cog.z, r.bias.force_x, r.bias.force_y, r.bias.force_z, r.bias.torque_x, r.bias.torque_y,
    r.bias.torque_z);
  res->success = true;
  res->message = buf;
}

}  // namespace ft_conditioner
