#include "ft_conditioner/wrench_conditioner_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
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

  param_cb_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) {return onParameters(p);});
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
  pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(out_topic, rclcpp::SensorDataQoS());
  sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
    in_topic, rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::WrenchStamped::SharedPtr m) {onRaw(m);});
  using Trigger = std_srvs::srv::Trigger;
  using Req = std::shared_ptr<Trigger::Request>;
  using Res = std::shared_ptr<Trigger::Response>;
  capture_srv_ = create_service<Trigger>("~/capture_pose", [this](const Req q, Res r) {captureSample(q, r);});
  identify_srv_ = create_service<Trigger>("~/identify", [this](const Req q, Res r) {identify(q, r);});
  clear_srv_ = create_service<Trigger>("~/clear_samples", [this](const Req q, Res r) {clearSamples(q, r);});
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
  for (const auto & p : params) {
    const auto & n = p.get_name();
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
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto offset = config_.tool_in_sensor;
  config_ = next;
  config_.tool_in_sensor = offset;
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
  geometry_msgs::msg::WrenchStamped out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = tool_frame_;
  out.wrench.force.x = w.force_x; out.wrench.force.y = w.force_y; out.wrench.force.z = w.force_z;
  out.wrench.torque.x = w.torque_x; out.wrench.torque.y = w.torque_y; out.wrench.torque.z = w.torque_z;
  pub_->publish(out);
  ++published_;
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
