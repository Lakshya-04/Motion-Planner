#include "ft_conditioner/wrench_conditioner_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
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

Vec3 vec(const std::vector<double> & v)
{
  return v.size() >= 3 ? Vec3{v[0], v[1], v[2]} : Vec3{};
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
  offset_from_tf_ = declare_parameter<bool>("tool_in_sensor_from_tf", true);
  const auto off_xyz = declare_parameter<std::vector<double>>("tool_in_sensor_xyz", {0.0, 0.0, 0.0});
  const auto off_rpy = declare_parameter<std::vector<double>>("tool_in_sensor_rpy", {0.0, 0.0, 0.0});
  config_.sign = declare_parameter<double>("sign", 1.0);
  config_.payload.mass_kg = declare_parameter<double>("payload_mass_kg", 0.0);
  config_.payload.cog = vec(declare_parameter<std::vector<double>>("payload_cog_xyz", {0.0, 0.0, 0.0}));
  const auto bias = declare_parameter<std::vector<double>>("bias", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  config_.gravity_world = vec(declare_parameter<std::vector<double>>("gravity_world", {0.0, 0.0, -9.80665}));
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
  offset_param_.p = vec(off_xyz);
  offset_param_.R = off_rpy.size() >= 3 ? fromRpy(off_rpy[0], off_rpy[1], off_rpy[2]) : Mat3{};

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

std::size_t WrenchConditionerNode::samples() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_.size();
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
    // The transform for this exact stamp is not there (yet): use the newest one rather than drop.
    try {
      r_world_sensor = toMat(
        tf_buffer_->lookupTransform(world_frame_, sensor_frame, tf2::TimePointZero).transform.rotation);
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
  if (offset_cached_) {
    out = offset_cache_;
    return true;
  }
  try {
    const auto t = tf_buffer_->lookupTransform(sensor_frame, tool_frame_, tf2::TimePointZero).transform;
    offset_cache_.p = {t.translation.x, t.translation.y, t.translation.z};
    offset_cache_.R = toMat(t.rotation);
    offset_cached_ = true;        // the sensor is rigidly mounted to the tool: look it up once
    out = offset_cache_;
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
      "no TF for '%s' (world '%s', tool '%s'): wrench dropped (%zu so far)", sensor.c_str(),
      world_frame_.c_str(), tool_frame_.c_str(), dropped_);
    return;
  }
  force_core::ConditionerConfig cfg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.tool_in_sensor = off;
    cfg = config_;
    last_raw_ = msg;
    last_r_ = r;
    have_last_ = true;
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
  if (!have_last_) {
    res->message = "no raw wrench with a usable TF received yet";
    return;
  }
  samples_.push_back({last_r_, toCore(last_raw_->wrench)});
  res->success = true;
  res->message = "captured pose " + std::to_string(samples_.size()) +
    " (arm still, no contact; need >= 4 with gravity pointing in clearly different directions)";
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
  std::lock_guard<std::mutex> lock(mutex_);
  const auto r = force_core::identifyPayload(samples_, config_.gravity_world, config_.sign);
  if (!r.ok) {
    res->message = r.message;
    return;
  }
  config_.payload = r.payload;
  config_.bias = r.bias;
  set_parameters({
    rclcpp::Parameter("payload_mass_kg", r.payload.mass_kg),
    rclcpp::Parameter("payload_cog_xyz", std::vector<double>{r.payload.cog.x, r.payload.cog.y, r.payload.cog.z}),
    rclcpp::Parameter("bias", std::vector<double>{r.bias.force_x, r.bias.force_y, r.bias.force_z,
        r.bias.torque_x, r.bias.torque_y, r.bias.torque_z})});
  char buf[640];
  std::snprintf(buf, sizeof(buf),
    "fit from %zu poses, residual %.4f N / %.5f N m. Parameters now in use; to keep them:\n"
    "payload_mass_kg: %.5f\npayload_cog_xyz: [%.5f, %.5f, %.5f]\nbias: [%.5f, %.5f, %.5f, %.5f, %.5f, %.5f]",
    r.samples, r.rms_force_n, r.rms_torque_nm, r.payload.mass_kg, r.payload.cog.x, r.payload.cog.y,
    r.payload.cog.z, r.bias.force_x, r.bias.force_y, r.bias.force_z, r.bias.torque_x, r.bias.torque_y,
    r.bias.torque_z);
  res->success = true;
  res->message = buf;
}

}  // namespace ft_conditioner
