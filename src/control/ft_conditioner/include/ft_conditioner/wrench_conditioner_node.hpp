// ROS shell around force_core::WrenchConditioner.
//
// In:  geometry_msgs/WrenchStamped, the raw sensor wrench in the sensor's own frame (what a real
//      sensor, or the Gazebo / MuJoCo backends, publish: payload weight and bias included).
// Out: geometry_msgs/WrenchStamped in the tool frame: what the environment exerts on the tool, at the
//      tool control point, in the tool axes (core_types::Wrench convention), ready for
//      ft_bridge_hardware / JTAC. See docs/ft_sensing.md.
//
// Frames come from TF: world <- sensor gives the sensor's orientation for the payload's weight, and
// sensor <- tool is the lever arm and the tool axes. Where a simulator has no TF frame for the sensor,
// give the tool offset as parameters instead (tool_in_sensor_from_tf:=false). If the transform at a
// wrench's stamp is not there yet, the newest one is used only while it is at most max_tf_age seconds
// older than the wrench; beyond that (TF publisher stalled) the wrench is dropped.
//
// Calibration: with the arm still and no contact, call ~/capture_pose in at least four poses that point
// gravity in clearly different directions in the sensor frame, then ~/identify. The node fits payload
// mass, centre of gravity and bias, starts using them, and returns a YAML snippet for the parameters.
// ~/capture_pose averages the raw wrenches of the last capture_window_s and refuses when the input is stale
// (newest older than capture_max_age_s), too short (< capture_min_samples) or not still (force spread above
// capture_max_spread_n, orientation drift above capture_max_rotation_rad).
#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "force_core/wrench_conditioning.hpp"

namespace ft_conditioner
{

/// True when a transform stamped `tf_stamp_ns` may stand in for a wrench stamped `msg_stamp_ns`.
/// `max_age_s <= 0` disables the limit; a zero stamp is a static transform, which never ages.
inline bool tfFreshEnough(std::int64_t msg_stamp_ns, std::int64_t tf_stamp_ns, double max_age_s)
{
  if (max_age_s <= 0.0 || tf_stamp_ns == 0) {return true;}
  return static_cast<double>(msg_stamp_ns - tf_stamp_ns) * 1e-9 <= max_age_s;
}

class WrenchConditionerNode : public rclcpp::Node
{
public:
  explicit WrenchConditionerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  std::size_t published() const {return published_;}
  std::size_t dropped() const {return dropped_;}
  std::size_t staleTf() const {return stale_tf_;}
  std::size_t samples() const;
  force_core::ConditionerConfig config() const;
  std::vector<force_core::CalibrationSample> calibrationSamples() const;

private:
  void onRaw(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  bool sensorOrientation(
    const std::string & sensor_frame, const rclcpp::Time & stamp, force_core::Mat3 & r_world_sensor);
  bool toolOffset(const std::string & sensor_frame, force_core::FrameOffset & out);
  void captureSample(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void identify(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void clearSamples(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  std::string world_frame_, tool_frame_, sensor_frame_param_;
  bool offset_from_tf_{true};
  force_core::FrameOffset offset_param_;
  bool offset_cached_{false};
  force_core::FrameOffset offset_cache_;

  mutable std::mutex mutex_;
  force_core::ConditionerConfig config_;
  std::vector<force_core::CalibrationSample> samples_;
  struct Recent
  {
    rclcpp::Time received;
    force_core::Mat3 r;
    core_types::Wrench raw;
  };
  std::deque<Recent> recent_;           // raw wrenches (with usable TF) of the last capture_window_s
  double capture_window_s_{0.5}, capture_max_age_s_{0.25}, capture_max_spread_n_{1.0};
  double capture_max_rotation_rad_{0.02};
  std::size_t capture_min_samples_{5};

  double max_tf_age_s_{0.1};
  std::size_t published_{0}, dropped_{0}, stamp_fallbacks_{0}, stale_tf_{0};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_srv_, identify_srv_, clear_srv_;
};

}  // namespace ft_conditioner
