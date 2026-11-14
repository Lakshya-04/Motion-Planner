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
// give the tool offset as parameters instead (tool_in_sensor_from_tf:=false).
//
// Calibration: with the arm still and no contact, call ~/capture_pose in at least four poses that point
// gravity in clearly different directions in the sensor frame, then ~/identify. The node fits payload
// mass, centre of gravity and bias, starts using them, and returns a YAML snippet for the parameters.
#pragma once

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

class WrenchConditionerNode : public rclcpp::Node
{
public:
  explicit WrenchConditionerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  std::size_t published() const {return published_;}
  std::size_t dropped() const {return dropped_;}
  std::size_t samples() const;
  force_core::ConditionerConfig config() const;

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
  geometry_msgs::msg::WrenchStamped::SharedPtr last_raw_;
  force_core::Mat3 last_r_;
  bool have_last_{false};

  std::size_t published_{0}, dropped_{0}, stamp_fallbacks_{0};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_srv_, identify_srv_, clear_srv_;
};

}  // namespace ft_conditioner
