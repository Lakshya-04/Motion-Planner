// ROS shell around the self filter: subscribes to a point cloud, looks up the link poses and the
// cloud frame in TF at the cloud's own time stamp, filters, and publishes the cloud without the
// robot (every point field untouched).
//
// Parameters (all optional except a robot description source):
//   robot_description        URDF text; if empty the node subscribes to the `robot_description`
//                            topic (transient local), as robot_state_publisher publishes it
//   root_link                frame the filter works in (default: the URDF root)
//   links                    links that count as "self" (default: all links with collision shapes)
//   padding.sphere / padding.box[3] / padding.cylinder_radius / padding.cylinder_height
//   padding_overrides        names of links with their own padding; each then reads
//                            padding_overrides.<link>.{sphere, box, cylinder_radius, cylinder_height}
//   min_sensor_distance      drop points closer than this to the cloud frame's origin
//   shadow                   also drop points hidden behind a link (off by default)
//   contact_link, contact_padding   reduced padding on one link while ~/contact_mode is true
//   tf_timeout_sec           how long to wait for a transform at the cloud's stamp
// Topics: cloud_in (sensor_msgs/PointCloud2), cloud_out, contact_mode (std_msgs/Bool).
#pragma once

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "robot_self_filter/core/robot_model.hpp"
#include "robot_self_filter/core/self_filter.hpp"

namespace robot_self_filter
{

class SelfFilterNode : public rclcpp::Node
{
public:
  explicit SelfFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// True once a robot description has been parsed and the filter is ready.
  bool ready() const {return filter_.has_value();}

private:
  void loadRobot(const std::string & urdf);
  SelfFilterConfig readConfig();
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  std::optional<UrdfRobot> robot_;
  std::optional<SelfFilter> filter_;
  std::string root_link_;
  std::string root_link_param_;
  double tf_timeout_sec_{0.1};
  bool contact_mode_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contact_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

}  // namespace robot_self_filter
