#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_self_filter/ros/self_filter_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_self_filter::SelfFilterNode>());
  rclcpp::shutdown();
  return 0;
}
