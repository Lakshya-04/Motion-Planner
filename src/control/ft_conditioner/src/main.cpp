#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ft_conditioner/wrench_conditioner_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ft_conditioner::WrenchConditionerNode>());
  rclcpp::shutdown();
  return 0;
}
