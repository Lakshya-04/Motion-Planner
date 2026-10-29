/**
 * @file rclcpp_logger.hpp
 * @brief rclcpp-backed implementation of core_types::Logger.
 * @details The concrete adapter that lets core logic (e.g.
 *          motion_planner::ManipulationManager, MotionExecReporter) log
 *          through rclcpp without depending on it directly. Nodes inject
 *          this at their own boundary (e.g. via
 *          ManipulationManager::setLogger()). Shared across ROS adapter
 *          packages so each one doesn't duplicate or depend on another just
 *          to reuse this.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "core_types/logger.hpp"

namespace ros_adapters
{

/**
 * @class RclcppLogger
 * @brief Wraps an rclcpp::Logger to satisfy the core_types::Logger port.
 */
class RclcppLogger : public core_types::Logger
{
public:
  /**
   * @brief Construct from an existing rclcpp logger (e.g. node->get_logger()).
   * @param[in] logger Logger instance to forward info/warn calls to.
   */
  explicit RclcppLogger(rclcpp::Logger logger)
  : logger_(std::move(logger)) {}

  void info(const std::string & message) const override
  {
    RCLCPP_INFO(logger_, "%s", message.c_str());
  }

  void warn(const std::string & message) const override
  {
    RCLCPP_WARN(logger_, "%s", message.c_str());
  }

private:
  rclcpp::Logger logger_;
};

}  // namespace ros_adapters
