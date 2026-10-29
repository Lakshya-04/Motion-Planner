/**
 * @file logger.hpp
 * @brief Minimal logging port so core logic never depends on rclcpp.
 * @details Adapters inject a concrete implementation (e.g. an rclcpp::Logger
 *          wrapper) at construction time. Only info/warn are declared since
 *          those are the only levels any current core-logic call site uses --
 *          extend when a real caller needs more.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <string>

namespace core_types
{

/**
 * @class Logger
 * @brief Abstract logging port implemented by a ROS (or other) adapter.
 */
class Logger
{
public:
  virtual ~Logger() = default;

  /**
   * @brief Log an informational message.
   * @param[in] message Fully-formatted message text.
   */
  virtual void info(const std::string & message) const = 0;

  /**
   * @brief Log a warning message.
   * @param[in] message Fully-formatted message text.
   */
  virtual void warn(const std::string & message) const = 0;
};

}  // namespace core_types
