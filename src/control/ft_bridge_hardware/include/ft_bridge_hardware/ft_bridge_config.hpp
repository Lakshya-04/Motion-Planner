/**
 * @file ft_bridge_config.hpp
 * @brief Pure parsing of the FT bridge hardware component's <param> block.
 * @details No rclcpp, no hardware_interface -- parseFtBridgeConfig() takes the plain
 *          std::unordered_map<std::string, std::string> hardware_interface::HardwareInfo already
 *          hands every component (its own <hardware><param>...</param></hardware> entries),
 *          exactly like lift_hardware's own parameter-parsing helpers. Kept separate from
 *          FtBridgeHardwareInterface so this parsing (and its defaulting/validation behavior) is
 *          directly gtest-able without a live ros2_control_node -- see this package's README for
 *          why the SensorInterface class itself is verified live instead, matching
 *          lift_hardware's own established convention (LiftHardwareInterface has no direct
 *          gtest; its control law/homing state machine/conversions do).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <string>
#include <unordered_map>

namespace ft_bridge_hardware
{

/**
 * @struct FtBridgeConfig
 * @brief Resolved, defaulted configuration for one FT bridge hardware component instance.
 */
struct FtBridgeConfig
{
  /// Real F/T sensor topic (geometry_msgs/WrenchStamped), used unless injection_mode is true.
  std::string source_topic{"/ufactory/uf_ftsensor_ext_states"};

  /// Test-only topic (geometry_msgs/WrenchStamped) this component reads from instead of
  /// source_topic when injection_mode is true -- the mechanism that lets the whole admittance
  /// loop be exercised in mock/ufactory_sim with scripted forces (force_control_derisking_plan.md
  /// §1.1's "highest-leverage item in the whole effort").
  std::string injection_topic{"~/injected_wrench"};

  /// Setpoint topic (geometry_msgs/Wrench, not stamped -- a target, not a live reading), always
  /// subscribed regardless of injection_mode. Biases the exported wrench by subtraction so
  /// admittance_controller's built-in zero-seeking law becomes target-seeking -- see
  /// wrench_bias.hpp. Defaults to all-zero (a no-op bias) until something publishes on it.
  std::string setpoint_topic{"~/target_wrench"};

  bool injection_mode{false};
};

/**
 * @brief Parse hardware_interface::HardwareInfo::hardware_parameters into an FtBridgeConfig.
 * @details Unrecognized keys are ignored (forward-compatible); missing keys keep FtBridgeConfig's
 *          own defaults. "injection_mode" accepts "true"/"false" case-sensitively, matching the
 *          same string-bool convention this codebase already uses for mock_sensor_commands --
 *          any other value (including empty/unset) resolves to false, never throws.
 */
inline FtBridgeConfig parseFtBridgeConfig(
  const std::unordered_map<std::string, std::string> & hardware_parameters)
{
  FtBridgeConfig config;

  const auto find_or =
    [&hardware_parameters](const std::string & key, const std::string & fallback) {
      const auto it = hardware_parameters.find(key);
      return it != hardware_parameters.end() ? it->second : fallback;
    };

  config.source_topic = find_or("source_topic", config.source_topic);
  config.injection_topic = find_or("injection_topic", config.injection_topic);
  config.setpoint_topic = find_or("setpoint_topic", config.setpoint_topic);
  config.injection_mode = find_or("injection_mode", "false") == "true";

  return config;
}

}  // namespace ft_bridge_hardware
