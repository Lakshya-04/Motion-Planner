/**
 * @file transport_factory.hpp
 * @brief Pick a transport by name (the `transport` hardware parameter).
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "lift_hardware/core/lift_transport.hpp"

namespace lift_hardware
{

/**
 * @brief Build the transport called `name`: "mock" (parameters `mock_stroke_m`,
 *        `mock_initial_height_m`, plus the velocity and acceleration limits of the config) or
 *        "device" (the unimplemented skeleton). Returns null and sets `error` for an unknown
 *        name or bad numbers.
 */
std::unique_ptr<LiftTransport> make_transport(
  const std::string & name, const std::unordered_map<std::string, std::string> & params,
  std::string * error);

}  // namespace lift_hardware
