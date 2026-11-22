#include "lift_hardware/core/transport_factory.hpp"

#include "lift_hardware/core/device_transport.hpp"
#include "lift_hardware/core/lift_config.hpp"
#include "lift_hardware/core/mock_transport.hpp"

namespace lift_hardware
{

std::unique_ptr<LiftTransport> make_transport(
  const std::string & name, const std::unordered_map<std::string, std::string> & params,
  std::string * error)
{
  if (name == "device") {
    return std::make_unique<DeviceTransport>();
  }
  if (name != "mock") {
    if (error != nullptr) {
      *error = "unknown transport '" + name + "' (mock | device)";
    }
    return nullptr;
  }
  const LiftConfig defaults;
  MockConfig mock;
  mock.max_velocity_mps = defaults.max_velocity_mps;
  mock.max_acceleration_mps2 = defaults.max_acceleration_mps2;
  if (
    !read_number(params, "mock_stroke_m", mock.stroke_m, error) ||
    !read_number(params, "mock_initial_height_m", mock.initial_height_m, error) ||
    !read_number(params, "max_velocity_mps", mock.max_velocity_mps, error) ||
    !read_number(params, "max_acceleration_mps2", mock.max_acceleration_mps2, error))
  {
    return nullptr;
  }
  if (
    !(mock.stroke_m > 0.0) || !(mock.max_velocity_mps > 0.0) ||
    !(mock.max_acceleration_mps2 > 0.0) || !(mock.initial_height_m >= 0.0) ||
    mock.initial_height_m > mock.stroke_m)
  {
    if (error != nullptr) {
      *error = "mock transport: need stroke > 0, limits > 0 and 0 <= initial height <= stroke";
    }
    return nullptr;
  }
  return std::make_unique<MockTransport>(mock);
}

}  // namespace lift_hardware
