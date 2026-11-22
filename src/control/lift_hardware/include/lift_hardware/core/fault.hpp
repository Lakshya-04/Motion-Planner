/**
 * @file fault.hpp
 * @brief Result codes and the device reading shared by every lift transport.
 */

#pragma once

namespace lift_hardware
{

/// Outcome of one transport call, also the fault the supervisor reports.
enum class Fault
{
  kNone = 0,
  kTimeout,      ///< the device did not answer in time (transient until repeated)
  kOutOfRange,   ///< a height outside the device's travel (rejected target or impossible reading)
  kLinkLost,     ///< the connection is gone; the transport must be connected again
  kDevice,       ///< the device itself reports a fault
  kUnsupported,  ///< the transport does not implement this operation
};

/// Short stable name, for logs and diagnostics.
const char * to_string(Fault fault);

/// What the device says it is doing.
enum class DeviceState
{
  kIdle,     ///< stationary
  kMoving,   ///< travelling toward a target
  kFaulted,  ///< stopped by the device because of a fault; `Reading::fault` says which
};

/// One poll of the device.
struct Reading
{
  double height_m{0.0};  ///< carriage height above the retracted position
  DeviceState state{DeviceState::kIdle};
  Fault fault{Fault::kNone};  ///< the device's own fault, meaningful when state is kFaulted
};

}  // namespace lift_hardware
