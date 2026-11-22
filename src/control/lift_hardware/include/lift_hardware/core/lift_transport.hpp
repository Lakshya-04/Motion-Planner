/**
 * @file lift_transport.hpp
 * @brief The seam between the lift logic and whatever talks to a lift.
 * @details A transport hides one kind of device (a simulation, a scripted test double, a real
 *          controller on some bus). It speaks metres above the retracted position and nothing
 *          else; protocols, units and register maps stay behind it.
 *
 *          Contract for implementers:
 *          - Every call returns quickly. The supervisor calls from a control loop. A transport
 *            whose device is slow keeps its own worker thread and answers from cached data.
 *          - A call returns `Fault::kNone` on success, otherwise the most specific fault it can
 *            name. `kLinkLost` means "connect again"; the supervisor never calls anything but
 *            `connect()` (and `disconnect()`) on a transport it believes is disconnected.
 *          - `move_to` only starts motion. The device moves at its own pace and `read` tells how
 *            far it got. A new `move_to` replaces the previous target.
 *          - `stop` must be safe to call at any time, including repeatedly and when idle.
 *          - Not thread safe: one owner calls it.
 */

#pragma once

#include "lift_hardware/core/fault.hpp"

namespace lift_hardware
{

class LiftTransport
{
public:
  virtual ~LiftTransport() = default;

  /// Open the link and make the device ready to answer.
  virtual Fault connect() = 0;

  /// Close the link. Always succeeds; safe when not connected.
  virtual void disconnect() = 0;

  /// Usable travel of the device in metres (> 0 on success).
  virtual Fault read_stroke(double & stroke_m) = 0;

  /// Start moving to `height_m` (metres above the retracted position).
  virtual Fault move_to(double height_m) = 0;

  /// Stop where the device is now.
  virtual Fault stop() = 0;

  /// Current height, device state and device fault.
  virtual Fault read(Reading & reading) = 0;
};

}  // namespace lift_hardware
