/**
 * @file device_transport.hpp
 * @brief Skeleton of a transport for a real lift controller. Deliberately not implemented.
 * @details This public repository carries no device protocol. A deployment supplies its own
 *          subclass of `LiftTransport` (in a private package or an out-of-tree one) and selects
 *          it by name, see `docs/lift.md`. This class marks the place and the checklist; every
 *          operation fails with `Fault::kUnsupported`, so choosing it by mistake is loud (the
 *          supervisor keeps retrying the connection and reports the fault) and never moves a
 *          lift.
 *
 *          A real implementation has to:
 *          1. `connect()`: open the link, check the device is the expected kind and is powered
 *             and initialised, bound every wait by a timeout, and return without blocking the
 *             control loop longer than a few milliseconds (otherwise run the I/O on a worker
 *             thread and answer from its cache).
 *          2. `read_stroke()`: return the device's own travel in metres. The supervisor never
 *             commands beyond it.
 *          3. `move_to()`: convert metres to the device's unit, refuse a target outside travel
 *             with `kOutOfRange`, and only start the motion.
 *          4. `stop()`: halt now, idempotent. Prefer the device's own stop over "move to the
 *             present height".
 *          5. `read()`: height in metres, whether it is moving, and the device's fault flags
 *             mapped to `Fault`. A silent device is `kTimeout`; a dead link is `kLinkLost`.
 *          6. Keep any watchdog the device requires alive inside these calls.
 */

#pragma once

#include "lift_hardware/core/lift_transport.hpp"

namespace lift_hardware
{

class DeviceTransport : public LiftTransport
{
public:
  Fault connect() override { return Fault::kUnsupported; }
  void disconnect() override {}
  Fault read_stroke(double &) override { return Fault::kUnsupported; }
  Fault move_to(double) override { return Fault::kUnsupported; }
  Fault stop() override { return Fault::kUnsupported; }
  Fault read(Reading &) override { return Fault::kUnsupported; }
};

}  // namespace lift_hardware
