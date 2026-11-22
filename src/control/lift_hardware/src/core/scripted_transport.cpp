#include "lift_hardware/core/scripted_transport.hpp"

#include <cmath>

namespace lift_hardware
{

ScriptedTransport::ScriptedTransport(double stroke_m, double initial_height_m)
: stroke_m_(stroke_m), height_m_(initial_height_m), target_m_(initial_height_m)
{
}

void ScriptedTransport::inject(Op op, Fault fault, int count)
{
  injections_[op] = Injection{fault, count};
}

void ScriptedTransport::clear_faults()
{
  injections_.clear();
}

Fault ScriptedTransport::take(Op op)
{
  const auto it = injections_.find(op);
  if (it == injections_.end() || it->second.remaining == 0) {
    return Fault::kNone;
  }
  const Fault fault = it->second.fault;
  if (it->second.remaining > 0) {
    --it->second.remaining;
  }
  return fault;
}

Fault ScriptedTransport::connect()
{
  ++connect_calls_;
  const Fault injected = take(Op::kConnect);
  if (injected != Fault::kNone) {
    return injected;
  }
  connected_ = true;
  return Fault::kNone;
}

void ScriptedTransport::disconnect()
{
  ++disconnect_calls_;
  connected_ = false;
}

Fault ScriptedTransport::read_stroke(double & stroke_m)
{
  const Fault injected = take(Op::kReadStroke);
  if (injected != Fault::kNone) {
    return injected;
  }
  if (!connected_) {
    return Fault::kLinkLost;
  }
  stroke_m = stroke_m_;
  return Fault::kNone;
}

Fault ScriptedTransport::move_to(double height_m)
{
  const Fault injected = take(Op::kMoveTo);
  if (injected != Fault::kNone) {
    return injected;
  }
  if (!connected_) {
    return Fault::kLinkLost;
  }
  moves_.push_back(height_m);
  if (!std::isfinite(height_m) || height_m < 0.0 || height_m > stroke_m_) {
    return Fault::kOutOfRange;
  }
  target_m_ = height_m;
  moving_ = true;
  return Fault::kNone;
}

Fault ScriptedTransport::stop()
{
  ++stop_calls_;
  const Fault injected = take(Op::kStop);
  if (injected != Fault::kNone) {
    return injected;
  }
  if (!connected_) {
    return Fault::kLinkLost;
  }
  target_m_ = height_m_;
  moving_ = false;
  return Fault::kNone;
}

Fault ScriptedTransport::read(Reading & reading)
{
  const Fault injected = take(Op::kRead);
  if (injected != Fault::kNone) {
    return injected;
  }
  if (!connected_) {
    return Fault::kLinkLost;
  }
  if (follow_moves_ && moving_) {
    height_m_ = target_m_;
    moving_ = false;
  }
  moving_ = std::abs(target_m_ - height_m_) > 1e-9;
  reading.height_m = height_m_;
  if (device_fault_ != Fault::kNone) {
    reading.state = DeviceState::kFaulted;
    reading.fault = device_fault_;
  } else {
    reading.state = moving_ ? DeviceState::kMoving : DeviceState::kIdle;
    reading.fault = Fault::kNone;
  }
  return Fault::kNone;
}

}  // namespace lift_hardware
