/**
 * @file scripted_transport.hpp
 * @brief Test double for the supervisor: instant motion, injectable faults, a call log.
 */

#pragma once

#include <map>
#include <vector>

#include "lift_hardware/core/lift_transport.hpp"

namespace lift_hardware
{

enum class Op
{
  kConnect,
  kReadStroke,
  kMoveTo,
  kStop,
  kRead,
};

/**
 * Behaves like a perfect device unless told otherwise: `move_to` makes the next `read` report the
 * target as reached (set `follow_moves` false to freeze the height and move it by hand with
 * `set_height`). `inject(op, fault, count)` makes the next `count` calls of `op` return `fault`
 * without any side effect (`count` < 0 means every call until `clear_faults()`).
 * `set_device_fault` makes reads report a device that stopped with that fault.
 */
class ScriptedTransport : public LiftTransport
{
public:
  explicit ScriptedTransport(double stroke_m = 0.8, double initial_height_m = 0.3);

  Fault connect() override;
  void disconnect() override;
  Fault read_stroke(double & stroke_m) override;
  Fault move_to(double height_m) override;
  Fault stop() override;
  Fault read(Reading & reading) override;

  // Scripting.
  void inject(Op op, Fault fault, int count = 1);
  void clear_faults();
  void set_height(double height_m) { height_m_ = height_m; }
  void set_stroke(double stroke_m) { stroke_m_ = stroke_m; }
  void set_device_fault(Fault fault) { device_fault_ = fault; }
  void set_follow_moves(bool follow) { follow_moves_ = follow; }

  // Inspection.
  bool connected() const { return connected_; }
  int connect_calls() const { return connect_calls_; }
  int stop_calls() const { return stop_calls_; }
  int disconnect_calls() const { return disconnect_calls_; }
  const std::vector<double> & moves() const { return moves_; }
  double height() const { return height_m_; }

private:
  struct Injection
  {
    Fault fault{Fault::kNone};
    int remaining{0};
  };
  Fault take(Op op);

  double stroke_m_;
  double height_m_;
  double target_m_;
  bool follow_moves_{true};
  bool connected_{false};
  bool moving_{false};
  Fault device_fault_{Fault::kNone};
  std::map<Op, Injection> injections_;
  int connect_calls_{0};
  int stop_calls_{0};
  int disconnect_calls_{0};
  std::vector<double> moves_;
};

}  // namespace lift_hardware
