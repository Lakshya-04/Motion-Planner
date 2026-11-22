/**
 * @file lift_supervisor.hpp
 * @brief ROS-free logic that runs one lift through a LiftTransport.
 * @details Owns the lifecycle a control loop needs and a transport should not have to know:
 *
 *          - Connecting: tries again with growing delays (`reconnect_*`) until it works.
 *          - Homing: after every connect the stroke is read and the lift is driven to
 *            `home_height_m`; commands are ignored until it is there and idle.
 *          - Ready: a command is clamped to [0, stroke], turned into a reference that respects
 *            the velocity and acceleration limits, and sent to the device; the device must arrive
 *            within `move_timeout_s` of the reference reaching the target.
 *          - Held: any fault the device or the checks report (timeout of a move, out-of-range
 *            height, device fault) stops the lift, keeps stopping until a stop is accepted, and
 *            holds the last good height. Commands are ignored until `reset_fault()`, which homes
 *            again.
 *          - Link loss (reported at once, or after `max_io_failures` failed calls in a row)
 *            returns to Connecting and homes again after reconnecting, because the position can
 *            no longer be trusted.
 *
 *          Time is seconds on any monotonic clock, passed in; nothing here sleeps or blocks, so a
 *          control loop can call `poll()` then `command()` every cycle.
 */

#pragma once

#include <memory>

#include "lift_hardware/core/fault.hpp"
#include "lift_hardware/core/lift_config.hpp"
#include "lift_hardware/core/lift_transport.hpp"
#include "lift_hardware/core/profile.hpp"

namespace lift_hardware
{

enum class Phase
{
  kDisconnected,  ///< not connected; waiting for the next attempt or trying now
  kHoming,        ///< connected, driving to the home height
  kReady,         ///< homed, following commands
  kHeld,          ///< stopped by a fault until reset_fault()
};

const char * to_string(Phase phase);

/// What the supervisor reports to the control loop each cycle.
struct Snapshot
{
  Phase phase{Phase::kDisconnected};
  Fault fault{Fault::kNone};  ///< why the lift is held or disconnected, kNone while healthy
  bool homed{false};
  bool stop_confirmed{true};  ///< while held: the transport accepted a stop
  double height_m{0.0};       ///< last good measured height
  double velocity_mps{0.0};   ///< filtered estimate, zero unless Ready or Homing
  double stroke_m{0.0};       ///< effective travel limit, 0 until connected
  double reference_m{0.0};    ///< the shaped setpoint last sent
};

class LiftSupervisor
{
public:
  /// `config` must pass `validate()`.
  LiftSupervisor(std::unique_ptr<LiftTransport> transport, const LiftConfig & config);

  /// Advance the state machine and read the device. Call once per cycle, before `command()`.
  void poll(double now_s);

  /// Hand over the commanded height (m). Ignored unless Ready; non-finite values are ignored.
  void command(double now_s, double target_m);

  /// Leave Held and home again. Returns false when not Held.
  bool reset_fault(double now_s);

  /// Stop the lift and close the link; `poll()` and `command()` do nothing afterwards.
  void shutdown();

  const Snapshot & snapshot() const { return snapshot_; }

  /// The command as limited by the stroke (what the shaped reference heads for).
  double clamped_target_m() const { return target_m_; }

private:
  void try_connect(double now_s);
  void begin_homing(double now_s);
  void poll_homing(double now_s);
  void poll_ready(double now_s);
  void poll_held(double now_s);
  bool read_device(double now_s, Reading & reading);
  void note_io_result(Fault fault, double now_s);
  void lose_link(double now_s);
  void enter_fault(Fault fault, double now_s);
  void update_velocity(double now_s, double height_m);

  std::unique_ptr<LiftTransport> transport_;
  LiftConfig config_;
  Snapshot snapshot_;

  bool shut_down_{false};
  double next_attempt_s_{0.0};
  double backoff_s_{0.0};

  int io_failures_{0};
  double phase_start_s_{0.0};
  bool home_sent_{false};

  // Ready: the shaped reference and what has been sent.
  Kinematics reference_;
  double target_m_{0.0};
  double last_shape_s_{0.0};
  bool shaping_started_{false};
  double last_send_s_{0.0};
  double last_sent_m_{0.0};
  bool sent_any_{false};
  bool at_target_{false};
  bool out_of_tolerance_{false};
  double out_of_tolerance_since_s_{0.0};

  // Velocity estimate.
  bool have_previous_height_{false};
  double previous_height_m_{0.0};
  double previous_height_s_{0.0};
};

}  // namespace lift_hardware
