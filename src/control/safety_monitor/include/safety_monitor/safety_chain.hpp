/**
 * @file safety_chain.hpp
 * @brief One latched OK / HOLD / ESTOP state machine over the independent safety inputs.
 * @details Collects force overload, torque overload, stale F/T, stale joint state, base obstacle
 *          hold / abort, the operator e-stop and a controller fault, and turns them into one state
 *          with defined outputs (stop the arm, stop the base, tool off, request power removal).
 *
 *          States (stop categories after IEC 60204-1 9.2.2 / ISO 13850, see docs/safety_chain.md):
 *            kOk     no stop; all outputs false.
 *            kHold   controlled stop with power kept (category 2 style): arm_stop, base_stop.
 *            kEstop  controlled stop, then power removal request (category 1 style): arm_stop,
 *                    base_stop, tool_off, power_removal_request.
 *
 *          Rules: each cause has a configurable severity (kNone = ignored, kHold, kEstop); the state
 *          only escalates on its own (OK -> HOLD -> ESTOP). kEstop is never cleared automatically.
 *          kHold clears automatically only if `hold_auto_clear` is set, after `hold_clear_time_s`
 *          with no cause active. Otherwise leaving a state needs reset(), which is refused while any
 *          cause is still active (also an unconsumed one-shot report) and, for the operator e-stop,
 *          while the e-stop input is still asserted. Instantaneous events (a force spike, an e-stop
 *          press, a fault pulse) are remembered until the next step(), so one between two steps is not
 *          lost. Missing, non-finite or NaN data is a trip (fail-safe), a stream never fed counts as
 *          stale once `require_*` is set.
 *
 *          The core has no clock and no I/O: every call passes `now_s`. Not thread safe.
 *
 *          NOT A CERTIFIED SAFETY FUNCTION: ordinary software on a general-purpose OS. A real
 *          e-stop is a hardware circuit that removes power without this code.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <array>
#include <cstddef>
#include <deque>
#include <string>

namespace safety_monitor
{

enum class ChainState { kOk = 0, kHold = 1, kEstop = 2 };

enum class ChainCause
{
  kForceOverload = 0,   ///< |F| above `force_abort_n` (tool profile overload_abort_n)
  kTorqueOverload,      ///< |tau| above `torque_abort_nm` (tool profile tangential_torque_abort_nm)
  kStaleFt,             ///< no valid F/T sample within `ft_stale_s`
  kStaleJointState,     ///< no valid joint state within `joint_stale_s`
  kBaseObstacleHold,    ///< the base navigator is holding for an obstacle
  kBaseObstacleAbort,   ///< the base navigator gave up on an obstacle
  kOperatorEstop,       ///< operator e-stop topic or service
  kControllerFault,     ///< arm controller or driver fault
  kCount
};

enum class ChainSeverity { kNone = 0, kHold = 1, kEstop = 2 };

enum class BaseObstacle { kNone, kHold, kAbort };

const char * toString(ChainState s);
const char * toString(ChainCause c);

struct ChainParams
{
  double force_abort_n{0.0};       ///< 0 disables the force check
  double torque_abort_nm{0.0};     ///< 0 disables the torque check
  double ft_stale_s{0.1};
  double joint_stale_s{0.1};
  bool require_ft{true};            ///< false: a never-fed F/T stream is not a trip (non-contact tools)
  bool require_joint_state{true};
  bool hold_auto_clear{false};      ///< HOLD may clear by itself; ESTOP never does
  double hold_clear_time_s{1.0};
  std::size_t max_events{1000};     ///< log capacity; the oldest are dropped
  /// Severity per cause, indexed by ChainCause.
  std::array<ChainSeverity, static_cast<std::size_t>(ChainCause::kCount)> severity{
    ChainSeverity::kEstop,   // force overload
    ChainSeverity::kEstop,   // torque overload
    ChainSeverity::kHold,    // stale F/T
    ChainSeverity::kEstop,   // stale joint state
    ChainSeverity::kHold,    // base obstacle hold
    ChainSeverity::kEstop,   // base obstacle abort
    ChainSeverity::kEstop,   // operator e-stop
    ChainSeverity::kEstop};  // controller fault
};

struct ChainOutputs
{
  ChainState state{ChainState::kOk};
  bool arm_stop{false};
  bool base_stop{false};
  bool tool_off{false};
  bool power_removal_request{false};
};

enum class ChainEventKind { kCauseAsserted, kStateChange, kResetAccepted, kResetRefused };

struct ChainEvent
{
  double t_s{0.0};
  ChainEventKind kind{ChainEventKind::kCauseAsserted};
  ChainCause cause{ChainCause::kCount};   ///< kCount when no single cause applies
  ChainState from{ChainState::kOk};
  ChainState to{ChainState::kOk};
  std::string detail;
};

class SafetyChain
{
public:
  explicit SafetyChain(const ChainParams & p = ChainParams{});

  /// Latest F/T magnitudes with the sample time. The peak since the last step() is kept.
  void updateFt(double force_n, double torque_nm, double stamp_s);
  void updateJointState(double stamp_s);
  void setBaseObstacle(BaseObstacle o);
  /// Level of the operator e-stop input; a rising edge is remembered until the next step().
  void setOperatorEstop(bool asserted);
  /// Level of the controller fault input; a rising edge is remembered until the next step().
  void setControllerFault(bool fault);

  /// Evaluate all causes at `now_s`, update the state and the log, return the outputs.
  ChainOutputs step(double now_s);
  /// Explicit operator reset. Returns true and goes to kOk only if no cause is active.
  bool reset(double now_s);

  ChainState state() const {return state_;}
  ChainOutputs outputs() const;
  const std::deque<ChainEvent> & events() const {return events_;}
  std::size_t droppedEvents() const {return dropped_;}
  /// First cause of the current non-OK state (kCount in kOk).
  ChainCause latchedCause() const {return latched_cause_;}

private:
  using Flags = std::array<bool, static_cast<std::size_t>(ChainCause::kCount)>;
  Flags activeLevels(double now_s) const;
  void log(double t, ChainEventKind k, ChainCause c, ChainState from, ChainState to, const std::string & d);

  ChainParams p_;
  ChainState state_{ChainState::kOk};
  ChainCause latched_cause_{ChainCause::kCount};
  // inputs
  double force_peak_n_{0.0}, torque_peak_nm_{0.0}, last_force_n_{0.0}, last_torque_nm_{0.0};
  bool ft_seen_{false}, joint_seen_{false}, ft_invalid_{false};
  double ft_stamp_s_{0.0}, joint_stamp_s_{0.0};
  BaseObstacle base_{BaseObstacle::kNone};
  bool estop_level_{false}, fault_level_{false}, estop_pending_{false}, fault_pending_{false};
  bool force_pending_{false}, torque_pending_{false};
  // bookkeeping
  Flags prev_active_{};
  double clear_since_s_{-1.0};
  std::deque<ChainEvent> events_;
  std::size_t dropped_{0};
};

}  // namespace safety_monitor
