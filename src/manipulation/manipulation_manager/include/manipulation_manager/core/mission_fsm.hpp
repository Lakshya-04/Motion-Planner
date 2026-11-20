/**
 * @file mission_fsm.hpp
 * @brief Mission state IDLE / ACTIVE / PAUSED / STOPPED / RECOVERY with pause and resume taking effect at segment
 *        boundaries.
 * @details ROS-free. `requestPause()` only sets a flag; the task loop calls `atBoundary()` between columns, and the
 *          state becomes PAUSED there (the loop then waits until `resume()`). Cancel is unchanged: `stop()` ends the
 *          mission from any state. RECOVERY is entered while the arm is freed (jog) and returns to ACTIVE.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <string>

namespace manipulation_manager
{

enum class MissionState { kIdle, kActive, kPaused, kStopped, kRecovery };

inline const char * toString(MissionState s)
{
  switch (s) {
    case MissionState::kIdle: return "IDLE";
    case MissionState::kActive: return "ACTIVE";
    case MissionState::kPaused: return "PAUSED";
    case MissionState::kStopped: return "STOPPED";
    case MissionState::kRecovery: return "RECOVERY";
  }
  return "IDLE";
}

class MissionFsm
{
public:
  MissionState state() const {return state_;}
  bool pauseRequested() const {return pause_requested_;}

  /// IDLE or STOPPED -> ACTIVE.
  bool start()
  {
    if (state_ != MissionState::kIdle && state_ != MissionState::kStopped) {return false;}
    state_ = MissionState::kActive;
    pause_requested_ = false;
    return true;
  }

  /// Takes effect at the next segment boundary. Only while ACTIVE.
  bool requestPause()
  {
    if (state_ != MissionState::kActive) {return false;}
    pause_requested_ = true;
    return true;
  }

  /// Called by the loop between columns. True when the loop must wait (state is PAUSED).
  bool atBoundary()
  {
    if (state_ == MissionState::kActive && pause_requested_) {
      state_ = MissionState::kPaused;
    }
    return state_ == MissionState::kPaused;
  }

  /// PAUSED -> ACTIVE; also withdraws a pause that has not reached a boundary yet.
  bool resume()
  {
    if (state_ == MissionState::kPaused) {
      state_ = MissionState::kActive;
      pause_requested_ = false;
      return true;
    }
    if (state_ == MissionState::kActive && pause_requested_) {
      pause_requested_ = false;
      return true;
    }
    return false;
  }

  bool enterRecovery()
  {
    if (state_ != MissionState::kActive) {return false;}
    state_ = MissionState::kRecovery;
    return true;
  }

  bool leaveRecovery()
  {
    if (state_ != MissionState::kRecovery) {return false;}
    state_ = MissionState::kActive;
    return true;
  }

  /// Cancel / end: any state but IDLE -> STOPPED.
  bool stop()
  {
    if (state_ == MissionState::kIdle || state_ == MissionState::kStopped) {return false;}
    state_ = MissionState::kStopped;
    pause_requested_ = false;
    return true;
  }

  void reset() {state_ = MissionState::kIdle; pause_requested_ = false;}

private:
  MissionState state_{MissionState::kIdle};
  bool pause_requested_{false};
};

/// Result of one typed mission command (mirrors coverage_task_msgs/srv/MissionCommand).
struct MissionCommandReply
{
  bool accepted{false};
  std::string state;
  bool pause_pending{false};
  std::string message;
};

/// Apply "pause", "resume" or "status" to the FSM. An unknown command changes nothing and is not accepted.
inline MissionCommandReply applyMissionCommand(MissionFsm & fsm, const std::string & command)
{
  MissionCommandReply r;
  if (command == "pause") {
    r.accepted = fsm.requestPause();
    r.message = r.accepted ? "pause requested, takes effect at the next column boundary" :
      "pause ignored: the mission is not active";
  } else if (command == "resume") {
    r.accepted = fsm.resume();
    r.message = r.accepted ? "resumed" : "resume ignored: nothing is paused or pending";
  } else if (command == "status") {
    r.accepted = true;
    r.message = "status";
  } else {
    r.message = "unknown command '" + command + "' (pause | resume | status)";
  }
  r.state = toString(fsm.state());
  r.pause_pending = fsm.pauseRequested();
  return r;
}

}  // namespace manipulation_manager
