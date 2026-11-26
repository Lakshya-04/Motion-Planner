/**
 * @file safety_chain_node_core.hpp
 * @brief ROS-free helpers of the opt-in `safety_chain_node`: message-to-sample conversion, severity
 *        parsing, state text and cause naming.
 * @details Keeps everything testable on the host; the node (`src/safety_chain_node.cpp`) only moves
 *          messages in and out. NOT A CERTIFIED SAFETY FUNCTION, see docs/safety_chain.md.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <deque>
#include <string>
#include <vector>

#include "safety_monitor/safety_chain.hpp"

namespace safety_monitor
{

/// |F| and |tau| of a wrench. A non-finite component gives a NaN magnitude, which the chain treats as a trip.
void wrenchMagnitudes(
  double fx, double fy, double fz, double tx, double ty, double tz,
  double & force_n, double & torque_nm);

/// "none" | "hold" | "estop" (case-insensitive) to a severity. Returns false for anything else.
bool parseSeverity(const std::string & text, ChainSeverity & out);
const char * toString(ChainSeverity s);

/// "OK" | "HOLD" | "ESTOP".
const char * stateLabel(ChainState s);

/// Causes asserted since the chain was last OK (back to the last accepted reset or auto-clear), oldest
/// first, without duplicates. Empty in kOk. Taken from the event log, so bounded by its capacity.
std::vector<std::string> activeCauseNames(
  const std::deque<ChainEvent> & events, ChainState current);

/// "OK" or "HOLD force_overload,stale_ft": the text published on the state topic.
std::string formatState(ChainState s, const std::vector<std::string> & causes);

/// Refusal reasons logged at exactly `now_s` (one reset() call), joined by "; ". Empty if none.
std::string refusalReasons(const std::deque<ChainEvent> & events, double now_s);

}  // namespace safety_monitor
