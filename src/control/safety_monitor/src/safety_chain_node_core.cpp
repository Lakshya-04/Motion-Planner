#include "safety_monitor/safety_chain_node_core.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>

namespace safety_monitor
{

void wrenchMagnitudes(
  double fx, double fy, double fz, double tx, double ty, double tz,
  double & force_n, double & torque_nm)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const bool f_ok = std::isfinite(fx) && std::isfinite(fy) && std::isfinite(fz);
  const bool t_ok = std::isfinite(tx) && std::isfinite(ty) && std::isfinite(tz);
  force_n = f_ok ? std::sqrt(fx * fx + fy * fy + fz * fz) : nan;
  torque_nm = t_ok ? std::sqrt(tx * tx + ty * ty + tz * tz) : nan;
}

bool parseSeverity(const std::string & text, ChainSeverity & out)
{
  std::string s = text;
  std::transform(s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  if (s == "none") {out = ChainSeverity::kNone;} else if (s == "hold") {
    out = ChainSeverity::kHold;
  } else if (s == "estop") {out = ChainSeverity::kEstop;} else {return false;}
  return true;
}

const char * toString(ChainSeverity s)
{
  switch (s) {
    case ChainSeverity::kNone: return "none";
    case ChainSeverity::kHold: return "hold";
    case ChainSeverity::kEstop: return "estop";
  }
  return "none";
}

const char * stateLabel(ChainState s)
{
  switch (s) {
    case ChainState::kOk: return "OK";
    case ChainState::kHold: return "HOLD";
    case ChainState::kEstop: return "ESTOP";
  }
  return "ESTOP";
}

std::vector<std::string> activeCauseNames(
  const std::deque<ChainEvent> & events, ChainState current)
{
  std::vector<std::string> out;
  if (current == ChainState::kOk) {return out;}
  // Walk back to the event that left kOk for the last time.
  std::size_t begin = 0;
  for (std::size_t i = events.size(); i > 0; --i) {
    const ChainEvent & e = events[i - 1];
    if (e.kind == ChainEventKind::kResetAccepted ||
      (e.kind == ChainEventKind::kStateChange && e.to == ChainState::kOk))
    {
      begin = i;
      break;
    }
  }
  for (std::size_t i = begin; i < events.size(); ++i) {
    if (events[i].kind != ChainEventKind::kCauseAsserted) {continue;}
    const std::string name = toString(events[i].cause);
    if (std::find(out.begin(), out.end(), name) == out.end()) {out.push_back(name);}
  }
  return out;
}

std::string formatState(ChainState s, const std::vector<std::string> & causes)
{
  std::string text = stateLabel(s);
  for (std::size_t i = 0; i < causes.size(); ++i) {
    text += (i == 0 ? " " : ",") + causes[i];
  }
  return text;
}

std::string refusalReasons(const std::deque<ChainEvent> & events, double now_s)
{
  std::string out;
  for (const ChainEvent & e : events) {
    if (e.kind != ChainEventKind::kResetRefused || e.t_s != now_s) {continue;}
    if (!out.empty()) {out += "; ";}
    out += e.detail;
  }
  return out;
}

}  // namespace safety_monitor
