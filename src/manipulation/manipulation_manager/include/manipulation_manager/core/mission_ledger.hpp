/**
 * @file mission_ledger.hpp
 * @brief Durable per (mission, stand, column) record of a coverage mission, persisted as JSON, so a manager restart can
 *        resume and rerun only the unfinished columns.
 * @details ROS-free, header-only (nlohmann_json). The file is written atomically (temp file in the same directory,
 *          then rename). A missing file is an empty ledger; a corrupted one is moved aside to `<path>.corrupt`
 *          and treated as empty, so a bad file never blocks a mission or resumes from wrong data.
 *          A column is the segment unit of the manager. The stand is recorded for information: stands are planned
 *          again on a resume, so only (mission, column) decides what is skipped.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace manipulation_manager
{

enum class LedgerState { kPending, kRunning, kDone, kSkipped, kFailed };

/// Why a column was skipped (hardware- or flow-origin, typed).
enum class SkipReason { kNone, kNavFailed, kDockFailed, kPlanFailed, kContactOverload };

inline const char * toString(LedgerState s)
{
  switch (s) {
    case LedgerState::kPending: return "PENDING";
    case LedgerState::kRunning: return "RUNNING";
    case LedgerState::kDone: return "DONE";
    case LedgerState::kSkipped: return "SKIPPED";
    case LedgerState::kFailed: return "FAILED";
  }
  return "PENDING";
}

inline const char * toString(SkipReason r)
{
  switch (r) {
    case SkipReason::kNone: return "NONE";
    case SkipReason::kNavFailed: return "NAV_FAILED";
    case SkipReason::kDockFailed: return "DOCK_FAILED";
    case SkipReason::kPlanFailed: return "PLAN_FAILED";
    case SkipReason::kContactOverload: return "CONTACT_OVERLOAD";
  }
  return "NONE";
}

inline bool parseLedgerState(const std::string & s, LedgerState & out)
{
  for (const auto v : {LedgerState::kPending, LedgerState::kRunning, LedgerState::kDone, LedgerState::kSkipped,
      LedgerState::kFailed})
  {
    if (s == toString(v)) {out = v; return true;}
  }
  return false;
}

inline bool parseSkipReason(const std::string & s, SkipReason & out)
{
  for (const auto v : {SkipReason::kNone, SkipReason::kNavFailed, SkipReason::kDockFailed, SkipReason::kPlanFailed,
      SkipReason::kContactOverload})
  {
    if (s == toString(v)) {out = v; return true;}
  }
  return false;
}

/// FNV-1a 64-bit of a byte string as 16 hex digits: the mission id of a plan file's content.
inline std::string missionIdFromBytes(const std::string & bytes)
{
  uint64_t h = 1469598103934665603ULL;
  for (const unsigned char c : bytes) {
    h ^= c;
    h *= 1099511628211ULL;
  }
  char buf[17];
  std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(h));
  return buf;
}

struct LedgerEntry
{
  LedgerState state{LedgerState::kPending};
  SkipReason reason{SkipReason::kNone};
  int stand{-1};
};

class MissionLedger
{
public:
  MissionLedger() = default;
  explicit MissionLedger(std::string path) : path_(std::move(path)) {}

  const std::string & path() const {return path_;}
  const std::string & mission() const {return mission_;}
  bool persistent() const {return !path_.empty();}

  /// Loads the file. Returns false when the file was corrupted (moved aside, ledger empty); true otherwise
  /// (also when there is no file).
  bool load()
  {
    entries_.clear();
    mission_.clear();
    if (path_.empty()) {return true;}
    std::ifstream in(path_);
    if (!in) {return true;}
    std::stringstream ss;
    ss << in.rdbuf();
    in.close();
    if (!parse(ss.str())) {
      entries_.clear();
      mission_.clear();
      std::rename(path_.c_str(), (path_ + ".corrupt").c_str());
      return false;
    }
    return true;
  }

  /// Starts a mission. A fresh start (resume false) or another mission id clears the entries; a resume of the same
  /// mission keeps them, and RUNNING entries (the process died in them) go back to PENDING.
  /// Returns true when entries of the same mission were kept.
  bool begin(const std::string & mission, size_t column_count, bool resume)
  {
    const bool keep = resume && mission == mission_ && !entries_.empty();
    if (!keep) {
      entries_.clear();
    }
    mission_ = mission;
    for (size_t c = 0; c < column_count; ++c) {
      entries_.emplace(c, LedgerEntry{});
    }
    for (auto & kv : entries_) {
      if (kv.second.state == LedgerState::kRunning) {
        kv.second.state = LedgerState::kPending;
      }
    }
    save();
    return keep;
  }

  void markRunning(size_t column, int stand) {set(column, {LedgerState::kRunning, SkipReason::kNone, stand});}
  void markDone(size_t column, int stand) {set(column, {LedgerState::kDone, SkipReason::kNone, stand});}
  void markFailed(size_t column, int stand) {set(column, {LedgerState::kFailed, SkipReason::kNone, stand});}
  void markSkipped(size_t column, int stand, SkipReason why) {set(column, {LedgerState::kSkipped, why, stand});}

  LedgerEntry get(size_t column) const
  {
    const auto it = entries_.find(column);
    return it == entries_.end() ? LedgerEntry{} : it->second;
  }

  bool isDone(size_t column) const {return get(column).state == LedgerState::kDone;}

  /// Columns a resume must not rerun: DONE, and skipped for a contact overload (never retried blindly).
  bool resumeSkips(size_t column) const
  {
    const auto e = get(column);
    return e.state == LedgerState::kDone ||
           (e.state == LedgerState::kSkipped && e.reason == SkipReason::kContactOverload);
  }

  std::vector<size_t> columnsIn(LedgerState s) const
  {
    std::vector<size_t> out;
    for (const auto & kv : entries_) {
      if (kv.second.state == s) {out.push_back(kv.first);}
    }
    return out;
  }

  size_t size() const {return entries_.size();}

  /// Atomic write: temp file next to the target, then rename. False when it could not be written.
  bool save() const
  {
    if (path_.empty()) {return true;}
    const std::string tmp = path_ + ".tmp";
    {
      std::ofstream out(tmp, std::ios::trunc);
      if (!out) {return false;}
      out << toJson().dump(1);
      out.flush();
      if (!out) {return false;}
    }
    return std::rename(tmp.c_str(), path_.c_str()) == 0;
  }

  nlohmann::json toJson() const
  {
    nlohmann::json cols = nlohmann::json::array();
    for (const auto & kv : entries_) {
      const auto & e = kv.second;
      cols.push_back(
        {{"column", kv.first}, {"stand", e.stand}, {"state", toString(e.state)}, {"reason", toString(e.reason)}});
    }
    return {{"version", 1}, {"mission", mission_}, {"columns", cols}};
  }

private:
  void set(size_t column, const LedgerEntry & e)
  {
    entries_[column] = e;
    save();
  }

  bool parse(const std::string & text)
  {
    try {
      const auto j = nlohmann::json::parse(text);
      if (!j.is_object() || j.value("version", 0) != 1 || !j.contains("mission") || !j["columns"].is_array()) {
        return false;
      }
      mission_ = j["mission"].get<std::string>();
      for (const auto & c : j["columns"]) {
        LedgerEntry e;
        if (!parseLedgerState(c.at("state").get<std::string>(), e.state) ||
          !parseSkipReason(c.at("reason").get<std::string>(), e.reason))
        {
          return false;
        }
        e.stand = c.at("stand").get<int>();
        entries_[c.at("column").get<size_t>()] = e;
      }
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  std::string path_;
  std::string mission_;
  std::map<size_t, LedgerEntry> entries_;
};

}  // namespace manipulation_manager
