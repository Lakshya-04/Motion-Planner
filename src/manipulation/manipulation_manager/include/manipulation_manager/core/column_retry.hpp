/**
 * @file column_retry.hpp
 * @brief Second chance for the columns that failed or ended partial in the main pass (ROS-free policy).
 * @details Opt-in (`retry_failed_columns`, off by default). After the main pass the node asks `planColumnRetries` what to
 *          retry and then drives `runColumnRetries` with a callback that performs one attempt (stow + prime, optionally a
 *          new stand, then the column). This header holds the decisions, not the motion:
 *            - what is retried: every column that ended partial or failed, except a contact overload (the cause is on the
 *              wall, not in the plan, same rule as the in-place retry). Columns that never ran (no stand reaches them) are
 *              not in the input at all.
 *            - in which order: by stand (the base drives least), then by column id.
 *            - with which variant, per attempt (`chooseVariant`): always from a clean pose (stow, then prime the
 *              continuity from the live state, the dock reference cleared); a different dock IK solution than the ones the
 *              earlier attempts docked with (`vary_ik`, the node excludes them in dock_ik_selector's DockIkExclusion), the
 *              other IK branch allowed from the second attempt on (`allow_branch_change`); a re-planned nearby stand
 *              (`restand`, only when `replan_stand` is on and the failure was navigation or dock/reach related; a
 *              navigation failure re-plans on the first attempt and does not vary the IK, as the IK was never reached).
 *            - when to stop: a cancel, an attempt the callback calls fatal (arm stuck and not recoverable), the attempts
 *              of a column (`max_retries`), and `stop_after_failed_columns` columns in a row that used all their attempts
 *              without success (the cause is then not the column; the rest are left as they were, recorded as such).
 *            - what is recorded: per attempt the column, stand, attempt number, variant, failure before, outcome and a note;
 *              per column the attempts and the final outcome; the columns not retried with the reason.
 *          Nothing here runs a motion, so it is tested with a fake attempt function.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <map>
#include <utility>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "manipulation_manager/core/coverage_task.hpp"

namespace manipulation_manager
{

/// Why a column ended as it did (the node maps its ledger reason to this).
enum class RetryFailure { kNone, kNav, kDock, kPlan, kOverload, kOther };

inline const char * toString(RetryFailure f)
{
  switch (f) {
    case RetryFailure::kNone: return "none";
    case RetryFailure::kNav: return "navigation";
    case RetryFailure::kDock: return "dock";
    case RetryFailure::kPlan: return "plan";
    case RetryFailure::kOverload: return "contact_overload";
    case RetryFailure::kOther: return "other";
  }
  return "other";
}

inline const char * toString(ColumnOutcome o)
{
  switch (o) {
    case ColumnOutcome::kDone: return "done";
    case ColumnOutcome::kPartial: return "partial";
    case ColumnOutcome::kFailed: return "failed";
  }
  return "failed";
}

struct ColumnRetryConfig
{
  bool enabled{false};
  int max_retries{1};                 ///< further attempts per column
  bool replan_stand{false};           ///< allow a re-planned stand (multi-stand runs only; the node decides)
  bool allow_branch_change{false};    ///< from the second attempt on, let the other IK branch through
  int stop_after_failed_columns{3};   ///< columns in a row that used all attempts and did not recover; 0 = never stop
};

struct ColumnFailure
{
  size_t column{0};
  int stand{-1};                      ///< stand the column ran on (-1 = single stand / unknown)
  ColumnOutcome outcome{ColumnOutcome::kFailed};
  RetryFailure kind{RetryFailure::kOther};
};

struct RetryVariant
{
  bool fresh_stow{true};              ///< stow the arm and prime from the clean pose first
  bool vary_ik{false};                ///< dock with a solution different from the earlier attempts'
  bool allow_branch_change{false};    ///< the different solution may be on the other IK branch
  bool restand{false};                ///< re-plan the stand for this column's cells and drive there
  std::string label() const
  {
    std::string s = fresh_stow ? "fresh_stow" : "in_place";
    if (vary_ik) {s += allow_branch_change ? "+next_ik_any_branch" : "+next_ik";}
    if (restand) {s += "+restand";}
    return s;
  }
};

struct ColumnRetryPlan
{
  size_t column{0};
  int stand{-1};
  ColumnOutcome first_outcome{ColumnOutcome::kFailed};
  RetryFailure first_kind{RetryFailure::kOther};
};

/// The variant of attempt `attempt` (1 = the first retry) after a failure of kind `kind`.
inline RetryVariant chooseVariant(RetryFailure kind, int attempt, const ColumnRetryConfig & cfg)
{
  RetryVariant v;
  const bool nav = kind == RetryFailure::kNav;
  const bool reach_related = nav || kind == RetryFailure::kDock;
  v.vary_ik = !nav;
  v.allow_branch_change = v.vary_ik && cfg.allow_branch_change && attempt >= 2;
  // a navigation failure goes to a new stand at once; a dock failure only after a different IK solution did not help
  v.restand = cfg.replan_stand && reach_related && (nav || attempt >= 2);
  return v;
}

/// What to retry and in which order (see the header comment). Overloads and kNone are dropped.
inline std::vector<ColumnRetryPlan> planColumnRetries(
  const std::vector<ColumnFailure> & failures, const ColumnRetryConfig & cfg,
  std::vector<std::pair<size_t, std::string>> * not_retried = nullptr)
{
  std::vector<ColumnRetryPlan> out;
  if (!cfg.enabled || cfg.max_retries <= 0) {
    return out;
  }
  for (const auto & f : failures) {
    if (f.outcome == ColumnOutcome::kDone) {
      continue;
    }
    if (f.kind == RetryFailure::kOverload) {
      if (not_retried) {not_retried->push_back({f.column, "contact overload is not retried"});}
      continue;
    }
    out.push_back({f.column, f.stand, f.outcome, f.kind});
  }
  std::stable_sort(
    out.begin(), out.end(), [](const ColumnRetryPlan & a, const ColumnRetryPlan & b) {
      return a.stand != b.stand ? a.stand < b.stand : a.column < b.column;
    });
  return out;
}

/// Result of one attempt, from the node.
struct RetryAttemptResult
{
  ColumnOutcome outcome{ColumnOutcome::kFailed};
  RetryFailure kind{RetryFailure::kOther};   ///< why it failed, when it did
  bool fatal{false};                         ///< stop the whole retry pass (arm stuck, shutdown)
  std::string note;
};

struct ColumnAttemptRecord
{
  size_t column{0};
  int stand{-1};
  int attempt{0};                     ///< 1 = first retry
  std::string variant;
  RetryFailure before{RetryFailure::kNone};
  ColumnOutcome outcome{ColumnOutcome::kFailed};
  RetryFailure kind{RetryFailure::kNone};
  std::string note;
};

struct ColumnRetryReport
{
  bool enabled{false};
  int max_retries{0};
  std::vector<ColumnAttemptRecord> attempts;
  std::map<size_t, ColumnOutcome> final_outcome;     ///< retried columns only
  std::vector<std::pair<size_t, std::string>> not_retried;   ///< column, reason
  bool stopped_early{false};
  std::string stop_reason;

  size_t attemptsOf(size_t column) const
  {
    return static_cast<size_t>(std::count_if(
        attempts.begin(), attempts.end(), [&](const ColumnAttemptRecord & a) {return a.column == column;}));
  }
};

using RetryAttemptFn = std::function<RetryAttemptResult(const ColumnRetryPlan &, int attempt, const RetryVariant &)>;

/// Runs the plan: for each column up to max_retries attempts through `attempt_fn`, until it is done. Stops on
/// `cancelled()`, a fatal attempt, or stop_after_failed_columns exhausted columns in a row.
inline ColumnRetryReport runColumnRetries(
  const std::vector<ColumnRetryPlan> & plan, const ColumnRetryConfig & cfg, const RetryAttemptFn & attempt_fn,
  const std::function<bool()> & cancelled = {})
{
  ColumnRetryReport rep;
  rep.enabled = cfg.enabled;
  rep.max_retries = cfg.max_retries;
  int exhausted_in_a_row = 0;
  for (size_t i = 0; i < plan.size(); ++i) {
    const auto & p = plan[i];
    const auto skip_rest = [&](const std::string & why) {
        rep.stopped_early = true;
        rep.stop_reason = why;
        for (size_t k = i; k < plan.size(); ++k) {
          rep.not_retried.push_back({plan[k].column, "retry pass stopped: " + why});
        }
      };
    if (cancelled && cancelled()) {
      skip_rest("canceled");
      return rep;
    }
    if (cfg.stop_after_failed_columns > 0 && exhausted_in_a_row >= cfg.stop_after_failed_columns) {
      skip_rest(std::to_string(exhausted_in_a_row) + " columns in a row did not recover");
      return rep;
    }
    ColumnOutcome last = p.first_outcome;
    RetryFailure kind = p.first_kind;
    bool fatal = false;
    for (int a = 1; a <= cfg.max_retries; ++a) {
      if (cancelled && cancelled()) {
        break;
      }
      const RetryVariant v = chooseVariant(kind, a, cfg);
      const RetryAttemptResult r = attempt_fn(p, a, v);
      ColumnAttemptRecord rec;
      rec.column = p.column;
      rec.stand = p.stand;
      rec.attempt = a;
      rec.variant = v.label();
      rec.before = kind;
      rec.outcome = r.outcome;
      rec.kind = r.outcome == ColumnOutcome::kDone ? RetryFailure::kNone : r.kind;
      rec.note = r.note;
      rep.attempts.push_back(rec);
      last = r.outcome;
      kind = r.kind;
      if (r.fatal) {
        fatal = true;
        break;
      }
      if (last == ColumnOutcome::kDone || kind == RetryFailure::kOverload) {
        break;   // done, or an overload on the retry: never retried further
      }
    }
    rep.final_outcome[p.column] = last;
    exhausted_in_a_row = last == ColumnOutcome::kDone ? 0 : exhausted_in_a_row + 1;
    if (fatal) {
      rep.stopped_early = true;
      rep.stop_reason = "fatal attempt on column " + std::to_string(p.column);
      for (size_t k = i + 1; k < plan.size(); ++k) {
        rep.not_retried.push_back({plan[k].column, "retry pass stopped: " + rep.stop_reason});
      }
      return rep;
    }
  }
  return rep;
}

inline nlohmann::json columnRetryToJson(const ColumnRetryReport & r)
{
  nlohmann::json attempts = nlohmann::json::array();
  for (const auto & a : r.attempts) {
    attempts.push_back(
      {{"column", a.column}, {"stand", a.stand}, {"attempt", a.attempt}, {"variant", a.variant},
        {"failure_before", toString(a.before)}, {"outcome", toString(a.outcome)}, {"failure", toString(a.kind)},
        {"note", a.note}});
  }
  nlohmann::json cols = nlohmann::json::array();
  for (const auto & [c, o] : r.final_outcome) {
    cols.push_back(
      {{"column", c}, {"attempts", r.attemptsOf(c)}, {"final_outcome", toString(o)},
        {"recovered", o == ColumnOutcome::kDone}});
  }
  nlohmann::json skipped = nlohmann::json::array();
  for (const auto & [c, why] : r.not_retried) {
    skipped.push_back({{"column", c}, {"reason", why}});
  }
  return {{"enabled", r.enabled}, {"max_retries", r.max_retries}, {"attempts", attempts}, {"columns", cols},
    {"not_retried", skipped}, {"stopped_early", r.stopped_early}, {"stop_reason", r.stop_reason}};
}

}  // namespace manipulation_manager
