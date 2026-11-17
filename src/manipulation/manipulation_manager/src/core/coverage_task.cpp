#include "manipulation_manager/core/coverage_task.hpp"

#include <algorithm>
#include <sstream>

namespace manipulation_manager
{

StandAssignment assignColumnsToStands(
  const std::vector<size_t> & poses_per_column,
  const std::vector<std::vector<size_t>> & stand_targets,
  const std::vector<size_t> & wanted)
{
  StandAssignment out;
  out.columns_per_stand.resize(stand_targets.size());

  // pose index -> column
  std::vector<size_t> column_of_pose;
  for (size_t c = 0; c < poses_per_column.size(); ++c) {
    column_of_pose.insert(column_of_pose.end(), poses_per_column[c], c);
  }
  // covered[stand][column] = poses of the column the stand reaches
  std::vector<std::vector<size_t>> covered(
    stand_targets.size(), std::vector<size_t>(poses_per_column.size(), 0));
  for (size_t s = 0; s < stand_targets.size(); ++s) {
    for (const size_t t : stand_targets[s]) {
      if (t < column_of_pose.size()) {
        ++covered[s][column_of_pose[t]];
      }
    }
  }

  std::vector<bool> use(poses_per_column.size(), wanted.empty());
  for (const size_t c : wanted) {
    if (c < use.size()) {
      use[c] = true;
    }
  }
  for (size_t c = 0; c < poses_per_column.size(); ++c) {
    if (!use[c] || poses_per_column[c] == 0) {
      continue;
    }
    size_t best = 0;
    size_t best_count = 0;
    for (size_t s = 0; s < stand_targets.size(); ++s) {
      if (covered[s][c] > best_count) {   // strict: the first stand wins a tie
        best_count = covered[s][c];
        best = s;
      }
    }
    if (best_count == 0) {
      out.unassigned.push_back(c);
    } else {
      out.columns_per_stand[best].push_back(c);
    }
  }
  return out;
}

std::vector<std::vector<size_t>> groupByLateralSpan(const std::vector<double> & lateral, double max_span)
{
  std::vector<size_t> order(lateral.size());
  for (size_t i = 0; i < order.size(); ++i) {
    order[i] = i;
  }
  std::stable_sort(
    order.begin(), order.end(), [&](size_t a, size_t b) {return lateral[a] < lateral[b];});
  std::vector<std::vector<size_t>> groups;
  for (const size_t i : order) {
    if (groups.empty() || (max_span > 0.0 && lateral[i] - lateral[groups.back().front()] > max_span)) {
      groups.emplace_back();
    }
    groups.back().push_back(i);
  }
  return groups;
}

const char * toString(TaskState s)
{
  switch (s) {
    case TaskState::kIdle: return "idle";
    case TaskState::kPlanningStands: return "planning_stands";
    case TaskState::kNavigating: return "navigating";
    case TaskState::kRunningColumns: return "running_columns";
    case TaskState::kSucceeded: return "succeeded";
    case TaskState::kFailed: return "failed";
    case TaskState::kCanceled: return "canceled";
  }
  return "unknown";
}

TaskStateMachine::TaskStateMachine(int nav_attempts)
: nav_attempts_(std::max(1, nav_attempts))
{
}

bool TaskStateMachine::terminal() const
{
  return state_ == TaskState::kSucceeded || state_ == TaskState::kFailed ||
         state_ == TaskState::kCanceled;
}

bool TaskStateMachine::startSingle()
{
  if (state_ != TaskState::kIdle) {
    return false;
  }
  stand_count_ = 1;
  state_ = TaskState::kRunningColumns;
  return true;
}

bool TaskStateMachine::startMulti()
{
  if (state_ != TaskState::kIdle) {
    return false;
  }
  state_ = TaskState::kPlanningStands;
  return true;
}

bool TaskStateMachine::standsPlanned(size_t stand_count)
{
  if (state_ != TaskState::kPlanningStands) {
    return false;
  }
  stand_count_ = stand_count;
  stand_ = 0;
  attempts_made_ = 0;
  state_ = stand_count == 0 ? TaskState::kFailed : TaskState::kNavigating;
  return true;
}

bool TaskStateMachine::standsFailed()
{
  if (state_ != TaskState::kPlanningStands) {
    return false;
  }
  state_ = TaskState::kFailed;
  return true;
}

void TaskStateMachine::nextStand()
{
  ++stand_;
  attempts_made_ = 0;
  state_ = stand_ >= stand_count_ ? TaskState::kSucceeded : TaskState::kNavigating;
}

NavVerdict TaskStateMachine::navigationResult(bool reached)
{
  if (state_ != TaskState::kNavigating) {
    return NavVerdict::kFinish;
  }
  ++attempts_made_;
  if (reached) {
    state_ = TaskState::kRunningColumns;
    return NavVerdict::kRun;
  }
  if (attempts_made_ < nav_attempts_) {
    return NavVerdict::kRetry;
  }
  nextStand();
  return state_ == TaskState::kSucceeded ? NavVerdict::kFinish : NavVerdict::kSkip;
}

bool TaskStateMachine::standFinished()
{
  if (state_ != TaskState::kRunningColumns || stand_count_ == 0) {
    return false;
  }
  nextStand();
  return true;
}

bool TaskStateMachine::singleFinished()
{
  if (state_ != TaskState::kRunningColumns) {
    return false;
  }
  state_ = TaskState::kSucceeded;
  return true;
}

bool TaskStateMachine::cancel()
{
  if (terminal()) {
    return false;
  }
  state_ = TaskState::kCanceled;
  return true;
}

void TaskTracker::beginSingle()
{
  multi_ = false;
  stands_.clear();
  done_.clear();
  partial_.clear();
  failed_.clear();
}

void TaskTracker::beginMulti(size_t n)
{
  const auto resumed = done_;     // columns settled before the stands were planned (a resume) stay counted
  const auto skipped = failed_;
  beginSingle();
  done_ = resumed;
  failed_ = skipped;
  multi_ = true;
  stands_.resize(n);
  for (size_t i = 0; i < n; ++i) {
    stands_[i].index = i;
  }
}

void TaskTracker::standReached(size_t stand, bool reached, const std::string & message)
{
  if (stand < stands_.size()) {
    stands_[stand].reached = reached;
    stands_[stand].message = message;
  }
}

void TaskTracker::columnFinished(size_t stand, size_t column, ColumnOutcome outcome)
{
  auto & all = outcome == ColumnOutcome::kDone ? done_ :
    outcome == ColumnOutcome::kPartial ? partial_ : failed_;
  all.push_back(column);
  if (stand < stands_.size()) {
    auto & per = outcome == ColumnOutcome::kDone ? stands_[stand].done :
      outcome == ColumnOutcome::kPartial ? stands_[stand].partial : stands_[stand].failed;
    per.push_back(column);
  }
}

void TaskTracker::columnSkipped(size_t column)
{
  failed_.push_back(column);
}

bool TaskTracker::success() const
{
  if (!partial_.empty() || !failed_.empty()) {
    return false;
  }
  return std::all_of(
    stands_.begin(), stands_.end(), [](const Stand & s) {
      return s.reached || (s.done.empty() && s.partial.empty() && s.failed.empty() && s.message.empty());
    });
}

std::string TaskTracker::summary() const
{
  std::ostringstream os;
  os << done_.size() << " done, " << partial_.size() << " partial, " << failed_.size() << " failed";
  for (const auto & s : stands_) {
    if (!s.reached && !s.message.empty()) {
      os << "; stand " << s.index << " not reached (" << s.message << ")";
    }
  }
  return os.str();
}

}  // namespace manipulation_manager
