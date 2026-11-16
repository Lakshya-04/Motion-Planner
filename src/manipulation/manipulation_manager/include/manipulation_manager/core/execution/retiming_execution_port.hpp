/**
 * @file retiming_execution_port.hpp
 * @brief ExecutionPort decorator that retimes the planned trajectory by segment role first.
 * @details Opt-in (the node builds it only with `retime:=true`). A request with a stroke speed
 *          is a contact seam (constant tool speed, jerk limited, a limit violation fails the
 *          segment with `time_limit_violation`); any other request is a transition (time
 *          optimal, violations only counted). Reversals split the path; the pieces are joined
 *          again with the stop at the cut, so the inner port still gets one trajectory.
 *          ROS-free: the retimer is the motion_planner::TimeParameterizer port.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "manipulation_manager/core/execution/execution_port.hpp"
#include "motion_planner/core/time_parameterization.hpp"

namespace manipulation_manager
{

class RetimingExecutionPort : public ExecutionPort
{
public:
  RetimingExecutionPort(
    std::shared_ptr<ExecutionPort> inner, std::shared_ptr<motion_planner::TimeParameterizer> retimer,
    std::vector<motion_planner::JointLimit> seam_limits,
    std::vector<motion_planner::JointLimit> transition_limits)
  : inner_(std::move(inner)), retimer_(std::move(retimer)),
    seam_limits_(std::move(seam_limits)), transition_limits_(std::move(transition_limits)) {}

  ExecutionResult execute(const ExecutionRequest & request) override
  {
    using motion_planner::SegmentRole;
    motion_planner::TimingRequest timing;
    timing.role = request.stroke_speed > 0.0 ? SegmentRole::kContactSeam : SegmentRole::kTransition;
    timing.stroke_speed = request.stroke_speed;
    timing.limits = timing.role == SegmentRole::kContactSeam ? seam_limits_ : transition_limits_;

    last_ = motion_planner::parameterizeSegment(*retimer_, request.trajectory, timing);
    if (!last_.success) {
      ExecutionResult fail;
      fail.message = std::string("retiming: ") + motion_planner::toString(last_.error) + ": " +
        last_.message;
      fail.failure_reason_code = last_.error == motion_planner::TimingError::kLimitViolation ?
        "time_limit_violation" : "retiming_failed";
      return fail;
    }
    ExecutionRequest timed = request;
    timed.trajectory = motion_planner::concatenate(last_.pieces);
    return inner_->execute(timed);
  }

  /// The outcome of the most recent retiming (violations on a transition live here).
  const motion_planner::TimingResult & lastTiming() const {return last_;}

private:
  std::shared_ptr<ExecutionPort> inner_;
  std::shared_ptr<motion_planner::TimeParameterizer> retimer_;
  std::vector<motion_planner::JointLimit> seam_limits_;
  std::vector<motion_planner::JointLimit> transition_limits_;
  motion_planner::TimingResult last_;
};

}  // namespace manipulation_manager
