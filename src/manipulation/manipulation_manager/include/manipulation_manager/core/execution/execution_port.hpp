/**
 * @file execution_port.hpp
 * @brief Port for executing an already-planned trajectory on the robot.
 * @details The other half of the plan/execute decoupling described in
 *          docs/design_principles.md:
 *          Planner::plan() only plans; ManipulationManager hands the
 *          resulting core_types::Trajectory to an ExecutionPort to
 *          actually move the robot. ROS-optional -- adapters (e.g.
 *          MoveItExecutionAdapter) convert to/from ROS message types at
 *          their own boundary.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <cstdint>
#include <string>

#include "core_types/trajectory.hpp"

namespace manipulation_manager
{

/**
 * @struct ExecutionRequest
 * @brief Input to ExecutionPort::execute() -- a planned trajectory to run.
 */
struct ExecutionRequest
{
  std::string planning_group;             ///< MoveIt planning group to execute on.
  core_types::Trajectory trajectory;  ///< The already-planned trajectory to run.
  double stroke_speed{0.0};           ///< Tool speed (m/s) of a contact seam; 0 = a transition.
};

/**
 * @struct ExecutionResult
 * @brief Outcome of ExecutionPort::execute().
 */
struct ExecutionResult
{
  bool success{false};                      ///< True if the trajectory executed successfully.
  std::string message;                      ///< Additional detail/error message.
  std::string failure_reason_code{"none"};  ///< Machine-readable failure reason.
  uint64_t execution_time_ms{0};            ///< Time spent executing.
};

/**
 * @class ExecutionPort
 * @brief Executes a planned trajectory on the robot. Never plans.
 */
class ExecutionPort
{
public:
  virtual ~ExecutionPort() = default;
  virtual ExecutionResult execute(const ExecutionRequest & request) = 0;
};

}  // namespace manipulation_manager
