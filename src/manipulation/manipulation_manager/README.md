# manipulation_manager

The orchestrator: sequences a coverage plan's motion segments through `motion_planner`'s `MotionPlanner`, executes them through the compliant controller's `FollowJointTrajectory` action, and reports per-segment and per-run outcomes. It also runs multi-stand missions, with pause, resume, retry and a mission ledger.

## Tier

ROS node (`manipulation_manager_node`, `tool_actuation_node`) over a mostly ROS-light core library (`manipulation_manager_core`: `ManipulationManager`, coverage source and feasibility analysis, mission FSM and ledger, tool actuation).

## Inputs and outputs

- Action server `execute_coverage` (`coverage_task_msgs/ExecuteCoverage`: single stand or several, selected columns, tool mode, resume). Service and topic `mission_command` (`MissionCommand`, and `std_msgs/String` `pause` / `resume`).
- Clients and publishers: action `navigate_to_stand` (`stand_navigator_msgs`), service `PlanStands` of `reachability_msgs`, the controller's `follow_joint_trajectory` and `/<controller_name>/target_force_n` (default controller `combined_arm_lift_controller`), `valve_topic` (default `/tool/valve`, `std_msgs/Bool`); reads `force_topic` (default `/ft_conditioner/wrench_conditioned`) for the peak force.
- Input plan: `coverage_output.json` from `coverage_planner` (parameter `coverage_json_path`), or with `coverage_source:=ros` the latched `CoveragePlan` on `coverage_plan_topic` (`docs/coverage_planner.md`).
- Ports you can implement or call from C++: `ExecutionPort` (`core/execution/execution_port.hpp`), `CoverageSource` (`core/coverage_source.hpp`), `ColumnReachabilityPort` (`core/column_reach_port.hpp`).
- Planning backends are chosen by name in a YAML (`planning_config` parameter; `config/planning.yaml`, `config/planning_curobo.yaml`).
- Parameters: many; see the `declare_parameter` calls in `src/ros/manipulation_manager_node.cpp` (profile, frames, contact and spray tool settings, base stance check, retries, ledger, `run_on_start`, ...).
- Launch: `launch/manipulation_manager.launch.py` (arguments such as `profile`, `arm`, `hardware`, `params_file`).

## Dependencies

- In this repo: `core_types`, `motion_planner`, `motion_planner_moveit`, `ros_adapters`, `mobile_manipulator_profile`, `mobile_manipulator_moveit_config`, `coverage_task_msgs`, `reachability_msgs`, `stand_navigator_msgs`.
- External: `rclcpp`, `rclcpp_action`, MoveIt (`moveit_ros_planning`), `control_msgs`, `nlohmann_json`.
- Needs ROS 2 and MoveIt. No GPU (cuRobo is optional through a plugin). The simulators supply the arm; no simulator is needed to build or unit-test.

## Use it on its own

```bash
ros2 launch manipulation_manager manipulation_manager.launch.py profile:=ur10e_telescopic
ros2 run manipulation_manager manipulation_manager_node --ros-args -p planning_config:=/path/planning.yaml
```

The end-to-end passes (planner, manager, MoveIt, controller, simulator) are scripted in `sim_mujoco` and `sim_gazebo`.

## Tests

158 test cases in 20 files under `test/` (counted from the sources): mission FSM and ledger, feasibility analysis, column ordering, execution ports, tool actuation, the manager core and others.

```bash
colcon test --packages-select manipulation_manager
```

## Status and limits

- Exercised in simulation passes only; nothing here has run on hardware.
- The base stance check only verifies; the manager never asks the reachability planner for a different base pose on its own.
- The tool-actuation feed is built but was not run against the node in a simulation at the time its doc was written, and nothing has run against a real valve.
- Time parameterisation is off by default (`retime:=true`).

## Related docs

- [architecture.md](../../../docs/architecture.md)
- [backend_selection.md](../../../docs/backend_selection.md)
- [stand_navigation.md](../../../docs/stand_navigation.md)
- [tools.md](../../../docs/tools.md)
- [ROADMAP.md](../../../docs/ROADMAP.md)
