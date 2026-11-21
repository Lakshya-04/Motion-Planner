# coverage_task_msgs

The task interface of the manipulation manager: run a coverage plan (one stand or several), and send a typed pause, resume or status command to the running mission.

## Tier

Message package (`rosidl` interfaces only, no code).

## Inputs and outputs

- Action `ExecuteCoverage`: goal fields `coverage_json_path`, `multi_stand`, `tool_mode` (`TOOL_PARAMETER`, `TOOL_CONTACT`, `TOOL_NON_CONTACT`), `columns`, `resume`; result with `success`, `message`, `columns_done` / `columns_partial` / `columns_failed`, per-stand outcomes and `peak_force_n`; feedback with `state`, `stand_index`, `column`, `mission_state` and counters.
- Service `MissionCommand`: `command` is `pause`, `resume` or `status`; the reply gives `accepted`, `state`, `pause_pending`, `message`.
- Message `StandOutcome`: one entry per planned stand (`stand_index`, `reached`, column lists).

## Dependencies

- In this repo: none.
- External: `rosidl_default_generators`, `rosidl_default_runtime`.
- Needs ROS 2 to generate the interfaces. No GPU. No simulator.

## Use it on its own

```xml
<depend>coverage_task_msgs</depend>
```

```bash
colcon build --packages-select coverage_task_msgs
ros2 interface show coverage_task_msgs/action/ExecuteCoverage
```

## Tests

No tests: the package only holds interface definitions. Servers and clients that use it carry the tests (`manipulation_manager`, and the `execute_coverage_client.py` scripts in `sim_mujoco` and `sim_gazebo`).

## Status and limits

- The goal and result fields follow what `manipulation_manager` implements today and may change with it.
- Used in simulation passes only; nothing here has run on hardware.

## Related docs

- [architecture.md](../../../docs/architecture.md)
- [stand_navigation.md](../../../docs/stand_navigation.md)
- [sim_mujoco.md](../../../docs/sim_mujoco.md)
