# Module map

The repository holds 33 ROS 2 packages under `src/`. This page says what each one is, what it needs and how the packages depend on each other, so you can lift one part out without taking the rest. Each package has its own README (inputs, outputs, tests, limits); the rules the layering follows are in [architecture.md](architecture.md) and [package_layout.md](package_layout.md).

Nothing here has run on hardware. Behaviour was checked in unit tests and in the MuJoCo and Gazebo simulations ([sim_vs_real.md](sim_vs_real.md)).

## Packages

Column meanings. **ROS-free**: the library at the core of the package has no `rclcpp` and no ROS message dependency (`yes`), only the part named in the tier is ROS-free (`partly`), or the package needs ROS (`no`). Every package is still built with `colcon` and `ament_cmake` or `ament_python`; "ROS-free" means it does not need a ROS runtime. **GPU** and **Simulator**: whether the package needs one to build or run. **Depends on** lists in-repo dependencies declared in `package.xml` (build, exec and export dependencies; test dependencies left out).

| Package | Area | Tier | ROS-free | Needs a GPU | Needs a simulator | Depends on (in this repo) |
|---|---|---|---|---|---|---|
| [`core_types`](../src/common/core_types/README.md) | common | ROS-free core library | yes | no | no | none |
| [`force_core`](../src/common/force_core/README.md) | common | ROS-free core library | yes | no | no | `core_types` |
| [`gravity_core`](../src/control/gravity_core/README.md) | control | ROS-free core library | yes | no | no | none |
| [`ros_adapters`](../src/common/ros_adapters/README.md) | common | ROS node or plugin (adapters) | no | no | no | `core_types` |
| [`mobile_manipulator_description`](../src/description/mobile_manipulator_description/README.md) | description | Description (xacro, profiles, launch) | partly | no | no | none |
| [`mobile_manipulator_moveit_config`](../src/description/mobile_manipulator_moveit_config/README.md) | description | ROS node or plugin (MoveIt config) | no | no | no | `compliant_trajectory_controller`, `ft_bridge_hardware`, `mobile_manipulator_description` |
| [`mobile_manipulator_profile`](../src/description/mobile_manipulator_profile/README.md) | description | ROS-free core library (header-only) | yes | no | no | `mobile_manipulator_description` |
| [`coverage_planner`](../src/perception/coverage_planner/README.md) | perception | ROS-free core library + ROS nodes | partly | no | no | `core_types`, `coverage_planner_msgs`, `ros_adapters`, `scene_model` |
| [`coverage_planner_msgs`](../src/perception/coverage_planner_msgs/README.md) | perception | Message package | no | no | no | none |
| [`robot_self_filter`](../src/perception/robot_self_filter/README.md) | perception | ROS-free core library + ROS node | yes | no | no | `scene_model` |
| [`scene_adapters`](../src/perception/scene_adapters/README.md) | perception | ROS-free core library | yes | no | no | `core_types`, `motion_planner`, `scene_model` |
| [`scene_model`](../src/perception/scene_model/README.md) | perception | ROS-free core library | yes | no | no | none |
| [`wall_detector`](../src/perception/wall_detector/README.md) | perception | ROS-free core library + ROS node | yes | no | no | `scene_model` |
| [`coverage_task_msgs`](../src/manipulation/coverage_task_msgs/README.md) | manipulation | Message package | no | no | no | none |
| [`curobo_bridge`](../src/manipulation/curobo_bridge/README.md) | manipulation | ROS node (Python) over a ROS-free planner | partly | needs a GPU to plan | no | `curobo_bridge_msgs` |
| [`curobo_bridge_msgs`](../src/manipulation/curobo_bridge_msgs/README.md) | manipulation | Message package | no | no | no | none |
| [`curobo_planner`](../src/manipulation/curobo_planner/README.md) | manipulation | ROS node or plugin (client) | no | no (the node it calls needs one) | no | `core_types`, `curobo_bridge_msgs`, `motion_planner`, `scene_model` |
| [`manipulation_bt_prototype`](../src/manipulation/manipulation_bt_prototype/README.md) | manipulation | ROS-free core library (prototype, opt-in) | yes | no | no | none |
| [`manipulation_manager`](../src/manipulation/manipulation_manager/README.md) | manipulation | ROS node | no | no | no | `core_types`, `coverage_task_msgs`, `mobile_manipulator_moveit_config`, `mobile_manipulator_profile`, `motion_planner`, `motion_planner_moveit`, `reachability_msgs`, `ros_adapters`, `stand_navigator_msgs` |
| [`motion_planner`](../src/manipulation/motion_planner/README.md) | manipulation | ROS-free core library | yes | no | no | `core_types` |
| [`motion_planner_moveit`](../src/manipulation/motion_planner_moveit/README.md) | manipulation | ROS node or plugin (MoveIt backend) | no | no | no | `core_types`, `mobile_manipulator_moveit_config`, `mobile_manipulator_profile`, `motion_planner`, `ros_adapters` |
| [`reachability`](../src/manipulation/reachability/README.md) | manipulation | ROS-free core library + tool + optional node | yes | no | no | `reachability_msgs` |
| [`reachability_moveit`](../src/manipulation/reachability_moveit/README.md) | manipulation | ROS-free verifier + MoveIt executables | partly | no | no | `mobile_manipulator_moveit_config`, `motion_planner`, `motion_planner_moveit`, `reachability` |
| [`reachability_msgs`](../src/manipulation/reachability_msgs/README.md) | manipulation | Message package | no | no | no | none |
| [`stand_navigator`](../src/manipulation/stand_navigator/README.md) | manipulation | ROS node (Python) over a ROS-free core | partly | no | no | `stand_navigator_msgs` |
| [`stand_navigator_msgs`](../src/manipulation/stand_navigator_msgs/README.md) | manipulation | Message package | no | no | no | none |
| [`vamp_planner`](../src/manipulation/vamp_planner/README.md) | manipulation | ROS-free core library + plugins | yes | no | no | `core_types`, `motion_planner` |
| [`compliant_trajectory_controller`](../src/control/compliant_trajectory_controller/README.md) | control | ROS node or plugin (controller) | no | no | no | `core_types`, `force_core`, `ft_bridge_hardware`, `mobile_manipulator_description`, `mobile_manipulator_profile`, `ros_adapters` |
| [`ft_bridge_hardware`](../src/control/ft_bridge_hardware/README.md) | control | ROS node or plugin (hardware component) | no | no | no | none |
| [`ft_conditioner`](../src/control/ft_conditioner/README.md) | control | ROS node | no | no | no | `core_types`, `force_core` |
| [`lift_hardware`](../src/control/lift_hardware/README.md) | control | ROS node or plugin (hardware component) over a ROS-free core | partly | no | no | none |
| [`sim_gazebo`](../src/simulation/sim_gazebo/README.md) | simulation | Simulator | no | no | is the simulator | `mobile_manipulator_description` |
| [`sim_realism`](../src/simulation/sim_realism/README.md) | simulation | ROS nodes (python) over ROS-free noise models | partly | no | no (works with both simulators) | none |
| [`sim_isaac`](../src/simulation/sim_isaac/README.md) | simulation | Simulator (stub) | n/a | no | is the simulator | none |
| [`sim_mujoco`](../src/simulation/sim_mujoco/README.md) | simulation | Simulator | partly | no | is the simulator | `coverage_planner`, `coverage_planner_msgs`, `coverage_task_msgs`, `ft_bridge_hardware`, `ft_conditioner`, `manipulation_manager`, `mobile_manipulator_description`, `mobile_manipulator_moveit_config`, `motion_planner_moveit`, `reachability`, `reachability_msgs`, `stand_navigator`, `stand_navigator_msgs`, `wall_detector` |

Dependencies used but not declared in `package.xml`, found in the build files: `reachability_moveit` also links `core_types`, `ros_adapters` and `reachability_msgs` (CMake `find_package`); `manipulation_bt_prototype` includes the core headers of `manipulation_manager` by relative path; the `sim_gazebo` scripts also use `mobile_manipulator_moveit_config`, `coverage_planner_msgs` and `coverage_task_msgs`.

## Dependency view

Layers, lowest first. A package depends only on packages in lower layers (the layer is its longest dependency chain in the table above). Within a layer there are no dependencies.

```text
Layer 0  leaves, depend on nothing in this repo
         core_types   scene_model   mobile_manipulator_description   ft_bridge_hardware
         coverage_planner_msgs  coverage_task_msgs  curobo_bridge_msgs  reachability_msgs  stand_navigator_msgs
         manipulation_bt_prototype   sim_isaac (stub)

Layer 1  one step up
         force_core (core_types)            motion_planner (core_types)        ros_adapters (core_types)
         mobile_manipulator_profile (description)
         reachability (reachability_msgs)   stand_navigator (stand_navigator_msgs)
         robot_self_filter, wall_detector (scene_model)
         curobo_bridge (curobo_bridge_msgs) sim_gazebo (description)

Layer 2  ft_conditioner (force_core)        scene_adapters (motion_planner, scene_model)
         vamp_planner (motion_planner)      curobo_planner (motion_planner, scene_model, curobo_bridge_msgs)
         coverage_planner (scene_model, coverage_planner_msgs, ros_adapters)
         compliant_trajectory_controller (force_core, ft_bridge_hardware, profile, description, ros_adapters)

Layer 3  mobile_manipulator_moveit_config (description, compliant_trajectory_controller, ft_bridge_hardware)
Layer 4  motion_planner_moveit (motion_planner, moveit_config, profile, ros_adapters)
Layer 5  manipulation_manager (motion_planner_moveit, moveit_config, reachability_msgs, stand_navigator_msgs, coverage_task_msgs, ...)
         reachability_moveit (reachability, motion_planner_moveit, moveit_config)
Layer 6  sim_mujoco (the whole stack: manager, planner, reachability, stand navigator, wall detector, conditioner, ...)
```

Reading it from the bottom: the ROS-free cores (`core_types`, `scene_model`, `force_core`, `motion_planner`, `reachability`, `scene_adapters`, `vamp_planner`, the self filter and wall detector cores) form the base; the ROS adapters, backends and the controller sit on them; the manager and the simulators sit on top.

Things to know before you lift a package:

- `mobile_manipulator_moveit_config` depends on `compliant_trajectory_controller` and `ft_bridge_hardware` (for `controller:=jtac`), so taking the MoveIt config also takes the controller. `motion_planner_moveit` and `manipulation_manager` sit above it.
- `motion_planner` is the planner contract. The MoveIt, VAMP and cuRobo backends are separate packages above it and are chosen by name in a YAML ([backend_selection.md](backend_selection.md)).
- The message packages are separate so a client can depend on the types without the server.

## Recipes

Each recipe is a set of packages plus the file or parameter that selects the behaviour. They are starting points assembled from the package READMEs, not tested bundles.

### 1. Only the coverage planner

Plan passes over a wall from a scene file or an occupancy grid, with no robot.

- Packages: `coverage_planner`, `coverage_planner_msgs`, `scene_model`, `core_types`, `ros_adapters`. Optional: `wall_detector` to produce `wall_scene.json` from a live point cloud.
- Selected by: `config/coverage_planner.yaml` (`coverage_pattern`, `swath`, `step`), `config/tool_profiles.yaml` (contact or non-contact tool), and the `GenerateCoveragePath` service request fields.
- Try: `ros2 launch coverage_planner wall_coverage_demo.launch.py wall_scene_json:=/path/wall_scene.json show_rviz:=true`
- See [coverage_patterns.md](coverage_patterns.md) and [tools.md](tools.md).

### 2. Bring your own arm through a profile

- Packages: `mobile_manipulator_description`, `mobile_manipulator_profile`, `mobile_manipulator_moveit_config` (plus `compliant_trajectory_controller` and `ft_bridge_hardware`, which the MoveIt config depends on).
- Selected by: a new `config/profiles/<name>_telescopic.yaml` (arm xacro arguments, joint names, DOF, controller family, home pose), an arm xacro, a generated collision matrix and a controller family folder in the MoveIt config; the four steps are in [robot_description.md](robot_description.md) ("Adding an arm").
- Try: `ros2 launch mobile_manipulator_moveit_config demo.launch.py profile:=<name>`
- Per-robot artefacts (SRDF, spheres, reachability map, ...) from one command: [robot_assets.md](robot_assets.md).

### 3. Swap the free-space planner (MoveIt, VAMP or cuRobo)

- Packages: `motion_planner` plus one backend: `motion_planner_moveit` (MoveIt/OMPL, needs MoveIt), `vamp_planner` (CPU, needs VAMP installed), or `curobo_planner` with `curobo_bridge` and `curobo_bridge_msgs` (needs a GPU for the node). Driven by `manipulation_manager`.
- Selected by: the `planning:` YAML given as the manager's `planning_config` parameter, for example `free_space: vamp` with `plugin_params: {vamp: {robot: ur5}}`. `manipulation_manager/config/planning.yaml` and `planning_curobo.yaml` are the shipped examples.
- Try: `ros2 run manipulation_manager manipulation_manager_node --ros-args -p planning_config:=/path/planning.yaml`
- To add your own backend, implement the ports and register a plugin: [backend_selection.md](backend_selection.md).

### 4. Just the compliant controller and the F/T conditioner

- Packages: `compliant_trajectory_controller`, `force_core`, `core_types`, `ros_adapters`, `ft_bridge_hardware`, `ft_conditioner`, and for the build `mobile_manipulator_description` and `mobile_manipulator_profile`.
- Selected by: the controller entry of `config/jtac_controller.yaml` (`type: compliant_trajectory_controller/JtacController`) with your joints and frames; the conditioner parameters (`input_topic`, `tool_frame`, `payload_mass_kg`, `bias`); the bridge hardware parameter `source_topic` set to the conditioner's `output_topic`. `config/hardware_template.yaml` lists every value to set.
- Try: `ros2 launch ft_conditioner ft_conditioner.launch.py input_topic:=/my_sensor/wrench payload_mass_kg:=0.8`, and `ros2 launch compliant_trajectory_controller jtac_compliance_demo.launch.py` for the controller on mock hardware.
- See [ft_sensing.md](ft_sensing.md) and [hardware_bringup.md](hardware_bringup.md).

### 5. Reachability maps for your own arm

- Packages: `reachability` (core and tool), `reachability_msgs` and the ROS node if you want services. Optional: `reachability_moveit` to confirm stands with IK and collision checks.
- Selected by: your arm as a chain YAML (`reachability/scripts/urdf_to_chain.py` from a URDF), the `rm4d_tool build` options, and for base-aware stands a profile with a `base:` section (`base_profile`). Which real platforms those bases are, with their sensor suites and sources: [base_catalog.md](base_catalog.md).
- Try: `rm4d_tool build chain.yaml arm.rm4d`, then `rm4d_tool bands arm.rm4d --wall-x 0.8 --z-min 0.1 --z-max 3.0 --lift-max 1.5`, then `ros2 launch reachability reachability.launch.py map_file:=arm.rm4d`.
- See [reachability.md](reachability.md).

### 6. Simulate with MuJoCo

- Packages: `sim_mujoco` on top of `mobile_manipulator_description` and `mobile_manipulator_moveit_config`; for contact work add `compliant_trajectory_controller`, `ft_bridge_hardware` and `ft_conditioner`; the scripted passes use the rest of the stack. The MuJoCo SDK is not bundled: set `MUJOCO_ROOT`.
- Selected by: the launch arguments `profile`, `scene`, `controller` (`jtc` or `jtac`), `conditioner`, `depth_sensor`, `base_motion`, and `hardware:=mujoco` in the description.
- Try: `ros2 launch sim_mujoco sim_mujoco.launch.py profile:=ur10e_telescopic`, then `src/simulation/sim_mujoco/scripts/run_smoke.sh`.
- `sim_gazebo` is the alternative backend (`hardware:=gazebo`, headless, no GPU): [sim_gazebo.md](sim_gazebo.md). [sim_mujoco.md](sim_mujoco.md) and [docker.md](docker.md) describe images and the SDK.
