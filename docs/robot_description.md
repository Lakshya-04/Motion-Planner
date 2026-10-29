# Robot description (`mobile_manipulator_description`)

A generic mobile-manipulator model: **base + lift + 6-DOF arm + tool = 7 actuated DOF**.
Every part is swappable behind a small contract, so the same planners, reachability maps and
controllers run on any combination. Starter configuration: a **telescopic lift** and a
**UR10e** arm. A dependency-free generic 6-DOF arm ships for tests and CI.

Package: [`src/description/mobile_manipulator_description`](../src/description/mobile_manipulator_description).

## Layout

```
urdf/
  mobile_manipulator.urdf.xacro   top level; args: arm, lift_stroke, lift_stages, tcp_xyz, ...
  base/generic_base.xacro         box base (replace with a real base)
  lift/telescopic_lift.xacro      master prismatic joint + 0..3 mimic stages
  arms/arm.xacro                  arm selector + the arm contract
  arms/generic_6dof_arm.xacro     reference arm, primitives only
  arms/ur_arm.xacro               UR family via ur_description (not bundled)
  tools/generic_tool.xacro        tool mount + TCP
srdf/mobile_manipulator.srdf.xacro   groups as chains: arm (6), lift_arm (7), tool
config/profiles/*.yaml           single source of joint names, order, limits, frames, home pose
test/test_description.py         contract tests (no install needed)
```

Frame chain: `base_footprint > base_link > lift_mount_link > lift_base_link > (lift_joint) >
lift_carriage_link > lift_output_link > arm_base_link > ... > arm_flange_link > tool_mount_link
> tcp_link`.

## Contracts (what a replacement must provide)

| Part | Provides |
|---|---|
| Base | `base_footprint`, `base_link`, `lift_mount_link` |
| Lift | `lift_base_link` (fixed to parent), `lift_joint` (prismatic, +z, lower 0, upper = stroke), `lift_output_link` (arm attaches here). Mimic stage joints are optional and never commanded |
| Arm | macro `(parent *origin prefix:='arm_')`; links `arm_base_link` (z up) and `arm_flange_link` (z out of the last link); exactly six actuated revolute joints between them |
| Tool | `tool_mount_link` and `tcp_link`; the TCP is what coverage poses and planners target |

Planning groups are **chains** (`arm_base_link -> tcp_link`, `base_link -> tcp_link`), so they
stay valid for any arm or lift without editing joint names. The profile YAML is the one place
that lists joint names and order; code and configs read it instead of repeating lists.

## Swapping parts

- **Arm:** write `urdf/arms/<name>.xacro` with a macro that follows the contract and add one
  branch in `arm.xacro`. Select with `arm:=<name>`. For the UR family use `arm:=ur10e`, `ur5e`,
  `ur20`, etc. (needs `ur_description`: `ros-<distro>-ur-description`, BSD-3, installed by the
  Docker image). An unknown arm name fails loudly instead of rendering a robot without an arm.
- **Lift:** `lift_stroke`, `lift_stages` (1-3) and `lift_max_velocity` are xacro args; a
  different lift (scissor, chain-driven, ...) is a new macro with the same three outputs.
- **Tool:** change `tcp_xyz` / `tcp_rpy`, or provide a tool macro with the two links.

## Lift modelling

One controlled joint carries the arm over the full stroke; the visible telescoping stages follow
it through `<mimic>` joints (each `stroke / stages` relative to the stage below). This keeps a
single commanded lift DOF for planning and control, avoids re-implementing a 50/50 split in
code, and is the same modelling pattern used by production telescopic lifts. Defaults: 1.5 m
stroke, two stages, 0.2 m/s (check these against your actual lift and set the args).

Note: mimic joints must appear in `/joint_states` (or be resolved by the planner), otherwise
collision geometry on the stage links sits at the wrong height. Publish them with a small
relay, or let the planner resolve mimics.

## Use

```bash
xacro urdf/mobile_manipulator.urdf.xacro arm:=ur10e -o robot.urdf
ros2 launch mobile_manipulator_description display.launch.py arm:=generic_6dof
python3 -m pytest src/description/mobile_manipulator_description/test
```

## Limits and next steps

- Collision geometry is primitives; generate a collision-sphere model per robot (FOAM, see
  [vamp_notes.md](vamp_notes.md)) for the VAMP / cuRobo backends and the
  [self filter](robot_self_filter.md).
- The SRDF carries structural exclusions only; generate the full allowed-collision matrix per
  arm.
- Add the `ros2_control` hardware block per backend (real, mock, sim) as a separate xacro.
- Verified here with `check_urdf` and the contract tests for the generic arm and UR10e (rendered
  with the Humble xacro). Not yet loaded into MoveIt.
