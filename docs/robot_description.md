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
| Base | macro `(prefix:='')`; `base_footprint` (ground, x forward), `base_link`, `lift_mount_link` (top face, z up); no actuated joint inside the base (wheels and steering are fixed links) |
| Lift | `lift_base_link` (fixed to parent), `lift_joint` (prismatic, +z, lower 0, upper = stroke), `lift_output_link` (arm attaches here). Mimic stage joints are optional and never commanded |
| Arm | macro `(parent *origin prefix:='arm_')`; links `arm_base_link` (z up) and `arm_flange_link` (z out of the last link); exactly six actuated revolute joints between them |
| Tool | `tool_mount_link` and `tcp_link`; the TCP is what coverage poses and planners target |

Planning groups are **chains** (`arm_base_link -> tcp_link`, `base_link -> tcp_link`), so they
stay valid for any arm or lift without editing joint names. The profile YAML is the one place
that lists joint names and order; code and configs read it instead of repeating lists.

## Swapping parts

- **Base:** select with `base:=<name>` (default `generic_base`); see [Bases](#bases). An unknown
  name fails loudly. To add one: write `urdf/base/<name>.xacro` with a `<name>_base` macro that
  satisfies the base contract (the shared `wheeled_base_common.xacro` has the four-wheel and
  two-wheel building blocks), add one branch in `urdf/base/base.xacro`, add the include in
  `mobile_manipulator.urdf.xacro`, and give the profile a `base:` section.
- **Arm:** write `urdf/arms/<name>.xacro` with a macro that follows the contract and add one
  branch in `arm.xacro`. Select with `arm:=<name>`. For the UR family use `arm:=ur10e`, `ur5e`,
  `ur20`, etc. (needs `ur_description`: `ros-<distro>-ur-description`, BSD-3, installed by the
  Docker image). An unknown arm name fails loudly instead of rendering a robot without an arm.
- **Lift:** `lift_stroke`, `lift_stages` (1-3) and `lift_max_velocity` are xacro args; a
  different lift (scissor, chain-driven, ...) is a new macro with the same three outputs.
- **Tool:** change `tcp_xyz` / `tcp_rpy`, or provide a tool macro with the two links.

## Bases

`urdf/base/base.xacro` is the selector (`base:=<name>`, same pattern as `arms/arm.xacro`). Every
base is built from primitives (a body box, cylinder wheels, sphere casters) from published
dimensions; no vendor mesh or URDF is bundled. Wheels, steering modules and casters are fixed
links, so the planning chain (lift + six arm joints) is the same for every base; the vehicle
controller drives the base through `cmd_vel`. Each macro exposes its dimensions as parameters.

| `base:=` | Vehicle | Drive | Strafe | Body L x W (m) | Top of base (m) | Wheel r / track / wheelbase (m) | v / w max (m/s, rad/s) | Mass (kg) |
|---|---|---|---|---|---|---|---|---|
| `generic_base` | box on the ground (default) | omni | yes | 0.90 x 0.60 | 0.35 | none | 0.5 / 1.0 | 60 |
| `ranger_mini` | AgileX Ranger Mini 2.0 | 4ws (omni, Ackermann, spin) | yes | 0.738 x 0.497 | 0.264 | 0.0825 / 0.364 / 0.494 | 1.5 / 1.5 | 70 |
| `scout_mini` | AgileX Scout Mini | skid | no | 0.612 x 0.38 | 0.235 | 0.0875 / 0.4535 / 0.463 | 1.7 / 2.0 | 23 |
| `scout` | AgileX Scout 2.0 | skid | no | 0.93 x 0.48 | 0.349 | 0.1575 / 0.582 / 0.498 | 1.5 / 1.5 | 65 |
| `husky` | Clearpath Husky A200 | skid | no | 0.99 x 0.45 | 0.39 | 0.165 / 0.5708 / 0.512 | 1.0 / 2.0 | 50 |
| `mpo_500` | Neobotix MPO-500 (mecanum) | omni | yes | 0.88 x 0.58 | 0.30 | 0.1 / 0.46 / 0.55 | 1.0 / 1.0 | 100 |
| `mir100` | MiR100-class | diff | no | 0.89 x 0.58 | 0.352 | 0.1 / 0.404 / none | 1.5 / 1.0 | 67 |
| `turtlebot3_burger` | TurtleBot3 Burger (CI, demos) | diff | no | 0.138 x 0.14 | 0.192 | 0.033 / 0.16 / none | 0.22 / 2.84 | 1 |
| `turtlebot3_waffle` | TurtleBot3 Waffle (CI, demos) | diff | no | 0.281 x 0.24 | 0.141 | 0.033 / 0.287 / none | 0.26 / 1.82 | 1.8 |

**The numbers are nominal**, taken from published spec sheets and rounded; wheel positions and
body outlines are simplified (wheels inside the body envelope carry no collision shape, casters
are visual only). Check every value against the unit you actually run (mounting plate height,
tyre wear, payload derating, firmware speed limits) and edit it in the base macro defaults
(`urdf/base/<name>.xacro`) and in the profile's `base:` section together: the selector passes only
`prefix`, so the profile does not drive the model, and `test/test_bases.py` fails when the two
disagree. The velocity and acceleration limits are the vehicle's, not a
recommendation for a loaded 1.5 m lift with an arm: derate them for the navigation stack.

Each profile has a `base:` section next to `lift:`: `name`, `drive_type` (`omni | diff | skid |
4ws`), `can_strafe`, `footprint` (rectangle in `base_footprint` enclosing body, wheels and
casters, for costmaps and base stationing), `wheel_radius_m`, `track_m`, `wheelbase_m`,
`max_linear_velocity_mps`, `max_angular_velocity_radps`, `max_linear_acceleration_mps2`,
`max_angular_acceleration_radps2`, `lift_mount_height_m` (`lift_mount_link` above the ground),
`ground_clearance_m`, `mass_kg`. Base stationing (`docs/reachability.md`) can use `can_strafe` and
the footprint as inputs: a strafing base can take any station pose, a diff/skid base needs a
heading along its motion. The URDF does not read the section; `BaseModel` in the reachability
package does (`rm4d_tool station --base-profile`), including `arm_mount_xy`, the arm base position
on the footprint. `test/test_bases.py` keeps the profile, the footprint, the mass and the macro in
agreement. Profiles per base: `ur10e_<base>_telescopic.yaml` (for example
`ur10e_ranger_mini_telescopic.yaml`); the TurtleBot3 profiles use the generic arm because a UR10e
would tip it. The `ur10e_scout_mini_telescopic.yaml` profile is a geometry example: the Scout
Mini's rated payload is far below a UR10e on a 1.5 m lift, so check the platform rating before
using it.

Swap steps: `xacro urdf/mobile_manipulator.urdf.xacro arm:=ur10e base:=ranger_mini`, or pick the
matching profile. The lift mounts at `lift_mount_link`; shift it with the `lift_mount_xyz` xacro
arg (or edit `mount_x`/`mount_y` in the base macro's defaults) if your lift is not at the body
centre. Vendor
description packages (for example `ranger_ros2`, `husky_description`, `turtlebot3_description`)
are not used; a branch that uses one when installed can be added the way `ur_arm.xacro` does for
UR, provided the vendor frames are re-parented to the three contract links.

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
- The SRDF carries the generated structural allowed-collision matrix (`scripts/generate_acm.py`);
  geometry-specific pairs can be added by sampling, see [moveit_config.md](moveit_config.md).
- `hardware:=mock` adds a ros2_control block (mock components); add real and sim backends as
  further branches of that one xacro.
- Verified here with `check_urdf` and the contract tests for the generic arm and UR10e (rendered
  with the Humble xacro). Loaded into MoveIt and planned with on Jazzy, see [moveit_config.md](moveit_config.md).
