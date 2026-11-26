# Robot description (`mobile_manipulator_description`)

A generic mobile-manipulator model: **base + lift + N-DOF arm + tool = 1 + N actuated DOF**
(7 for a 6-DOF arm, 8 for a 7-DOF one). Every part is swappable behind a small contract, so
the same planners, reachability maps and controllers run on any combination. Starter
configuration: a **telescopic lift** and a **UR10e** arm. A dependency-free generic 6-DOF arm
and primitives-only 7-DOF arms (Panda, FR3, iiwa 7, Gen3, xArm 7; published kinematics and limits,
no vendor meshes) ship for tests and CI, so the stack is exercised on 6 and 7 joints; see [Arms](#arms).

Package: [`src/description/mobile_manipulator_description`](../src/description/mobile_manipulator_description).

## Layout

```
urdf/
  mobile_manipulator.urdf.xacro   top level; args: arm, lift_stroke, lift_stages, tcp_xyz, ...
  base/generic_base.xacro         box base (replace with a real base)
  lift/lift.xacro                 lift selector: telescopic | scissor | fixed | none
  lift/telescopic_lift.xacro      master prismatic joint + 0..3 mimic stages
  lift/scissor_lift.xacro         master prismatic joint + visual scissor linkage on mimic joints
  arms/arm.xacro                  arm selector + the arm contract
  arms/generic_6dof_arm.xacro     reference 6-DOF arm, primitives only
  arms/panda_arm.xacro            7-DOF Franka Panda-like arm, primitives only
  arms/serial_arm_common.xacro    shared link / joint macros of the arms below (cylinders on the chain)
  arms/{fr3,iiwa7,kinova_gen3,xarm7}_arm.xacro   7-DOF arms from published tables, primitives only
  arms/ur_arm.xacro               UR family via ur_description (not bundled)
  tools/generic_tool.xacro        tool mount + TCP
srdf/mobile_manipulator.srdf.xacro   groups as chains (names from the profile): arm (N), lift_arm (1 + N), tool
srdf/acm_<arm>.xacro             generated allowed-collision matrix per arm (scripts/generate_acm.py)
config/profiles/*.yaml           single source of group names, joint names + order (so the DOF), frames, home pose
mobile_manipulator_description/  Python profile loader (load_profile)
test/                            contract, profile and arm-swap tests (no install needed)
```

Frame chain: `base_footprint > base_link > lift_mount_link > lift_base_link > (lift_joint) >
lift_carriage_link > lift_output_link > arm_base_link > ... > arm_flange_link > tool_mount_link
> tcp_link`.

## Contracts (what a replacement must provide)

| Part | Provides |
|---|---|
| Base | macro `(prefix:='')`; `base_footprint` (ground, x forward), `base_link`, `lift_mount_link` (top face, z up); no actuated joint inside the base (wheels and steering are fixed links) |
| Lift | `lift_base_link` (fixed to parent), `lift_joint` (prismatic, +z, lower 0, upper = stroke), `lift_output_link` (arm attaches here). Mimic stage joints are optional and never commanded |
| Arm | macro `(parent *origin prefix:='arm_')`; links `arm_base_link` (z up) and `arm_flange_link` (z out of the last link); N actuated revolute joints between them, in kinematic order (N is what the arm has: 6 for UR and the generic arm, 7 for the Panda-like arm). The profile lists them in `joints.arm`, which is where N is stated |
| Tool | `tool_mount_link` and `tcp_link`; the TCP is what coverage poses and planners target |

Planning groups are **chains** (`arm_base_link -> tcp_link`, `base_link -> tcp_link`), so they
stay valid for any arm or lift without editing joint names. The profile YAML is the one place
that lists group names, joint names and order; code and configs read it instead of repeating
lists:

```yaml
groups: {arm: arm, whole_body: lift_arm, tool: tool}   # planning-group names
arm:    {dof: 6, controller_family: ur}                # dof is checked against joints.arm
joints: {lift: lift_joint, arm: [...N names...], mimic: [...]}
home:   {lift_joint: 0.4, <arm joint>: ..., ...}       # every actuated joint
```

Readers: the xacro files (`urdf/arm_joint_names.xacro`: SRDF groups, home state, ros2_control
block), the Python loader (`mobile_manipulator_description.load_profile(name_or_path)`, fields
`arm_joints`, `arm_dof`, `dof`, `joints`, `whole_body_group`, `arm_group`, `tcp_link`,
`home_positions`, `frames`) and the header-only C++ loader in `mobile_manipulator_profile`
(`loadProfile`, same fields). The xacro `profile` argument selects the file; when empty it follows
`arm` (`ur*` -> `ur10e_telescopic`, otherwise `<arm>_telescopic`).

## Swapping parts

- **Base:** select with `base:=<name>` (default `generic_base`); see [Bases](#bases). An unknown
  name fails loudly. To add one: write `urdf/base/<name>.xacro` with a `<name>_base` macro that
  satisfies the base contract (the shared `wheeled_base_common.xacro` has the four-wheel and
  two-wheel building blocks), add one branch in `urdf/base/base.xacro`, add the include in
  `mobile_manipulator.urdf.xacro`, and give the profile a `base:` section.
- **Arm:** see "Adding an arm" below. Select with `arm:=<name>` (the `ur*` names need
  `ur_description`: `ros-<distro>-ur-description`, BSD-3, installed by the Docker image). An
  unknown arm name fails loudly instead of rendering a robot without an arm.
- **Lift:** `lift:=telescopic|scissor|fixed|none`, see [Lift variants](#lift-variants). All of them
  keep the three lift outputs, so a different lift (chain-driven, ...) is a new macro with the same
  outputs plus one branch in `lift/lift.xacro`. An unknown name fails loudly.
- **Tool:** change `tcp_xyz` / `tcp_rpy`, or provide a tool macro with the two links.

## Adding an arm

1. `urdf/arms/<name>.xacro`: a macro (for a primitives-only serial arm, build it from `serial_arm_common.xacro` like `fr3_arm.xacro`) `<name>_arm(parent *origin prefix)` that follows the contract
   above (any number N of revolute joints). Include it from `mobile_manipulator.urdf.xacro` and add
   one branch for it in `arms/arm.xacro`.
2. `config/profiles/<name>_telescopic.yaml`: copy a shipped profile, set `robot.xacro_args.arm`,
   the joint names in `joints.arm`, `arm.dof`, `arm.controller_family`, and a collision-free `home`
   pose with a value for every joint.
3. `scripts/generate_acm.py --arm <name> --out srdf/acm_<name>.xacro` (the SRDF includes
   `acm_<name>.xacro`; the UR range shares `acm_ur.xacro`). A test fails if it is stale.
4. `mobile_manipulator_moveit_config/config/<controller_family>/`: `moveit_controllers.yaml`,
   `ros2_controllers.yaml`, `jtac_controllers.yaml` with the lift joint and the arm joints, and a
   `joint_limits.yaml` if the arm's limits differ from the shared file.

The swap tests (`test/test_arm_swap.py`, and `test_moveit_swap.py` in the MoveIt package) then cover
the new profile as soon as its name is in their profile lists. The ros2_control and SRDF blocks
support up to 12 arm joints.

## Arms

All arms are primitives (cylinders along the kinematic chain) with the published joint frames and
limits; no vendor mesh or description package is bundled or required (the `ur*` family is the one
exception and uses `ur_description` from apt). Joint names are `arm_joint1..arm_joint7` for the
7-DOF arms, the UR names for UR. "Status" says what was checked: **kinematics checked** means the
joint frames and limits were compared with the vendor's public description (read, not copied; no
code or mesh is used, ADR 0002) and a test pins the zero-pose geometry; **not on hardware** applies to
every row, nothing here has driven a real arm.

| `arm:=` | Arm | DOF | Source of the parameters | Status |
|---|---|---|---|---|
| `generic_6dof` | reference arm, UR-like proportions | 6 | ours | tests and CI |
| `ur3e`, `ur5e`, `ur10e`, `ur16e`, `ur20` (also `ur3`, `ur5`, `ur10`, `ur30`) | Universal Robots | 6 | `ur_description` (BSD-3, installed by the image), per `ur_type` | UR10e verified in the image; the other profiles (`ur3e_telescopic` ...) render the same macro and are in the swap tests, but only where `ur_description` is installed (skipped in a plain checkout) |
| `panda` | Franka Emika Panda | 7 | published Craig DH table and limits (d1 0.333, d3 0.316, d5 0.384, a4 0.0825, a5 -0.0825, a7 0.088, flange 0.107) | kinematics checked against franka_description `kinematics.yaml`; limits are the older Panda ones |
| `fr3` | Franka Research 3 | 7 | same DH table; limits, velocities and torques from franka_description `robots/fr3/joint_limits.yaml` (Apache-2.0) | kinematics checked; no hardware |
| `iiwa7` | KUKA LBR iiwa 7 R800 | 7 | datasheet link lengths (0.34 / 0.40 / 0.40 / 0.126 m), axes z, y, z, -y, z, y, z as in the `iiwa_ros2` iiwa14 description; limits +-170 / 120 / 170 / 120 / 170 / 120 / 175 deg | geometry from the datasheet; velocity and torque rows not cross-checked against a vendor description of the 7 kg arm (the available one is the iiwa14) |
| `kinova_gen3` | Kinova Gen3 7-DOF (no vision module, no gripper) | 7 | `gen3_macro.xacro` of `ros2_kortex` (BSD-3) = the Gen3 user-guide DH table | kinematics checked. The four joints that are continuous on the real arm (1, 3, 5, 7) are modelled revolute +-2 pi |
| `xarm7` | UFactory xArm 7 | 7 | `xarm7_default_kinematics.yaml`, `xarm7.urdf.xacro` and `joint_limits.yaml` of `xarm_ros2` (BSD-3) | kinematics checked. Effort limits are the vendor's peak-torque figures |

Profiles: `<arm>_telescopic.yaml` for each row (`panda_telescopic`, `fr3_telescopic`, `iiwa7_telescopic`,
`kinova_gen3_telescopic`, `xarm7_telescopic`, `ur3e_telescopic`, `ur5e_telescopic`, `ur10e_telescopic`,
`ur16e_telescopic`, `ur20_telescopic`, `generic_6dof_telescopic`). The home pose of each new arm is a
moderate ready posture chosen by hand (Gen3: the vendor's Home action) and **has not been checked for
self-collision in a simulator**; run `robot_assets.py check` for the arm before trusting it. The
default `tcp_xyz` of the new profiles is a nominal gripper-class offset (0.10 to 0.17 m), edit it. A lift
payload of 150 kg says nothing about whether a small arm such as the xArm 7 or a UR3e makes sense on it.
The four new 7-DOF arms share the `arm7` MoveIt controller family (same joint names); velocity limits come
from the URDF, only the accelerations are set in `config/arm7/joint_limits.yaml` (a guess, not a vendor
figure). `hardware:=real` drivers for `fr3` and `xarm7` are listed in `config/hardware_drivers.yaml` (not
run); `iiwa7` and `kinova_gen3` have none, so `real` fails loudly for them.

The 7-DOF `home` postures are unchecked, and the MuJoCo pass scripts no longer depend on them: with `MP_PROFILE` set to
one of these profiles they derive the pre-contact pose by planning (`ready_pose.py --pose`, wall placement from
`profile_reach.py`; docs/sim_mujoco.md, "Running another robot profile"). That path has not been run in a simulator.

What is not covered: meshes, inertias (placeholder 2 kg per link), joint friction, cable limits, the
vendors' own collision models, and the redundancy of a 7-DOF arm in IK (the null space is left to the
IK solver; for the reachability map see [reachability.md](reachability.md), "Redundant arms").

## Bases

`urdf/base/base.xacro` is the selector (`base:=<name>`, same pattern as `arms/arm.xacro`). Every
base is built from primitives (a body box, cylinder wheels, sphere casters) from published
dimensions; no vendor mesh or URDF is bundled. Wheels, steering modules and casters are fixed
links, so the planning chain (lift + the profile's arm joints) is the same for every base; the vehicle
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
| `jackal` | Clearpath Jackal (4-wheel differential / skid) | diff | no | 0.508 x 0.37 | 0.25 | 0.098 / 0.3756 / 0.262 | 2.0 / 4.0 | 17 |
| `dingo_o` | Clearpath Dingo-O (mecanum) | omni | yes | 0.506 x 0.40 | 0.172 | 0.05 / 0.404 / 0.3 | 1.1 / 2.0 | 17 |
| `tiago_style` | round differential base, torso-lift robot style | diff | no | 0.54 x 0.54 (box) | 0.30 | 0.0985 / 0.4044 / none | 1.0 / 1.0 | 60 |
| `fetch_style` | round differential base, torso-lift robot style | diff | no | 0.56 x 0.56 (box) | 0.30 | 0.0605 / 0.3748 / none | 1.0 / 2.0 | 70 |

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

The last four rows are approximate (written from the public spec sheets without a unit to check
against; the round bases are modelled as their bounding box, so the footprint is conservative). The
torso lift of `tiago_style` / `fetch_style` is not part of the base: it is the profile's `lift:` as
a one-carriage column (`telescopic`, `stages: 0`, stroke 0.35 / 0.386 m), the existing
[column-lift recipe](#lift-variants), so no new lift macro was needed. Profiles:
`kinova_gen3_jackal_nolift`, `xarm7_dingo_o_nolift` (small bases, short pedestal, no lift),
`fr3_tiago_style_torso`, `generic_6dof_fetch_style_torso`. The sizes are payload-honest only for the
small arms chosen; check the platform rating.

What each platform ships with (lidars, cameras) and which numbers have a source is in
[base_catalog.md](base_catalog.md); the profile's optional `sensors:` list holds the sensors (nothing reads it yet).

Swap steps: `xacro urdf/mobile_manipulator.urdf.xacro arm:=ur10e base:=ranger_mini`, or pick the
matching profile. The lift mounts at `lift_mount_link`; shift it with the `lift_mount_xyz` xacro
arg (or edit `mount_x`/`mount_y` in the base macro's defaults) if your lift is not at the body
centre. Vendor
description packages (for example `ranger_ros2`, `husky_description`, `turtlebot3_description`)
are not used; a branch that uses one when installed can be added the way `ur_arm.xacro` does for
UR, provided the vendor frames are re-parented to the three contract links.

## Lift variants

Selected with `lift:=<type>`; the profile's `lift:` section is the one place that states the values
(`scripts/profile_xacro_args.py` turns it into xacro args, and the asset command and the tests
use it). Every variant has `lift_base_link`, `lift_carriage_link`, `lift_joint`, `lift_output_link`,
so SRDF chains, reachability and planners do not change. Published lifts of known mobile manipulators and
commercial columns, with sources, are in [lift_catalog.md](lift_catalog.md).

| `lift:` | `lift_joint` | Extra links | Notes |
|---|---|---|---|
| `telescopic` | prismatic, 0..`stroke` | `lift_stage_N_link` (`stages` 0-3, mimic) | default; `stages:=0` is the recipe for a chain / column lift below |
| `scissor` | prismatic, 0..`stroke` | `lift_scissor_K_{center,a,b}` per X-level (`stages` levels, mimic) | box primitives; angles are linearly interpolated, so the linkage is a visual envelope (conservative vertically, up to about 2 cm narrower sideways), the platform height is exact |
| `fixed` | fixed, pinned at `lift_fixed_height` (default 0.4) above retracted | the telescopic column | the older `lift_fixed_height:=<m>` on any variant does the same; this is the unified name |
| `none` | fixed | a pedestal | arm mounted directly on the base at `lift_mount_height` (default 0.3) |

Profile `lift:` keys: `type`, `stroke_m`, `stages`, `mount_height_m` (`lift_mount_height`: height of
`lift_output_link` above `lift_base_link` when retracted; for a scissor this is the minimum height),
`max_velocity_mps`, `max_acceleration_mps2` (URDF cannot carry an acceleration: it is not wired into MoveIt yet,
so copy it into `mobile_manipulator_moveit_config/config/joint_limits.yaml` by hand), and `payload_kg` (a note about
the real lift's rating, not used by the model), and `geometry` (below). With `none` / `fixed` the lift joint is not actuated:
the SRDF omits it from the home state, the ros2_control block does not command it, `lift_arm` has the
same six joints as `arm`, and the profile sets `joints.lift: null`.

The structural collision matrix has its own files for `none` and `scissor`
(`srdf/acm_<family>_<lift>.xacro`, `scripts/generate_acm.py --lift`); `telescopic` and `fixed` share the
column's. Profiles: `ur10e_telescopic`, `ur10e_scissor`, `ur10e_nolift`, `generic_6dof_telescopic`,
`ur10e_lift_reference`.

**Column geometry (telescopic and fixed).** Without a `geometry` section the stages are cylinders as tall
as the column. With one, the column is described from primitives (boxes or cylinders, no meshes), so the
model can follow a measured lift:

```yaml
lift:
  type: telescopic
  stroke_m: 0.8
  stages: 2
  mount_height_m: 0.738        # lift_output_link above lift_base_link, retracted
  geometry:
    shape: box                 # cylinder (default) | box, for the stage tubes
    column: {size_m: [0.186, 0.186, 0.648]}                 # fixed outer tube: x, y, height
    stages:                                                 # outermost first, one entry per stage
      - {size_m: [0.164, 0.164, 0.690], rest_z_m: -0.014}   # outer x, y, height; bottom above the column's bottom, retracted
      - {size_m: [0.146, 0.146, 0.490], rest_z_m: 0.213}
    carriage: {size_m: [0.212, 0.22, 0.035]}                # plate on the last stage
```

`scripts/profile_xacro_args.py` turns it into `lift_shape`, `lift_column_size`, `lift_stage_sizes`,
`lift_stage_rest_z` and `lift_carriage_size`. The stack must be continuous or the xacro refuses to render:
the top of the last stage (`rest_z_m` + height) equals `mount_height_m` minus the arm mount height
(`arm_mount_xyz` z, so set it to the plate's thickness here), every stage is narrower than the one below,
and there is one entry per stage. Whether neighbouring tubes still overlap at full extension depends on the
stroke and is checked by `test/test_lifts.py` (`_stack_violations`), not by the xacro: a stroke the tubes
cannot cover leaves a gap. The first stage moves `stroke / stages` and the last one the whole stroke, so
`ur10e_lift_reference` (0.8 m, two stages: the middle tube 0.4 m, the top tube 0.8 m) models a three-tube
column. The reachability helper `armBaseHeightAtLiftZero` still assumes the default 0.8 m column and 0.02 m
arm mount, so for such a profile its value is not the arm base height (unfixed).

The commanded joint is driven on hardware by the `lift_hardware` package ([lift.md](lift.md)).

**Recipe: chain / column lift.** A chain-driven or rigid-chain column lift behaves, to the planner,
like any single-axis vertical lift: one prismatic joint, `lower 0`, `upper = stroke`. Use
`lift:=telescopic lift_stages:=0` (a column with one carriage, no visible stages), set `lift_stroke`,
`lift_max_velocity` and `lift_max_acceleration` from the datasheet, and put the rated payload in
`lift.payload_kg`. For a column whose geometry matters (a chain housing, a guide rail), copy
`lift/telescopic_lift.xacro`, replace the visuals / collision boxes, keep the four contract names, and
add a branch to `lift/lift.xacro`; mimic joints are only needed for parts that visibly follow the
carriage. Leave the stroke honest: chain lifts often have a mechanical limit above the usable stroke.

## Lift modelling

One controlled joint carries the arm over the full stroke; the visible telescoping stages follow
it through `<mimic>` joints (each `stroke / stages` relative to the stage below). This keeps a
single commanded lift DOF for planning and control, avoids re-implementing a 50/50 split in
code, and is the same modelling pattern used by production telescopic lifts. Defaults: 1.5 m
stroke, two stages, 0.2 m/s (check these against your actual lift and set them in the profile).

Note: mimic joints must appear in `/joint_states` (or be resolved by the planner), otherwise
collision geometry on the stage links sits at the wrong height. Publish them with a small
relay, or let the planner resolve mimics.

## Use

```bash
xacro urdf/mobile_manipulator.urdf.xacro arm:=ur10e -o robot.urdf
xacro urdf/mobile_manipulator.urdf.xacro arm:=panda -o robot.urdf   # 7-DOF arm
ros2 launch mobile_manipulator_description display.launch.py arm:=generic_6dof
python3 -m pytest src/description/mobile_manipulator_description/test
```

## Limits and next steps

- Collision geometry is primitives; generate a collision-sphere model per robot (FOAM, see
  [vamp_notes.md](vamp_notes.md)) for the VAMP / cuRobo backends and the
  [self filter](robot_self_filter.md).
- The SRDF carries the generated structural allowed-collision matrix (`scripts/generate_acm.py`);
  geometry-specific pairs can be added by sampling, see [moveit_config.md](moveit_config.md).
- `hardware:=real` selects a vendor ros2_control driver from the arm (`config/hardware_drivers.yaml`, not vendored; `hardware_bringup.md` section 0a).
- `hardware:=mock` adds a ros2_control block (mock components); add real and sim backends as
  further branches of that one xacro.
- The 7-DOF arms use cylinders along the real segments, so their collision geometry is an
  approximation; check it before relying on tight clearances.
- Verified here with `check_urdf` and the contract tests for the generic arm, the Panda-like arm and UR10e (rendered
  with the Humble xacro). Loaded into MoveIt and planned with on Jazzy, see [moveit_config.md](moveit_config.md).

## Home-pose check

`test/test_home_pose.py` renders every profile and checks that its `home` pose lists exactly the
actuated joints, lies inside the URDF joint limits, and does not self-collide in a capsule model
built from the URDF collision shapes (arm and tool links as inscribed capsules, the lift column as
oriented boxes; links up to two moving joints apart are not checked). It runs on the host (xacro,
urdf_parser_py, numpy) and skips UR profiles without `ur_description`. It is the same question the
`check` stage of [robot_assets.md](robot_assets.md#self-collision-sanity) asks of the FOAM spheres,
answered earlier and without Docker; it is a planning-grade check on primitives, not a mesh check.

## Joint movement cost (`joint_costs`)

An optional array in the profile, one entry per joint in `joints` order (lift first, then the arm), each finite and above 0.
Planners and the dock IK choice compare paths by each joint's travel as a fraction of its range (1 / range), multiplied by its
entry: a higher entry makes that joint costlier to move, so plans prefer the others. `[2.0, 1, 1, 1, 1, 1, 1]` makes the lift
twice as costly as an arm joint. Without the key the values are lift 2.0 and arm joints 1.0 (no lift: all 1.0).

It is read in one place and handed to every consumer: the manager passes it to the dock IK choice and, as the plugin param
`joint_costs`, to the free-space planners (`moveit_ompl`); the cuRobo node takes `profile:=<name>` or `joint_costs`. It only
ranks finished candidates and guides shortcutting, so it acts where those modes are on (`free_space_optimize`,
`dock_ik_continuity`); the planners' own internal costs (OMPL path length, cuRobo trajectory optimisation, VAMP) are not weighted.
A size that does not match `joints` is an error when the profile is loaded.
