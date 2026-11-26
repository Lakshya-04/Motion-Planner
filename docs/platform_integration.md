# Platform integration: vendor SDKs and description packages

Research and a decision, no code. The question: our bases (Ranger Mini 2.0, Scout, Scout Mini, Husky, Jackal,
Dingo-O, MiR100, MPO-500, TurtleBot3, TIAGo-style, Fetch-style) are generic xacro with nominal numbers
([robot_description.md](robot_description.md)). Should we use the vendors' own packages, and as git submodules?

Everything here was read from GitHub (repository metadata and files through the API) and the ROS Jazzy
`rosdistro` file on 2026-11-26. Nothing was cloned, built or run, and nothing was tried on a robot. Where a
number comes from vendor source it is the number in the commit named below, not a datasheet.

## Decision in short

- **No git submodules for vendor code.** [ADR 0002](adr/0002-third-party-backends.md) and
  [THIRD_PARTY.md](../THIRD_PARTY.md) say third-party code is fetched at a pinned commit and never copied in. A
  submodule is not a copy, so it would not break that, but it buys nothing here (see "Submodules" below).
  Submodules stay what [development_plan.md](development_plan.md) section 12 says: a way to split our own packages.
- **Default: apt, where the distro already ships the vendor packages** (Clearpath description, config, messages
  and simulator; TurtleBot3). **Otherwise a pinned vcstool file**, [`docker/platforms.repos`](../docker/platforms.repos),
  fetched on demand into a workspace, not into this repository. The pins there are exact commits.
- **Vendor drivers are never linked into our packages.** Anything that talks to a base goes through ROS
  interfaces (cmd_vel in, odom and state out) in a thin adapter package that we write. Same rule as
  [hardware_drivers.yaml](../src/description/mobile_manipulator_description/config/hardware_drivers.yaml) for arms.
- **Everything vendor-side is optional.** The generic xacro bases stay the default and the CI path. A vendor
  description would be switched on by an argument, with the primitive model as fallback.
- **Four platforms have nothing usable on Jazzy today** (MiR100, MPO-500 licence, TIAGo, Fetch). They are
  left out of the pin file and the reasons are below.

## Per platform

Licence column: the GitHub licence field, then what the repository's own LICENSE file and the package manifest (package.xml)
say. "Jazzy" means a branch or a rosdistro entry for Jazzy, not "it builds": nothing was built.

### AgileX Ranger Mini 2.0

| Item | Finding |
|---|---|
| Packages | [agilexrobotics/ranger_ros2](https://github.com/agilexrobotics/ranger_ros2) (base node, messages, bringup), on top of [agilexrobotics/ugv_sdk](https://github.com/agilexrobotics/ugv_sdk) (CAN protocol, C++) |
| Licence | ranger_ros2: BSD-3-Clause (LICENSE file read, matches GitHub). ugv_sdk: Apache-2.0 LICENSE file, but its package.xml says BSD: **conflict, ask upstream**; we would rely on the LICENSE file |
| Jazzy | ranger_ros2 has a jazzy branch, last commit 2024-10-21 (default branch is humble, pushed 2026-11-23). ugv_sdk has only main (2026-08-11) and no tags. Neither is in the Jazzy rosdistro. Jazzy gap: the jazzy branch is 25 months behind humble |
| Description and meshes | **None.** ranger_ros2 has no description package. Meshes exist only in [agilexrobotics/ugv_gazebo_sim](https://github.com/agilexrobotics/ugv_gazebo_sim) (ROS 1 catkin, Gazebo classic, package.xml says BSD, no LICENSE file and no GitHub licence): **unlicensed as published, not usable, not pinned** |
| Driver | ranger_base node, CAN through ugv_sdk (gs_usb adapter, scripts in the bringup package) |
| Drive modes | The ROS node picks the mode from the Twist: sideways velocity set means parallel (crab); otherwise a turn radius below the minimum turn radius means spinning, else dual Ackermann. Side-slip is chosen only for the V1 model with zero forward speed. The node then calls the SDK's mode setter, which sends a CAN command. There is **no service, topic or parameter to ask for a mode**: a planner cannot pick one except by shaping its Twist |
| cmd_vel and limits | Subscribes /cmd_vel. Mini V2 constants in ranger_params.hpp: linear 1.5 m/s, angular 4.8 rad/s (spin), steering clamp 0.4782 rad (Ackermann) and 1.570 rad (parallel), minimum turn radius 0.4764 m, track 0.364 m, wheelbase 0.494 m. Those two geometry numbers equal our ranger_mini xacro defaults |
| Sensors | The repositories ship no sensor driver or sensor list. Not documented in what was read: **unverified** |
| Use here | Driver for hardware:=real and the mode logic, nothing for the model |

**Decision:** consume ugv_sdk and ranger_ros2 from the pin file. Keep our own primitive xacro (there is nothing
to replace it with). Write a small adapter that turns a base command into a Twist that lands in the wanted
mode (spin: zero linear; crab: sideways component; Ackermann: radius above 0.4764 m) and checks the mode the
robot reports in its motion state. Hardware use waits on the licence answer for ugv_sdk.

### AgileX Scout and Scout Mini

| Item | Finding |
|---|---|
| Packages | [agilexrobotics/scout_ros2](https://github.com/agilexrobotics/scout_ros2): scout_base, scout_description, scout_msgs; uses ugv_sdk (above) |
| Licence | Apache-2.0 LICENSE file and GitHub field; package.xml of the packages says BSD: **same conflict as ugv_sdk** |
| Jazzy | No jazzy branch by that name on main/humble. There is a branch literally named origin/jazzy (commit 48f70cb, 2024-10-21): it passes parameter defaults to declare_parameter, which main (2025-04-25) does not. Not in the rosdistro |
| Description and meshes | scout_description: ament_cmake with no dependencies, so it is separable. It holds one model, scout_v2, with COLLADA meshes (base, wheels, a Hokuyo). **No Scout Mini model** in it. Mesh licence is not stated apart from the package licence |
| Driver | scout_base node, CAN through ugv_sdk; handles Scout, Scout Mini and Scout Mini Omni, chosen by the is_scout_mini and is_omni_wheel parameters |
| Drive modes | Skid-steer for Scout and Scout Mini: one mode. The omni variant adds a sideways velocity: the node passes the Twist's sideways part to the SDK only for that variant |
| cmd_vel and limits | /cmd_vel straight to the SDK command (linear x and angular z, plus y for omni). The ROS node applies no clamp that I could find; limits live in the SDK or the firmware (not read) |
| Sensors | None in the repository besides the Hokuyo mesh. Not documented: **unverified** |
| Use here | Driver for hardware:=real; meshes for sim fidelity of the full-size Scout only |

**Decision:** consume the `origin/jazzy` commit through the pin file, driver only. Do not use the description
for anything we ship: it does not cover Scout Mini and the licence is contradictory. Our primitive models stay.

### Clearpath Husky (A200), Jackal (J100), Dingo-O (DO100)

Clearpath is the only vendor with a maintained Jazzy line, a binary release and a description that is
generated from one robot config.

| Item | Finding |
|---|---|
| Packages | [clearpath_common](https://github.com/clearpathrobotics/clearpath_common) (platform, sensor, mount and manipulator descriptions, control config), [clearpath_config](https://github.com/clearpathrobotics/clearpath_config) (the robot config schema, Python), [clearpath_msgs](https://github.com/clearpathrobotics/clearpath_msgs), [clearpath_robot](https://github.com/clearpathrobotics/clearpath_robot) (hardware interfaces, motor drivers, on-robot bringup), [clearpath_ros2_socketcan_interface](https://github.com/clearpathrobotics/clearpath_ros2_socketcan_interface), [clearpath_simulator](https://github.com/clearpathrobotics/clearpath_simulator) (Gazebo) |
| Licence | BSD-3-Clause in every one (GitHub field and LICENSE file head read). Mesh files are not licensed separately in what was read: not checked file by file |
| Jazzy | Yes: jazzy branches, all pushed within the last month except socketcan (2025-06-12). Rosdistro Jazzy has releases (apt) of clearpath_common 2.9.19, clearpath_config 2.9.8, clearpath_msgs 2.7.0, clearpath_simulator 2.9.6, clearpath_desktop 2.9.1. **clearpath_robot is not in the Jazzy rosdistro**: source, or Clearpath's own robot image (not checked) |
| Description and meshes | clearpath_platform_description: ament_cmake, runtime deps only robot_state_publisher, urdf and xacro, so it is separable. Platform folders: a200 (Husky), j100 (Jackal), do100 and do150 (Dingo-O), dd100 and dd150 (Dingo-D), a300, r100, w200. **Catch:** the platform xacro's default controller-file argument uses $(find clearpath_control), so clearpath_control has to be installed too. The intended path is to generate the whole URDF from a robot config with clearpath_generator_common |
| Driver | clearpath_hardware_interfaces plugin (named in the Dingo xacro as the real hardware plugin), CAN through the socketcan interface and the motor drivers. Not used by us: hardware stays on Clearpath's stack |
| Drive modes | None to select. Husky and Jackal: differential drive controller. Dingo-O: mecanum drive controller (omni) |
| cmd_vel and limits | Controller config in clearpath_control. Husky: linear 1.0 m/s, angular 1.0 rad/s. Jackal: 2.0 and 4.0. Dingo-O: x and y 1.3 m/s, angular 4.0 rad/s, acceleration limits in the same files. /cmd_vel goes through a twist_mux first |
| Sensors | Documented as xacro and config in clearpath_sensors_description: Hokuyo UST, SICK LMS1xx, Velodyne, Ouster OS1, Hesai, Seyond, Intel RealSense, Luxonis OAK-D, StereoLabs ZED, Axis and FLIR cameras, several IMU and GNSS units |
| Use here | Description and meshes for sim fidelity; the sensor catalogue to drive our sensor-suite profiles; Gazebo launch as a cross-check of our own sim_gazebo; driver only if someone runs Clearpath's stack on the robot |

**Decision:** apt for common, config, msgs, simulator (they are released for Jazzy, so the distro already
pins them); the pin file carries the same repositories plus clearpath_robot and socketcan for the hardware side,
for people who want an exact commit. Use the platform description and the sensor catalogue through an optional
switch, with our generic models as fallback. This is the one case where "use the vendor's own description" is
fully realistic. Our wrapper has to pull in clearpath_control for the xacro defaults, or pass the controller
argument.

### MiR100

| Item | Finding |
|---|---|
| Packages | [DFKI-NI/mir_robot](https://github.com/DFKI-NI/mir_robot): community driver (a reverse ROS bridge to the robot's own ROS), description, Gazebo, navigation. DFKI states it is not affiliated with MiR. Supported: MiR100, MiR200, MiR500, MiR software 2.8.3.1 only |
| Licence | BSD-3-Clause (GitHub field; LICENSE file reads "Copyright (c) 2018, DFKI GmbH" with the usual BSD text) |
| Jazzy | **No.** The Jazzy rosdistro lists mir_robot with branch rolling, but rolling and humble are the same commit (7d9c729, 2021-06-11) and its packages are catkin (ROS 1). The active branch is noetic (2026-11-17). A third-party ROS 2 port exists, [niemsoen/mir_robot](https://github.com/niemsoen/mir_robot) (branch humble-devel, commit c302eae, 2024-03-26, BSD-3-Clause, ament_cmake, adds a REST API package); no Jazzy branch and not tested here |
| Other | [open-rmf/fleet_adapter_mir](https://github.com/open-rmf/fleet_adapter_mir), Apache-2.0, talks to the MiR through its REST API (fleet adapter, not a velocity driver) |
| Drive modes | Differential drive: one mode |
| Sensors | Not verified: I did not fetch MiR's own documentation, and the description package was not read for sensors |
| Use here | Nothing today |

**Decision:** nothing in the pin file. A ROS 1 bridge is no basis for Jazzy, and a one-person fork on Humble is
not something to pin for a product. If MiR100 becomes a real target, write the adapter against MiR's REST
API (missions and positions, not raw velocity, since MiR's controller owns motion and safety) after reading MiR's
own API documentation, which I have not seen. The niemsoen description could then be tried as a model source.

### Neobotix MPO-500

| Item | Finding |
|---|---|
| Packages | [neobotix/neo_mpo_500-2](https://github.com/neobotix/neo_mpo_500-2) (robot model, configs, launch), [neo_common2](https://github.com/neobotix/neo_common2), [neo_relayboard_v2-2](https://github.com/neobotix/neo_relayboard_v2-2) (serial driver for the relay board), [neo_kinematics_mecanum2](https://github.com/neobotix/neo_kinematics_mecanum2), [neo_nav2_bringup](https://github.com/neobotix/neo_nav2_bringup) |
| Licence | **Unclear.** GitHub reports none for neo_mpo_500-2 and the repository has no LICENSE file. The package.xml files say "BSD" with no version or text. Only neo_nav2_bringup (Apache-2.0) has a licence field. Treat as unlicensed until Neobotix answers. Elsewhere in the organisation there is a **GPL-3.0** repository (neo_waypoint_follower) and an **AGPL-3.0** one (neo_pick_2_place): keep them out |
| Jazzy | Yes: jazzy branches on all of the above. neo_mpo_500-2 jazzy: commit c87c582, 2026-03-04. Most are in the Jazzy rosdistro only as a few message and nav packages, not the robot packages |
| Description and meshes | In neo_mpo_500-2 itself: xacro bodies, omni wheel macro, SICK S300 and microScan3 macros, COLLADA meshes. Not separable cleanly: the same package carries bringup and has exec dependencies on the UR driver, phidgets and neo_nav2_bringup |
| Driver | Relay board over serial (a port parameter, device path under /dev) plus the mecanum kinematics node |
| Drive modes | Omni (mecanum): one mode, x, y and yaw |
| cmd_vel and limits | Navigation config: forward 0.6 m/s, reverse 0.2, sideways 0.5, forward acceleration 0.25. Kinematics config: wheel diameter 0.254 m, length 0.5 m, width 0.548 m (wheel centres). Teleop scale 0.6 |
| Sensors | SICK S300 and microScan3 laser scanners (xacro and meshes), a RealSense launch file, a Phidgets IMU config |
| Use here | Would be: meshes and the mecanum kinematics. Blocked by the licence |

**Decision:** not in the pin file, only a commented pin to keep the commit on record. Ask Neobotix (the README
gives a ROS contact address) for a licence statement before anyone fetches it. We may read its published numbers
(wheel size, speeds) and note them in [THIRD_PARTY.md](../THIRD_PARTY.md) as "read for published parameters",
as the arm macros already do.

### ROBOTIS TurtleBot3 Burger and Waffle

| Item | Finding |
|---|---|
| Packages | [ROBOTIS-GIT/turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) (description, node, bringup, navigation2, cartographer), [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs), [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) (Gazebo) |
| Licence | Apache-2.0 (LICENSE files read; description package.xml says Apache 2.0). Mesh licences not checked separately |
| Jazzy | Yes: jazzy branches (turtlebot3 2026-06-15, msgs 2025-04-30, simulations 2025-06-27) and Jazzy releases in the rosdistro |
| Description and meshes | turtlebot3_description: depends only on urdf and xacro, so it is separable. Models: burger, waffle, waffle_pi. Sensor meshes: LDS, RealSense R200, Astra |
| Driver | turtlebot3_node over the OpenCR board (Dynamixel protocol 2.0). Burger: wheel separation 0.160 m, wheel radius 0.033 m |
| Drive modes | Differential: one mode |
| cmd_vel and limits | cmd_vel subscriber in the node. The hardware limits were not read. Burger navigation config uses a 0.3 m/s forward cap for the planner |
| Sensors | LDS-01, LDS-02 or LDS-03 chosen by an environment variable in the bringup launch; camera on waffle_pi; R200 and Astra meshes in the description |
| Use here | Description and meshes; Gazebo as a cross-check; driver for a real Burger or Waffle |

**Decision:** apt (`ros-jazzy-turtlebot3*` through rosdep), pin file for people who want the commit. This is
the second realistic case. TurtleBot3 cannot carry a manipulator plus lift as we model it, so the real use is
sim fidelity and the navigation configs, not a product path.

### PAL TIAGo

| Item | Finding |
|---|---|
| Packages | [pal-robotics/tiago_robot](https://github.com/pal-robotics/tiago_robot) and about a dozen sibling repositories; base is PMB2: [pal-robotics/pmb2_robot](https://github.com/pal-robotics/pmb2_robot) |
| Licence | Apache-2.0 (tiago_robot LICENSE file read; description package.xml says Apache License 2.0) |
| Jazzy | **No.** Default branch humble-devel (2026-11-18); other branches are foxy, galactic and the older alum-devel (2025-09-16). No Jazzy branch, not in the Jazzy rosdistro. I did not establish which PAL release name matches Jazzy |
| Description | tiago_description needs pmb2_description, omni_base_description, the PAL gripper and hand descriptions, launch_pal and pal_urdf_utils: not separable |
| Use here | Nothing |

**Decision:** omit. Our TIAGo-style profile is a torso lift on a generic base, not a TIAGo, and a Humble-only
stack with the PAL tool chain behind it is a poor fit. Revisit when PAL publishes a Jazzy branch.

### Fetch

[fetchrobotics/fetch_ros](https://github.com/fetchrobotics/fetch_ros) (the ZebraDevs address redirects to it): ROS 1 only (branches `indigo-devel`, `melodic-devel`, `ros1`), last push 2024-08-20, GitHub reports no licence and
no LICENSE file was found by the licence API. A web search found no ROS 2 or Jazzy port. **Omit.** Keep the
fetch-style base as a generic model and call it that.

## How to consume them

| Option | Where it fits | Where it does not |
|---|---|---|
| (a) Git submodules under a third_party path | Defensible for the three AgileX repositories: no apt release, one pinned commit each, shows in git submodule status | See "Submodules" below. Wrong for everything with an apt release, and for anything with an unresolved licence |
| (b) vcstool pin file, fetched on demand | AgileX trio, clearpath_robot, and the exact-commit path for the apt-released ones. Same idea as the VAMP pin | One file fetches everything in it; split per vendor if that gets annoying |
| (c) apt and rosdep | Clearpath common, config, msgs, simulator; TurtleBot3. The distro maintains the pin | Not available for AgileX, Neobotix robot packages, MiR on Jazzy |
| (d) Thin adapter in this repository | The only place base-driver code of ours lives: cmd_vel and odom in and out, mode shaping for the Ranger, no vendor headers | Does not replace (a) to (c): it needs the vendor driver installed |

### Submodules: where the idea is right, and where it is not

Right: our own packages ([development_plan.md](development_plan.md) section 12), and any vendor repository we
had to patch. ADR 0002 rules out patching, so the second case is closed.

Not right for vendors:

- A plain clone would not fetch them, which is good, but `git clone --recursive`, IDE "init submodules" buttons and
  CI checkout flags would, and the repository would then show third-party trees in its working copy and in
  GitHub's tree view. Our stance is "no third-party source in this repository"; a submodule is a pointer, but
  the tool chain treats it as part of the tree.
- GitHub source archives and release zips leave submodules out, so anything that builds from an archive breaks
  silently.
- Four of the packages we would pin have unresolved or conflicting licences. A submodule pointer makes them
  look like part of the project.
- We already have the vcstool package in the image and the pattern of an exact pin outside the tree (VAMP). One
  mechanism is cheaper than two.
- The one thing submodules do better: the pin shows up in `git diff` and review of the tree itself. The pin file
  is a normal text file in review too, so this is a small loss.

### Maintenance and what stays optional

- **Pin bumps:** one commit per vendor when we choose to move; look at the changelog first. Clearpath moves
  weekly (jazzy branch pushes in November), so pin to a release tag unless there is a reason not to. Tags
  exist on clearpath_common (2.9.19 is the newest seen).
- **Jazzy gaps:** AgileX has one stale Jazzy branch (2024-10) and no tags on ugv_sdk, so a bump there is rare and
  manual; PAL, MiR, Fetch have no Jazzy line; Neobotix has Jazzy branches but no licence.
- **Optional:** every vendor package. CI keeps building and testing against the generic bases. A vendor
  description is selected by an argument and falls back to the primitive model; nothing in this repository
  imports a vendor package at build time. The switch and the adapter are not written yet.
- **Coordinate frames:** vendors use their own names (TurtleBot3 uses a footprint frame, Clearpath a base link).
  The adapter and the description switch need a frame mapping to our profile frames.

## Licence red flags

| Item | Problem | Rule |
|---|---|---|
| ugv_sdk, scout_ros2 | LICENSE file is Apache-2.0, package.xml says BSD | Ask upstream. Both are permissive, so installing is fine; do not copy in |
| neo_mpo_500-2 and the other Neobotix robot packages | No LICENSE file, GitHub shows none, package.xml says BSD only | Unclear: do not fetch or use until Neobotix confirms |
| Neobotix neo_waypoint_follower (GPL-3.0), neo_pick_2_place (AGPL-3.0) | Copyleft | Never vendored or linked into our Apache-2.0 packages; only installed as a separate dependency if ever needed |
| ugv_gazebo_sim | No LICENSE file, no GitHub licence, package.xml says BSD | Unlicensed as published; not pinned |
| fetch_ros | No licence found | Not used |
| Meshes of any vendor | A package licence does not always cover its meshes (the UR meshes are the example already in THIRD_PARTY.md) | Meshes are fetched, referenced, never checked in; read the mesh terms before shipping an image |
| DFKI and niemsoen mir_robot | BSD-3-Clause, fine, but the code is not Jazzy | Not used |

No GPL code is in the pin file. BSD-3-Clause and Apache-2.0 are compatible with Apache-2.0 as installed
dependencies. If an image containing these is published, its notices have to include theirs.

## What I could not verify

- Whether anything builds or runs on Jazzy: no build was attempted.
- Real sensor suites for AgileX bases and the MiR100 (no vendor page fetched), and TurtleBot3 hardware speed limits.
- Mesh licences for any vendor, and whether Clearpath's apt packages equal the pinned commits.
- MiR's own REST API documentation, and the MiR100 sensor list.
- How Clearpath ships `clearpath_robot` on its robot image.
- Whether ugv_sdk builds as an ament package: its CMake has a catkin branch; the ranger and scout READMEs build
  it in a colcon workspace.
- The meaning of PAL's `alum` and `erbium` branch names with respect to ROS distributions.
