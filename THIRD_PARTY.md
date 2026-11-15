# Third-party software

This repository is Apache-2.0 ([LICENSE](LICENSE)). It contains **no vendored third-party source or
binaries**: everything below is fetched at build time (apt, pip, a pinned git commit) or is a dependency
declared in a `package.xml`. The only data files checked in are small test fixtures we wrote (`.pcd`
clouds, one cube `.stl`). Robot meshes are not shipped: the UR10e meshes come from the `ur_description`
package installed by apt, other bases and tools are described with primitives.

Licences below were read from each project's own `LICENSE` / `COPYING` file or the GitHub licence field
on 2026-11-15 unless marked **unverified**. A licence name is not legal advice; check the exact version you
ship. Where a project bundles parts under other licences it is noted.

## Used or built against today

| Software | Licence | How it is used | Pinned / from |
|---|---|---|---|
| [VAMP](https://github.com/KavrakiLab/vamp) | Apache-2.0 | SIMD collision-checking planner; `vamp_planner` compiles against it | Docker `vamp-build` stage, commit `27cb9b6`, installed to `/opt/vamp` |
| VAMP's fetched dependencies: `nigh` (BSD-3-Clause), `pdqsort` (Zlib), `SIMDxorshift` (Apache-2.0), Eigen 3.4 (MPL-2.0), **`cddlib` (GPL-2.0)** | as listed | VAMP's CMake fetches them; the Docker stage copies their headers next to VAMP's into `/opt/vamp/include` because VAMP's headers include them (`docs/vamp_notes.md`) | with VAMP. cddlib's GPL-2.0 is the one copyleft item in the planning path: review it before distributing a binary or image that includes `vamp_planner`. Checked 2026-11-15 in VAMP's `cmake/Dependencies.cmake`: cddlib is built as a static library and used only to convert convex polytopes (`polytope_convert.hh`), so it is linked into anything built from VAMP's headers. Resolution: this repository ships source and recipes only, never a built image or binary; users build VAMP themselves, and anyone who redistributes such a build must satisfy cddlib's GPL-2.0. Our planning path does not use polytope obstacles |
| [FOAM](https://github.com/CoMMALab/foam) | MIT | URDF to collision spheres (offline robot asset generation) | `docker/tools/foam.Dockerfile`, commit `116928f71aaa`, separate image |
| [Cricket](https://github.com/CoMMALab/cricket) | MIT | Spheres to a VAMP robot header (offline) | `docker/tools/build_robot_tools.sh`, commit `a2e664f97b6a`, separate image |
| [cuRobo](https://github.com/NVlabs/curobo) v2 | Apache-2.0; bundled example assets carry their own licences | GPU planner, IK and ESDF behind `curobo_bridge` | `docker/requirements-gpu.txt`, commit `78fd485`, pip from git. Which example assets the pip install includes was not checked (**unverified**) |
| PyTorch (CUDA 12.8 wheels) | BSD-3-Clause style (its `LICENSE`; GitHub reports no SPDX id) | cuRobo's tensor backend | `requirements-gpu.txt`, `torch==2.8.0` |
| NVIDIA CUDA runtime libraries (pip wheels pulled by torch and `jax[cuda12]`) | NVIDIA's own licence, text not read (**unverified**) | GPU execution | pip |
| [MuJoCo](https://github.com/google-deepmind/mujoco) 4.0.1 | Apache-2.0 | Physics for `sim_mujoco` (C SDK, mounted tarball, `MUJOCO_ROOT`); Python `mujoco-core` for tools and cross-check | SDK from the release page; `requirements-sim.txt` |
| MuJoCo MJX, [JAX](https://github.com/jax-ml/jax) | Apache-2.0 (MJX is part of the MuJoCo repository) | Batched cross-check of kinematics and collisions | `requirements-sim.txt` |
| [MoveIt 2](https://github.com/moveit/moveit2) | BSD-3-Clause | Planning scene, OMPL, Cartesian planning, kinematics plugins, `moveit_configs_utils` | apt `ros-jazzy-moveit` |
| [OMPL](https://github.com/ompl/ompl), [FCL](https://github.com/flexible-collision-library/fcl) | BSD-3-Clause; BSD | Free-space planning and collision checking through MoveIt | apt |
| [TRAC-IK](https://github.com/traclabs/trac_ik), [pick_ik](https://github.com/PickNikRobotics/pick_ik) | BSD-3-Clause | IK solver plugins for MoveIt (TRAC-IK is the default; pick_ik is opt-in) | apt, `package.xml` of the MoveIt config |
| KDL ([orocos_kinematics_dynamics](https://github.com/orocos/orocos_kinematics_dynamics)) | LGPL-2.1 | IK and kinematics through MoveIt's KDL plugin and `kinematics_interface_kdl`; used as a shared library | apt |
| [ros2_control](https://github.com/ros-controls/ros2_control), [ros2_controllers](https://github.com/ros-controls/ros2_controllers) | Apache-2.0 | Controller manager, hardware interfaces, joint trajectory and admittance controllers; JTAC builds on this stack | apt |
| [Gazebo Harmonic](https://github.com/gazebosim/gz-sim), [ros_gz](https://github.com/gazebosim/ros_gz), [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control) | Apache-2.0 | Headless physics backend `sim_gazebo`; `ft_bridge_hardware` derives from a `gz_ros2_control` base class when it is present | apt, Docker `sim-gazebo` target |
| [ur_description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) | BSD-3-Clause; the meshes of newer models (UR15, UR18, UR20, UR30 and others) are under the vendor's "Terms and Conditions for use of Graphical Documentation", which is not OSI open source. The UR10e meshes are not in that list, per the upstream README read on the `rolling` branch | UR10e arm model for the starter configuration | apt `ros-jazzy-ur-description` (the distro package's own licence text was not read: **unverified**) |
| [Eigen](https://gitlab.com/libeigen/eigen) | MPL-2.0 | Linear algebra in most C++ packages | apt |
| [yaml-cpp](https://github.com/jbeder/yaml-cpp) | MIT | Scene and profile files (`scene_model`, `mobile_manipulator_profile`) | apt |
| [PCL](https://github.com/PointCloudLibrary/pcl) | BSD (its `LICENSE.txt`) | `.pcd` reading in `coverage_planner`; message conversion | apt |
| [nlohmann/json](https://github.com/nlohmann/json) | MIT | `wall_scene.json` and plan output | apt |
| [tinyxml2](https://github.com/leethomason/tinyxml2) | Zlib | URDF parsing in `robot_self_filter` | apt |
| [OctoMap](https://github.com/OctoMap/octomap) | BSD (the `octomap` library) | Obstacle octomaps for MoveIt, `scene_adapters` | apt; `docker/build.sh` pins its CMake dir to avoid a version clash |
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio) | BSD-2-Clause | Forward kinematics in `reachability` | apt |
| trimesh (MIT), SciPy (BSD-3-Clause), NumPy (BSD-3-Clause per upstream; file not fully read) | as listed | Mesh handling and numerics in `curobo_bridge` and the Docker tools | pip |
| ROS 2 Jazzy core (`rclcpp`, `tf2`, `robot_state_publisher`, `xacro`, `rviz2`, `urdf`, ...) | Apache-2.0 or BSD-3-Clause per package; not checked package by package (**unverified**) | Middleware and tooling for every package | `ros:jazzy-ros-base` image, apt |

## Installed in the image but not used by any package

`BehaviorTree.CPP` (MIT, `docker/apt-extra.txt`), `libccd` (listed in `apt-extra.txt`; used as a MoveIt/FCL
dependency, licence not read: **unverified**), `joint_state_publisher_gui`.

## Considered or planned, not used today ([ROADMAP.md](docs/ROADMAP.md) section 17)

| Software | Licence | Intended use |
|---|---|---|
| [Nav2](https://github.com/ros-planning/navigation2) | Apache-2.0 overall; `nav2_amcl` is LGPL-2.1 | Navigation to base stands, docking |
| [Noether](https://github.com/ros-industrial/noether) | Apache-2.0 | Mesh coverage |
| MuJoCo Warp | Apache-2.0 | Parallel gain sweeps |
| [mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control) | Apache-2.0 | Possible base-motion and sensor plugins; it pins MuJoCo 3.12 |

Licences for these four are as verified on 2026-11-15 and recorded in the roadmap.

## Notices

There is no `NOTICE` file because nothing is redistributed in source or binary form by this repository. A
Docker image built from it contains the software above under their licences; if you publish an image, bundle
the notices of what it contains (the cddlib and KDL entries need particular attention).
