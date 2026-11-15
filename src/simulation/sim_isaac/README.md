# sim_isaac (stub)

Placeholder for an NVIDIA Isaac Sim backend. **Nothing here simulates anything yet.**
`ros2 launch sim_isaac isaac.launch.py` exits with a message saying so.

The design and the contract every simulation backend has to meet are in
[docs/simulation.md](../../../docs/simulation.md). Checklist for turning this into a backend:

- [ ] Decide the Isaac Sim version and check the licence terms for the deployment.
- [ ] URDF to USD import from a robot profile (collision as convex hulls).
- [ ] ros2_control hardware interface (topic-based, via the Isaac Sim ROS 2 bridge) and a
      `hardware:=isaac` branch in `mobile_manipulator.ros2_control.xacro`.
- [ ] `/clock`, joint states and commands with the profile's joint names and order.
- [ ] Tool wrench at `tcp_link` with the sign convention checked against a wall push.
- [ ] Wall world generated from `wall_scene.json` / `SceneModel`.
- [ ] Base `cmd_vel` and `odom`.
- [ ] Headless run as a test in a container; runs as its own compose service (Isaac Sim ships as its
      own image).
