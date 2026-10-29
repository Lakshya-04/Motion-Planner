# Documentation

| Document | What it covers |
|---|---|
| [architecture.md](architecture.md) | The modular pipeline, every port, selection by config, surface and policy slots |
| [package_layout.md](package_layout.md) | `core/` + `ros/` split per module, how modules connect (pluginlib in-process, ROS interfaces across processes) |
| [design_principles.md](design_principles.md) | Rules and lessons learned (frames, the slow lift axis, timing, safety) |
| [curobo_v2_notes.md](curobo_v2_notes.md) | cuRobo v2 API, scene/perception inputs, how it maps onto our ports |
| [vamp_notes.md](vamp_notes.md) | VAMP: what it does, upstream state, robot generation, our wrapper layout |
| [learned_recovery.md](learned_recovery.md) | Research on recovery to a known state: RL, imitation learning, MPC, safety filters; recommended staged architecture |
| [adr/](adr/) | Decision records (modular pipeline, upstream backends, core/ros split, C++ standard) |
