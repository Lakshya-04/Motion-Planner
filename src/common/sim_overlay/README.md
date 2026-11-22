# sim_overlay

Draws the coverage plan columns and stand poses (the `mp_rviz` topics `/mp_viz/plan_columns` and `/mp_viz/stands`) inside Gazebo and MuJoCo. Full description, commands and limits: [docs/sim_overlay.md](../../../docs/sim_overlay.md).

- `sim_overlay/overlay.py`: marker dicts to sim-neutral primitives (pure python), fixed frame to world transform.
- `scripts/gz_overlay.py`: Gazebo via the gz-sim `/marker` service (`gz service`), `--dry-run` prints the commands.
- `sim_overlay/mujoco_overlay.py`, `scripts/overlay_dump.py`, `patches/mujoco_viewer_hook.patch`: MuJoCo `user_scn` adapter, JSON feed, viewer hook (not applied).

Honest limits: host-tested only (`python3 -m pytest src/common/sim_overlay`); never run in a simulator. Gazebo needs the GUI; labels do not face the camera; MuJoCo is capped by `user_scn.maxgeom`.
