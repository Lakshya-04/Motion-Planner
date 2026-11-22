# Simulator overlay (`sim_overlay`)

Shows the coverage plan (the wall columns) and the stand / navigation target poses inside the simulators, not only in RViz.

**Status: host-tested only. Nothing was run in Gazebo or MuJoCo.** The unit tests cover the geometry, the `gz` command text (dry run) and the MuJoCo geom adapter (against the `mujoco` python package where it is installed, else fakes). Whether the markers actually appear is untested.

## What it shows

Input is the topic contract of `mp_rviz` (`mp_viz_scene`): `/mp_viz/plan_columns` (one LINE_STRIP per column, connectors grey) and `/mp_viz/stands` (per stand an ARROW, a footprint LINE_STRIP and a text label). `sim_overlay` subscribes to those `MarkerArray` topics and does not recompute the plan or the stands, so `mp_viz_scene` (or any publisher of the same markers) must be running.

Markers become sim-neutral primitives (polyline, arrow, box outline, sphere, label) with colours, moved into the simulator world by a fixed transform `frame_x frame_y frame_z frame_yaw` (default identity: the marker frame is the sim world). Set it if the plan frame is not the sim world frame.

## Gazebo (Harmonic, gz-sim 8)

`scripts/gz_overlay.py` calls the `/marker` service (`gz.msgs.Marker`, reply `gz.msgs.Boolean`) through the `gz service` command. Each marker is a `LINE_STRIP` (arrows are a shaft plus two head strokes, boxes a 16-point strip) or `TEXT`; ids and namespaces `mp_overlay/<marker ns>` make a re-send replace the old marker (`ADD_MODIFY`), and `DELETE_ALL` per namespace is the clear. No world file is edited. The `/marker` service is served by the gz **GUI** (MarkerManager), so run Gazebo with `gui:=true`; a headless `-s` server draws nothing. `TEXT` has no billboard mode in this message, so labels do not turn to the camera.

```bash
ros2 run sim_overlay gz_overlay.py                       # live, follows the topics
ros2 run sim_overlay gz_overlay.py --ros-args -p dry_run:=true   # print the gz commands only
python3 src/common/sim_overlay/scripts/gz_overlay.py --from-json /tmp/mp_sim_overlay.json --dry-run   # no ROS
python3 src/common/sim_overlay/scripts/gz_overlay.py --from-json /tmp/mp_sim_overlay.json --clear      # remove them
```

## MuJoCo

`sim_overlay/mujoco_overlay.py` has `draw(user_scn, primitives)` (capsule connectors via `mjv_connector`, spheres, labels as geom labels; respects `user_scn.maxgeom`, returns the number of primitives dropped) and `load_overlay(path)`. The viewer (`sim_mujoco/scripts/viewer.py`) is a separate non-ROS-feed process, so `scripts/overlay_dump.py` writes the primitives as JSON (atomic rename) and the viewer polls the file's mtime.

```bash
ros2 run sim_overlay overlay_dump.py --ros-args -p path:=/tmp/mp_sim_overlay.json
# one-time hook, not applied to the repo:
git apply src/common/sim_overlay/patches/mujoco_viewer_hook.patch
SIM_MUJOCO_OVERLAY_JSON=/tmp/mp_sim_overlay.json python3 <your usual viewer.py command>
```

The patch adds an env variable `SIM_MUJOCO_OVERLAY_JSON` (off when unset), and in the viewer loop, before `viewer.sync()`, redraws `viewer.user_scn` when that file's mtime changed. It imports `sim_overlay`, so the workspace must be sourced. The marker frame to MuJoCo world transform is set on `overlay_dump.py` (same `frame_*` parameters).

## Limits

- Not run in any simulator; appearance, line widths and text size are unchecked.
- Gazebo: needs the GUI; the service name `/marker` is the documented default and may differ per gz version or world namespace (`service` parameter). One `gz service` process per marker, so a plan of hundreds of segments takes a while to draw.
- MuJoCo: the overlay is capped by `user_scn.maxgeom` (default 1000); a box costs 15 capsules, so large plans drop later primitives (reported by `draw`).
- Both are static snapshots of the last marker state; no animation, no robot-relative frames (no TF lookup).
