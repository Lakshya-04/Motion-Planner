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

## Coverage paint: live done / not done map

Paints the wall like a cleaning pass: red = not done, green = done, orange = failed or partial column, plus a percent-covered number. Core: `sim_overlay/coverage_paint.py` (ROS-free, numpy only). A 2 cm grid on the wall plane from the wall json (`wall_scene.json` or a recorder's `run.csv.wall.json`: origin, u, v, extents). A cell is DONE when the tool swath (default 0.14 m, the sim scripts' `MULTI_STAND_SWATH`) passed over it while in contact: force >= `force_threshold_n` (1 N) AND the tcp within `plane_tol_m` (3 cm) of the wall plane. The swath is a round-nosed band around the segment between two consecutive in-contact samples (a flat tool has square ends, so about half a swath beyond a stroke end is over-credited). FAILED cells are the not-yet-done cells of a column the manager reports failed or partial.

Failed columns need the column geometry: `plan_file` (`coverage_output.json`); ASSUMPTION: manager column k = the k-th non-connector segment of the file. The ExecuteCoverage feedback carries only counts, so `coverage_paint.py` infers per column: when `column` moves on, the column is failed if `columns_failed` rose, else done (`FeedbackTracker`). Exact lists from the action result can be published as JSON on `~/outcome` (`{"done": [], "partial": [3], "failed": [5]}`). Only contact marks DONE, a column the manager calls done is not painted by itself.

The node publishes `/mp_viz/coverage_paint` (MarkerArray: one TRIANGLE_LIST per status from merged rectangles, wall outline, percent text) and `/mp_viz/coverage_percent`. Gazebo and MuJoCo reuse the existing nodes: `OverlayState` turns TRIANGLE_LIST into a filled `tiles` primitive (`gz_commands`: one TRIANGLE_LIST gz marker, both windings; `mujoco_overlay`: thin box geoms), so nothing else was duplicated.

```bash
# RViz: add a MarkerArray display on /mp_viz/coverage_paint (fixed frame = the wall json frame_id, or frame_id:=)
ros2 launch sim_overlay coverage_paint.launch.py wall_scene_json:=/path/wall_scene.json source:=mujoco   # or gazebo
# Gazebo (GUI on): the same launch draws it in the gz GUI
ros2 launch sim_overlay coverage_paint.launch.py wall_scene_json:=... source:=gazebo feed:=gazebo plan_file:=coverage_output.json
# MuJoCo: writes /tmp/mp_sim_overlay.json, the viewer hook (patches/mujoco_viewer_hook.patch, not applied) polls it
ros2 launch sim_overlay coverage_paint.launch.py wall_scene_json:=... source:=mujoco feed:=mujoco
ros2 service call /coverage_paint/save_png std_srvs/srv/Trigger   # PNG of the live map (png_path)
```

Force inputs: `source:=mujoco` reads `/sim_mujoco/ft_ground_truth`, `source:=gazebo` `/ft_sensor/ground_truth_wrench` (both WrenchStamped, `|Fz|`, like `record_run.py`); `force_msg:=info` reads `/ft_sensor/ground_truth_info` (`max_point_norm`). TCP pose: TF `tcp_frame` (tcp_link) in `frame_id`, which must be the frame of the wall json. Parameters: `cell_m`, `swath_m`, `force_threshold_n` (<= 0 ignores force, for a spray pass), `plane_tol_m`, `sample_hz`, `publish_hz`, `lift_m`.

### Replay from a recorded run (report / README picture)

```bash
python3 src/common/sim_overlay/scripts/coverage_replay.py run.csv [--wall run.csv.wall.json] [--out map.png] \
    [--plan coverage_output.json --failed 5 --partial 3] [--overlay-json feed.json] [--require-active]
```

Reads `t,x,y,z,force[,active]` and the wall json (default `run.csv.wall.json`), writes a matplotlib PNG (default `run.coverage.png`) and prints a JSON line with `percent_done`, areas and sample counts. `--overlay-json` writes the primitives for the MuJoCo feed / `gz_overlay.py --from-json`. `docs/media/coverage_paint_example.png` is made from SYNTHETIC strokes (not a simulator run).

Not run: the node under ROS, RViz, Gazebo or MuJoCo; host pytest covers the core, replay CLI, marker to primitive path, gz command text and the MuJoCo box adapter (fake `mujoco`). The Gazebo `/marker` service limits above apply (GUI only, one `gz service` per marker; this adds about 8 calls per refresh).

## Limits

- Not run in any simulator; appearance, line widths and text size are unchecked.
- Gazebo: needs the GUI; the service name `/marker` is the documented default and may differ per gz version or world namespace (`service` parameter). One `gz service` process per marker, so a plan of hundreds of segments takes a while to draw.
- MuJoCo: the overlay is capped by `user_scn.maxgeom` (default 1000); a box costs 15 capsules, so large plans drop later primitives (reported by `draw`).
- Both are static snapshots of the last marker state; no animation, no robot-relative frames (no TF lookup).
