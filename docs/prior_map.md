# Prior map input (`wall_detector/core/prior_map`)

Status: host-tested only (ROS-free g++/gtest, `prior_map_test`, 10 tests). Not run on the robot, in a
simulator or on a real building model. Not wired into any node or launch file: the entry points are the
C++ API and a small command line tool.

An external wall or room model (a mesh, or a hand-written list of planar patches) becomes the same
`wall_scene.json` the coverage planner already reads (`loadWallScene`), so a wall can be planned before it
has been scanned, and a scan can be checked against, and completed by, the model.

```
.obj / .ply mesh ----priorMapFromMesh---->  PriorMap  ----priorMapToWallSceneJson---->  wall_scene.json
prior_surfaces .json --parsePriorSurfacesJson-->  (planes, openings, boxes)                (walls + obstacles)
PriorSurface + measured PlaneFit --mergeWithScan--> MergeResult --mergedWallSceneJson--> wall_scene.json
```

Code: `src/perception/wall_detector/{include,src}/.../prior_map.*`, tool `prior_map_to_wall_scene`
(`src/perception/wall_detector/src/tools/prior_map_to_wall_scene.cpp`), fixtures `src/perception/wall_detector/test/data/{room.obj,window_wall.json}`. It lives in
`wall_detector` because that package already owns the `wall_scene.json` writer; the OBJ reader is
`scene_model::loadObj` (reused, not copied). The PLY reader there is point-cloud only, so a small ASCII PLY
mesh reader (vertices and face lists) is added here.

## Conventions

| Thing | Convention | Basis |
|---|---|---|
| Units, frames | metres; every coordinate in the one named `frame_id` of the file (default `map`). `transformPriorMap` re-expresses it in another frame given the transform | [REP-103](https://www.ros.org/reps/rep-0103.html) (SI units, right-handed), [REP-105](https://www.ros.org/reps/rep-0105.html) (`map` / `odom` / `base_link`); resolving the transform (TF) is the caller's job |
| Plane frame | `normal` points to the robot side, `u = up x normal` (horizontal along a wall), `v = normal x u` (up the wall), `u x v = normal`; identical to `scene_model::PlaneFit` and `wall_scene.json` (`basis_u`, `basis_v`, `normal_outward`) | this repository |
| Polygons | vertex lists, planar, no repeated closing vertex; the winding gives the normal by the right-hand rule when no `normal` is given | the OBJ and PLY face conventions (counter-clockwise seen from the front) |
| Openings | holes in a surface (door, window, vent) with their own polygon in the same plane; the model of a wall with openings (a wall, its voids) follows [IFC](https://standards.buildingsmart.org/IFC/RELEASE/IFC4/ADD2_TC1/HTML/) (`IfcWall` with `IfcOpeningElement` voids filled by `IfcDoor` / `IfcWindow`), reduced to a polygon | buildingSMART IFC4 |
| Coverage planner side | an opening becomes a flush obstacle region (`height 0`, bounding box in the wall's `u, v`; `kind` and `name` are kept), a box in front of the wall an obstacle region with its protrusion | `wall_scene.json` `obstacles[]`, `uv_on_wall` |
| Mesh formats | Wavefront OBJ (`v` / `f`, polygons fan-triangulated), ASCII PLY with a vertex list; no assumption on units or up axis: use `mesh_scale` (e.g. 0.001 for mm) and `transformPriorMap` (a Y-up export needs a 90 degree turn) | OBJ spec (Alias Wavefront), PLY (Turk, Stanford) |

## The `prior_surfaces` JSON

```json
{
  "format": "prior_surfaces", "version": 1, "frame_id": "map", "units": "meters",
  "surfaces": [
    {"id": "north_wall",
     "polygon": [[0, 3, 0], [4, 3, 0], [4, 3, 2.5], [0, 3, 2.5]],
     "normal": [0, -1, 0],
     "openings": [{"kind": "window", "polygon": [[1, 3, 1], [2, 3, 1], [2, 3, 2], [1, 3, 2]]}]}
  ],
  "obstacles": [{"id": "cabinet", "center": [0.5, 2.8, 0.9], "size": [0.6, 0.4, 1.8], "yaw_deg": 0}]
}
```

* `surfaces[].polygon`: at least 3 vertices, planar within `coplanar_tol_m` (1 cm), non-zero area.
* `normal` (optional): must agree with the polygon's plane within `normal_tol_deg` (it only chooses the sign);
  without it the winding decides. With `robot_position` set, every normal is turned to face it and each flip is
  recorded in `PriorMap::warnings` (and in the output's `warnings`).
* `openings[]`: polygon in the surface's plane and inside its extent (within the tolerance), else rejected.
* `obstacles[]`: boxes (`size` full extents, `yaw_deg` about +z). They go to the walls they stand within 0.6 m
  of (or 5 cm behind) as footprints clipped to the wall.
* Anything malformed throws `std::runtime_error("prior map: ...")` with the reason (also: duplicate ids,
  non-numeric or non-finite numbers, a mesh with no usable plane, a face index out of range, binary PLY, an
  unknown extension).

## Mesh to planes

Triangles are grouped by plane (normals within `normal_tol_deg` of each other, either sign, plane offsets within
`plane_offset_tol_m`), then split into connected pieces (shared welded vertices, `weld_tol_m`) so two separate
walls in one plane stay two surfaces. Each piece becomes a surface whose polygon is the **bounding rectangle**
of the piece in its `(u, v)`. Pieces under `min_area_m2` and horizontal pieces (floors, ceilings, unless
`include_horizontal`) are dropped with a warning.

## Merge with a scan

`mergeWithScan(prior, measured)` compares a prior surface with a measured `PlaneFit` (what `detectWall`
returns):

* same plane = normals within `max_angle_deg` (5) and plane-to-plane distance within `max_offset_m` (5 cm), and
  the normals do not point opposite ways;
* same plane: the **measured pose is kept unchanged**, the prior's openings are carried over (re-expressed in the
  measured plane's `(u, v)`), and `mergedWallSceneJson` writes them as obstacle regions next to the measured wall;
* otherwise nothing is carried over;
* either way `offset_m` (signed, along the prior normal), `angle_deg`, `normal_flipped` and a message are
  returned, and `prior_merge` in the JSON repeats them. The prior never moves the measurement and a
  disagreement is reported, never absorbed.

## Not supported

* **IFC (STEP) parsing**, `IfcOpenShell`, gbXML, Revit / CAD files. Export the walls to OBJ / PLY, or write the
  JSON. A real BIM import should reuse IfcOpenShell (LGPL) to get plane geometry and openings and feed this
  JSON, not re-implement IFC.
* Curved surfaces, vaulted ceilings: only planar polygons; a mesh is reduced to planes within the tolerances.
* Non-rectangular wall outlines from a mesh (the rectangle is used) and openings recovered from a mesh (give them
  in the JSON). A non-rectangular opening becomes its bounding box in the coverage planner, which is
  conservative (the box is avoided).
* Semantic layers (materials, wall type, fire rating, storey), levels, units other than metres, binary PLY,
  textures, STL as a prior.
* Merging more than one scan, loop closure or registering the prior to the robot's map: the prior is assumed to
  be in the same frame as the scan (give the transform to `transformPriorMap`). A wrong frame is a wrong wall;
  `mergeWithScan` will say "not the same plane" rather than shift anything.
* Moving obstacle boxes by anything but a yaw and a translation (`transformPriorMap` refuses a tilting transform
  when boxes are present).

## Tool

```
prior_map_to_wall_scene IN.{obj,ply,json} OUT.json [--frame F] [--scale S] [--robot X Y Z]
```

Writes `wall_scene.json` (one wall per surface, id = position in the list, `name` = the surface id) and prints
the warnings. Checked on the host: the output of both fixtures is read by `loadWallScene` (the window wall gives
3 obstacle regions: window, door, cabinet).

## Tests

`bash docker/tools/run_host_core_tests.sh -k prior_map` (10 tests): box room OBJ to 4 walls with the right
extents and orientation, outward-facing mesh flipped and reported, PLY in mm, disconnected coplanar walls, a wall
with window and door and a box to `wall_scene.json`, window position, merge with 2 cm and 1 degree, disagreement
(20 cm, 8 degrees, flipped normal) reported without moving the measurement, malformed input rejected, frame
change.
