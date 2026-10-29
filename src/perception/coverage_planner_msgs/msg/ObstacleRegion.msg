# A single detected obstacle's 2-D footprint, already expressed in its parent wall's own (u, v)
# frame (matches wall_scene.json's "uv_on_wall" field directly -- no 3-D projection needed here).
# Local to coverage_planner -- not interfaces/ObstacleRegion.

uint32 id
uint32 parent_wall
float64 u_min
float64 u_max
float64 v_min
float64 v_max
