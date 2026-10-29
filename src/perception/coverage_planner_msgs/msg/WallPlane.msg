# A single detected wall plane: origin + in-plane orthonormal basis (u, v) + outward normal,
# plus that wall's own planar extent in (u, v). Mirrors wall_scene.json's per-wall schema.
# Local to coverage_planner -- not interfaces/WallPlane -- so this package has no
# dependency on any sibling * package.
#
# pose.header.frame_id is the frame this wall's geometry is expressed in (no separate frame_id
# field -- PoseStamped already carries one). pose.pose.position is the wall's own origin;
# pose.pose.orientation is the quaternion whose rotation matrix columns are (basis_u, basis_v,
# normal_outward), in that order -- see wall_basis.hpp's encodeWallBasis()/decomposeWallBasis().

uint32 id
geometry_msgs/PoseStamped pose
float64 u_min
float64 u_max
float64 v_min
float64 v_max

# Plane-fit quality, carried through for the wall-scene loader's flatness gate -- a wall whose
# spread exceeds a configurable threshold should be refused rather than silently coverage-planned
# as if it were flat.
float64 plane_rms_mm
float64 spread_p95_mm
float64 spread_p99_mm
