# Segment metadata attached to a generated coverage path.
# Indexes are inclusive and reference nav_msgs/Path.poses.
# Local to coverage_planner -- not interfaces/MotionPlanSegment -- so this package has no
# dependency on any sibling * package. Force-control fields (force_stage/target_force_n) are
# deliberately not carried here -- that's a motion-execution concern, out of scope for coverage
# path generation.

string label
uint32 start_index
uint32 end_index
# Equal to start_index/end_index for every segment except a connector, which widens these two to
# also cover the neighboring strips' own joint poses -- so per-segment line rendering draws the
# joint edges. Never use these for copying/extracting a segment's own poses; use start_index/
# end_index for that, or the joint poses get duplicated into the copy.
uint32 render_start_index
uint32 render_end_index
string frame_id
string mode
string planner
bool requires_contact
bool requires_retraction
float32 priority
string[] tags
