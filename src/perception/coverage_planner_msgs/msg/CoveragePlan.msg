# A whole wall's coverage plan, published once wall_coverage_service_node finishes a run started
# through ~/start. Wire-format twin of GenerateCoveragePath.srv's own response, plus the
# expandCoveragePasses() lifting wall_coverage_demo_node also applies before its own output.

bool success
string message

# Final, lifted/expanded coverage path (world-frame, oriented, standoff-offset).
nav_msgs/Path path
coverage_planner_msgs/MotionPlanSegment[] segments

float32 total_distance   # Total path length in meters
float32 coverage_area    # Estimated coverage area in m^2
int32 num_waypoints      # Number of waypoints in path
float32 planning_time    # Time taken to generate the flat path, in seconds
string pattern_used      # Actual pattern used
