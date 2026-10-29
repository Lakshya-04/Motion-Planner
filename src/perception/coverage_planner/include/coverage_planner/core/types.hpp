// Plain C++ data types for the coverage planner core. No ROS dependency.
//
// Field names deliberately mirror the ROS message types (header.frame_id, info.width,
// pose.pose.position, ...) so the ros/ layer can convert with simple field copies and the
// algorithms read the same as they did against messages. Time stamps are not modelled: the
// core is a pure function of its inputs, and the ROS layer stamps outputs itself.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace coverage_planner
{

struct Header
{
  std::string frame_id;
};

struct Point
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Point32
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct Pose
{
  Point position;
  Quaternion orientation;
};

struct PoseStamped
{
  Header header;
  Pose pose;
};

struct Path
{
  Header header;
  std::vector<PoseStamped> poses;
};

struct Polygon
{
  std::vector<Point32> points;
};

struct PolygonStamped
{
  Header header;
  Polygon polygon;
};

struct MapMetaData
{
  double resolution{0.0};  ///< Cell edge length (m).
  uint32_t width{0};       ///< Cells along x.
  uint32_t height{0};      ///< Cells along y.
  Pose origin;             ///< Pose of cell (0, 0) in the grid frame.
};

/// Occupancy values: -1 unknown, 0 free, 100 occupied (row-major, y * width + x).
struct OccupancyGrid
{
  Header header;
  MapMetaData info;
  std::vector<int8_t> data;
};

/// A coverage segment: an inclusive index range into Path::poses plus execution intent.
struct MotionPlanSegment
{
  std::string label;
  uint32_t start_index{0};
  uint32_t end_index{0};
  uint32_t render_start_index{0};
  uint32_t render_end_index{0};
  std::string frame_id;
  std::string mode;
  std::string planner;
  bool requires_contact{false};
  bool requires_retraction{false};
  float priority{0.0F};
  std::vector<std::string> tags;
};

/// A fitted wall plane. `pose` has x = u axis, y = v axis, z = outward normal.
struct WallPlane
{
  uint32_t id{0};
  PoseStamped pose;
  double u_min{0.0};
  double u_max{0.0};
  double v_min{0.0};
  double v_max{0.0};
  double plane_rms_mm{0.0};
  double spread_p95_mm{0.0};
  double spread_p99_mm{0.0};
};

struct ObstacleRegion
{
  uint32_t id{0};
  uint32_t parent_wall{0};
  double u_min{0.0};
  double u_max{0.0};
  double v_min{0.0};
  double v_max{0.0};
};

}  // namespace coverage_planner
