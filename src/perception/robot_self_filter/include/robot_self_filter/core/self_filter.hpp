// Removes the robot's own body from a point cloud. ROS-free: the caller supplies the link poses
// (from joint states + TF at the cloud's time stamp), the filter does geometry only.
//
// Design and rules: docs/robot_self_filter.md.
#pragma once

#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "robot_self_filter/core/robot_model.hpp"
#include "scene_model/core/types.hpp"

namespace robot_self_filter
{

/// Padding added to a shape (m). Box: per axis. Cylinder: radius and half... full height.
struct Padding
{
  double sphere_radius{0.02};
  Vec3 box{0.02, 0.02, 0.02};
  double cylinder_radius{0.02};
  double cylinder_height{0.02};   ///< Added to the full height (so half of it on each end).
};

struct SelfFilterConfig
{
  Padding padding;                                  ///< Default for every link.
  std::map<std::string, Padding> link_padding;      ///< Per-link override (thin clips vs big housings).
  std::set<std::string> links;                      ///< Which links count as "self"; empty = all with shapes.

  /// Drop points closer than this to the sensor origin (its own mount, cables). Needs the origin.
  double min_sensor_distance{0.0};

  /// Also drop points hidden behind a robot link as seen from the sensor origin. Off by default:
  /// it deletes real surfaces behind the robot (it removed ceiling points above the robot in an
  /// earlier stack). Needs the origin.
  bool shadow{false};

  /// Extra padding in metres per (m/s) of link speed: the arm moves between the pose the cloud was
  /// taken at and the pose that is known. Needs link speeds in the input.
  double speed_padding_gain{0.0};

  /// While the tool touches a surface, a padded tool shape would delete the surface next to it.
  /// In contact mode this link uses `contact_padding` (default none) instead of its normal padding.
  std::string contact_link;
  double contact_padding{0.0};
};

struct SelfFilterInput
{
  const scene_model::PointCloud * cloud{nullptr};
  Pose cloud_to_root{Pose::Identity()};        ///< Cloud frame -> robot root link frame.
  LinkPoses link_poses;                        ///< Root frame, at the cloud's time stamp.
  std::map<std::string, double> link_speed;    ///< Optional.
  bool contact_mode{false};
  std::optional<Vec3> sensor_origin;           ///< In the root frame; needed for shadow / min distance.
};

struct SelfFilterResult
{
  bool success{false};
  std::string message;
  scene_model::PointCloud filtered;            ///< Points kept; labels and colors stay aligned.
  std::vector<bool> removed;                   ///< One entry per input point.
  size_t removed_count{0};
  std::map<std::string, size_t> removed_per_link;  ///< "<min_distance>" for the distance rule.
};

class SelfFilter
{
public:
  SelfFilter(std::vector<LinkShapes> shapes, SelfFilterConfig config);

  const SelfFilterConfig & config() const {return config_;}

  /// Fails (success = false, with a reason) when a configured link has no pose, the cloud is
  /// missing, or shadow / min-distance rules are on without a sensor origin.
  SelfFilterResult filter(const SelfFilterInput & input) const;

private:
  std::vector<LinkShapes> shapes_;
  SelfFilterConfig config_;
};

/// Geometry helpers, exposed for tests and reuse.
bool pointInShape(const Shape & s, const Pose & world_from_link, const Vec3 & p_root,
  double pad_a, const Vec3 & pad_box, double pad_h);

/// Does the segment a -> b (root frame) touch the (inflated) shape?
bool segmentTouchesShape(const Shape & s, const Pose & world_from_link, const Vec3 & a, const Vec3 & b,
  double pad_a, const Vec3 & pad_box, double pad_h);

}  // namespace robot_self_filter
