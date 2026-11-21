#include "coverage_planner/core/wall_pose_lift.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>


#include "coverage_planner/core/wall_basis.hpp"

namespace coverage_planner
{


namespace
{

Eigen::Vector3d toEigen(const Point & point)
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}

Quaternion toQuaternionMsg(const Eigen::Quaterniond & quaternion)
{
  Quaternion msg;
  msg.x = quaternion.x();
  msg.y = quaternion.y();
  msg.z = quaternion.z();
  msg.w = quaternion.w();
  return msg;
}

/// Which wall (u, v) boundary a pose is nearest to, and how far from it (meters; clamped >= 0).
/// Distances/coordinates here are computed against the WALL's own basis_u/basis_v directly (not
/// the tool frame's x/y tangents below) -- those two only coincide in direction up to a sign that
/// depends on the wall's own u/v/normal handedness, and u_min/u_max/v_min/v_max are defined in the
/// wall's own basis, so mixing the two would silently swap which physical edge gets tilted toward.
enum class NearestEdge { kUMin, kUMax, kVMin, kVMax };

struct EdgeProximity
{
  NearestEdge edge;
  double distance_m;
};

EdgeProximity findNearestEdge(double u, double v, const WallPlane & wall)
{
  const double du_min = std::max(0.0, u - wall.u_min);
  const double du_max = std::max(0.0, wall.u_max - u);
  const double dv_min = std::max(0.0, v - wall.v_min);
  const double dv_max = std::max(0.0, wall.v_max - v);

  EdgeProximity nearest{NearestEdge::kUMin, du_min};
  if (du_max < nearest.distance_m) {
    nearest = {NearestEdge::kUMax, du_max};
  }
  if (dv_min < nearest.distance_m) {
    nearest = {NearestEdge::kVMin, dv_min};
  }
  if (dv_max < nearest.distance_m) {
    nearest = {NearestEdge::kVMax, dv_max};
  }
  return nearest;
}

/// Rotates z_nominal toward `target_direction` (a wall-in-plane unit vector, e.g. -basis_v_unit
/// to extend reach past the bottom edge) by `angle_rad`. Deriving the rotation axis directly from
/// z_nominal and the actual target -- rather than a hand-picked per-edge axis/sign table -- keeps
/// this correct regardless of whether the wall's own (basis_u, basis_v, normal_outward) happens to
/// be a right- or left-handed set relative to the tool frame built below.
Eigen::Vector3d rotateTowardDirection(
  const Eigen::Vector3d & z_nominal, const Eigen::Vector3d & target_direction, double angle_rad)
{
  const Eigen::Vector3d target_in_plane =
    (target_direction - z_nominal * z_nominal.dot(target_direction)).normalized();
  const Eigen::Vector3d axis = z_nominal.cross(target_in_plane).normalized();
  const Eigen::AngleAxisd rotation(angle_rad, axis);
  return (rotation * z_nominal).normalized();
}

constexpr double kMaxAxisTiltDeg = 85.0;

}  // namespace

Path liftCoveragePathPose(
  const Path & flat_path, const WallPlane & wall,
  const coverage_planner::ToolTask & tool_task)
{
  const WallBasis wall_basis = decomposeWallBasis(wall);
  const Eigen::Vector3d wall_origin = wall_basis.origin;
  const Eigen::Vector3d basis_u_unit = wall_basis.basis_u.normalized();
  const Eigen::Vector3d basis_v_unit = wall_basis.basis_v.normalized();
  const Eigen::Vector3d normal = wall_basis.normal_outward.normalized();
  const Eigen::Vector3d z_nominal = -normal;

  // Gram-Schmidt: orthogonalize basis_u against the normal so a slightly-imperfect plane fit
  // still yields an exactly-orthonormal tool frame.
  Eigen::Vector3d x_tangent = (basis_u_unit - z_nominal * z_nominal.dot(basis_u_unit)).normalized();
  Eigen::Vector3d y_tangent = z_nominal.cross(x_tangent).normalized();
  // Re-orthogonalize x against y, z for a numerically clean right-handed frame (x*y=z).
  x_tangent = y_tangent.cross(z_nominal).normalized();

  // Intended fixed tool tilt (profile geometry): rotate the nominal frame about the in-surface
  // axis (given in the contact frame, so it follows tool_frame_yaw_deg). Skipped entirely at 0
  // so the poses stay bit-identical to the untilted lift. It is applied before the edge tilt.
  const bool fixed_tilt = tool_task.tool_tilt_deg != 0.0;
  Eigen::Vector3d z_start = z_nominal;
  Eigen::Vector3d x_start = x_tangent;
  if (fixed_tilt) {
    const double yaw_rad = tool_task.tool_frame_yaw_deg * M_PI / 180.0;
    const double ax = tool_task.tool_tilt_axis[0];
    const double ay = tool_task.tool_tilt_axis[1];
    Eigen::Vector3d axis = (ax * x_tangent + ay * y_tangent);
    if (axis.norm() > 1e-12) {
      axis = (Eigen::AngleAxisd(yaw_rad, z_nominal) * axis).normalized();
      // Defensive bound; the loader already refuses a total (fixed + edge) tilt of 85 degrees.
      const double tilt_rad =
        std::clamp(tool_task.tool_tilt_deg, -kMaxAxisTiltDeg, kMaxAxisTiltDeg) * M_PI / 180.0;
      const Eigen::AngleAxisd tilt(tilt_rad, axis);
      z_start = (tilt * z_nominal).normalized();
      x_start = (tilt * x_tangent).normalized();
    }
  }

  const bool tilt_enabled = tool_task.tool_normal_policy == "tilt_at_edges" && !tool_task.contact &&
                            tool_task.max_edge_tilt_deg > 0.0 && tool_task.standoff_m > 0.0;
  const double margin_m = tool_task.standoff_m;
  const double max_tilt_rad = tool_task.max_edge_tilt_deg * M_PI / 180.0;

  Path lifted;
  lifted.header = flat_path.header;
  lifted.poses.reserve(flat_path.poses.size());

  for (const auto & flat_pose : flat_path.poses) {
    const Eigen::Vector3d position = toEigen(flat_pose.pose.position);
    const Eigen::Vector3d relative = position - wall_origin;
    // u, v computed in the WALL's own basis (matches wall.u_min/u_max/v_min/v_max exactly) --
    // deliberately NOT x_tangent/y_tangent, see findNearestEdge's doc comment above.
    const double u = relative.dot(basis_u_unit);
    const double v = relative.dot(basis_v_unit);

    Eigen::Vector3d z_axis = z_start;
    if (tilt_enabled) {
      const EdgeProximity nearest = findNearestEdge(u, v, wall);
      if (nearest.distance_m < margin_m) {
        const double fraction = 1.0 - (nearest.distance_m / margin_m);
        Eigen::Vector3d target_direction;
        switch (nearest.edge) {
          case NearestEdge::kUMin:
            target_direction = -basis_u_unit;
            break;
          case NearestEdge::kUMax:
            target_direction = basis_u_unit;
            break;
          case NearestEdge::kVMin:
            target_direction = -basis_v_unit;
            break;
          case NearestEdge::kVMax:
            target_direction = basis_v_unit;
            break;
        }
        z_axis = rotateTowardDirection(z_start, target_direction, fraction * max_tilt_rad);
      }
    }
    // Never let the fixed plus edge tilt turn the axis more than kMaxAxisTiltDeg from into-wall.
    const double cos_limit = std::cos(kMaxAxisTiltDeg * M_PI / 180.0);
    if (z_axis.dot(z_nominal) < cos_limit) {
      const Eigen::Vector3d side = z_axis - z_nominal * z_axis.dot(z_nominal);
      const Eigen::Vector3d dir = side.norm() > 1e-12 ? Eigen::Vector3d(side.normalized()) :
        x_tangent;
      z_axis = (z_nominal * cos_limit + dir * std::sqrt(1.0 - cos_limit * cos_limit)).normalized();
    }

    Eigen::Vector3d x_final = (x_start - z_axis * z_axis.dot(x_start)).normalized();
    Eigen::Vector3d y_final = z_axis.cross(x_final).normalized();

    // Rotate the commanded tool frame's own X/Y about its Z (contact-normal) axis -- e.g. to
    // swap which physical axis of an asymmetric tool tracks "up the wall" vs. "across it".
    // Applied to x_final/y_final (not x_tangent/y_tangent, computed once above): z_axis can vary
    // per-pose when tilt_enabled, so this must use THIS pose's own z_axis to stay orthonormal.
    if (tool_task.tool_frame_yaw_deg != 0.0) {
      const double yaw_rad = tool_task.tool_frame_yaw_deg * M_PI / 180.0;
      x_final = (Eigen::AngleAxisd(yaw_rad, z_axis) * x_final).normalized();
      y_final = z_axis.cross(x_final).normalized();
    }

    Eigen::Matrix3d rotation;
    rotation.col(0) = x_final;
    rotation.col(1) = y_final;
    rotation.col(2) = z_axis;
    Eigen::Quaterniond quaternion(rotation);
    quaternion.normalize();

    // The standoff is measured along the tool axis, so the axis meets the strip point at that
    // distance. Without a fixed tilt z_start == -normal and this is the plain normal offset.
    const Eigen::Vector3d offset_position = position - z_start * tool_task.standoff_m;

    PoseStamped lifted_pose;
    lifted_pose.header = flat_pose.header;
    lifted_pose.pose.position.x = offset_position.x();
    lifted_pose.pose.position.y = offset_position.y();
    lifted_pose.pose.position.z = offset_position.z();
    lifted_pose.pose.orientation = toQuaternionMsg(quaternion);
    lifted.poses.push_back(lifted_pose);
  }

  return lifted;
}

}  // namespace coverage_planner
