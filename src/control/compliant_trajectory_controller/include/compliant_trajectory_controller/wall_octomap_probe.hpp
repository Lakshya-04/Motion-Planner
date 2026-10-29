/**
 * @file wall_octomap_probe.hpp
 * @brief Reusable ground-truth query against the LIVE pushed wall octomap: exact-point occupancy
 *        (used by jtac_wall_approach_test_node.cpp for post-hoc verification) and signed
 *        penetration depth along a direction (used by wall_contact_force_simulator_node.cpp to
 *        turn real geometry into a mock contact wrench).
 * @details Extracted from jtac_wall_approach_test_node.cpp's own WallOctomapContactMonitor once a
 *          second consumer needed the identical octomap-fetch + FK setup (this package's own
 *          "3+ real call sites" bar for pulling a reusable piece out of a single test file) --
 *          not a new mechanism, the same GetPlanningScene(WORLD_OBJECT_GEOMETRY|OCTOMAP) fetch and
 *          kinematics_interface_kdl FK this file's own git history already proved live.
 *
 *          Deliberately owns its OWN kinematics_interface_kdl instance rather than sharing
 *          JtacController's/AdmittanceRule's -- same duplication-for-independence reasoning
 *          already accepted elsewhere in this package (e.g. redundancy_kinematics_): this probe
 *          must be able to answer "is the real robot touching the real wall" without any
 *          dependency on whichever compliance backend is under test, including a mock one that
 *          might itself be wrong.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <octomap/octomap.h>

#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace compliant_trajectory_controller
{

class WallOctomapProbe
{
public:
  explicit WallOctomapProbe(rclcpp::Node::SharedPtr node);

  /// Fetches robot_description, the live pushed octomap, and initializes FK against them. Returns
  /// false (logged) on any failure -- a caller should treat that as "no real ground truth
  /// available," not silently skip the check.
  bool initialize(const std::string & kinematics_base, const std::string & kinematics_tip);

  /// Diagnostic detail behind a single contact check -- lets a caller report EXACTLY how solid a
  /// "contact" claim is (the raw FK point itself genuinely inside occupied space, vs. only a
  /// neighbor within the tolerance margin).
  struct ContactCheckResult
  {
    bool contact{false};
    bool exact_point_occupied{false};  ///< True only if margin_offset_m == 0.0 AND contact.
    double margin_offset_m{0.0};
  };

  /// True if the tip's own FK position at this joint configuration falls inside, or within one
  /// octomap voxel of, a real occupied voxel of the LIVE pushed octomap. See checkContact()'s own
  /// implementation comment for why dx=0 is always tried first.
  ContactCheckResult checkContact(const std::vector<double> & joint_positions) const;

  /// Signed penetration depth (meters) of the CONTACT SURFACE POINT -- the tip's own FK point,
  /// shifted `contact_surface_offset_m` further along `direction_in_kinematics_base` (a unit
  /// vector, e.g. the tip's own local Z axis expressed in kinematics_base -- the tool's approach
  /// normal) -- along that same direction. contact_surface_offset_m exists in case the bare TCP
  /// FRAME ORIGIN is not exactly where a real tool physically ends -- for mop's own tool
  /// (cam_mop_ft.stl, tool_profiles.yaml's mesh_origin_rpy=[3.14,-1.57,0]), it is NOT: transforming
  /// the mesh's own vertices through that rotation into the flange frame gives a contact-normal
  /// (Z) extent of [-0.004, +0.162]m, and tcp_xyz.z=0.165 places link_tcp 3mm PAST the mesh's own
  /// farthest point -- i.e. link_tcp already sits at (very slightly past) the tool's real physical
  /// end, not 5cm short of it. An earlier, WRONG 0.05 value here (claimed to be bisected via
  /// /check_state_validity) credited the simulated tool with reach it does not have, and was
  /// caught only by a human watching RViz notice the tool never visibly touched the wall even
  /// when this class reported contact. Default 0.0 matches the mesh; don't reintroduce a positive
  /// offset without re-deriving it from the actual mesh geometry, not a live bisection (a
  /// bisection against the wrong wall standoff, or along an off-axis direction, can look
  /// plausible while being wrong -- this is what happened the first time).
  ///
  /// 0.0 if the (offset) contact point is NOT inside occupied space (still in free air --
  /// callers should treat this as "no contact force," not "small negative force"). If it IS
  /// occupied, marches BACKWARD along -direction (away from the wall) in `step_m` increments,
  /// up to `max_search_m`, until reaching a free sample -- the distance traveled is the
  /// penetration depth. Returns 0.0 (with a logged warning) if no free sample is found within
  /// max_search_m -- a genuine "how did we get this deep" anomaly, not silently clamped.
  // step_m default 0.0002 (0.2mm) -- found live that 0.002 (2mm) was far coarser than the actual
  // physical depths involved (a K=5000N/m mock wall settles around ~1mm at a few newtons): the
  // backward search's FIRST candidate at d=step_m was already free almost every tick, so every
  // "in contact" reading quantized to a fixed force = stiffness*step_m regardless of true depth --
  // a bang-bang 0N/~10N oscillation, never a smooth spring, and it never let the admittance loop
  // converge onto any target force in between.
  double computeSignedPenetrationM(
    const std::vector<double> & joint_positions, const Eigen::Vector3d & direction_in_kinematics_base,
    double contact_surface_offset_m = 0.0, double step_m = 0.0002, double max_search_m = 0.03) const;

  /// FK convenience -- both consumers need the tip's own transform (checkContact()'s own
  /// translation, and the simulator's own approach-normal direction from the tip's local Z).
  bool calculateTipTransform(
    const std::vector<double> & joint_positions, Eigen::Isometry3d & tip_in_kinematics_base) const;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr kinematics_node_;
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>
    kinematics_loader_;
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;
  std::string kinematics_tip_;
  std::unique_ptr<octomap::OcTree> octree_;
  Eigen::Isometry3d octomap_origin_{Eigen::Isometry3d::Identity()};
  std::string octomap_frame_id_;
};

}  // namespace compliant_trajectory_controller
