/**
 * @file jtac_compliance_demo_node.cpp
 * @brief Live RViz demo of AdmittanceRule's redundancy split between the lift and the arm.
 * @details Runs the SAME AdmittanceRule at the SAME 100 Hz rate used by
 *          test_admittance_rule_integration.cpp (the admittance-rule integration test), against the SAME synthetic 7-joint chain
 *          (urdf/jtac_demo_robot.urdf), closed loop exactly as that test does (the compliant
 *          output feeds back in as "current" every cycle). It is not a new experiment -- it is a
 *          visualization of the two experiments already proven there
 *          (ReportsWhichJointAbsorbsAVerticalPush / ...HorizontalPush): a vertical push shows
 *          ~98% of the resulting motion at the lift, a horizontal push shows exactly 0%, because
 *          the lift's Jacobian column is fixed at [0,0,1,0,0,0] for every configuration -- a
 *          prismatic joint mounted at the chain root literally cannot move the tool sideways.
 *
 *          Publishes two independent joint_states streams -- "actual" (the compliance-shifted
 *          pose) and "reference" (the fixed pose the trajectory backbone commanded) -- so two
 *          robot_state_publisher instances (one plain, one with frame_prefix:=ghost_, see
 *          launch/jtac_compliance_demo.launch.py) can render them as an overlaid ghost in RViz.
 *          Also publishes a force arrow and a text readout of the live lift-share percentage.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <algorithm>
#include <cmath>
#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "admittance_controller/admittance_rule.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "mobile_manipulator_profile/profile.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "urdf/model.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr double kDt = 0.01;  // 100 Hz -- same as combined_arm_lift_controllers.yaml.

// Same diagnostic gains as the admittance-rule integration test suite, satisfying M > K*(zeta*dt)^2 on every Cartesian
// channel -- see test_admittance_rule_integration.cpp's own comment for the full derivation and
// why mop's actual shipped gains diverge at this rate. This demo is about the redundancy split,
// not about mop's specific (and currently broken) tuning.
//
// EXPERIMENT (2026-08-21, live wall_ceiling comparison): XY triple reduced to exactly match the
// Z triple -- was mass=4.0/damping_ratio=5.59/stiffness=1000.0 (X/Y) vs mass=2.0/damping_ratio=
// 6.0/stiffness=300.0 (Z), a real "arm feels 3.3x stiffer than the lift" asymmetry. Removed here to observe
// unified behavior; shared by BOTH pose_set schedules (buildParameters() is not duplicated per
// mode), so this also changes redundancy_split mode's tuning, not only wall_ceiling's.
constexpr double kDiagnosticMassXYZ = 2.0;
constexpr double kDiagnosticDampingRatioXYZ = 6.0;
constexpr double kStiffnessXYZ = 300.0;
// Was a literal 5.0 inline in buildParameters() -- named so tickWallCeilingWeighted()'s own
// reimplementation (see its own doc comment for why it can't reuse AdmittanceRule::update()
// directly) uses the identical value, keeping an equal-weights weighted run and the
// AdmittanceRule-driven baseline comparable.
constexpr double kJointDamping = 5.0;

// Moving joints of the URDF chain base -> tip, in chain order: the DOF of the demo and the names
// it commands. Fixed and mimic joints do not count. Throws if `tip` is not below `base`.
std::vector<std::string> chainJointNames(
  const std::string & urdf_xml, const std::string & base, const std::string & tip)
{
  urdf::Model model;
  if (!model.initString(urdf_xml)) {
    throw std::runtime_error("robot_description is not a valid URDF");
  }
  auto link = model.getLink(tip);
  if (!link) {
    throw std::runtime_error("kinematics_tip '" + tip + "' is not a link of the URDF");
  }
  std::vector<std::string> names;
  while (link && link->name != base) {
    const auto joint = link->parent_joint;
    if (!joint) {
      throw std::runtime_error("'" + base + "' is not an ancestor of '" + tip + "'");
    }
    if (joint->type != urdf::Joint::FIXED && !joint->mimic) {
      names.push_back(joint->name);
    }
    link = link->getParent();
  }
  std::reverse(names.begin(), names.end());
  return names;
}

// Default reference pose for the REAL robot_description chain (rendered via
// robot.urdf.xacro), vetted with pose_conditioning_probe (this package): condition number
// ~19.5 at link6, versus effectively singular (condition number = inf, smallest singular value
// ~1e-17) at BOTH the all-zero pose and the SRDF "home" group_state (joint1=0.7854, rest 0) --
// this arm's zero configuration is a genuine wrist singularity, the same class of trap the
// synthetic chain of the admittance-rule integration test's all-zero pose was, just independently rediscovered on real geometry.
// Overridable via the reference_pose parameter for a different URDF.
const std::vector<double> kDefaultReferencePose{0.40, 0.0, -0.5, 1.2, 0.0, 0.6, 0.0};

// "Home-like" validated seed posture reused verbatim from
// workflow_orchestrator/validate_7dof_sync.py's own DEFAULT_NULLSPACE_SEED (joint3=-25deg,
// joint5=65deg) -- this demo doubles as a preview of a pose already trusted for real 7-DOF sync
// testing, rather than an arbitrary new one. Vetted for THIS use with pose_conditioning_probe
// (condition number 10.72 at link_tcp) and reference_pose_collision_probe (collision-free,
// arm_7dof).
//
// joint1 yawed +45deg from validate_7dof_sync.py's own joint1=0: at joint1=0 the tool's local Z
// pointed diagonally in base frame ([-0.706, -0.708, 0.002], the +45deg arm-mount attach_rpy
// rotating what would otherwise be pure -X into that diagonal) -- +45deg is the actual front of
// the robot, and lands the tool's local Z on pure base -Y ([0.0016, -0.999999, 0.0]), a cleaner
// "tool facing a wall" configuration to read pushes against. -45deg was the other candidate
// (lands on pure -X instead, condition number identical); re-verified collision-free and
// condition number 10.72 (unchanged -- a yaw about the tool's own approach axis doesn't affect
// conditioning) at this value too.
const std::vector<double> kHomeWallPose{
  0.384, 0.7853981633974483, 0.0, -0.4363323129985824, 0.0, 1.1344640137963142, 0.0};

// Same posture as kHomeWallPose with the shoulder (joint2) raised +90deg and the lift extended
// toward the top of its travel -- found by sweeping from kHomeWallPose and vetted the same way
// (condition number 41.56, collision-free): the tool's own local Z axis points almost exactly
// vertically upward in base frame ([0.003, 0.0, 1.0]), i.e. "tool facing a ceiling" -- the
// joint1 yaw carries through unchanged (a yaw about the vertical axis doesn't disturb alignment
// with that same vertical axis).
const std::vector<double> kCeilingPose{
  0.7, 0.7853981633974483, 1.5707963267948966, -0.4363323129985824, 0.0, 1.1344640137963142,
  0.0};

enum class DemoPhase
{
  kSettle,
  kVerticalPush,
  kVerticalHold,
  kVerticalRelease,
  kHorizontalPush,
  kHorizontalHold,
  kHorizontalRelease,
};

struct PhasePlan
{
  DemoPhase phase;
  double duration_s;
  double target_force_n;   ///< 0.0 during settle/release.
  int cartesian_axis;      ///< 0=X (wall-like), 2=Z (ceiling-like). Unused when force is 0.
  const char * label;
};

const std::vector<PhasePlan> kSchedule{
  {DemoPhase::kSettle, 3.0, 0.0, 2, "settled at reference"},
  {DemoPhase::kVerticalPush, 2.0, 6.0, 2, "CEILING-like push (Cartesian Z)"},
  {DemoPhase::kVerticalHold, 4.0, 6.0, 2, "holding vertical push"},
  {DemoPhase::kVerticalRelease, 3.0, 0.0, 2, "releasing"},
  {DemoPhase::kHorizontalPush, 2.0, 6.0, 0, "WALL-like push (Cartesian X)"},
  {DemoPhase::kHorizontalHold, 4.0, 6.0, 0, "holding horizontal push"},
  {DemoPhase::kHorizontalRelease, 3.0, 0.0, 0, "releasing"},
};

// --- pose_set:=wall_ceiling: a second, independent schedule against REAL wall/ceiling postures,
//     pushing directly along the TOOL's own local axes (kinematics.tip == ft_sensor.frame.id, so
//     a "local" wrench needs no rotation) rather than base-frame axes -- see WallCeilingPhaseKind
//     and JtacComplianceDemoNode::tick()'s own branch for why kSchedule's base-axis convention
//     doesn't represent "push into the wall" once the reference pose isn't axis-aligned. ---
enum class WallCeilingPhaseKind
{
  kMoveToPose,  ///< Directly repositions current_/reference_ to *target_pose over duration_s.
  kSettle,
  kPush,
  kHold,
  kRelease,
};

struct WallCeilingPhase
{
  WallCeilingPhaseKind kind;
  double duration_s;
  const std::vector<double> * target_pose;  ///< kMoveToPose only.
  int local_axis;                           ///< 0=local X, 1=local Y, 2=local Z (tool normal).
  double target_force_n;                    ///< 0.0 outside kPush/kHold.
  const char * label;
};

constexpr double kWallCeilingForceN = 8.0;

const std::vector<WallCeilingPhase> kWallCeilingSchedule{
  {WallCeilingPhaseKind::kSettle, 2.0, nullptr, 2, 0.0, "settled at HOME/WALL pose"},
  {WallCeilingPhaseKind::kPush, 1.5, nullptr, 2, kWallCeilingForceN,
    "WALL: pushing tool +Z (into the wall)"},
  {WallCeilingPhaseKind::kHold, 3.0, nullptr, 2, kWallCeilingForceN, "WALL: holding tool +Z"},
  {WallCeilingPhaseKind::kRelease, 2.0, nullptr, 2, 0.0, "WALL: releasing"},
  {WallCeilingPhaseKind::kPush, 1.5, nullptr, 0, kWallCeilingForceN,
    "WALL: pushing tool +X (tangential/shear)"},
  {WallCeilingPhaseKind::kHold, 3.0, nullptr, 0, kWallCeilingForceN, "WALL: holding tool +X"},
  {WallCeilingPhaseKind::kRelease, 2.0, nullptr, 0, 0.0, "WALL: releasing"},
  {WallCeilingPhaseKind::kPush, 1.5, nullptr, 1, kWallCeilingForceN,
    "WALL: pushing tool +Y (tangential/shear)"},
  {WallCeilingPhaseKind::kHold, 3.0, nullptr, 1, kWallCeilingForceN, "WALL: holding tool +Y"},
  {WallCeilingPhaseKind::kRelease, 2.0, nullptr, 1, 0.0, "WALL: releasing"},
  {WallCeilingPhaseKind::kMoveToPose, 3.0, &kCeilingPose, 2, 0.0, "moving to CEILING pose"},
  {WallCeilingPhaseKind::kSettle, 2.0, nullptr, 2, 0.0, "settled at CEILING pose"},
  {WallCeilingPhaseKind::kPush, 1.5, nullptr, 2, kWallCeilingForceN,
    "CEILING: pushing tool +Z (into the ceiling)"},
  {WallCeilingPhaseKind::kHold, 3.0, nullptr, 2, kWallCeilingForceN, "CEILING: holding tool +Z"},
  {WallCeilingPhaseKind::kRelease, 2.0, nullptr, 2, 0.0, "CEILING: releasing"},
  {WallCeilingPhaseKind::kPush, 1.5, nullptr, 0, kWallCeilingForceN,
    "CEILING: pushing tool +X (tangential/shear)"},
  {WallCeilingPhaseKind::kHold, 3.0, nullptr, 0, kWallCeilingForceN, "CEILING: holding tool +X"},
  {WallCeilingPhaseKind::kRelease, 2.0, nullptr, 0, 0.0, "CEILING: releasing"},
  {WallCeilingPhaseKind::kMoveToPose, 3.0, &kHomeWallPose, 2, 0.0,
    "moving back to HOME/WALL pose"},
};

trajectory_msgs::msg::JointTrajectoryPoint makePoint(const std::vector<double> & positions)
{
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.velocities.assign(positions.size(), 0.0);
  point.accelerations.assign(positions.size(), 0.0);
  return point;
}

std::string readFile(const std::string & path)
{
  std::ifstream file(path);
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

}  // namespace

class JtacComplianceDemoNode : public rclcpp::Node
{
public:
  JtacComplianceDemoNode()
  : Node("jtac_compliance_demo_node")
  {
    // robot_description defaults to the synthetic chain (as in the admittance-rule integration test) (self-contained, no external
    // xacro render needed) but is meant to be overridden -- see
    // launch/jtac_compliance_demo.launch.py's real_robot argument, which renders
    // robot.urdf.xacro at launch time and passes the result here.
    declare_parameter("robot_description", std::string(""));
    // Empty reference_pose: kDefaultReferencePose for a 7-joint chain, else the profile's home pose.
    declare_parameter("reference_pose", std::vector<double>{});
    declare_parameter("profile", std::string(mobile_manipulator_profile::kDefaultProfile));
    declare_parameter("kinematics_tip", std::string("link6"));
    // "redundancy_split" (default): the original kSchedule, base-frame axis pushes at whatever
    // reference_pose is given. "wall_ceiling": kWallCeilingSchedule, tool-local-frame pushes at
    // kHomeWallPose/kCeilingPose -- see those constants' own doc comments.
    declare_parameter("pose_set", std::string("redundancy_split"));
    // pose_set:=wall_ceiling only -- see tickWallCeilingWeighted()'s own doc comment for the
    // mechanism (a self-contained weighted-pseudo-inverse loop, not an AdmittanceRule parameter:
    // AdmittanceRule::update() has no seam to swap in a weighted joint conversion). Default
    // false: the feature itself stays opt-in until ported into JtacController for real
    // deployment -- this demo node is still the prototyping sandbox, not the production path.
    declare_parameter("enable_redundancy_weighting", false);
    // DESIGN CHOICE (2026-08-21, reversible): lift weighted 10x each arm joint -- "arm more
    // compliant than the lift" as the default bias whenever this feature IS enabled, not a
    // physically-derived number. Live-verified range at kCeilingPose's +Z push (lift share of
    // the resulting motion): 1.2% at an extreme 1000:1 ratio, ~93-96% at the natural (all-1.0)
    // unweighted split, 100% at the opposite 1:1000 extreme -- 10:1 sits clearly on the
    // arm-compliant side without being the maximal, all-or-nothing extreme used to make that
    // range measurable in the first place. Swap freely via this same parameter; nothing else
    // depends on this specific ratio. The mechanism is drawn from
    // the literature (Whitney 1969, Khatib's dynamically-consistent
    // generalized inverse, Chan & Dubey 1995) this mechanism is drawn from.
    // Empty = the lift (joint 0) weighted 10x every other joint, sized to the chain's DOF.
    declare_parameter("redundancy_weights", std::vector<double>{});

    std::string urdf = get_parameter("robot_description").as_string();
    if (urdf.empty()) {
      const std::string urdf_path =
        ament_index_cpp::get_package_share_directory("compliant_trajectory_controller") +
        "/urdf/jtac_demo_robot.urdf";
      urdf = readFile(urdf_path);
      RCLCPP_INFO(
        get_logger(), "No robot_description parameter given; using the synthetic demo URDF.");
    }
    if (urdf.empty()) {
      RCLCPP_FATAL(
        get_logger(),
        "robot_description is empty and the fallback demo URDF is missing.");
      throw std::runtime_error("missing URDF");
    }

    kinematics_tip_ = get_parameter("kinematics_tip").as_string();
    // DOF and joint names come from the URDF chain, not from a constant.
    joint_names_ = chainJointNames(urdf, "base_link", kinematics_tip_);
    num_joints_ = joint_names_.size();
    if (num_joints_ == 0) {
      RCLCPP_FATAL(get_logger(), "No moving joints between base_link and '%s'.",
        kinematics_tip_.c_str());
      throw std::runtime_error("empty chain");
    }
    RCLCPP_INFO(get_logger(), "Chain base_link -> %s has %zu joints.", kinematics_tip_.c_str(),
      num_joints_);

    reference_pose_ = get_parameter("reference_pose").as_double_array();
    if (reference_pose_.empty()) {
      std::string profile_warning;
      const auto profile = mobile_manipulator_profile::loadProfileOrDefaults(
        get_parameter("profile").as_string(), profile_warning);
      if (!profile_warning.empty()) {
        RCLCPP_WARN(get_logger(), "%s", profile_warning.c_str());
      }
      if (num_joints_ == kDefaultReferencePose.size()) {
        reference_pose_ = kDefaultReferencePose;
      } else if (profile.dof() == num_joints_ && profile.joints() == joint_names_) {
        reference_pose_ = profile.homePositions();
      } else {
        RCLCPP_FATAL(
          get_logger(), "Chain has %zu joints: pass reference_pose with %zu values.",
          num_joints_, num_joints_);
        throw std::runtime_error("no default reference_pose for this DOF");
      }
    }
    if (reference_pose_.size() != num_joints_) {
      RCLCPP_FATAL(
        get_logger(), "reference_pose must have %zu elements, got %zu.", num_joints_,
        reference_pose_.size());
      throw std::runtime_error("bad reference_pose size");
    }
    pose_set_ = get_parameter("pose_set").as_string();
    if (pose_set_ == "wall_ceiling") {
      // kHomeWallPose/kCeilingPose were only vetted (pose_conditioning_probe,
      // reference_pose_collision_probe) at kinematics.tip==link_tcp -- the real controlled tool
      // point once the mop tool is attached, per this repo's own convention. A run against
      // link6 or the synthetic demo chain would be pushing on an unvetted, and likely
      // self-colliding or wrong-frame, pose.
      if (num_joints_ != kHomeWallPose.size()) {
        RCLCPP_FATAL(
          get_logger(),
          "pose_set:=wall_ceiling uses poses vetted for a %zu-joint chain; this one has %zu.",
          kHomeWallPose.size(), num_joints_);
        throw std::runtime_error("wall_ceiling pose_set needs a 7-joint chain");
      }
      if (kinematics_tip_ != "link_tcp") {
        RCLCPP_FATAL(
          get_logger(),
          "pose_set:=wall_ceiling requires kinematics_tip:=link_tcp (i.e. real_robot:=true on "
          "the launch file) -- got '%s'.", kinematics_tip_.c_str());
        throw std::runtime_error("wall_ceiling pose_set requires kinematics_tip=link_tcp");
      }
      reference_pose_ = kHomeWallPose;
    }

    enable_redundancy_weighting_ = get_parameter("enable_redundancy_weighting").as_bool();
    redundancy_weights_ = get_parameter("redundancy_weights").as_double_array();
    if (redundancy_weights_.empty()) {
      redundancy_weights_.assign(num_joints_, 1.0);
      redundancy_weights_[0] = 10.0;
    }
    if (enable_redundancy_weighting_) {
      if (pose_set_ != "wall_ceiling") {
        RCLCPP_FATAL(
          get_logger(), "enable_redundancy_weighting:=true requires pose_set:=wall_ceiling.");
        throw std::runtime_error("redundancy weighting requires pose_set=wall_ceiling");
      }
      if (redundancy_weights_.size() != num_joints_) {
        RCLCPP_FATAL(
          get_logger(), "redundancy_weights must have %zu elements, got %zu.", num_joints_,
          redundancy_weights_.size());
        throw std::runtime_error("bad redundancy_weights size");
      }
      for (double w : redundancy_weights_) {
        if (w <= 0.0) {
          RCLCPP_FATAL(get_logger(), "redundancy_weights entries must be > 0 (got %f).", w);
          throw std::runtime_error("non-positive redundancy_weights entry");
        }
      }
    }
    weighted_joint_vel_.assign(num_joints_, 0.0);

    rclcpp::NodeOptions rule_node_options;
    rule_node_options.parameter_overrides(buildParameters(urdf, kinematics_tip_, joint_names_));
    rule_node_ =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "jtac_demo_admittance_rule",
      rule_node_options);

    param_listener_ = std::make_shared<admittance_controller::ParamListener>(
      rule_node_->get_node_parameters_interface(), rule_node_->get_logger());
    rule_ = std::make_unique<admittance_controller::AdmittanceRule>(param_listener_);
    if (rule_->configure(rule_node_, num_joints_, urdf) != controller_interface::return_type::OK) {
      RCLCPP_FATAL(get_logger(), "AdmittanceRule::configure() failed.");
      throw std::runtime_error("configure failed");
    }

    kinematics_loader_ =
      std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
      "kinematics_interface", "kinematics_interface::KinematicsInterface");
    kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
      kinematics_loader_->createUnmanagedInstance("kinematics_interface_kdl/KinematicsInterfaceKDL"));
    // "kinematics": rule_node_'s own parameter_overrides (buildParameters()) already declares
    // "kinematics.tip" = kinematics_tip_.
    if (!kinematics_->initialize(
        urdf, rule_node_->get_node_parameters_interface(), "kinematics"))
    {
      RCLCPP_FATAL(get_logger(), "kinematics initialize() failed.");
      throw std::runtime_error("kinematics init failed");
    }

    ASSERT_liftColumnIsPureZ();

    Eigen::Isometry3d base_to_ft;
    Eigen::VectorXd q0 =
      Eigen::VectorXd::Map(reference_pose_.data(), static_cast<long>(reference_pose_.size()));
    kinematics_->calculate_link_transform(q0, kinematics_tip_, base_to_ft);
    base_to_sensor_rotation_ = base_to_ft.rotation();

    actual_joint_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10));
    reference_joint_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>("/ghost/joint_states", rclcpp::QoS(10));
    marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/jtac_demo/markers", rclcpp::QoS(10));

    current_ = makePoint(reference_pose_);
    reference_ = makePoint(reference_pose_);
    phase_start_ = now();

    if (pose_set_ == "wall_ceiling") {
      if (enable_redundancy_weighting_) {
        std::ostringstream weights_oss;
        for (double w : redundancy_weights_) {weights_oss << w << " ";}
        RCLCPP_INFO(
          get_logger(), "JTAC compliance demo running (pose_set=wall_ceiling, "
          "enable_redundancy_weighting=true): self-contained weighted-pseudo-inverse loop, "
          "redundancy_weights=[%s] (lift, joint1..6).", weights_oss.str().c_str());
      } else {
        RCLCPP_INFO(
          get_logger(), "JTAC compliance demo running (pose_set=wall_ceiling): tool-local X/Y/Z "
          "pushes at a real HOME/WALL pose, then the same at a raised CEILING pose. Watch the "
          "lift column and the ghost overlay.");
      }
    } else {
      RCLCPP_INFO(
        get_logger(), "JTAC compliance demo running: cycling CEILING-like (vertical, lift-heavy) "
        "and WALL-like (horizontal, arm-only) pushes. Watch the lift column and the ghost "
        "overlay.");
    }

    timer_ = create_wall_timer(
      std::chrono::duration<double>(kDt), [this]() {tick();});
  }

private:
  static std::vector<rclcpp::Parameter> buildParameters(
    const std::string & urdf, const std::string & tip,
    const std::vector<std::string> & joint_names)
  {
    return {
      rclcpp::Parameter("robot_description", urdf),
      rclcpp::Parameter("joints", joint_names),
      rclcpp::Parameter("command_interfaces", std::vector<std::string>{"position", "velocity"}),
      rclcpp::Parameter("state_interfaces", std::vector<std::string>{"position", "velocity"}),
      rclcpp::Parameter(
        "chainable_command_interfaces", std::vector<std::string>{"position", "velocity"}),
      rclcpp::Parameter(
        "kinematics.plugin_name", std::string("kinematics_interface_kdl/KinematicsInterfaceKDL")),
      rclcpp::Parameter("kinematics.plugin_package", std::string("kinematics_interface")),
      rclcpp::Parameter("kinematics.base", std::string("base_link")),
      // Same link as ft_sensor.frame.id/gravity_compensation.frame.id below -- the FT sensor
      // mounts at the wrist, and this demo has no separate tool-tip link unless one is added via
      // extra xacro args (robot.urdf.xacro's add_other_geometry), so tip == sensor mount here.
      rclcpp::Parameter("kinematics.tip", tip),
      rclcpp::Parameter("kinematics.alpha", 0.0005),
      rclcpp::Parameter("ft_sensor.name", std::string("demo_ft")),
      rclcpp::Parameter("ft_sensor.frame.id", tip),
      rclcpp::Parameter("ft_sensor.filter_coefficient", 1.0),
      rclcpp::Parameter("control.frame.id", std::string("base_link")),
      rclcpp::Parameter("fixed_world_frame.frame.id", std::string("base_link")),
      rclcpp::Parameter("gravity_compensation.frame.id", tip),
      rclcpp::Parameter("gravity_compensation.CoG.pos", std::vector<double>{0.0, 0.0, 0.0}),
      rclcpp::Parameter("gravity_compensation.CoG.force", 0.0),
      // All three translational axes selected simultaneously: whichever schedule is active
      // alternates which axis actually receives nonzero force, so selection never has to be
      // reconfigured mid-run. Y stays unselected-in-effect for kSchedule (its own phases never
      // set a nonzero Y force), so this is a no-op for that schedule's own observable behavior.
      rclcpp::Parameter(
        "admittance.selected_axes", std::vector<bool>{true, true, true, false, false, false}),
      rclcpp::Parameter(
        "admittance.mass",
        std::vector<double>{
          kDiagnosticMassXYZ, kDiagnosticMassXYZ, kDiagnosticMassXYZ, 0.5, 0.5, 0.5}),
      rclcpp::Parameter(
        "admittance.stiffness",
        std::vector<double>{
          kStiffnessXYZ, kStiffnessXYZ, kStiffnessXYZ, 50.0, 50.0, 50.0}),
      rclcpp::Parameter(
        "admittance.damping_ratio",
        std::vector<double>{
          kDiagnosticDampingRatioXYZ, kDiagnosticDampingRatioXYZ, kDiagnosticDampingRatioXYZ, 1.0,
          1.0, 1.0}),
      rclcpp::Parameter("admittance.joint_damping", kJointDamping),
      rclcpp::Parameter("enable_parameter_update_without_reactivation", false),
    };
  }

  /// Same invariant as LiftJacobianColumnIsPurelyVerticalByConstruction in the gtest -- checked
  /// again here at startup because this node's whole visual point rests on it, and a silent
  /// failure would make the demo misleading rather than merely wrong.
  void ASSERT_liftColumnIsPureZ()
  {
    Eigen::VectorXd q =
      Eigen::VectorXd::Map(reference_pose_.data(), static_cast<long>(reference_pose_.size()));
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, num_joints_);
    if (!kinematics_->calculate_jacobian(q, kinematics_tip_, jacobian)) {
      throw std::runtime_error("calculate_jacobian failed at startup");
    }
    if (std::abs(jacobian(0, 0)) > 1e-6 || std::abs(jacobian(1, 0)) > 1e-6) {
      throw std::runtime_error(
              "lift Jacobian column is not purely vertical -- URDF changed under this demo's assumption");
    }
  }

  const PhasePlan & currentPlan() const {return kSchedule[schedule_index_];}

  void advanceScheduleIfDue(double elapsed_in_phase_s)
  {
    if (elapsed_in_phase_s < currentPlan().duration_s) {
      return;
    }
    schedule_index_ = (schedule_index_ + 1) % kSchedule.size();
    phase_start_ = now();
  }

  geometry_msgs::msg::Wrench sensorFrameWrenchFor(const PhasePlan & plan) const
  {
    Eigen::Vector3d base_force = Eigen::Vector3d::Zero();
    base_force[plan.cartesian_axis] = plan.target_force_n;
    const Eigen::Vector3d sensor_force = base_to_sensor_rotation_.transpose() * base_force;
    geometry_msgs::msg::Wrench wrench;
    wrench.force.x = sensor_force.x();
    wrench.force.y = sensor_force.y();
    wrench.force.z = sensor_force.z();
    return wrench;
  }

  /// pose_set:=wall_ceiling only: a push "along local axis k" needs NO rotation at all, unlike
  /// sensorFrameWrenchFor() above -- ft_sensor.frame.id IS kinematics_tip_ (see buildParameters()),
  /// so the sensor frame already IS the tool frame, and setting wrench components directly here
  /// means exactly "push along the tool's own X/Y/Z", regardless of which way that maps to base
  /// frame at the current reference pose (unlike kHomeWallPose, where TCP Z is NOT aligned with
  /// any single base axis -- see that constant's own doc comment).
  static geometry_msgs::msg::Wrench sensorLocalWrenchFor(const WallCeilingPhase & plan)
  {
    geometry_msgs::msg::Wrench wrench;
    double * const components[3] = {&wrench.force.x, &wrench.force.y, &wrench.force.z};
    *components[plan.local_axis] = plan.target_force_n;
    return wrench;
  }

  /// Projects each joint's current offset from the reference pose through the Jacobian to get its
  /// contribution to Cartesian motion along `axis`, and returns the lift's (joint 0) share of the
  /// total -- the same computation the gtest's ReportsWhichJointAbsorbsA*Push tests perform.
  double liftShareAlongAxis(int axis) const
  {
    Eigen::VectorXd q =
      Eigen::VectorXd::Map(current_.positions.data(), static_cast<long>(current_.positions.size()));
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, num_joints_);
    if (!kinematics_->calculate_jacobian(q, kinematics_tip_, jacobian)) {
      return 0.0;
    }
    double total_abs = 0.0;
    double lift_abs = 0.0;
    for (size_t i = 0; i < num_joints_; ++i) {
      const double offset = current_.positions[i] - reference_pose_[i];
      const double contribution = jacobian(axis, static_cast<long>(i)) * offset;
      total_abs += std::abs(contribution);
      if (i == 0) {
        lift_abs = std::abs(contribution);
      }
    }
    return total_abs > 1e-9 ? lift_abs / total_abs : 0.0;
  }

  /// Generalization of liftShareAlongAxis() for a push direction that isn't a pure base axis --
  /// exactly the wall_ceiling case, since kHomeWallPose's tool-local Z projects roughly equally
  /// onto base X and Y (see that constant's own doc comment). liftShareAlongAxis(k) is the
  /// special case of this with direction = the k'th base unit vector.
  double liftShareAlongDirection(const Eigen::Vector3d & direction_in_base) const
  {
    Eigen::VectorXd q =
      Eigen::VectorXd::Map(current_.positions.data(), static_cast<long>(current_.positions.size()));
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, num_joints_);
    if (!kinematics_->calculate_jacobian(q, kinematics_tip_, jacobian)) {
      return 0.0;
    }
    double total_abs = 0.0;
    double lift_abs = 0.0;
    for (size_t i = 0; i < num_joints_; ++i) {
      const double offset = current_.positions[i] - reference_pose_[i];
      const double contribution = direction_in_base.dot(jacobian.block<3, 1>(0, i)) * offset;
      total_abs += std::abs(contribution);
      if (i == 0) {
        lift_abs = std::abs(contribution);
      }
    }
    return total_abs > 1e-9 ? lift_abs / total_abs : 0.0;
  }

  /// base_to_sensor_rotation_ depends on the reference pose -- recomputed once at startup, and
  /// again here after every wall_ceiling kMoveToPose phase (the only way reference_pose_ changes
  /// after startup).
  void recomputeBaseToSensorRotation()
  {
    Eigen::Isometry3d base_to_ft;
    Eigen::VectorXd q0 =
      Eigen::VectorXd::Map(reference_pose_.data(), static_cast<long>(reference_pose_.size()));
    kinematics_->calculate_link_transform(q0, kinematics_tip_, base_to_ft);
    base_to_sensor_rotation_ = base_to_ft.rotation();
  }

  const WallCeilingPhase & currentWallCeilingPlan() const
  {
    return kWallCeilingSchedule[schedule_index_];
  }

  void advanceWallCeilingScheduleIfDue(double elapsed_in_phase_s)
  {
    if (elapsed_in_phase_s < currentWallCeilingPlan().duration_s) {
      return;
    }
    if (currentWallCeilingPlan().kind == WallCeilingPhaseKind::kMoveToPose) {
      // Snap exactly onto the target rather than trusting the last interpolated sample (avoids
      // compounding floating-point drift across repeated wall<->ceiling cycles), and refresh
      // every reference_pose_-derived quantity before the next phase pushes against it.
      reference_pose_ = *currentWallCeilingPlan().target_pose;
      current_.positions = reference_pose_;
      current_.velocities.assign(num_joints_, 0.0);
      reference_.positions = reference_pose_;
      weighted_joint_vel_.assign(num_joints_, 0.0);
      recomputeBaseToSensorRotation();
    }
    schedule_index_ = (schedule_index_ + 1) % kWallCeilingSchedule.size();
    phase_start_ = now();
  }

  void tick()
  {
    if (pose_set_ == "wall_ceiling") {
      tickWallCeiling();
    } else {
      tickRedundancySplit();
    }
  }

  void tickRedundancySplit()
  {
    const auto & plan = currentPlan();
    const double elapsed_in_phase_s = (now() - phase_start_).seconds();
    advanceScheduleIfDue(elapsed_in_phase_s);

    const auto wrench = sensorFrameWrenchFor(plan);
    // Must be pre-sized like current_/reference_, not default-constructed: AdmittanceRule::
    // update() writes into desired.positions[i] assuming the vector is already the right length.
    trajectory_msgs::msg::JointTrajectoryPoint desired = makePoint(reference_pose_);
    const auto result = rule_->update(
      current_, wrench, reference_, rclcpp::Duration::from_seconds(kDt), desired);
    if (result == controller_interface::return_type::OK) {
      current_.positions = desired.positions;
      current_.velocities = desired.velocities;
    }

    const double lift_share = plan.target_force_n >
      1e-6 ? liftShareAlongAxis(plan.cartesian_axis) : 0.0;
    publishJointStates();
    publishMarkers(plan.label, plan.target_force_n, wrench, lift_share);
  }

  void tickWallCeiling()
  {
    const auto & plan = currentWallCeilingPlan();
    const double elapsed_in_phase_s = (now() - phase_start_).seconds();

    if (plan.kind == WallCeilingPhaseKind::kMoveToPose) {
      // A plain repositioning move at zero load -- current_/reference_ are driven directly
      // together (no offset, so nothing for AdmittanceRule to do) rather than run through it,
      // avoiding any transient from suddenly relocating an active spring's own reference.
      const double t = std::clamp(elapsed_in_phase_s / plan.duration_s, 0.0, 1.0);
      const auto & target = *plan.target_pose;
      std::vector<double> interpolated(num_joints_);
      for (size_t i = 0; i < num_joints_; ++i) {
        interpolated[i] = reference_pose_[i] + t * (target[i] - reference_pose_[i]);
      }
      current_.positions = interpolated;
      current_.velocities.assign(num_joints_, 0.0);
      reference_.positions = interpolated;

      publishJointStates();
      geometry_msgs::msg::Wrench zero_wrench;
      publishMarkers(plan.label, 0.0, zero_wrench, 0.0);
      advanceWallCeilingScheduleIfDue(elapsed_in_phase_s);
      return;
    }

    if (enable_redundancy_weighting_) {
      tickWallCeilingWeighted(plan);
    } else {
      const auto wrench = sensorLocalWrenchFor(plan);
      trajectory_msgs::msg::JointTrajectoryPoint desired = makePoint(reference_pose_);
      const auto result = rule_->update(
        current_, wrench, reference_, rclcpp::Duration::from_seconds(kDt), desired);
      if (result == controller_interface::return_type::OK) {
        current_.positions = desired.positions;
        current_.velocities = desired.velocities;
      }

      const double lift_share = plan.target_force_n > 1e-6 ?
        liftShareAlongDirection(base_to_sensor_rotation_.col(plan.local_axis)) : 0.0;
      publishJointStates();
      publishMarkers(plan.label, plan.target_force_n, wrench, lift_share);
    }

    advanceWallCeilingScheduleIfDue(elapsed_in_phase_s);
  }

  /// Self-contained weighted-pseudo-inverse compliance loop -- deliberately NOT going through
  /// AdmittanceRule::update() (see this class's own commit/doc history: update() computes the
  /// Cartesian spring law AND calls the unweighted convert_cartesian_deltas_to_joint_deltas() in
  /// the same function body, with no seam to swap in a weighted conversion from outside). Only
  /// translation, in base frame, using the SAME mass/stiffness/damping_ratio/joint_damping
  /// values buildParameters() gives AdmittanceRule -- no wall_ceiling phase ever selects a
  /// rotational axis, so reproducing AdmittanceRule's general 6-DOF/gravity-compensation/
  /// arbitrary-control-frame machinery is unnecessary here, and reusing its exact gains makes an
  /// all-1.0 redundancy_weights run directly comparable to the AdmittanceRule-driven baseline
  /// recorded in reference_trajectories/.
  ///
  /// Math: X_ddot = (F_base - D*X_dot - K*X) / M (translation only, D from damping_ratio same as
  /// buildParameters() derives it); joint_acc = J_W+ * X_ddot - joint_damping*joint_vel, where
  /// J_W+ = W^-1 J^T (J W^-1 J^T)^-1 is the weighted pseudo-inverse (W = diag(redundancy_weights),
  /// J the 3xN linear-velocity Jacobian rows) -- reduces to the plain Moore-Penrose pseudo-inverse
  /// when every weight is equal, by construction. Drawn from
  /// the literature (Whitney 1969, Khatib's dynamically-consistent
  /// generalized inverse, Chan & Dubey 1995's weighted least-norm).
  void tickWallCeilingWeighted(const WallCeilingPhase & plan)
  {
    const auto wrench = sensorLocalWrenchFor(plan);
    const Eigen::Vector3d f_sensor(wrench.force.x, wrench.force.y, wrench.force.z);
    const Eigen::Vector3d f_base = base_to_sensor_rotation_ * f_sensor;

    Eigen::VectorXd q_current =
      Eigen::VectorXd::Map(current_.positions.data(), static_cast<long>(current_.positions.size()));
    Eigen::VectorXd q_reference =
      Eigen::VectorXd::Map(reference_pose_.data(), static_cast<long>(reference_pose_.size()));
    Eigen::Isometry3d current_tf, reference_tf;
    kinematics_->calculate_link_transform(q_current, kinematics_tip_, current_tf);
    kinematics_->calculate_link_transform(q_reference, kinematics_tip_, reference_tf);
    const Eigen::Vector3d x = current_tf.translation() - reference_tf.translation();

    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, num_joints_);
    kinematics_->calculate_jacobian(q_current, kinematics_tip_, jacobian);
    const Eigen::Matrix<double, 3, Eigen::Dynamic> j_lin = jacobian.topRows<3>();

    Eigen::VectorXd joint_vel = Eigen::VectorXd::Map(
      weighted_joint_vel_.data(), static_cast<long>(weighted_joint_vel_.size()));
    const Eigen::Vector3d x_dot = j_lin * joint_vel;

    const double raw_damping =
      kDiagnosticDampingRatioXYZ * 2.0 * std::sqrt(kDiagnosticMassXYZ * kStiffnessXYZ);
    const Eigen::Vector3d x_ddot =
      (f_base - raw_damping * x_dot - kStiffnessXYZ * x) / kDiagnosticMassXYZ;

    const Eigen::VectorXd weights = Eigen::VectorXd::Map(
      redundancy_weights_.data(), static_cast<long>(redundancy_weights_.size()));
    const Eigen::MatrixXd w_inv = weights.cwiseInverse().asDiagonal();
    const Eigen::Matrix3d j_w_jt = j_lin * w_inv * j_lin.transpose();
    const Eigen::MatrixXd j_w_pinv = w_inv * j_lin.transpose() * j_w_jt.inverse();

    Eigen::VectorXd joint_acc = j_w_pinv * x_ddot;
    joint_acc -= kJointDamping * joint_vel;

    joint_vel += joint_acc * kDt;
    const Eigen::VectorXd joint_pos = q_current + joint_vel * kDt;

    for (size_t i = 0; i < num_joints_; ++i) {
      current_.positions[i] = joint_pos[i];
      current_.velocities[i] = joint_vel[i];
      weighted_joint_vel_[i] = joint_vel[i];
    }

    const double lift_share = plan.target_force_n > 1e-6 ?
      liftShareAlongDirection(base_to_sensor_rotation_.col(plan.local_axis)) : 0.0;
    publishJointStates();
    publishMarkers(plan.label, plan.target_force_n, wrench, lift_share);
  }

  void publishJointStates()
  {
    const auto stamp = now();

    sensor_msgs::msg::JointState actual_msg;
    actual_msg.header.stamp = stamp;
    actual_msg.name = joint_names_;
    actual_msg.position = current_.positions;
    actual_joint_state_pub_->publish(actual_msg);

    sensor_msgs::msg::JointState reference_msg;
    reference_msg.header.stamp = stamp;
    reference_msg.name = joint_names_;
    reference_msg.position = reference_pose_;
    reference_joint_state_pub_->publish(reference_msg);
  }

  void publishMarkers(
    const std::string & label, double target_force_n,
    const geometry_msgs::msg::Wrench & sensor_wrench, double lift_share)
  {
    Eigen::VectorXd q_actual =
      Eigen::VectorXd::Map(current_.positions.data(), static_cast<long>(current_.positions.size()));
    Eigen::Isometry3d actual_tcp;
    kinematics_->calculate_link_transform(q_actual, kinematics_tip_, actual_tcp);

    visualization_msgs::msg::MarkerArray array;
    const auto stamp = now();

    // Force arrow: applied wrench, rotated into base frame, drawn from the wrist outward.
    const Eigen::Vector3d sensor_force(sensor_wrench.force.x, sensor_wrench.force.y,
      sensor_wrench.force.z);
    const Eigen::Vector3d base_force = base_to_sensor_rotation_ * sensor_force;
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "base_link";
    arrow.header.stamp = stamp;
    arrow.ns = "jtac_demo";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.01;
    arrow.scale.y = 0.02;
    arrow.scale.z = 0.02;
    arrow.color.r = 1.0F;
    arrow.color.g = 0.2F;
    arrow.color.b = 0.2F;
    arrow.color.a = base_force.norm() > 1e-3 ? 1.0F : 0.0F;
    geometry_msgs::msg::Point start;
    start.x = actual_tcp.translation().x();
    start.y = actual_tcp.translation().y();
    start.z = actual_tcp.translation().z();
    geometry_msgs::msg::Point end = start;
    constexpr double kArrowScalePerNewton = 0.02;
    end.x -= base_force.x() * kArrowScalePerNewton;
    end.y -= base_force.y() * kArrowScalePerNewton;
    end.z -= base_force.z() * kArrowScalePerNewton;
    arrow.points = {start, end};
    array.markers.push_back(arrow);

    // Text readout: phase, force, and the live lift-share measurement for whichever axis is
    // active -- the number this whole demo exists to make visible.
    visualization_msgs::msg::Marker text;
    text.header.frame_id = "base_link";
    text.header.stamp = stamp;
    text.ns = "jtac_demo";
    text.id = 1;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = 0.0;
    text.pose.position.y = 0.0;
    text.pose.position.z = 1.1;
    text.scale.z = 0.06;
    text.color.r = 1.0F;
    text.color.g = 1.0F;
    text.color.b = 1.0F;
    text.color.a = 1.0F;
    std::ostringstream oss;
    oss << label << "\nforce = " << target_force_n << " N";
    if (target_force_n > 1e-6) {
      oss << "\nlift share of motion = " << std::round(lift_share * 1000.0) / 10.0 << "%";
    }
    text.text = oss.str();
    array.markers.push_back(text);

    marker_pub_->publish(array);
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> rule_node_;
  std::shared_ptr<admittance_controller::ParamListener> param_listener_;
  std::unique_ptr<admittance_controller::AdmittanceRule> rule_;
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>
  kinematics_loader_;
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;
  Eigen::Matrix3d base_to_sensor_rotation_;
  std::vector<double> reference_pose_;
  std::string kinematics_tip_;
  std::vector<std::string> joint_names_;  ///< Moving joints of the chain base_link -> tip.
  size_t num_joints_{0};                    ///< Their number (the DOF), from the URDF.
  std::string pose_set_;
  bool enable_redundancy_weighting_{false};
  std::vector<double> redundancy_weights_;
  /// tickWallCeilingWeighted()'s own persistent joint-velocity integrator state -- see that
  /// method's own doc comment. Never reset mid-schedule (matching AdmittanceRule's own
  /// internal joint_vel, which also persists continuously across phases); zeroed only at a
  /// kMoveToPose finalization, alongside current_/reference_'s own reset there.
  std::vector<double> weighted_joint_vel_;

  trajectory_msgs::msg::JointTrajectoryPoint current_;
  trajectory_msgs::msg::JointTrajectoryPoint reference_;
  size_t schedule_index_{0};
  rclcpp::Time phase_start_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr actual_joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_joint_state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<JtacComplianceDemoNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("jtac_compliance_demo_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
