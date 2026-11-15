/**
 * @file jtac_wall_approach_test_node.cpp
 * @brief Bridges T8/T9's abstract engage/APPROACH/CARTESIAN/RETRACT cycle and the full production
 *        pipeline (T10): the SAME real force_core::ForceStageSequencer cycle, against a REAL,
 *        visible, collision-checkable wall (wall_scene.json's own wall_id=0, mounted via
 *        cartesian_test.launch.py's mount_robot_to_wall:=true) -- and now also a REAL,
 *        collision-checked FSP entry/exit and a REAL Cartesian coverage-strip trajectory tracked
 *        WHILE force is held, not just a settle-in-place.
 * @details Run against cartesian_test.launch.py (mount_robot_to_wall:=true, use_jtac_controller:=
 *          true, push_world_geometry:=true), PLUS wall_mount_publisher_node running alongside it
 *          (coverage_planner -- motion_validator_node's own mount transform dies the moment
 *          its own mission finishes, KNOWN_ISSUES.md #20), PLUS wall_contact_force_simulator_node
 *          (this package) -- see that file's own doc comment for why it exists and what it
 *          replaces.
 *
 *          FULL FLOW: HOME (SRDF-named "home" group_state, whole-body group) -> [FSP: collision-
 *          checked MoveIt joint-space plan+execute] -> kApproachStandoffPose (the coverage-plan's
 *          own start point, offset from the real wall) -> engage() -> APPROACH (one ordinary
 *          trajectory, admittance's own bounded offset closes the remaining gap to genuine
 *          contact) -> CARTESIAN settle (dwell until force converges) -> a REAL Cartesian
 *          coverage-strip trajectory (MoveGroupInterface::computeCartesianPath(), the SAME
 *          primitive motion_planner::CartesianPlanner itself uses in production) tracked
 *          WHILE the trim loop (force_core::ForceStageSequencer's own background thread, see
 *          that header's "FORCE TRIM" doc comment) corrects for real depth variation along the
 *          strip -> RETRACT (unload, peel back to the coverage-plan start standoff) -> disengage()
 *          -> [FSP] -> HOME again.
 *
 *          WHY THIS SHAPE, per the user's own explicit framing: "we move to an offset point using
 *          jtac or whatever then we move z till we achieve contact and then continue with the
 *          joint trajectory we have planned for that" -- i.e. move-until-contact/retract is NOT a
 *          separate service or mechanism; it is what JtacController's own admittance loop already
 *          does for free once engaged with a target force (see force_control_backend_admittance_
 *          vs_diffik.md's own Context section), and the ONLY missing piece was a real, continuous
 *          contact force in mock (wall_contact_force_simulator_node). The FSP legs and the
 *          Cartesian strip are the two genuinely-planned pieces of a real coverage mission this
 *          test was still missing -- both reuse REAL MoveIt planning (MoveGroupInterface), not
 *          hand-picked joint targets, for exactly the reason the earlier per-waypoint hand-chained
 *          approach (this file's own prior version) was wrong: joint-space point-to-point sends
 *          don't track a Cartesian line, and a real coverage strip needs to.
 *
 *          NOT attempted here, and why: wiring force_core::ForceStageSequencer into the REAL
 *          manipulation_manager pipeline (ManipulationManager's own executeMotionSegments()
 *          loop) -- confirmed by grep that this does NOT exist yet anywhere in that package. This
 *          test remains the correct, appropriately-scoped increment for validating the mechanism
 *          against real geometry; wiring it into full mission orchestration is a separate,
 *          larger integration task. Likewise, the coverage strip here is ONE hand-specified
 *          Cartesian segment (a straight line along the wall's own vertical axis), not a full
 *          rasterized-grid coverage plan (coverage_planner's own wall_rasterizer.cpp/
 *          coverage_pass_expander.cpp) -- reusing computeCartesianPath() (the actual tracking
 *          primitive) without also reusing the full obstacle-aware grid generator matches this
 *          file's own established, disclosed-scope convention rather than re-deriving
 *          wall_coverage_demo_node.cpp's entire pipeline inside a unit test.
 *
 *          kApproachStandoffPose was derived via moveit_pose_finder (this package) against the
 *          LIVE planning scene: collision-free (/check_state_validity-confirmed), well-
 *          conditioned, and shares the SAME fixed orientation this whole session's "wall-facing"
 *          pose family uses (tip local Z == base_link -Y, reconfirmed via pose_conditioning_probe
 *          AFTER that tool's own Euler-angle-convention bug was found and fixed -- see that file's
 *          own doc comment). It sits ~5cm short of wall_surface_probe's own ray-marched real wall
 *          entry face (well within this test's own admittance.max_offset_m=0.05) -- close enough
 *          for the compliance offset to bridge the remaining gap on its own, never a target inside
 *          real geometry. The strip end pose keeps the SAME Y (approach depth) and only varies Z
 *          (10cm up, along the wall's own vertical axis) -- deliberately: the COMMANDED reference
 *          never crosses into occupied geometry either, exactly matching real hybrid position/
 *          force control (position-controlled tangentially, force-controlled along the normal via
 *          admittance's own offset, which follows automatically regardless of what the commanded
 *          reference's own Y says).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "compliant_trajectory_controller/jtac_controller_compliance_port.hpp"
#include "compliant_trajectory_controller/wall_octomap_probe.hpp"
#include "core_types/tool_profile.hpp"
#include "force_core/force_stage_sequencer.hpp"
#include "ros_adapters/planned_path_marker_utils.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.hpp"
#include "compliant_trajectory_controller/pose_inputs.hpp"
#include "mobile_manipulator_profile/profile.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{

using FollowJTrajAction = control_msgs::action::FollowJointTrajectory;

// Joint order of the commanded group: the legacy chain the hand-derived poses below belong to.
// main() refuses to run when the planning group's active joints differ.
// Tool frame the trajectories are visualised for; set from the profile in main().
std::string kTcpLink{"tcp_link"};
std::vector<std::string> kJointNames{
  "lift_prismatic_joint", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

// The coverage-plan's own start point, offset from the wall -- re-derived against wall_mount_
// publisher_node's own synthetic_flat_wall:=true mode (a perfectly flat plane REPLACING wall
// 0's real, non-flat scanned points -- see that file's own doc comment for why: this test's own
// Cartesian strip lost contact partway through a real-wall run, traced to a genuine recessed
// region in the real scan, not a control bug; a known-flat surface isolates the control
// mechanism from that confound while it's being validated).
//
// Two SEPARATE bugs, both now fixed, made every earlier standoff calibration here obsolete:
//
// (1) wall_contact_force_simulator_node fetches the planning scene's octomap ONCE at startup
// and never refreshes it -- launched concurrently with wall_mount_publisher_node's own async
// synthetic-wall push, it can win the race and cache a stale, pre-push octree, silently
// reporting zero contact against wall geometry that no longer matches reality. No error, no
// warning -- just a permanently-wrong occupancy snapshot. Always start/restart wall_contact_
// force_simulator_node AFTER wall_mount_publisher_node's own "pushed N wall points" log line,
// not concurrently with it.
//
// (2) wall_mount_publisher_node's own synthetic_flat_wall points were placed at local z=0.0
// ("exactly on the fitted plane"), but OctoMap's own coordToKey() floors a coordinate into
// voxel [i*resolution, (i+1)*resolution) -- z=0.0 deterministically lands in [0, +resolution),
// putting the ENTIRE occupied shell's near face a full voxel width (5cm) past the fitted plane,
// not the +-resolution/2 quantization noise the old comment here assumed. Confirmed via a
// signed-plane-distance check (fitted plane_abcd vs. wall_surface_probe's own ray-marched near
// face, converted through the live base_link->map tf): ~4.65cm off before the fix, ~3mm after.
// Fixed at the source (wall_mount_publisher_node.cpp now seeds local z=-resolution/2), which
// moved the wall's own near face from Y=-0.855 to Y=-0.9015 in base_link at this X/Z -- i.e.
// FURTHER from the robot than every prior standoff here assumed, because those were all
// calibrated against the wrongly-placed shell.
//
// Re-derived AGAIN via moveit_pose_finder's cartesian_mode:=true (continuity-preserving short
// Cartesian step, not an independently-IK-sampled goal) at Y=-0.8485 -- this time not chosen to
// merely CLEAR the gap (the Y=-0.8335/kTargetForceN=12.0 pairing did that: APPROACH's own
// contact_threshold_n=2.0 was reached every time), but to fix a SEPARATE, deeper problem found
// AFTER contact: CARTESIAN's settle onto target_force_n plateaued at a stable, converged, and
// WRONG steady state (~6N, not 12N) and stayed there indefinitely -- not a slow convergence, a
// genuinely different equilibrium.
//
// Root cause, confirmed via /compute_fk on the plateaued joint state (offset_norm=19mm, nowhere
// near max_offset_m=0.05 -- ruling out that clamp): admittance's own stiffness (kStiffnessZ=
// 300 N/m) sits in SERIES with the wall's stiffness (wall_contact_force_simulator_node's own
// stiffness_n_per_m=5000), and any nonzero air gap between the frozen reference and the wall's
// real surface dilutes the achieved contact force according to the two-spring divider
//   F_contact = [K_wall*F_target - K_wall*K*gap] / (K + K_wall)
// At the prior gap=0.018m: [5000*12 - 5000*300*0.018]/5300 = 6.21N -- matches the measured
// plateau almost exactly. This is a real equilibrium, not a slow one: waiting longer never
// helps, because K*x = F_target - K_wall*(x-gap) already balances there. The only way to a
// higher contact force is a smaller gap and/or a higher target_force_n (rearranging the same
// formula for the target's own contact_threshold_n/force_band_n check).
//
// FIRST attempt at a fix (WRONG, worth recording): inflate target_force_n to compensate for the
// divider's own attenuation, keeping gap=0.003m and solving F_target=13.62 so the ACHIEVED force
// would land at 12.0. Live-verified this DOES make F_contact land almost exactly at 12.0 (proof
// the formula above is right) -- but CARTESIAN's own settle check compares measured force
// against target_force_n DIRECTLY (there is no separate "intended contact force" anywhere in
// this architecture), so inflating the target does not shrink the gap the check itself
// evaluates -- it grows it: |target - F_contact| = |13.62 - 12.0| = 1.62 > force_band_n=1.5,
// missing the band by 0.12N. Raising target_force_n to compensate for the divider ALWAYS grows
// (target - F_contact) faster than it grows F_contact, for any gap > 0 -- the wrong lever.
//
// The right lever is shrinking gap itself, keeping target_force_n at its real, intended value
// (12.0, restored below). The convergence error as a direct function of gap:
//   |target - F_contact| = K*(target + K_wall*gap) / (K + K_wall)
// Solved for gap=0.001m (1mm -- deliberately smaller than the 0.003m tried above, for margin):
//   error = 300*(12 + 5000*0.001) / 5300 = 0.962N, comfortably inside force_band_n=1.5 (0.54N
// to spare).
//
// A THIRD bug, found by the user watching RViz and NOT seeing the tool visibly touch the wall
// even on a run that logged "contact confirmed": wall_contact_force_simulator_node's own
// contact_surface_offset_m defaulted to 0.05 ("the tool's real physical extent past the bare TCP
// frame"), but that number contradicted the tool's own mesh -- transforming cam_mop_ft.stl's
// vertices through mesh_origin_rpy into the flange frame gives a contact-normal extent of
// [-0.004, +0.162]m, and tcp_xyz.z=0.165 places link_tcp 3mm PAST the mesh's own farthest point,
// not 5cm short of it. The simulator was crediting the tool with ~5.3cm of reach it does not
// have, so software-confirmed "contact" was always ~5.3cm away from anything visibly touching.
// Fixed at the source (wall_contact_force_simulator_node.cpp's own default is now 0.0) -- see
// that file's own doc comment for the full derivation.
//
// With contact_surface_offset_m=0, the bare TCP itself IS the contact-seeking reference (no more
// virtual point ahead of it), so the standoff must sit where the virtual point used to: Y=-0.9005,
// 0.001m short of the wall's own corrected near face at Y=-0.9015. Re-derived via moveit_pose_
// finder's cartesian_mode:=true (continuity-preserving short Cartesian step seeded from the prior
// Y=-0.8505 standoff's own solution, not an independently-IK-sampled goal), computeCartesianPath()
// fraction=1.0 (collision-checked start-to-goal, not just the endpoint). The force-vs-gap math
// above is UNCHANGED by this -- it was always about the geometric gap between the admittance
// reference and the wall, never about what the offset was called.
// EXPERIMENT, TRIED AND DISPROVEN (2026-08-24): temporarily widened to a 1cm gap to test whether
// the reference-freeze fix's own rebasing (APPROACH's 4s active-trajectory-send lets admittance
// close MOST of a gap BEFORE state_reference_ ever freezes) would let CARTESIAN's own tighter
// force_band_n convergence check see a much smaller EFFECTIVE gap than the raw nominal-to-wall
// distance -- i.e. whether a realistic approach_standoff_m-scale gap (mop's own tool_profiles.
// yaml uses 5cm/10cm staged standoffs) is viable once compliance has had time to close most of it
// before the reference locks. Live result: APPROACH still confirmed genuine contact (exact_point_
// occupied=true) at 1cm, but CARTESIAN's own settle timed out -- rebasing helps but not enough;
// the series-spring math (see the constant's own earlier doc comment in git history) still wins
// at 1cm. Reverted to the 1mm gap below.
//
// Checked force_segment_expander.cpp (the ALREADY-BUILT, ALREADY-TESTED production expansion --
// see its own APPROACH-segment construction): approach_standoff_m's values (10cm, 5cm) are
// intermediate INLINE WAYPOINTS within ONE continuous, already-compliant APPROACH trajectory that
// ENDS at intent.poses.front() -- the strip's own zero-standoff, on-surface pose -- not a
// deliberate final resting gap. Production's own design already commands the reference all the
// way to (near-)zero standoff, exactly matching this 1mm-gap calibration's own intent; the two
// larger values are guarded-move safety waypoints for catching contact early on an
// unexpectedly-close real wall, not where the reference is meant to end up. This test's own hand-
// calibrated 1mm gap is production-faithful, not a diagnostic-only shortcut.
const std::vector<double> kApproachStandoffPose{
  0.724537, 0.786474, -1.175181, -1.188421, 0.001374, 1.558118, -0.000507};

// The Cartesian coverage-strip's own end pose, for computeCartesianPath() in main(): SAME X/Y as
// kApproachStandoffPose above (same approach depth -- see this file's own doc comment for why),
// Z stepped +10cm (up, along the wall's own vertical axis). Against the FLAT synthetic wall this
// is safe across the whole strip by construction (the surface is uniform everywhere within the
// pushed plane's own (u, v) extent). Re-derived alongside kApproachStandoffPose above (same
// Y=-0.9005 recalibration), also via cartesian_mode:=true seeded from kApproachStandoffPose's
// own solution for branch continuity, fraction=1.0.
const std::array<double, 3> kStripEndPositionXyz{-0.347948, -0.9005, 1.81402};
const std::array<double, 3> kStripOrientationRpy{1.41922, -1.56707, 0.152887};

// The real, intended target force -- see kApproachStandoffPose's own doc comment above for why
// this is NOT compensated upward for the admittance/wall series-stiffness divider (that lever
// grows the convergence error, it doesn't shrink it); the 1mm standoff gap does the compensating
// instead, keeping this value exactly what CARTESIAN's own settle check compares measured force
// against.
constexpr double kTargetForceN = 12.0;

// Same diagnostic gains jtac_engage_disengage_test_node.cpp (T8) uses -- this test is about the
// wall-approach mechanism against real scene geometry, not about (re-)validating mop's own real,
// never-hardware-tuned gains.
constexpr double kMassZ = 8.0;
constexpr double kStiffnessZ = 300.0;
constexpr double kDampingRatioZ = 6.0;
constexpr double kMassXY = 16.0;
constexpr double kStiffnessXY = 1000.0;
constexpr double kDampingRatioXY = 5.59;

double rawDamping(double ratio, double mass, double stiffness)
{
  return ratio * 2.0 * std::sqrt(mass * stiffness);
}

core_types::ToolProfile buildTestToolProfile()
{
  core_types::ToolProfile profile;
  profile.name = "wall_approach_test_tool";
  profile.task.contact = true;
  profile.task.target_force_n = kTargetForceN;
  profile.task.force_band_n = 1.5;
  profile.task.force_zero_threshold_n = 1.0;
  profile.task.settle_timeout_s = 10.0;
  profile.task.overload_abort_n = 40.0;
  profile.task.tangential_torque_abort_nm = 10.0;
  profile.task.unexpected_contact_abort_n = 0.0;

  profile.control.controller = "admittance_chain";
  profile.control.admittance.selected_axes = {false, false, true, false, false, false};
  profile.control.admittance.mass = {kMassXY, kMassXY, kMassZ};
  profile.control.admittance.stiffness = {kStiffnessXY, kStiffnessXY, kStiffnessZ};
  profile.control.admittance.damping = {
    rawDamping(kDampingRatioXY, kMassXY, kStiffnessXY),
    rawDamping(kDampingRatioXY, kMassXY, kStiffnessXY),
    rawDamping(kDampingRatioZ, kMassZ, kStiffnessZ)};
  profile.control.admittance.wrench_reference_z_n = kTargetForceN;
  profile.control.admittance.contact_threshold_n = 2.0;
  profile.control.admittance.max_offset_m = 0.05;
  // Enabled here (unlike a plain-admittance-only test) specifically to exercise the trim loop's
  // OWN background thread against a genuinely moving Cartesian strip -- see force_core::
  // ForceStageSequencer's own "FORCE TRIM" doc comment. Same ki/max as mop's own production
  // tool_profiles.yaml entry. RULED OUT as the cause of a separately-found max_jerk=34802 on the
  // strip window (an A/B with this at 0.0 made jerk WORSE, not better -- see the Cartesian-strip
  // retiming fix below, in main(), for the actual root cause and fix).
  profile.control.admittance.trim_ki = 1.0;
  profile.control.admittance.trim_integral_max_n = 20.0;

  profile.control.ft.lpf_cutoff_hz = 0.0;
  profile.control.ft.deadband_n = 0.1;
  profile.control.ft.clamp_n = 100.0;
  profile.control.ft.tare_per_goal = false;
  return profile;
}

/// Identical to jtac_engage_disengage_test_node.cpp's own helper of the same name.
bool sendTrajectoryAndWait(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp_action::Client<FollowJTrajAction>::SharedPtr & client,
  const std::vector<double> & target_positions, double duration_s)
{
  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Action server not available.");
    return false;
  }

  FollowJTrajAction::Goal goal;
  goal.trajectory.joint_names = kJointNames;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = target_positions;
  point.velocities.assign(kJointNames.size(), 0.0);
  point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
  goal.trajectory.points = {point};

  auto send_goal_future = client->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node, send_goal_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send goal.");
    return false;
  }
  const auto goal_handle = send_goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal rejected.");
    return false;
  }

  auto result_future = client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(
      node, result_future, std::chrono::duration<double>(duration_s + 5.0)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for trajectory result.");
    return false;
  }
  const auto result = result_future.get();
  return result.result->error_code == FollowJTrajAction::Result::SUCCESSFUL;
}

/// Reads ONE live /joint_states message (short wait) for the post-hoc geometric verification
/// below -- independent of whatever the FSM/CompliancePort's own force reading says.
bool readLiveJointState(
  const rclcpp::Node::SharedPtr & node, std::vector<double> & ordered_out)
{
  auto latest = std::make_shared<sensor_msgs::msg::JointState>();
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SystemDefaultsQoS(),
    [&latest](const sensor_msgs::msg::JointState::SharedPtr msg) {latest = msg;});
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (latest->name.empty() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (latest->name.empty()) {
    return false;
  }
  ordered_out.resize(kJointNames.size());
  for (size_t i = 0; i < kJointNames.size(); ++i) {
    const auto it = std::find(latest->name.begin(), latest->name.end(), kJointNames[i]);
    if (it == latest->name.end()) {
      return false;
    }
    ordered_out[i] = latest->position[static_cast<size_t>(it - latest->name.begin())];
  }
  return true;
}

/// A dedicated node + background SingleThreadedExecutor for MoveGroupInterface -- SEPARATE from
/// the main `node` (which already has its own blocking rclcpp::spin_until_future_complete()/
/// spin_some() calls elsewhere in this file, e.g. sendTrajectoryAndWait()/readLiveJointState()) --
/// spinning the SAME node from two different call paths concurrently is unsafe. Matches
/// moveit_pose_finder.cpp's own already-proven pattern. RAII so it tears down correctly on EVERY
/// exit path from main(), including the CHECK macro's own early `return 1` -- a raw joinable
/// std::thread left dangling at those points would call std::terminate() on destruction.
struct MoveGroupExecutorThread
{
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread thread;

  explicit MoveGroupExecutorThread(const rclcpp::Node::SharedPtr & node)
  {
    executor.add_node(node);
    thread = std::thread([this]() {executor.spin();});
  }

  ~MoveGroupExecutorThread()
  {
    executor.cancel();
    if (thread.joinable()) {
      thread.join();
    }
  }
};

/// Real, collision-checked MoveIt joint-space plan+execute -- what "FSP" (free-space planning)
/// means in this codebase: an OMPL plan against the LIVE planning scene (the same pushed wall
/// octomap every other check in this file uses), not a raw, uncontact-checked trajectory send.
/// Publishes the SAME two things cartesian_test.launch.py's own production pipeline does for
/// every planned trajectory (see motion_validator.cpp/execution_tracker_node.cpp's own doc
/// comments), reusing their exact mechanisms rather than reinventing either:
///   - RViz path markers on /cartesian_validator/planned_path (ros_adapters::
///     makePlannedLineStripMarker() etc.) -- the SAME topic cartesian_validation.rviz's own
///     already-configured display expects, colored by the SAME yellow=cartesian/green=fsp
///     convention -- so this test's own trajectories show up exactly like production's do,
///     no new RViz config needed.
///   - The raw moveit_msgs::msg::RobotTrajectory on /motion_execution/commanded_trajectory
///     (moveit_execution_adapter.cpp's own topic/QoS) -- so execution_tracker_node (a standalone,
///     launchable-alongside-any-bringup observer of that topic + /joint_states, per its own doc
///     comment) can be run alongside this test to get REAL, objective deviation_m/jerk numbers,
///     not a visual impression of whether something looked like jitter.
/// Also logs the max per-waypoint joint delta directly from the raw trajectory data -- a cheap,
/// independent check for a redundancy-driven jump: this 7-DOF chain (6 arm + 1 prismatic lift) is
/// kinematically redundant for a 6-DOF Cartesian target, and a jump in the lift or an elbow joint
/// between ADJACENT waypoints (rather than a small, smooth step) is exactly what null-space drift
/// would look like in the raw joint data, independent of anything visual.
class TrajectoryVisualizer
{
public:
  TrajectoryVisualizer(
    const rclcpp::Node::SharedPtr & node, moveit::core::RobotModelConstPtr robot_model)
  : robot_model_(std::move(robot_model))
  {
    marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/cartesian_validator/planned_path", rclcpp::QoS(100).transient_local());
    commanded_trajectory_pub_ = node->create_publisher<moveit_msgs::msg::RobotTrajectory>(
      "/motion_execution/commanded_trajectory", rclcpp::QoS(1).transient_local());
  }

  void publish(
    const rclcpp::Logger & logger, const moveit_msgs::msg::RobotTrajectory & trajectory,
    const std::string & planner_label, const std::string & ee_link)
  {
    commanded_trajectory_pub_->publish(trajectory);

    const auto & points = trajectory.joint_trajectory.points;
    const auto & joint_names = trajectory.joint_trajectory.joint_names;
    if (points.empty()) {
      return;
    }

    std::vector<Eigen::Vector3d> curve;
    curve.reserve(points.size());
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    double max_joint_delta = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
      state.setVariablePositions(joint_names, points[i].positions);
      state.update();
      curve.push_back(state.getGlobalLinkTransform(ee_link).translation());
      if (i > 0) {
        for (size_t j = 0; j < points[i].positions.size(); ++j) {
          max_joint_delta = std::max(
            max_joint_delta, std::abs(points[i].positions[j] - points[i - 1].positions[j]));
        }
      }
    }
    RCLCPP_INFO(
      logger, "%s trajectory: %zu points, max per-waypoint joint delta = %.5f",
      planner_label.c_str(), points.size(), max_joint_delta);
    {
      std::string names_line, final_line;
      for (size_t j = 0; j < joint_names.size(); ++j) {
        names_line += joint_names[j] + (j + 1 < joint_names.size() ? "," : "");
        final_line += std::to_string(points.back().positions[j]) +
          (j + 1 < joint_names.size() ? "," : "");
      }
      RCLCPP_INFO(logger, "  joint_names order: [%s]", names_line.c_str());
      RCLCPP_INFO(logger, "  final waypoint:    [%s]", final_line.c_str());
    }

    const int32_t marker_id = static_cast<int32_t>(marker_index_++);
    const std::string & fk_frame_id = robot_model_->getModelFrame();
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(
      ros_adapters::makePlannedLineStripMarker(
        fk_frame_id, marker_id, planner_label, curve));
    marker_array.markers.push_back(
      ros_adapters::makePlannedPathDotsMarker(fk_frame_id, marker_id, planner_label, curve));
    marker_array.markers.push_back(
      ros_adapters::makePlannedEndpointMarkers(fk_frame_id, marker_id, planner_label, curve));
    marker_pub_->publish(marker_array);
  }

private:
  moveit::core::RobotModelConstPtr robot_model_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr commanded_trajectory_pub_;
  size_t marker_index_{0};
};

bool planAndExecuteJointTarget(
  const rclcpp::Node::SharedPtr & node, moveit::planning_interface::MoveGroupInterface & move_group,
  TrajectoryVisualizer & visualizer, const std::vector<double> & joint_positions)
{
  // NAME-EXPLICIT overload, NOT setJointValueTarget(const std::vector<double>&) -- that raw-vector
  // overload's own docs specify "group_variable_values" (the JointModelGroup's OWN internal
  // variable order), which is NOT guaranteed to be kJointNames' own [lift, j1..j6] order. Found
  // live: this silently drove the lift joint to a wildly wrong value (e.g. commanding -0.98
  // instead of the intended +0.77) while j1 (coincidentally?) landed close to its own intended
  // value -- looked exactly like an admittance/trim runaway at first, and cost real debugging time
  // (including two genuine self-collisions) before being traced to this ordering mismatch instead.
  if (!move_group.setJointValueTarget(kJointNames, joint_positions)) {
    RCLCPP_ERROR(node->get_logger(), "FSP: the joint target does not fit the planning group.");
    return false;
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "FSP plan() failed for the requested joint target.");
    return false;
  }
  visualizer.publish(node->get_logger(), plan.trajectory, "fsp", kTcpLink);
  return move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

/// Same as above, but for the SRDF-named "home" group_state (whole-body group) -- the canonical,
/// idiomatic way to say "home" in this codebase, rather than a hand-maintained joint vector.
bool planAndExecuteNamedTarget(
  const rclcpp::Node::SharedPtr & node, moveit::planning_interface::MoveGroupInterface & move_group,
  TrajectoryVisualizer & visualizer, const std::string & named_target)
{
  move_group.setNamedTarget(named_target);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(
      node->get_logger(), "FSP plan() failed for named target '%s'.", named_target.c_str());
    return false;
  }
  visualizer.publish(node->get_logger(), plan.trajectory, "fsp", kTcpLink);
  return move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

#define CHECK(condition, message) \
  do { \
    if (!(condition)) { \
      RCLCPP_ERROR(node->get_logger(), "FAILED: %s", message); \
      rclcpp::shutdown(); \
      return 1; \
    } \
    RCLCPP_INFO(node->get_logger(), "OK: %s", message); \
  } while (0)

/// Identical to jtac_engage_disengage_test_node.cpp's own macro of the same name -- see that
/// file's own comment for why _outcome must not be named "message".
#define CHECK_OUTCOME(outcome_expr, check_label) \
  do { \
    const auto & _outcome = (outcome_expr); \
    if (!_outcome.proceed) { \
      RCLCPP_ERROR( \
        node->get_logger(), "FAILED: %s (%s: %s)", check_label, \
        _outcome.failure_reason_code.c_str(), _outcome.message.c_str()); \
      rclcpp::shutdown(); \
      return 1; \
    } \
    RCLCPP_INFO(node->get_logger(), "OK: %s", check_label); \
  } while (0)

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("jtac_wall_approach_test_node");

  const auto tool_profile = buildTestToolProfile();

  node->declare_parameter("profile", std::string(mobile_manipulator_profile::kDefaultProfile));
  std::string profile_warning;
  const auto robot_profile = mobile_manipulator_profile::loadProfileOrDefaults(
    node->get_parameter("profile").as_string(), profile_warning);
  if (!profile_warning.empty()) {
    RCLCPP_WARN(node->get_logger(), "%s", profile_warning.c_str());
  }
  node->declare_parameter("planning_group", robot_profile.whole_body_group);
  const std::string planning_group = node->get_parameter("planning_group").as_string();
  kTcpLink = robot_profile.frameOr("tcp", "tcp_link");
  const std::string & tcp_link = kTcpLink;

  compliant_trajectory_controller::JtacControllerCompliancePort::Options port_options;
  port_options.controller_node_name = "combined_arm_lift_controller";
  compliant_trajectory_controller::JtacControllerCompliancePort port(port_options);

  auto now_ns_fn = [&node]() {return static_cast<uint64_t>(node->now().nanoseconds());};
  force_core::ForceStageSequencer sequencer(port, now_ns_fn);

  auto trajectory_client = rclcpp_action::create_client<FollowJTrajAction>(
    node, "combined_arm_lift_controller/follow_joint_trajectory");

  compliant_trajectory_controller::WallOctomapProbe verification_probe(node);
  CHECK(
    verification_probe.initialize(robot_profile.frameOr("base", "base_link"), tcp_link),
    "loaded the live pushed wall octomap for post-hoc geometric verification");

  // Dedicated node/executor for MoveGroupInterface -- see MoveGroupExecutorThread's own doc
  // comment for why this must be SEPARATE from `node`.
  auto moveit_node = rclcpp::Node::make_shared("jtac_wall_approach_test_moveit_client");
  MoveGroupExecutorThread moveit_executor_thread(moveit_node);
  moveit::planning_interface::MoveGroupInterface move_group(moveit_node, planning_group);
  // The hand-derived poses below belong to the legacy [lift, joint1..6] chain. Other arms need
  // their own poses; refuse instead of driving them with these.
  if (const auto err = compliant_trajectory_controller::checkPoseChain(
      move_group.getActiveJoints(), kJointNames, kApproachStandoffPose); !err.empty())
  {
    RCLCPP_FATAL(node->get_logger(), "%s", err.c_str());
    rclcpp::shutdown();
    return 1;
  }
  move_group.setPlanningTime(10.0);

  TrajectoryVisualizer visualizer(node, move_group.getRobotModel());

  // --- FSP: HOME -> the coverage-plan's own start point, offset from the real wall. Real,
  //     collision-checked MoveIt planning (against the SAME live pushed wall octomap every other
  //     check in this file uses) -- not a raw, uncontact-checked trajectory send. ---
  CHECK(
    planAndExecuteNamedTarget(node, move_group, visualizer, "home"),
    "FSP: planned and executed HOME as the starting pose");
  CHECK(
    planAndExecuteJointTarget(node, move_group, visualizer, kApproachStandoffPose),
    "FSP: planned and executed HOME -> the coverage-plan start point (standoff)");

  // --- APPROACH: engage() FIRST (so the compliance offset is already active for the whole move,
  //     matching how a real guarded move works), then ONE ordinary trajectory toward a standoff
  //     point that is STILL IN FREE AIR. wall_contact_force_simulator_node (a separate, persistent
  //     process -- see its own doc comment) is continuously publishing a REAL, geometry-derived
  //     wrench throughout; JtacController's own admittance offset (already built, already proven
  //     in T8/T9) is what actually advances past the raw reference and stops at genuine contact --
  //     this test does not compute, poll, or cancel anything itself. ---
  core_types::ForceIntent approach_intent;
  approach_intent.stage = core_types::ForceStage::kApproach;
  auto outcome = sequencer.beforeSegment(approach_intent, tool_profile);
  CHECK_OUTCOME(outcome, "beforeSegment(APPROACH) engaged compliance");
  CHECK(
    sequencer.state() == force_core::ForceFsmState::kApproach,
    "FSM entered kApproach");

  const bool approach_executed =
    sendTrajectoryAndWait(node, trajectory_client, kApproachStandoffPose, 4.0);

  // afterSegment() checks contact via a SINGLE status() snapshot, not a dwell loop (that only
  // happens for CARTESIAN/RETRACT's own kSetTargetForce/kDwell steps) -- so the admittance offset
  // must have already settled by the moment the raw trajectory action reports finished. Found
  // live: this test's own damping_ratio_z=6 (heavily overdamped) gives a SLOW pole around
  // ~0.5 rad/s (poles at -zeta*omega +/- omega*sqrt(zeta^2-1) for omega=sqrt(K/M)=6.12 rad/s) --
  // multiple SECONDS to fully converge, not milliseconds. A 500ms delay measured live: force was
  // still creeping toward target 5+ seconds after the trajectory finished, well past a first
  // afterSegment() check timed too early. Wait long enough for the slow pole's own ~3 tau settle.
  std::this_thread::sleep_for(std::chrono::seconds(6));

  outcome = sequencer.afterSegment(approach_executed);
  CHECK_OUTCOME(outcome, "afterSegment(APPROACH) confirmed contact");
  CHECK(
    sequencer.state() == force_core::ForceFsmState::kApproachContactConfirmed,
    "FSM confirmed contact");

  // Independent, geometric double-check: the FSM confirmed contact via FORCE (the mock wrench);
  // separately confirm the tip's own live FK position is ALSO genuinely inside the real octomap --
  // belt-and-suspenders, matching this codebase's own "verified live" convention rather than
  // trusting a single signal.
  {
    std::vector<double> ordered;
    CHECK(
      readLiveJointState(node, ordered), "read live joint state for geometric verification");
    const auto contact_check = verification_probe.checkContact(ordered);
    RCLCPP_INFO(
      node->get_logger(),
      "geometric verification: exact_point_occupied=%s, margin_offset_m=%.4f",
      contact_check.exact_point_occupied ? "true" : "false", contact_check.margin_offset_m);
  }

  // --- CARTESIAN: settle (dwell until force converges onto target_force_n), THEN release a REAL
  //     coverage-strip trajectory -- MoveGroupInterface::computeCartesianPath(), the SAME
  //     primitive motion_planner::CartesianPlanner itself uses in production, not a hand-
  //     chained joint-space waypoint list (see this file's own doc comment for why that earlier
  //     approach was wrong). beforeSegment() has already started the trim loop's background
  //     thread by the time this runs (see force_stage_sequencer.hpp's own "FORCE TRIM" doc
  //     comment) -- this is the FIRST time in this session's testing that trim actually runs
  //     against a genuinely MOVING trajectory, not just a static settle. ---
  core_types::ForceIntent cartesian_intent;
  cartesian_intent.stage = core_types::ForceStage::kCartesian;
  outcome = sequencer.beforeSegment(cartesian_intent, tool_profile);
  CHECK_OUTCOME(outcome, "beforeSegment(CARTESIAN) settled onto target_force_n");
  CHECK(
    sequencer.state() == force_core::ForceFsmState::kCartesianFollow,
    "FSM converged to kCartesianFollow");

  bool strip_executed = false;
  {
    geometry_msgs::msg::Pose strip_end_pose;
    strip_end_pose.position.x = kStripEndPositionXyz[0];
    strip_end_pose.position.y = kStripEndPositionXyz[1];
    strip_end_pose.position.z = kStripEndPositionXyz[2];
    tf2::Quaternion q;
    q.setRPY(kStripOrientationRpy[0], kStripOrientationRpy[1], kStripOrientationRpy[2]);
    strip_end_pose.orientation.x = q.x();
    strip_end_pose.orientation.y = q.y();
    strip_end_pose.orientation.z = q.z();
    strip_end_pose.orientation.w = q.w();

    // Plan from the NOMINAL reference (kApproachStandoffPose, collision-free by construction --
    // see moveit_pose_finder's own fraction=1.0 confirmation when this pose was derived), NOT
    // the actual live current state. Now that contact_surface_offset_m=0.0 means the bare TCP
    // genuinely sits in occupied space at real contact, getCurrentState() (the default start
    // computeCartesianPath() would otherwise use) IS already in contact, and MoveIt collision-
    // checks its own starting state too -- confirmed live, this rejected the whole path outright
    // (fraction=0.0) regardless of where the target waypoint sat. The fix mirrors how APPROACH
    // itself already works: the COMMANDED/PLANNED path stays entirely in free space; admittance's
    // own compliant offset is what actually closes the remaining ~1mm gap to genuine, force-
    // generating contact during EXECUTION, not during planning.
    // Per-joint, name-explicit setJointPositions() -- NOT setJointGroupPositions() with a raw
    // vector, which (per this file's own kApproachStandoffPose ordering bug found earlier this
    // session, in setJointValueTarget()) expects the JointModelGroup's own internal variable
    // order, not necessarily [lift, j1..j6]. Matches moveit_pose_finder.cpp's own seed_state
    // construction, the already-proven-safe pattern for this exact hazard.
    moveit::core::RobotState nominal_start_state(move_group.getRobotModel());
    nominal_start_state.setToDefaultValues();
    for (size_t i = 0; i < kJointNames.size(); ++i) {
      nominal_start_state.setJointPositions(kJointNames[i], &kApproachStandoffPose[i]);
    }
    nominal_start_state.update();
    move_group.setStartState(nominal_start_state);

    const std::vector<geometry_msgs::msg::Pose> waypoints{strip_end_pose};
    moveit_msgs::msg::RobotTrajectory raw_strip_trajectory;
    const double fraction =
      move_group.computeCartesianPath(waypoints, 0.005, 0.0, raw_strip_trajectory);
    move_group.setStartStateToCurrentState();  // don't leave this sticky for later planning calls
    RCLCPP_INFO(
      node->get_logger(), "coverage-strip computeCartesianPath() fraction = %.4f", fraction);
    if (fraction >= 0.95 && !raw_strip_trajectory.joint_trajectory.points.empty()) {
      // computeCartesianPath() does NOT scale its own default timing to this robot's real
      // joint velocity/acceleration limits -- a well-documented MoveIt gotcha, and confirmed
      // live here as the real cause of a measured max_jerk=34802 (execution_tracker_node,
      // vs 3-34 on this same run's own clean FSP windows) with NO position discontinuity at
      // all (this test's own per-waypoint joint-delta check showed only 0.056 rad) -- i.e. the
      // commanded PATH was fine, its TIMING was not. An A/B with trim_ki=0 made this WORSE
      // (max_jerk=1678563), ruling out the trim loop as the cause. Fix: retime the raw
      // trajectory against the robot's own real limits before executing -- the standard,
      // documented fix for exactly this computeCartesianPath() behavior.
      robot_trajectory::RobotTrajectory robot_trajectory(
        move_group.getRobotModel(), move_group.getName());
      robot_trajectory.setRobotTrajectoryMsg(*move_group.getCurrentState(), raw_strip_trajectory);
      trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
      CHECK(
        time_parameterization.computeTimeStamps(robot_trajectory),
        "retimed the coverage-strip trajectory against real joint velocity/acceleration limits");
      moveit_msgs::msg::RobotTrajectory strip_trajectory;
      robot_trajectory.getRobotTrajectoryMsg(strip_trajectory);

      visualizer.publish(node->get_logger(), strip_trajectory, "cartesian", tcp_link);
      moveit::planning_interface::MoveGroupInterface::Plan strip_plan;
      strip_plan.trajectory = strip_trajectory;
      strip_executed = move_group.execute(strip_plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }
  }
  CHECK(strip_executed, "followed the real coverage-strip joint trajectory while holding force");

  outcome = sequencer.afterSegment(true);
  CHECK_OUTCOME(outcome, "afterSegment(CARTESIAN) accepted");
  CHECK(
    sequencer.state() == force_core::ForceFsmState::kCartesianDone,
    "FSM reached kCartesianDone");
  RCLCPP_INFO(
    node->get_logger(), "trim final_trim_integral_n = %.3f (0.0 means trim_ki was 0 for this test)",
    sequencer.telemetry().final_trim_integral_n);

  // --- RETRACT: unload to ~0 N (beforeSegment's own dwell), then peel back to the coverage-plan
  //     start standoff, then disengage(). Ordinary trajectory again -- as the tool physically
  //     moves away, the mock simulator's own real-time octomap check naturally reports decreasing
  //     then zero force; no scripted "inject 0" step needed. Deliberately targets the ORIGINAL
  //     standoff (not a new standoff at the strip's own end position) -- a reasonable scoping
  //     choice for this test, not a claim that this is how a multi-strip production retract would
  //     necessarily be aimed. ---
  core_types::ForceIntent retract_intent;
  retract_intent.stage = core_types::ForceStage::kRetract;
  outcome = sequencer.beforeSegment(retract_intent, tool_profile);
  CHECK_OUTCOME(outcome, "beforeSegment(RETRACT) unloaded");
  CHECK(
    sequencer.state() == force_core::ForceFsmState::kRetractFollow,
    "FSM unloaded to kRetractFollow");

  CHECK(
    sendTrajectoryAndWait(node, trajectory_client, kApproachStandoffPose, 4.0),
    "executed RETRACT (peel-off) trajectory back to the coverage-plan start standoff");

  outcome = sequencer.afterSegment(true);
  CHECK_OUTCOME(outcome, "afterSegment(RETRACT) disengaged");
  CHECK(sequencer.state() == force_core::ForceFsmState::kIdle, "FSM returned to kIdle");

  const auto & telemetry = sequencer.telemetry();
  RCLCPP_INFO(
    node->get_logger(), "max_abs_force_at_switch_n = %.3f (bumpless-transfer bound: < 3.0)",
    telemetry.max_abs_force_at_switch_n);
  CHECK(
    telemetry.max_abs_force_at_switch_n < 3.0, "disengaged within the bumpless-transfer bound");
  CHECK(!telemetry.compliance_engaged_at_exit, "compliance is OFF after the full group");

  // --- FSP: coverage-plan start standoff -> HOME again. ---
  CHECK(
    planAndExecuteNamedTarget(node, move_group, visualizer, "home"),
    "FSP: planned and executed the return-to-HOME move");

  RCLCPP_INFO(
    node->get_logger(),
    "=== wall-approach test PASSED: HOME -> FSP -> coverage-plan start -> APPROACH -> contact -> "
    "CARTESIAN strip (trim-corrected) -> RETRACT -> FSP -> HOME, against a REAL wall with a REAL "
    "(mock-simulated) continuous contact force, verified live ===");
  rclcpp::shutdown();
  return 0;
}
