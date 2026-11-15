/**
 * @file wall_contact_force_simulator_node.cpp
 * @brief Persistent MOCK contact-force sensor: continuously reports a physically-plausible wrench
 *        derived from REAL penetration into the live pushed wall octomap, replacing the one-shot
 *        scripted force injection jtac_wall_approach_test_node.cpp and the engage test used until now.
 * @details This is the wall-contact-model gap wall-approach testing kept running into: mock has
 *          no contact-dynamics model at all, so nothing at the CONTROLLER level (JtacController's
 *          own admittance loop, already built and already proven in the engage and wall-approach tests) can genuinely "move
 *          until contact" -- there was no real-time force for it to react to. Every workaround
 *          tried before this (hand-derived through-wall poses, waypoint chains, per-step
 *          cancel-on-octomap-contact logic in the TEST node) was compensating for that missing
 *          piece from the wrong layer. This node fixes the actual gap instead:
 *
 *            - free air (tip's own FK point not inside any occupied octomap voxel): publishes
 *              ZERO force. The "seek toward the wall" behavior is NOT faked here -- that is
 *              JtacController's own job, driven by ForceIntent's target_force_n bias
 *              (see jtac_controller_compliance_port.hpp's own doc comment on how that bias is
 *              applied). This node's only job is to report the RAW, UNBIASED measured force a
 *              real F/T sensor would see, exactly like the real one will on hardware.
 *            - in contact (tip's own FK point IS inside an occupied voxel): computes penetration
 *              depth by marching backward along the tool's own local Z axis (the approach normal,
 *              taken from live FK's own rotation matrix -- NOT a hardcoded axis, so this works for
 *              any tool orientation, not just this session's fixed -Y-in-base-link family) until
 *              reaching free space again (WallOctomapProbe::computeSignedPenetrationM()), then
 *              applies a simple spring model force_n = stiffness_n_per_m * penetration_m, clamped
 *              to max_force_n as a safety bound against a runaway/anomalous reading.
 *
 *          Publishes geometry_msgs::msg::WrenchStamped on the SAME topic jtac_wall_approach_test_
 *          node.cpp's own injectWrenchFor() already used (frame_id="link_tcp", force.z = -force_n
 *          -- this package's own established sign convention: normalForceN(w) = -w.force_z,
 *          positive while loaded), at rate_hz, continuously, for as long as this node runs --
 *          matching wall_mount_publisher_node's own "persistent node, not a one-shot script"
 *          pattern (a one-shot process here would silently stop feeding the controller the
 *          instant it exited, exactly the TF-lifecycle bug wall_mount_publisher_node avoids:
 *          a one-shot process loses what it published the moment it exits).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "compliant_trajectory_controller/wall_octomap_probe.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{
const std::vector<std::string> kJointNames{
  "lift_prismatic_joint", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("wall_contact_force_simulator_node");

  node->declare_parameter("kinematics_base", std::string("base_link"));
  node->declare_parameter("kinematics_tip", std::string("link_tcp"));
  node->declare_parameter("wrench_topic", std::string(
      "/ft_bridge_hardware_uf_ft_sensor/injected_wrench"));
  // A rigid-wall-scale spring, not a soft one -- deliberately gives a SHALLOW equilibrium
  // penetration for a realistic target force (e.g. 6N / 5000 N/m = 1.2mm), matching what a
  // genuinely stiff wall would do, not an arbitrarily squishy mock surface.
  node->declare_parameter("stiffness_n_per_m", 5000.0);
  node->declare_parameter("max_force_n", 50.0);
  node->declare_parameter("rate_hz", 100.0);
  // How far the tool's REAL physical contact surface sits past the bare TCP frame origin, along
  // the approach normal. WAS 0.05 ("bisecting /check_state_validity against the live scene"),
  // but that value contradicted the tool's own mesh: transforming cam_mop_ft.stl's own vertices
  // through mesh_origin_rpy=[3.14,-1.57,0] (tool_profiles.yaml's own mop.geometry) into the
  // flange frame gives a contact-normal (Z) extent of [-0.004, +0.162]m -- the mesh's farthest
  // point is at Z=0.162, and tcp_xyz.z=0.165 places link_tcp 3mm PAST that, not 5cm short of it.
  // The mesh does not reach past link_tcp at all; the prior 0.05 credited the simulated tool with
  // ~5.3cm of reach it does not have, so "contact confirmed" in software (a virtual point 5cm
  // past the bare TCP touching the wall) never matched a visibly touching mesh in RViz -- caught
  // live by the user watching the same run this file's own test node was validated against.
  node->declare_parameter("contact_surface_offset_m", 0.0);

  const auto kinematics_base = node->get_parameter("kinematics_base").as_string();
  const auto kinematics_tip = node->get_parameter("kinematics_tip").as_string();
  const auto wrench_topic = node->get_parameter("wrench_topic").as_string();
  const auto stiffness_n_per_m = node->get_parameter("stiffness_n_per_m").as_double();
  const auto max_force_n = node->get_parameter("max_force_n").as_double();
  const auto rate_hz = node->get_parameter("rate_hz").as_double();
  const auto contact_surface_offset_m = node->get_parameter("contact_surface_offset_m").as_double();

  compliant_trajectory_controller::WallOctomapProbe probe(node);
  if (!probe.initialize(kinematics_base, kinematics_tip)) {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize WallOctomapProbe. Exiting.");
    rclcpp::shutdown();
    return 1;
  }

  auto latest_joint_state = std::make_shared<sensor_msgs::msg::JointState>();
  std::mutex joint_state_mutex;
  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SystemDefaultsQoS(),
    [&latest_joint_state, &joint_state_mutex](const sensor_msgs::msg::JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(joint_state_mutex);
      latest_joint_state = msg;
    });

  auto wrench_pub = node->create_publisher<geometry_msgs::msg::WrenchStamped>(
    wrench_topic, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    node->get_logger(),
    "wall_contact_force_simulator_node: publishing on '%s' at %.1fHz -- stiffness=%.1fN/m, "
    "max_force=%.1fN, contact_surface_offset_m=%.3f, spinning indefinitely.",
    wrench_topic.c_str(), rate_hz, stiffness_n_per_m, max_force_n, contact_surface_offset_m);

  rclcpp::Rate rate(rate_hz);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    sensor_msgs::msg::JointState::SharedPtr state_copy;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex);
      state_copy = latest_joint_state;
    }
    if (!state_copy->name.empty()) {
      std::vector<double> ordered(kJointNames.size());
      bool have_all = true;
      for (size_t i = 0; i < kJointNames.size(); ++i) {
        const auto it = std::find(state_copy->name.begin(), state_copy->name.end(), kJointNames[i]);
        if (it == state_copy->name.end()) {
          have_all = false;
          break;
        }
        ordered[i] = state_copy->position[static_cast<size_t>(it - state_copy->name.begin())];
      }
      if (have_all) {
        Eigen::Isometry3d tip_in_base;
        if (probe.calculateTipTransform(ordered, tip_in_base)) {
          const Eigen::Vector3d approach_normal_in_base = tip_in_base.rotation() * Eigen::Vector3d::UnitZ();
          const double penetration_m = probe.computeSignedPenetrationM(
            ordered, approach_normal_in_base, contact_surface_offset_m);
          const double force_n = std::min(stiffness_n_per_m * penetration_m, max_force_n);

          geometry_msgs::msg::WrenchStamped msg;
          msg.header.stamp = node->now();
          msg.header.frame_id = kinematics_tip;
          msg.wrench.force.z = -force_n;  // this package's own convention: negative while loaded.
          wrench_pub->publish(msg);
        }
      }
    }
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
