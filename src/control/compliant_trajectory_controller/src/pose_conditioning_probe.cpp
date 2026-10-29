/**
 * @file pose_conditioning_probe.cpp
 * @brief Reusable CLI tool: reports a kinematic chain's Jacobian condition number at a given pose.
 * @details Standalone from AdmittanceRule -- this only needs a URDF and a KinematicsInterface, so
 *          it is useful any time a reference pose needs vetting for near-singularity before
 *          trusting a compliance experiment against it (T4a's own kReferencePose was chosen this
 *          way by hand after the all-zero pose collapsed the Jacobian pseudo-inverse during R5/R10
 *          mock testing; this tool makes that check repeatable instead of ad hoc).
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
std::string readFile(const std::string & path)
{
  std::ifstream file(path);
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pose_conditioning_probe");

  node->declare_parameter("robot_description_file", std::string(""));
  node->declare_parameter("tip", std::string("link6"));
  node->declare_parameter(
    "joint_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  node->declare_parameter(
    "joint_names", std::vector<std::string>{
    "lift_prismatic_joint", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});

  const auto urdf_path = node->get_parameter("robot_description_file").as_string();
  const auto tip = node->get_parameter("tip").as_string();
  const auto pose = node->get_parameter("joint_pose").as_double_array();

  if (urdf_path.empty()) {
    std::cerr << "Pass -p robot_description_file:=<path to a flat .urdf>\n";
    return 1;
  }
  const std::string urdf = readFile(urdf_path);
  if (urdf.empty()) {
    std::cerr << "Could not read '" << urdf_path << "'\n";
    return 1;
  }

  node->declare_parameter("robot_description", urdf);
  node->declare_parameter(
    "kinematics.plugin_name", std::string("kinematics_interface_kdl/KinematicsInterfaceKDL"));
  node->declare_parameter("kinematics.plugin_package", std::string("kinematics_interface"));
  node->declare_parameter("kinematics.base", std::string("base_link"));
  node->declare_parameter("kinematics.tip", tip);
  node->declare_parameter("kinematics.alpha", 0.0005);

  auto loader = std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
    "kinematics_interface", "kinematics_interface::KinematicsInterface");
  auto kinematics = std::unique_ptr<kinematics_interface::KinematicsInterface>(
    loader->createUnmanagedInstance("kinematics_interface_kdl/KinematicsInterfaceKDL"));
  // "kinematics": node already declares "kinematics.tip" = tip above.
  if (!kinematics->initialize(urdf, node->get_node_parameters_interface(), "kinematics")) {
    std::cerr << "kinematics->initialize() failed\n";
    return 1;
  }

  if (pose.size() != 7) {
    std::cerr << "joint_pose must have exactly 7 elements, got " << pose.size() << "\n";
    return 1;
  }
  Eigen::VectorXd q = Eigen::VectorXd::Map(pose.data(), static_cast<long>(pose.size()));

  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, 7);
  if (!kinematics->calculate_jacobian(q, tip, jacobian)) {
    std::cerr << "calculate_jacobian() failed at this pose\n";
    return 1;
  }

  Eigen::Isometry3d transform;
  kinematics->calculate_link_transform(q, tip, transform);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  const auto & singular_values = svd.singularValues();
  const double smallest = singular_values(singular_values.size() - 1);
  const double largest = singular_values(0);
  const double condition_number = smallest >
    1e-12 ? largest / smallest : std::numeric_limits<double>::infinity();

  std::cout << "pose = [";
  for (double v : pose) {
    std::cout << v << " ";
  }
  std::cout << "]\n";
  std::cout << "tip translation = [" << transform.translation().transpose() << "]\n";
  // tf2::Quaternion::setRPY(roll, pitch, yaw) builds R = Rz(yaw)*Ry(pitch)*Rx(roll) (bullet/tf2's
  // own convention). Eigen's eulerAngles(a0,a1,a2) returns angles (t0,t1,t2) with
  // R = Rot(axis[a0],t0)*Rot(axis[a1],t1)*Rot(axis[a2],t2) -- so matching tf2's convention needs
  // eulerAngles(2,1,0) (Z,Y,X order), NOT eulerAngles(0,1,2) (which builds Rx*Ry*Rz instead, a
  // DIFFERENT matrix product that only coincidentally matches tf2's for symmetric rotations).
  // Found live: eulerAngles(0,1,2)'s output, fed back into moveit_pose_finder's orientation_rpy
  // (-> tf2::Quaternion::setRPY), silently reproduced a 90-degree-wrong orientation (tip local Z
  // landed on base +X instead of the intended -Y) for a pitch-near-zero pose -- moveit_pose_finder
  // itself never errors on this, it just plans/solves IK for the WRONG target.
  const Eigen::Vector3d zyx = transform.rotation().eulerAngles(2, 1, 0);
  const Eigen::Vector3d rpy(zyx(2), zyx(1), zyx(0));  // (roll, pitch, yaw)
  std::cout << "tip orientation (roll pitch yaw, for moveit_pose_finder's orientation_rpy) = ["
            << rpy.transpose() << "]\n";
  std::cout << "singular values = [" << singular_values.transpose() << "]\n";
  std::cout << "condition number = " << condition_number << "\n";
  std::cout << "lift column (col 0) = [" << jacobian.col(0).transpose() << "]\n";
  // Which base-frame direction the tip's own local Z axis points -- useful for picking a
  // reference pose meant to represent "tool normal faces the wall" (roughly horizontal) or
  // "tool normal faces the ceiling" (roughly vertical) without having to guess from joint angles.
  const Eigen::Vector3d tip_z_in_base = transform.rotation() * Eigen::Vector3d::UnitZ();
  std::cout << "tip local Z in base frame = [" << tip_z_in_base.transpose() << "]\n";

  rclcpp::shutdown();
  return 0;
}
