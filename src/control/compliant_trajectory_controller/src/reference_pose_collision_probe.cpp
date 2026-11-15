/**
 * @file reference_pose_collision_probe.cpp
 * @brief Reusable CLI tool: checks a candidate reference pose for self-collision.
 * @details Uses the SAME collision-checking call production motion planning uses --
 *          planning_scene::PlanningScene::isStateColliding() (see
 *          motion_planner/src/moveit_planning_backend.cpp's fallback path, which this
 *          mirrors) -- rather than a bespoke geometric check, so a pose cleared here is cleared
 *          by the actual planner's own definition of collision-free, not an approximation of it.
 *
 *          pose_conditioning_probe (this package) checks Jacobian conditioning; this checks
 *          collision. A reference pose for a live compliance demo needs both: the admittance-rule integration test's own
 *          all-zero-pose trap and this package's real-robot self-collision incident (a pose
 *          vetted for conditioning alone, at the wrong base_type/attach_rpy, still collided) are
 *          two independent ways a "reasonable-looking" pose turns out to be unusable.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "moveit/planning_scene/planning_scene.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"
#include "compliant_trajectory_controller/pose_inputs.hpp"
#include "mobile_manipulator_profile/profile.hpp"
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
  auto node = std::make_shared<rclcpp::Node>("reference_pose_collision_probe");

  node->declare_parameter("urdf_file", std::string(""));
  node->declare_parameter("srdf_file", std::string(""));
  // Names come from the robot profile (`profile` parameter, config/profiles/<name>.yaml); the
  // explicit parameters below still win when set.
  node->declare_parameter("profile", std::string(mobile_manipulator_profile::kDefaultProfile));
  std::string profile_warning;
  const auto profile = mobile_manipulator_profile::loadProfileOrDefaults(
    node->get_parameter("profile").as_string(), profile_warning);
  if (!profile_warning.empty()) {
    RCLCPP_WARN(node->get_logger(), "%s", profile_warning.c_str());
  }
  // Empty group / joint_names / joint_pose: the profile's whole-body group, its joints in
  // order, and its home pose.
  node->declare_parameter("group", std::string(""));
  node->declare_parameter("joint_names", std::vector<std::string>{});
  node->declare_parameter("joint_pose", std::vector<double>{});

  const auto urdf_path = node->get_parameter("urdf_file").as_string();
  const auto srdf_path = node->get_parameter("srdf_file").as_string();
  std::string group = node->get_parameter("group").as_string();
  if (group.empty()) {
    group = profile.whole_body_group;
  }
  auto joint_names = node->get_parameter("joint_names").as_string_array();
  auto pose = node->get_parameter("joint_pose").as_double_array();
  if (urdf_path.empty() || srdf_path.empty()) {
    std::cerr << "Pass -p urdf_file:=<path> -p srdf_file:=<path> (both flat, xacro-rendered)\n";
    return 1;
  }
  if (const auto err = compliant_trajectory_controller::resolveJointInputs(
      profile, joint_names, pose); !err.empty())
  {
    std::cerr << "Cannot judge a pose: " << err << "\n";
    return 1;
  }
  const std::string urdf = readFile(urdf_path);
  const std::string srdf = readFile(srdf_path);
  if (urdf.empty() || srdf.empty()) {
    std::cerr << "Could not read '" << urdf_path << "' or '" << srdf_path << "'\n";
    return 1;
  }

  robot_model_loader::RobotModelLoader::Options options(urdf, srdf);
  options.load_kinematics_solvers = false;
  robot_model_loader::RobotModelLoader loader(node, options);
  const auto robot_model = loader.getModel();
  if (!robot_model) {
    std::cerr << "Failed to build a RobotModel from the given URDF/SRDF.\n";
    return 1;
  }
  if (!robot_model->hasJointModelGroup(group)) {
    std::cerr << "Group '" << group << "' not found in the SRDF.\n";
    return 1;
  }

  // setJointPositions() does NOT clamp or reject out-of-range values -- it happily computes FK
  // for a pose the real robot could never reach, with no error and no warning. That gap let a
  // genuinely unreachable pose (joint3=1.2 against an actual limit of 0.0611) through this exact
  // tool's earlier checks undetected. Check explicitly, every time, before trusting anything else.
  bool any_out_of_bounds = false;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    const auto * joint_model = robot_model->getJointModel(joint_names[i]);
    if (!joint_model) {
      std::cerr << "Joint '" << joint_names[i] << "' not found in the URDF.\n";
      return 1;
    }
    const auto & bounds = joint_model->getVariableBounds(joint_names[i]);
    if (pose[i] < bounds.min_position_ || pose[i] > bounds.max_position_) {
      std::cerr << "OUT OF BOUNDS: " << joint_names[i] << " = " << pose[i] << " not in ["
                << bounds.min_position_ << ", " << bounds.max_position_
                << "] -- this pose is not physically reachable.\n";
      any_out_of_bounds = true;
    }
  }
  if (any_out_of_bounds) {
    return 1;
  }

  planning_scene::PlanningScene planning_scene(robot_model);
  moveit::core::RobotState & state = planning_scene.getCurrentStateNonConst();
  state.setToDefaultValues();
  for (size_t i = 0; i < joint_names.size(); ++i) {
    state.setJointPositions(joint_names[i], &pose[i]);
  }
  state.update();

  const bool colliding = planning_scene.isStateColliding(state, group, true);

  std::cout << "pose = [";
  for (double v : pose) {
    std::cout << v << " ";
  }
  std::cout << "]\n";
  std::cout << "group = " << group << "\n";
  std::cout << "SELF-COLLIDING = " << (colliding ? "YES" : "no") << "\n";
  if (colliding) {
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 20;
    collision_detection::CollisionResult result;
    planning_scene.checkSelfCollision(request, result, state);
    for (const auto & contact_pair : result.contacts) {
      std::cout << "  colliding pair: " << contact_pair.first.first << " <-> "
                << contact_pair.first.second << "\n";
    }
  }

  rclcpp::shutdown();
  return colliding ? 1 : 0;
}
