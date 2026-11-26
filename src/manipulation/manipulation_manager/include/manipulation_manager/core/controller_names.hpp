#pragma once

#include <string>

namespace manipulation_manager
{

/// The ROS name of a controller interface hanging off the controller node: "/<controller_name>/<leaf>".
/// `controller_name` may carry a leading "/" or namespace ("/ns/ctrl"); it is not doubled. This is the
/// ros2_control convention (each controller is a node; its action server and topics are under its name).
inline std::string controllerInterfaceName(const std::string & controller_name, const std::string & leaf)
{
  std::string base = controller_name;
  while (!base.empty() && base.front() == '/') {
    base.erase(base.begin());
  }
  while (!base.empty() && base.back() == '/') {
    base.pop_back();
  }
  return "/" + base + "/" + leaf;
}

/// An explicit full name wins; an empty `override_name` means the controller-relative default. Lets a
/// deployment point one interface (the follow_joint_trajectory action, the force-setpoint topic) somewhere
/// else without renaming the controller.
inline std::string resolveInterfaceName(
  const std::string & override_name, const std::string & controller_name, const std::string & leaf)
{
  return override_name.empty() ? controllerInterfaceName(controller_name, leaf) : override_name;
}

}  // namespace manipulation_manager
