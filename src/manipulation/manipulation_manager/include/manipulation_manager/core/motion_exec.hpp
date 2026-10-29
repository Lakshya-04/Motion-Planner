#pragma once

#include <cstddef>
#include <string>

namespace manipulation_manager
{

struct MotionExec
{
  bool success{false};
  std::string status;
  std::string message;
  size_t waypoints_completed{0};
};

}  // namespace manipulation_manager
