#include "lift_hardware/core/fault.hpp"

namespace lift_hardware
{

const char * to_string(Fault fault)
{
  switch (fault) {
    case Fault::kNone:
      return "none";
    case Fault::kTimeout:
      return "timeout";
    case Fault::kOutOfRange:
      return "out_of_range";
    case Fault::kLinkLost:
      return "link_lost";
    case Fault::kDevice:
      return "device";
    case Fault::kUnsupported:
      return "unsupported";
  }
  return "unknown";
}

}  // namespace lift_hardware
