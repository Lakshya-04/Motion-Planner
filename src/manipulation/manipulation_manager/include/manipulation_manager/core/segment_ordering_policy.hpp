#pragma once

#include <cstddef>
#include <vector>

namespace manipulation_manager
{

class SegmentOrderingPolicy
{
public:
  virtual ~SegmentOrderingPolicy() = default;
  virtual std::vector<size_t> buildExecutionOrder(size_t segment_count) const = 0;
};

class InputOrderSegmentOrderingPolicy : public SegmentOrderingPolicy
{
public:
  std::vector<size_t> buildExecutionOrder(size_t segment_count) const override
  {
    std::vector<size_t> order;
    order.reserve(segment_count);
    for (size_t i = 0; i < segment_count; ++i) {
      order.push_back(i);
    }
    return order;
  }
};

}  // namespace manipulation_manager
