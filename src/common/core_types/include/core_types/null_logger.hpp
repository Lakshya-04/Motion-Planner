/**
 * @file null_logger.hpp
 * @brief No-op Logger implementation.
 * @details Used as a default before an adapter injects a real one (e.g.
 *          RclcppLogger in motion_planner). Keeps core code usable and
 *          testable without requiring a logging backend.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include "core_types/logger.hpp"

namespace core_types
{

/**
 * @class NullLogger
 * @brief Logger implementation that silently discards every message.
 */
class NullLogger : public Logger
{
public:
  void info(const std::string &) const override {}
  void warn(const std::string &) const override {}
};

}  // namespace core_types
