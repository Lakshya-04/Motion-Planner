#include "manipulation_manager/core/stand_check.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace manipulation_manager
{

const char * toString(StandReason r)
{
  switch (r) {
    case StandReason::kWithinTolerance: return "within_tolerance";
    case StandReason::kPositionOutOfTolerance: return "position_out_of_tolerance";
    case StandReason::kYawOutOfTolerance: return "yaw_out_of_tolerance";
    case StandReason::kPositionAndYawOutOfTolerance: return "position_and_yaw_out_of_tolerance";
    case StandReason::kInvalidMeasurement: return "invalid_measurement";
    case StandReason::kRedocksExhausted: return "redocks_exhausted";
  }
  return "unknown";
}

StandCheck::StandCheck(const StandCheckConfig & cfg) : cfg_(cfg)
{
  cfg_.min_tol_m = std::max(0.0, cfg_.min_tol_m);
  cfg_.max_tol_m = std::max(cfg_.min_tol_m, cfg_.max_tol_m);
  cfg_.margin_scale = std::max(0.0, cfg_.margin_scale);
  cfg_.fallback_tol_m = std::clamp(cfg_.fallback_tol_m, cfg_.min_tol_m, cfg_.max_tol_m);
  cfg_.yaw_tol_rad = std::max(0.0, cfg_.yaw_tol_rad);
  cfg_.max_redocks = std::max(0, cfg_.max_redocks);
}

double StandCheck::toleranceM(double reach_margin_m) const
{
  if (!std::isfinite(reach_margin_m) || reach_margin_m < 0.0) {
    return cfg_.fallback_tol_m;
  }
  return std::clamp(cfg_.margin_scale * reach_margin_m, cfg_.min_tol_m, cfg_.max_tol_m);
}

StandCheckResult StandCheck::evaluate(
  const StandAchieved & a, double reach_margin_m, int redocks_done) const
{
  StandCheckResult r;
  r.tol_m = toleranceM(reach_margin_m);
  r.tol_yaw_rad = cfg_.yaw_tol_rad;
  char buf[160];

  if (!std::isfinite(a.position_error_m) || !std::isfinite(a.yaw_error_rad) ||
    a.position_error_m < 0.0)
  {
    r.decision = StandDecision::kAbort;
    r.reason = StandReason::kInvalidMeasurement;
    r.stand_failed = true;
    r.message = "stand check: the arrival error is not a valid measurement";
    return r;
  }
  const bool pos_bad = a.position_error_m > r.tol_m;
  const bool yaw_bad = std::fabs(a.yaw_error_rad) > r.tol_yaw_rad;
  std::snprintf(
    buf, sizeof(buf), "error %.1f mm (tol %.1f mm), yaw %.3f rad (tol %.3f rad)",
    a.position_error_m * 1000.0, r.tol_m * 1000.0, a.yaw_error_rad, r.tol_yaw_rad);

  if (!pos_bad && !yaw_bad) {
    r.decision = StandDecision::kGo;
    r.reason = StandReason::kWithinTolerance;
    r.message = std::string("stand check ok: ") + buf;
    return r;
  }
  if (redocks_done >= cfg_.max_redocks) {
    r.decision = StandDecision::kAbort;
    r.reason = StandReason::kRedocksExhausted;
    r.stand_failed = true;
    r.message = std::string("stand check: out of tolerance after ") + std::to_string(redocks_done) +
      " re-dock(s), stand failed: " + buf;
    return r;
  }
  r.decision = StandDecision::kRedock;
  r.reason = pos_bad && yaw_bad ? StandReason::kPositionAndYawOutOfTolerance :
    pos_bad ? StandReason::kPositionOutOfTolerance : StandReason::kYawOutOfTolerance;
  r.message = std::string("stand check: ") + toString(r.reason) + ", re-docking: " + buf;
  return r;
}

}  // namespace manipulation_manager
