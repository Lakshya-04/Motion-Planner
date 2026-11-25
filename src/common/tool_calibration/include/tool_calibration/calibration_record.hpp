/**
 * @file calibration_record.hpp
 * @brief Tool calibration record: TCP, payload, bias, provenance, residuals and validity, with
 *        JSON read/write and the check a contact tool must pass before it is used. ROS-free.
 * @details A record says what was measured, how, when, by whom and how well, and for how long it
 *          may be trusted: `max_age_hours` after the calibration date, and only while the tool has
 *          not been re-mounted (`mounting_events` at calibration time against the current count).
 *          `requireValidCalibration` is the loader-side gate: a contact tool needs a valid record
 *          (the force and contact semantics rest on the TCP being right), a non-contact tool
 *          does not. Limits: the date is an ISO 8601 UTC string "YYYY-MM-DDTHH:MM:SSZ", the
 *          clock and the mounting counter are supplied by the caller, and the record is not
 *          cross-checked against the profile's `tcp_xyz`.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "core_types/tool_profile.hpp"

namespace tool_calibration
{

/// Parses "YYYY-MM-DDTHH:MM:SSZ" (UTC) to seconds since the Unix epoch. nullopt when malformed.
inline std::optional<std::int64_t> parseIso8601Utc(const std::string & s)
{
  if (s.size() != 20 || s[4] != '-' || s[7] != '-' || s[10] != 'T' || s[13] != ':' ||
    s[16] != ':' || s[19] != 'Z')
  {
    return std::nullopt;
  }
  auto num = [&s](std::size_t pos, std::size_t len) -> int {
      int v = 0;
      for (std::size_t i = pos; i < pos + len; ++i) {
        if (s[i] < '0' || s[i] > '9') {
          return -1;
        }
        v = v * 10 + (s[i] - '0');
      }
      return v;
    };
  const int y = num(0, 4), mo = num(5, 2), d = num(8, 2), h = num(11, 2), mi = num(14, 2),
    se = num(17, 2);
  if (y < 0 || mo < 1 || mo > 12 || d < 1 || h < 0 || h > 23 || mi < 0 || mi > 59 ||
    se < 0 || se > 60)
  {
    return std::nullopt;
  }
  const bool leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
  static const int kDays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (d > kDays[mo - 1] + ((mo == 2 && leap) ? 1 : 0)) {
    return std::nullopt;  // e.g. 2026-02-31
  }
  // Days from civil (H. Hinnant).
  const int yy = y - (mo <= 2 ? 1 : 0);
  const int era = (yy >= 0 ? yy : yy - 399) / 400;
  const int yoe = yy - era * 400;
  const int doy = (153 * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const std::int64_t days = static_cast<std::int64_t>(era) * 146097 + doe - 719468;
  return days * 86400 + h * 3600 + mi * 60 + se;
}

/// Outcome of a validity check, saying whether the re-mount check could run.
struct ValidityReport
{
  bool valid{false};
  /// True when a current mounting counter was given and compared. False means the re-mount check
  /// was SKIPPED: a valid result then only covers the date, expiry and finite fields.
  bool mounting_checked{false};
  std::string error;  ///< Empty when valid.
};

/// One tool's calibration result with provenance and validity.
struct ToolCalibrationRecord
{
  std::string tool_name;                       ///< Tool profile key the record belongs to.
  std::array<double, 3> tcp_xyz{0, 0, 0};      ///< Flange -> contact frame translation (m).
  std::array<double, 3> tcp_rpy{0, 0, 0};      ///< Flange -> contact frame rotation (rad).
  double payload_mass_kg{0.0};                 ///< Tool payload mass.
  std::array<double, 3> payload_cog_m{0, 0, 0}; ///< Payload centre of gravity in the sensor frame.
  std::array<double, 6> ft_bias{0, 0, 0, 0, 0, 0}; ///< F/T bias (N, Nm).
  std::string method;                          ///< "pivot" | "plane_touch" | "fixture" | "cad" | ...
  std::string date;                            ///< Calibration time, "YYYY-MM-DDTHH:MM:SSZ" (UTC).
  std::string operator_id;                     ///< Who ran it.
  std::string robot_id;                        ///< Which robot.
  double rms_m{0.0};                           ///< Residual RMS of the fit (m).
  double max_residual_m{0.0};                  ///< Worst single residual (m).
  double tilt_spread_deg{0.0};                 ///< Spread of a tilt fit (deg); 0 when none was run.
  double max_age_hours{0.0};                   ///< Record expires this long after `date`; must be > 0.
  std::uint32_t mounting_events{0};            ///< Mounting-event counter at calibration time.

  /// Empty when the record is usable at `now_epoch_s` (and, when given, with the tool mounted
  /// `current_mounting_events` times in total); otherwise the reason it is not.
  std::string validityError(
    std::int64_t now_epoch_s,
    std::optional<std::uint32_t> current_mounting_events = std::nullopt) const
  {
    auto finite = [](double v) {return std::isfinite(v);};
    for (double v : tcp_xyz) {
      if (!finite(v)) {return "tcp_xyz is not finite";}
    }
    for (double v : tcp_rpy) {
      if (!finite(v)) {return "tcp_rpy is not finite";}
    }
    for (double v : payload_cog_m) {
      if (!finite(v)) {return "payload_cog_m is not finite";}
    }
    for (double v : ft_bias) {
      if (!finite(v)) {return "ft_bias is not finite";}
    }
    if (!finite(payload_mass_kg) || payload_mass_kg < 0.0) {
      return "payload_mass_kg must be finite and >= 0";
    }
    if (!(max_age_hours > 0.0) || !finite(max_age_hours)) {
      return "max_age_hours must be > 0";
    }
    const auto t0 = parseIso8601Utc(date);
    if (!t0) {
      return "date is not an ISO 8601 UTC timestamp (YYYY-MM-DDTHH:MM:SSZ)";
    }
    if (*t0 > now_epoch_s + 300) {
      return "calibration date is in the future";
    }
    if (static_cast<double>(now_epoch_s - *t0) > max_age_hours * 3600.0) {
      return "calibration expired (older than max_age_hours)";
    }
    if (current_mounting_events && *current_mounting_events != mounting_events) {
      return "tool was re-mounted since the calibration (mounting_events changed)";
    }
    return "";
  }

  /// Like validityError(), with the explicit statement of whether the re-mount check was done.
  ValidityReport validityReport(
    std::int64_t now_epoch_s,
    std::optional<std::uint32_t> current_mounting_events = std::nullopt) const
  {
    ValidityReport rep;
    rep.error = validityError(now_epoch_s, current_mounting_events);
    rep.valid = rep.error.empty();
    rep.mounting_checked = current_mounting_events.has_value();
    return rep;
  }

  bool isValid(
    std::int64_t now_epoch_s,
    std::optional<std::uint32_t> current_mounting_events = std::nullopt) const
  {
    return validityError(now_epoch_s, current_mounting_events).empty();
  }
};

/// JSON form of a record (schema_version 1).
inline nlohmann::json toJson(const ToolCalibrationRecord & r)
{
  nlohmann::json j;
  j["schema_version"] = 1;
  j["tool_name"] = r.tool_name;
  j["tcp_xyz"] = r.tcp_xyz;
  j["tcp_rpy"] = r.tcp_rpy;
  j["payload"] = {{"mass_kg", r.payload_mass_kg}, {"cog_m", r.payload_cog_m}};
  j["ft_bias"] = r.ft_bias;
  j["method"] = r.method;
  j["date"] = r.date;
  j["operator_id"] = r.operator_id;
  j["robot_id"] = r.robot_id;
  j["residuals"] = {{"rms_m", r.rms_m}, {"max_residual_m", r.max_residual_m},
    {"tilt_spread_deg", r.tilt_spread_deg}};
  j["validity"] = {{"max_age_hours", r.max_age_hours}, {"mounting_events", r.mounting_events}};
  return j;
}

/// Record from JSON. @throws std::runtime_error for a missing or mistyped field, a non-finite
/// number, or an unsupported schema_version.
inline ToolCalibrationRecord fromJson(const nlohmann::json & j)
{
  auto fail = [](const std::string & what) {
      throw std::runtime_error("calibration record: " + what);
    };
  try {
    if (!j.is_object()) {
      fail("not a JSON object");
    }
    if (j.value("schema_version", 0) != 1) {
      fail("unsupported or missing schema_version (expected 1)");
    }
    ToolCalibrationRecord r;
    r.tool_name = j.at("tool_name").get<std::string>();
    r.tcp_xyz = j.at("tcp_xyz").get<std::array<double, 3>>();
    r.tcp_rpy = j.at("tcp_rpy").get<std::array<double, 3>>();
    r.payload_mass_kg = j.at("payload").at("mass_kg").get<double>();
    r.payload_cog_m = j.at("payload").at("cog_m").get<std::array<double, 3>>();
    r.ft_bias = j.at("ft_bias").get<std::array<double, 6>>();
    r.method = j.at("method").get<std::string>();
    r.date = j.at("date").get<std::string>();
    r.operator_id = j.at("operator_id").get<std::string>();
    r.robot_id = j.at("robot_id").get<std::string>();
    r.rms_m = j.at("residuals").at("rms_m").get<double>();
    r.max_residual_m = j.at("residuals").at("max_residual_m").get<double>();
    r.tilt_spread_deg = j.at("residuals").value("tilt_spread_deg", 0.0);
    r.max_age_hours = j.at("validity").at("max_age_hours").get<double>();
    r.mounting_events = j.at("validity").at("mounting_events").get<std::uint32_t>();
    const double all[] = {r.tcp_xyz[0], r.tcp_xyz[1], r.tcp_xyz[2], r.tcp_rpy[0], r.tcp_rpy[1],
      r.tcp_rpy[2], r.payload_mass_kg, r.payload_cog_m[0], r.payload_cog_m[1], r.payload_cog_m[2],
      r.ft_bias[0], r.ft_bias[1], r.ft_bias[2], r.ft_bias[3], r.ft_bias[4], r.ft_bias[5], r.rms_m,
      r.max_residual_m, r.tilt_spread_deg, r.max_age_hours};
    for (double v : all) {
      if (!std::isfinite(v)) {
        fail("contains a non-finite number");
      }
    }
    return r;
  } catch (const nlohmann::json::exception & e) {
    throw std::runtime_error(std::string("calibration record: ") + e.what());
  }
}

/// Parses a record from JSON text. @throws std::runtime_error (also for malformed JSON).
inline ToolCalibrationRecord parseRecord(const std::string & text)
{
  try {
    return fromJson(nlohmann::json::parse(text));
  } catch (const nlohmann::json::parse_error & e) {
    throw std::runtime_error(std::string("calibration record: ") + e.what());
  }
}

/**
 * @brief Loader gate: a contact tool needs a valid calibration record, a non-contact tool does not.
 * @throws std::runtime_error naming the tool for a contact tool with no record, a record for a
 *         different tool, an invalid (expired, re-mounted, malformed) one, or no current mounting
 *         counter (required for a contact tool).
 */
inline void requireValidCalibration(
  const core_types::ToolProfile & tool, const std::optional<ToolCalibrationRecord> & record,
  std::int64_t now_epoch_s, std::optional<std::uint32_t> current_mounting_events = std::nullopt)
{
  if (!tool.task.contact) {
    return;
  }
  if (!record) {
    throw std::runtime_error(
            "tool '" + tool.name + "': contact tool needs a calibration record, none given");
  }
  if (record->tool_name != tool.name) {
    throw std::runtime_error(
            "tool '" + tool.name + "': calibration record is for tool '" + record->tool_name + "'");
  }
  const std::string err = record->validityError(now_epoch_s, current_mounting_events);
  if (!err.empty()) {
    throw std::runtime_error("tool '" + tool.name + "': calibration refused: " + err);
  }
  // A contact tool's TCP is only trusted for the mounting it was measured on: the caller must say
  // how often the tool has been mounted, or the re-mount check would pass silently.
  if (!current_mounting_events) {
    throw std::runtime_error(
            "tool '" + tool.name + "': calibration refused: the current mounting counter is "
            "required for a contact tool (the re-mount check cannot be skipped)");
  }
}

}  // namespace tool_calibration
