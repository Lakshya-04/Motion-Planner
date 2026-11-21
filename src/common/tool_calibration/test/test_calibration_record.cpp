#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "tool_calibration/calibration_record.hpp"

using tool_calibration::ToolCalibrationRecord;

namespace
{
constexpr std::int64_t kT0 = 1763683200;  // 2025-11-21T00:00:00Z

ToolCalibrationRecord makeRecord()
{
  ToolCalibrationRecord r;
  r.tool_name = "contact_wiper";
  r.tcp_xyz = {0.001, -0.002, 0.185};
  r.tcp_rpy = {0.0, 0.01, 0.0};
  r.payload_mass_kg = 0.84;
  r.payload_cog_m = {0.0, 0.0, 0.05};
  r.ft_bias = {0.1, -0.2, 0.3, 0.01, 0.02, -0.03};
  r.method = "plane_touch";
  r.date = "2025-11-21T00:00:00Z";
  r.operator_id = "op-7";
  r.robot_id = "robot-a";
  r.rms_m = 0.0004;
  r.max_residual_m = 0.0009;
  r.tilt_spread_deg = 0.2;
  r.max_age_hours = 24.0;
  r.mounting_events = 3;
  return r;
}

core_types::ToolProfile contactTool()
{
  core_types::ToolProfile t;
  t.name = "contact_wiper";
  t.task.contact = true;
  return t;
}
}  // namespace

TEST(CalibrationRecord, Iso8601Parsing)
{
  EXPECT_EQ(tool_calibration::parseIso8601Utc("1970-01-01T00:00:00Z"), 0);
  EXPECT_EQ(tool_calibration::parseIso8601Utc("2025-11-21T00:00:00Z"), kT0);
  EXPECT_EQ(tool_calibration::parseIso8601Utc("2000-03-01T12:34:56Z"), 951914096);
  EXPECT_FALSE(tool_calibration::parseIso8601Utc("2025-11-21 00:00:00"));
  EXPECT_FALSE(tool_calibration::parseIso8601Utc("2025-13-21T00:00:00Z"));
  EXPECT_FALSE(tool_calibration::parseIso8601Utc("2025-11-2xT00:00:00Z"));
  EXPECT_FALSE(tool_calibration::parseIso8601Utc(""));
}

TEST(CalibrationRecord, JsonRoundTrip)
{
  const ToolCalibrationRecord r = makeRecord();
  const auto back = tool_calibration::fromJson(tool_calibration::toJson(r));
  EXPECT_EQ(back.tool_name, r.tool_name);
  EXPECT_EQ(back.tcp_xyz, r.tcp_xyz);
  EXPECT_EQ(back.tcp_rpy, r.tcp_rpy);
  EXPECT_DOUBLE_EQ(back.payload_mass_kg, r.payload_mass_kg);
  EXPECT_EQ(back.payload_cog_m, r.payload_cog_m);
  EXPECT_EQ(back.ft_bias, r.ft_bias);
  EXPECT_EQ(back.method, r.method);
  EXPECT_EQ(back.date, r.date);
  EXPECT_EQ(back.operator_id, r.operator_id);
  EXPECT_EQ(back.robot_id, r.robot_id);
  EXPECT_DOUBLE_EQ(back.rms_m, r.rms_m);
  EXPECT_DOUBLE_EQ(back.max_residual_m, r.max_residual_m);
  EXPECT_DOUBLE_EQ(back.tilt_spread_deg, r.tilt_spread_deg);
  EXPECT_DOUBLE_EQ(back.max_age_hours, r.max_age_hours);
  EXPECT_EQ(back.mounting_events, r.mounting_events);
}

TEST(CalibrationRecord, TextRoundTrip)
{
  const auto back = tool_calibration::parseRecord(tool_calibration::toJson(makeRecord()).dump(2));
  EXPECT_EQ(back.tool_name, "contact_wiper");
  EXPECT_TRUE(back.isValid(kT0 + 3600));
}

TEST(CalibrationRecord, ReadRejectsMissingField)
{
  auto j = tool_calibration::toJson(makeRecord());
  j.erase("method");
  EXPECT_THROW(tool_calibration::fromJson(j), std::runtime_error);
  j = tool_calibration::toJson(makeRecord());
  j["validity"].erase("max_age_hours");
  EXPECT_THROW(tool_calibration::fromJson(j), std::runtime_error);
}

TEST(CalibrationRecord, ReadRejectsWrongTypeAndShape)
{
  auto j = tool_calibration::toJson(makeRecord());
  j["tcp_xyz"] = {1.0, 2.0};
  EXPECT_THROW(tool_calibration::fromJson(j), std::runtime_error);
  j = tool_calibration::toJson(makeRecord());
  j["date"] = 5;
  EXPECT_THROW(tool_calibration::fromJson(j), std::runtime_error);
  EXPECT_THROW(tool_calibration::fromJson(nlohmann::json::array()), std::runtime_error);
}

TEST(CalibrationRecord, ReadRejectsBadSchemaVersionAndBadText)
{
  auto j = tool_calibration::toJson(makeRecord());
  j["schema_version"] = 2;
  EXPECT_THROW(tool_calibration::fromJson(j), std::runtime_error);
  EXPECT_THROW(tool_calibration::parseRecord("{not json"), std::runtime_error);
}

TEST(CalibrationRecord, ReadRejectsNonFinite)
{
  auto j = tool_calibration::toJson(makeRecord());
  j["residuals"]["rms_m"] = std::nan("");  // serialised as null by nlohmann -> type error
  EXPECT_THROW(tool_calibration::fromJson(j), std::runtime_error);
}

TEST(CalibrationRecord, ValidInsideWindow)
{
  const auto r = makeRecord();
  EXPECT_TRUE(r.isValid(kT0));
  EXPECT_TRUE(r.isValid(kT0 + 23 * 3600));
  EXPECT_TRUE(r.isValid(kT0 + 24 * 3600));  // exactly at the limit still valid
}

TEST(CalibrationRecord, ExpiresAfterMaxAge)
{
  const auto r = makeRecord();
  EXPECT_FALSE(r.isValid(kT0 + 24 * 3600 + 1));
  EXPECT_NE(r.validityError(kT0 + 48 * 3600).find("expired"), std::string::npos);
}

TEST(CalibrationRecord, FutureDateIsInvalid)
{
  const auto r = makeRecord();
  EXPECT_FALSE(r.isValid(kT0 - 3600));
  EXPECT_TRUE(r.isValid(kT0 - 100));  // small clock skew tolerated
}

TEST(CalibrationRecord, MountingEventInvalidates)
{
  const auto r = makeRecord();
  EXPECT_TRUE(r.isValid(kT0 + 60, 3u));
  EXPECT_FALSE(r.isValid(kT0 + 60, 4u));
  EXPECT_NE(r.validityError(kT0 + 60, 4u).find("re-mounted"), std::string::npos);
  EXPECT_TRUE(r.isValid(kT0 + 60));  // counter unknown: not checked
}

TEST(CalibrationRecord, RejectsNonPositiveMaxAgeAndBadDate)
{
  auto r = makeRecord();
  r.max_age_hours = 0.0;
  EXPECT_FALSE(r.isValid(kT0));
  r = makeRecord();
  r.date = "yesterday";
  EXPECT_FALSE(r.isValid(kT0));
}

TEST(CalibrationRecord, RejectsNonFiniteValues)
{
  auto r = makeRecord();
  r.tcp_xyz[1] = INFINITY;
  EXPECT_FALSE(r.isValid(kT0));
  r = makeRecord();
  r.payload_mass_kg = -1.0;
  EXPECT_FALSE(r.isValid(kT0));
  r = makeRecord();
  r.ft_bias[4] = std::nan("");
  EXPECT_FALSE(r.isValid(kT0));
}

TEST(RequireValidCalibration, ContactToolWithoutRecordIsRefused)
{
  EXPECT_THROW(
    tool_calibration::requireValidCalibration(contactTool(), std::nullopt, kT0),
    std::runtime_error);
}

TEST(RequireValidCalibration, ContactToolWithValidRecordPasses)
{
  EXPECT_NO_THROW(
    tool_calibration::requireValidCalibration(contactTool(), makeRecord(), kT0 + 3600, 3u));
}

TEST(RequireValidCalibration, ContactToolWithoutTheMountingCounterIsRefused)
{
  try {
    tool_calibration::requireValidCalibration(contactTool(), makeRecord(), kT0 + 3600);
    FAIL() << "expected a refusal";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("mounting counter"), std::string::npos) << e.what();
  }
}

TEST(CalibrationRecord, ReportSaysWhetherTheRemountCheckRan)
{
  const auto r = makeRecord();
  const auto without = r.validityReport(kT0 + 60);
  EXPECT_TRUE(without.valid);
  EXPECT_FALSE(without.mounting_checked);
  const auto with = r.validityReport(kT0 + 60, 3u);
  EXPECT_TRUE(with.valid);
  EXPECT_TRUE(with.mounting_checked);
  const auto remounted = r.validityReport(kT0 + 60, 4u);
  EXPECT_FALSE(remounted.valid);
  EXPECT_TRUE(remounted.mounting_checked);
  EXPECT_FALSE(remounted.error.empty());
}

TEST(ParseIso8601, RejectsImpossibleCalendarDates)
{
  using tool_calibration::parseIso8601Utc;
  EXPECT_FALSE(parseIso8601Utc("2026-02-31T00:00:00Z"));
  EXPECT_FALSE(parseIso8601Utc("2026-02-29T00:00:00Z"));  // 2026 is not a leap year
  EXPECT_FALSE(parseIso8601Utc("2025-04-31T00:00:00Z"));
  EXPECT_FALSE(parseIso8601Utc("2025-06-31T00:00:00Z"));
  EXPECT_FALSE(parseIso8601Utc("2025-09-31T00:00:00Z"));
  EXPECT_FALSE(parseIso8601Utc("2025-11-31T00:00:00Z"));
  EXPECT_FALSE(parseIso8601Utc("1900-02-29T00:00:00Z"));  // century, not a leap year
  EXPECT_TRUE(parseIso8601Utc("2024-02-29T00:00:00Z"));   // leap year
  EXPECT_TRUE(parseIso8601Utc("2000-02-29T00:00:00Z"));   // divisible by 400
  EXPECT_TRUE(parseIso8601Utc("2025-12-31T23:59:59Z"));
  EXPECT_FALSE(parseIso8601Utc("2025-01-00T00:00:00Z"));
  EXPECT_FALSE(parseIso8601Utc("2025-13-01T00:00:00Z"));
}

TEST(RequireValidCalibration, ExpiredRecordIsRefused)
{
  try {
    tool_calibration::requireValidCalibration(contactTool(), makeRecord(), kT0 + 100 * 3600);
    FAIL() << "expected a refusal";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("contact_wiper"), std::string::npos);
    EXPECT_NE(std::string(e.what()).find("expired"), std::string::npos);
  }
}

TEST(RequireValidCalibration, RemountedToolIsRefused)
{
  EXPECT_THROW(
    tool_calibration::requireValidCalibration(contactTool(), makeRecord(), kT0 + 60, 4u),
    std::runtime_error);
}

TEST(RequireValidCalibration, RecordForAnotherToolIsRefused)
{
  auto r = makeRecord();
  r.tool_name = "other";
  EXPECT_THROW(
    tool_calibration::requireValidCalibration(contactTool(), r, kT0 + 60), std::runtime_error);
}

TEST(RequireValidCalibration, NonContactToolNeedsNoRecord)
{
  core_types::ToolProfile spray;
  spray.name = "spray_gun";
  spray.task.contact = false;
  EXPECT_NO_THROW(tool_calibration::requireValidCalibration(spray, std::nullopt, kT0));
  // Even an expired or foreign record is not examined for a non-contact tool.
  EXPECT_NO_THROW(tool_calibration::requireValidCalibration(spray, makeRecord(), kT0 + 1000 * 3600));
}
