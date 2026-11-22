#include <gtest/gtest.h>

#include "lift_hardware/core/lift_config.hpp"
#include "lift_hardware/core/transport_factory.hpp"

namespace lift_hardware
{
namespace
{

TEST(LiftConfig, DefaultsAreValid)
{
  EXPECT_EQ(validate(LiftConfig{}), "");
  std::string error;
  const LiftConfig c = parse_lift_config({}, &error);
  EXPECT_EQ(error, "");
  EXPECT_DOUBLE_EQ(c.max_velocity_mps, LiftConfig{}.max_velocity_mps);
}

TEST(LiftConfig, ReadsEachKey)
{
  std::string error;
  const LiftConfig c = parse_lift_config(
    {{"stroke_m", "0.8"}, {"max_velocity_mps", "0.15"}, {"max_acceleration_mps2", "0.08"},
     {"home_height_m", "0.01"}, {"max_io_failures", "7"}, {"reconnect_factor", "1.5"}},
    &error);
  EXPECT_EQ(error, "");
  EXPECT_DOUBLE_EQ(c.stroke_m, 0.8);
  EXPECT_DOUBLE_EQ(c.max_velocity_mps, 0.15);
  EXPECT_DOUBLE_EQ(c.max_acceleration_mps2, 0.08);
  EXPECT_DOUBLE_EQ(c.home_height_m, 0.01);
  EXPECT_EQ(c.max_io_failures, 7);
  EXPECT_DOUBLE_EQ(c.reconnect_factor, 1.5);
}

TEST(LiftConfig, RejectsText)
{
  std::string error;
  parse_lift_config({{"max_velocity_mps", "fast"}}, &error);
  EXPECT_NE(error.find("max_velocity_mps"), std::string::npos);
  error.clear();
  parse_lift_config({{"stroke_m", "0.8m"}}, &error);
  EXPECT_FALSE(error.empty());
  error.clear();
  parse_lift_config({{"stroke_m", "nan"}}, &error);
  EXPECT_FALSE(error.empty());
}

TEST(LiftConfig, RejectsValuesThatCannotWork)
{
  const std::pair<const char *, const char *> bad[] = {
    {"max_velocity_mps", "0"}, {"max_acceleration_mps2", "-1"}, {"tolerance_m", "0"},
    {"stroke_m", "-0.1"}, {"reconnect_factor", "0.5"}, {"reconnect_initial_s", "0"},
    {"max_io_failures", "0"}, {"max_io_failures", "2.5"}, {"home_timeout_s", "0"}};
  for (const auto & [key, value] : bad) {
    std::string error;
    parse_lift_config({{key, value}}, &error);
    EXPECT_FALSE(error.empty()) << key << "=" << value;
  }
  std::string error;
  parse_lift_config({{"stroke_m", "0.5"}, {"home_height_m", "0.6"}}, &error);
  EXPECT_NE(error.find("home_height_m"), std::string::npos);
  error.clear();
  parse_lift_config({{"reconnect_initial_s", "5"}, {"reconnect_max_s", "1"}}, &error);
  EXPECT_NE(error.find("reconnect_max_s"), std::string::npos);
}

TEST(TransportFactory, BuildsTheMockAndTheSkeletonAndRejectsTheRest)
{
  std::string error;
  EXPECT_NE(make_transport("mock", {}, &error), nullptr);
  EXPECT_NE(make_transport("device", {}, &error), nullptr);
  EXPECT_EQ(make_transport("serial", {}, &error), nullptr);
  EXPECT_NE(error.find("serial"), std::string::npos);
  error.clear();
  EXPECT_EQ(make_transport("mock", {{"mock_stroke_m", "-1"}}, &error), nullptr);
  EXPECT_FALSE(error.empty());
  error.clear();
  EXPECT_EQ(
    make_transport("mock", {{"mock_stroke_m", "0.5"}, {"mock_initial_height_m", "0.9"}}, &error),
    nullptr);
  EXPECT_FALSE(error.empty());
}

TEST(TransportFactory, TheSkeletonRefusesEveryOperation)
{
  std::string error;
  auto transport = make_transport("device", {}, &error);
  double stroke = 0.0;
  Reading reading;
  EXPECT_EQ(transport->connect(), Fault::kUnsupported);
  EXPECT_EQ(transport->read_stroke(stroke), Fault::kUnsupported);
  EXPECT_EQ(transport->move_to(0.1), Fault::kUnsupported);
  EXPECT_EQ(transport->stop(), Fault::kUnsupported);
  EXPECT_EQ(transport->read(reading), Fault::kUnsupported);
}

}  // namespace
}  // namespace lift_hardware
