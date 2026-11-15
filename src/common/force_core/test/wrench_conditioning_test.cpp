#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include "force_core/wrench_conditioning.hpp"

using force_core::CalibrationSample;
using force_core::ConditionerConfig;
using force_core::FrameOffset;
using force_core::Mat3;
using force_core::Payload;
using force_core::Vec3;
using force_core::WrenchConditioner;
using core_types::Wrench;

namespace
{
constexpr double kPi = 3.14159265358979323846;

Mat3 rotY(double a)
{
  Mat3 r;
  r.m = {std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a)};
  return r;
}
Mat3 rotX(double a)
{
  Mat3 r;
  r.m = {1, 0, 0, 0, std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a)};
  return r;
}
Mat3 mul(const Mat3 & a, const Mat3 & b)
{
  Mat3 r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double s = 0;
      for (int k = 0; k < 3; ++k) {s += a.at(i, k) * b.at(k, j);}
      r.m[3 * i + j] = s;
    }
  }
  return r;
}
Wrench w(double fx, double fy, double fz, double tx, double ty, double tz)
{
  Wrench x;
  x.force_x = fx; x.force_y = fy; x.force_z = fz; x.torque_x = tx; x.torque_y = ty; x.torque_z = tz;
  return x;
}
void expectNear(const Wrench & a, const Wrench & b, double tol)
{
  EXPECT_NEAR(a.force_x, b.force_x, tol);
  EXPECT_NEAR(a.force_y, b.force_y, tol);
  EXPECT_NEAR(a.force_z, b.force_z, tol);
  EXPECT_NEAR(a.torque_x, b.torque_x, tol);
  EXPECT_NEAR(a.torque_y, b.torque_y, tol);
  EXPECT_NEAR(a.torque_z, b.torque_z, tol);
}

// Tool origin 0.2 m beyond the sensor along the sensor's z, its axes turned 90 degrees about y.
FrameOffset offsetTool()
{
  FrameOffset o;
  o.p = {0.0, 0.0, 0.2};
  o.R = rotY(kPi / 2);
  return o;
}

// What a sensor reads when `f_tool` (in tool axes, acting at the tool origin, no torque there) loads it.
Wrench sensorReading(const Wrench & contact_at_tool, const FrameOffset & o)
{
  const Vec3 f_t{contact_at_tool.force_x, contact_at_tool.force_y, contact_at_tool.force_z};
  const Vec3 f_s = force_core::rotate(o.R, f_t);
  const Vec3 tq = force_core::cross(o.p, f_s);           // moment of that force about the sensor origin
  return w(f_s.x, f_s.y, f_s.z, tq.x, tq.y, tq.z);
}
}  // namespace

TEST(WrenchMove, RecoversAPointContactAtTheToolThroughTheLeverArm)
{
  const FrameOffset o = offsetTool();
  const Wrench contact = w(1.5, -2.0, -15.0, 0.0, 0.0, 0.0);        // pressing in, a little friction
  const Wrench raw = sensorReading(contact, o);
  // the sensor sees a real torque from the lever arm, the tool point does not
  EXPECT_GT(std::abs(raw.torque_x) + std::abs(raw.torque_y), 1.0);
  expectNear(force_core::moveWrench(raw, o), contact, 1e-9);
}

TEST(WrenchMove, RotationAloneGetsTheTorqueWrong)
{
  const FrameOffset o = offsetTool();
  const Wrench raw = sensorReading(w(1.5, -2.0, -15.0, 0, 0, 0), o);
  FrameOffset no_arm = o;
  no_arm.p = {0, 0, 0};                                              // what a rotate-only frame change does
  const Wrench rotated_only = force_core::moveWrench(raw, no_arm);
  EXPECT_GT(std::abs(rotated_only.torque_x) + std::abs(rotated_only.torque_y), 1.0);
}

TEST(WrenchMove, IdentityOffsetChangesNothing)
{
  const Wrench raw = w(1, 2, 3, 0.1, 0.2, 0.3);
  expectNear(force_core::moveWrench(raw, FrameOffset{}), raw, 1e-12);
}

TEST(WrenchConditioner, RemovesBiasAndPayloadWeightInAnyOrientation)
{
  ConditionerConfig c;
  c.payload = {1.2, {0.0, 0.01, 0.05}};
  c.bias = w(0.3, -0.2, 0.5, 0.01, -0.02, 0.005);
  c.tool_in_sensor = offsetTool();
  const WrenchConditioner cond(c);
  for (const double a : {0.0, 0.7, 1.9, -1.2}) {
    const Mat3 r = mul(rotX(a), rotY(0.5 * a + 0.3));
    const Wrench g = force_core::gravityWrench(c.payload, r, c.gravity_world);
    // free of contact: sensor = payload weight + bias
    const Wrench raw = w(g.force_x + c.bias.force_x, g.force_y + c.bias.force_y, g.force_z + c.bias.force_z,
      g.torque_x + c.bias.torque_x, g.torque_y + c.bias.torque_y, g.torque_z + c.bias.torque_z);
    expectNear(cond.condition(raw, r), Wrench{}, 1e-9);
  }
}

TEST(WrenchConditioner, ContactPassesThroughOnTopOfTheCompensation)
{
  ConditionerConfig c;
  c.payload = {0.8, {0.0, 0.0, 0.04}};
  c.tool_in_sensor = offsetTool();
  const WrenchConditioner cond(c);
  const Mat3 r = mul(rotX(1.1), rotY(-0.4));
  const Wrench contact = w(0.5, 1.0, -12.0, 0, 0, 0);               // at the tool point, tool axes
  const Wrench load = sensorReading(contact, c.tool_in_sensor);
  const Wrench g = force_core::gravityWrench(c.payload, r, c.gravity_world);
  const Wrench raw = w(load.force_x + g.force_x, load.force_y + g.force_y, load.force_z + g.force_z,
    load.torque_x + g.torque_x, load.torque_y + g.torque_y, load.torque_z + g.torque_z);
  expectNear(cond.condition(raw, r), contact, 1e-9);
  EXPECT_NEAR(core_types::normalForceN(cond.condition(raw, r)), 12.0, 1e-9);   // +12 N of contact
}

TEST(WrenchConditioner, SignFlipsASensorMeasuredTheOtherWay)
{
  ConditionerConfig c;
  c.sign = -1.0;
  const WrenchConditioner cond(c);
  expectNear(cond.condition(w(1, 2, 3, 4, 5, 6), Mat3{}), w(-1, -2, -3, -4, -5, -6), 1e-12);
}

TEST(PayloadIdentification, RecoversMassCentreAndBiasFromStaticPoses)
{
  const Payload truth{1.7, {0.012, -0.008, 0.061}};
  const Wrench bias = w(0.4, -0.3, 0.2, 0.02, -0.01, 0.03);
  const Vec3 gravity{0, 0, -9.80665};
  std::vector<CalibrationSample> s;
  for (const double a : {0.0, 0.9, 1.8, 2.7, -1.0, -2.0}) {
    CalibrationSample p;
    p.r_world_sensor = mul(rotX(a), rotY(0.6 * a - 0.5));
    const Wrench g = force_core::gravityWrench(truth, p.r_world_sensor, gravity);
    p.raw = w(g.force_x + bias.force_x, g.force_y + bias.force_y, g.force_z + bias.force_z,
      g.torque_x + bias.torque_x, g.torque_y + bias.torque_y, g.torque_z + bias.torque_z);
    s.push_back(p);
  }
  const auto r = force_core::identifyPayload(s, gravity);
  ASSERT_TRUE(r.ok) << r.message;
  EXPECT_NEAR(r.payload.mass_kg, truth.mass_kg, 1e-9);
  EXPECT_NEAR(r.payload.cog.x, truth.cog.x, 1e-9);
  EXPECT_NEAR(r.payload.cog.y, truth.cog.y, 1e-9);
  EXPECT_NEAR(r.payload.cog.z, truth.cog.z, 1e-9);
  expectNear(r.bias, bias, 1e-9);
  EXPECT_LT(r.rms_force_n, 1e-9);
}

TEST(PayloadIdentification, RejectsPosesThatAllPointTheSameWay)
{
  std::vector<CalibrationSample> s(6);
  for (auto & p : s) {p.raw = w(0, 0, -11.0, 0.1, 0, 0);}              // identical orientation each time
  const auto r = force_core::identifyPayload(s, Vec3{0, 0, -9.80665});
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.message.find("do not excite"), std::string::npos);
}

TEST(PayloadIdentification, NeedsEnoughPoses)
{
  const auto r = force_core::identifyPayload(std::vector<CalibrationSample>(2), Vec3{0, 0, -9.80665});
  EXPECT_FALSE(r.ok);
}

namespace
{
std::vector<CalibrationSample> posesFor(
  const Payload & truth, const Wrench & bias, const Vec3 & gravity, const std::vector<std::pair<double, double>> & angles,
  double noise)
{
  std::vector<CalibrationSample> s;
  unsigned k = 0;
  for (const auto & [ax, ay] : angles) {
    CalibrationSample p;
    p.r_world_sensor = mul(rotX(0.3 + ax), rotY(-0.5 + ay));
    const Wrench g = force_core::gravityWrench(truth, p.r_world_sensor, gravity);
    const auto n = [&]() {return noise * std::sin(12.9898 * static_cast<double>(++k));};
    p.raw = w(g.force_x + bias.force_x + n(), g.force_y + bias.force_y + n(),
      g.force_z + bias.force_z + n(), g.torque_x + bias.torque_x + n() * 0.1,
      g.torque_y + bias.torque_y + n() * 0.1, g.torque_z + bias.torque_z + n() * 0.1);
    s.push_back(p);
  }
  return s;
}
}  // namespace

TEST(PayloadIdentification, RejectsNearlyIdenticalPosesWithNoise)
{
  // Six poses within 0.1 degree of each other: the fit would be noise, not a payload.
  const auto s = posesFor(Payload{1.7, {0.012, -0.008, 0.061}}, w(0.4, -0.3, 0.2, 0.02, -0.01, 0.03),
    Vec3{0, 0, -9.80665}, {{0.0, 0.0}, {0.02, 0.01}, {-0.015, 0.02}, {0.01, -0.02}, {-0.02, 0.015}, {0.005, -0.01}}, 0.05);
  const auto r = force_core::identifyPayload(s, Vec3{0, 0, -9.80665});
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.message.find("do not excite"), std::string::npos);
}

TEST(PayloadIdentification, RejectsNegativeMassAsWrongSensorSign)
{
  const Vec3 gravity{0, 0, -9.80665};
  const auto s = posesFor(Payload{1.7, {0.012, -0.008, 0.061}}, w(0.4, -0.3, 0.2, 0.02, -0.01, 0.03),
    gravity, {{0.0, 0.0}, {0.9, 0.5}, {1.8, -0.4}, {2.7, 0.3}, {-1.0, 0.8}, {-2.0, -0.7}}, 0.0);
  const auto wrong = force_core::identifyPayload(s, gravity, -1.0);   // sensor read with the wrong sign
  EXPECT_FALSE(wrong.ok);
  EXPECT_NE(wrong.message.find("negative mass"), std::string::npos);
  EXPECT_TRUE(force_core::identifyPayload(s, gravity, 1.0).ok);
}
