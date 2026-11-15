/**
 * @file wrench_conditioning.hpp
 * @brief Raw force/torque sensor reading -> the contact wrench at the tool point.
 * @details A wrist sensor reports a wrench at ITS origin, in ITS axes, including the weight of
 *          everything beyond it, with an offset. The force layer wants "what the environment does to
 *          the tool", at the tool control point, in the tool axes (core_types::Wrench). Four steps,
 *          in this order, each separately testable:
 *            1. sign       flip a sensor whose measure direction is the opposite (+1 / -1);
 *            2. bias       subtract the no-load offset (identified together with the payload);
 *            3. gravity    subtract the weight of the payload (mass, centre of gravity in the sensor
 *                          frame) for the sensor's current orientation: static compensation, no
 *                          inertial terms, so use it at low accelerations;
 *            4. move       re-express at the tool origin and in the tool axes, with the lever-arm term
 *                          torque_T = R^T (torque_S - p x force_S), not just a rotation.
 *          Plain C++ (no Eigen, no ROS, no clock), like the rest of force_core.
 *
 *          Frames: S is the sensor frame, T the tool frame (link_tcp). `FrameOffset` gives T in S:
 *          `p` is T's origin in S coordinates and the columns of `R` are T's axes in S, so
 *          v_S = R v_T. The sensor orientation in the world, R_ws, comes from forward kinematics
 *          (v_W = R_ws v_S); gravity points along `gravity` in the world.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "core_types/wrench.hpp"

namespace force_core
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// Row-major 3x3 matrix.
struct Mat3
{
  std::array<double, 9> m{1, 0, 0, 0, 1, 0, 0, 0, 1};
  double at(int r, int c) const {return m[3 * r + c];}
};

Vec3 operator+(const Vec3 & a, const Vec3 & b);
Vec3 operator-(const Vec3 & a, const Vec3 & b);
Vec3 operator*(double s, const Vec3 & a);
Vec3 cross(const Vec3 & a, const Vec3 & b);
Vec3 rotate(const Mat3 & r, const Vec3 & v);            ///< r * v
Vec3 rotateInverse(const Mat3 & r, const Vec3 & v);     ///< r^T * v
Mat3 matrixFromQuaternion(double qx, double qy, double qz, double qw);

/// The tool frame T expressed in the sensor frame S.
struct FrameOffset
{
  Vec3 p;      ///< T's origin in S coordinates (m).
  Mat3 R;      ///< Columns are T's axes in S coordinates: v_S = R v_T.
};

/// Re-express a wrench measured at S's origin in S's axes at T's origin in T's axes.
core_types::Wrench moveWrench(const core_types::Wrench & at_sensor, const FrameOffset & tool_in_sensor);

/// Everything beyond the sensor that hangs on it: the tool, the plate, the adapter.
struct Payload
{
  double mass_kg{0.0};
  Vec3 cog;                         ///< Centre of gravity in the sensor frame (m).
};

/// The wrench the payload's weight puts on the sensor, in S's axes at S's origin.
core_types::Wrench gravityWrench(const Payload & payload, const Mat3 & r_world_sensor, const Vec3 & gravity_world);

struct ConditionerConfig
{
  double sign{1.0};                          ///< +1 or -1: maps the reading to "load acting on the tool side".
  core_types::Wrench bias;                   ///< No-load offset of the sign-applied reading, in S.
  Payload payload;
  FrameOffset tool_in_sensor;
  Vec3 gravity_world{0.0, 0.0, -9.80665};
};

class WrenchConditioner
{
public:
  explicit WrenchConditioner(ConditionerConfig config);

  /// Contact wrench at the tool point in the tool axes (environment on tool).
  core_types::Wrench condition(const core_types::Wrench & raw, const Mat3 & r_world_sensor) const;

  const ConditionerConfig & config() const {return config_;}

private:
  ConditionerConfig config_;
};

/// One static, contact-free reading with the sensor's orientation at that moment.
struct CalibrationSample
{
  Mat3 r_world_sensor;
  core_types::Wrench raw;
};

struct CalibrationResult
{
  bool ok{false};
  Payload payload;
  core_types::Wrench bias;
  double rms_force_n{0.0};          ///< Residual of the fit over the samples.
  double rms_torque_nm{0.0};
  std::size_t samples{0};
  std::string message;
};

/// Least squares for payload mass, centre of gravity and the bias from static poses without
/// contact. Needs at least four poses whose gravity directions in the sensor frame differ by at least
/// about 10 degrees (largest pairwise angle); a degenerate set is rejected, not guessed. A negative fitted
/// mass (wrong sensor sign) is rejected with a message saying so.
CalibrationResult identifyPayload(
  const std::vector<CalibrationSample> & samples, const Vec3 & gravity_world, double sign = 1.0);

}  // namespace force_core
