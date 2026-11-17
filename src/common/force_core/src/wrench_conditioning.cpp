#include "force_core/wrench_conditioning.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "force_core/payload_excitation.hpp"

namespace force_core
{

Vec3 operator+(const Vec3 & a, const Vec3 & b) {return {a.x + b.x, a.y + b.y, a.z + b.z};}
Vec3 operator-(const Vec3 & a, const Vec3 & b) {return {a.x - b.x, a.y - b.y, a.z - b.z};}
Vec3 operator*(double s, const Vec3 & a) {return {s * a.x, s * a.y, s * a.z};}

Vec3 cross(const Vec3 & a, const Vec3 & b)
{
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

Vec3 rotate(const Mat3 & r, const Vec3 & v)
{
  return {
    r.at(0, 0) * v.x + r.at(0, 1) * v.y + r.at(0, 2) * v.z,
    r.at(1, 0) * v.x + r.at(1, 1) * v.y + r.at(1, 2) * v.z,
    r.at(2, 0) * v.x + r.at(2, 1) * v.y + r.at(2, 2) * v.z};
}

Vec3 rotateInverse(const Mat3 & r, const Vec3 & v)
{
  return {
    r.at(0, 0) * v.x + r.at(1, 0) * v.y + r.at(2, 0) * v.z,
    r.at(0, 1) * v.x + r.at(1, 1) * v.y + r.at(2, 1) * v.z,
    r.at(0, 2) * v.x + r.at(1, 2) * v.y + r.at(2, 2) * v.z};
}

Mat3 matrixFromQuaternion(double qx, double qy, double qz, double qw)
{
  const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (n < 1e-12) {return Mat3{};}
  qx /= n; qy /= n; qz /= n; qw /= n;
  Mat3 r;
  r.m = {
    1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw),
    2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw),
    2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)};
  return r;
}

namespace
{
Vec3 force(const core_types::Wrench & w) {return {w.force_x, w.force_y, w.force_z};}
Vec3 torque(const core_types::Wrench & w) {return {w.torque_x, w.torque_y, w.torque_z};}
core_types::Wrench make(const Vec3 & f, const Vec3 & t)
{
  core_types::Wrench w;
  w.force_x = f.x; w.force_y = f.y; w.force_z = f.z;
  w.torque_x = t.x; w.torque_y = t.y; w.torque_z = t.z;
  return w;
}
}  // namespace

core_types::Wrench moveWrench(const core_types::Wrench & at_sensor, const FrameOffset & tool_in_sensor)
{
  const Vec3 f = force(at_sensor);
  // The same load, referred to T's origin instead of S's: the force is unchanged, the torque loses
  // the moment of that force about the new point (p is T's origin seen from S).
  const Vec3 t = torque(at_sensor) - cross(tool_in_sensor.p, f);
  return make(rotateInverse(tool_in_sensor.R, f), rotateInverse(tool_in_sensor.R, t));
}

core_types::Wrench gravityWrench(
  const Payload & payload, const Mat3 & r_world_sensor, const Vec3 & gravity_world)
{
  const Vec3 weight = payload.mass_kg * rotateInverse(r_world_sensor, gravity_world);   // in S
  return make(weight, cross(payload.cog, weight));
}

WrenchConditioner::WrenchConditioner(ConditionerConfig config)
: config_(std::move(config))
{
}

core_types::Wrench WrenchConditioner::condition(
  const core_types::Wrench & raw, const Mat3 & r_world_sensor) const
{
  const Vec3 f0 = config_.sign * force(raw) - force(config_.bias);
  const Vec3 t0 = config_.sign * torque(raw) - torque(config_.bias);
  const core_types::Wrench g = gravityWrench(config_.payload, r_world_sensor, config_.gravity_world);
  return moveWrench(make(f0 - force(g), t0 - torque(g)), config_.tool_in_sensor);
}

namespace
{
// Solve the n x n system a x = b (row-major a, destroyed) by Gaussian elimination with partial
// pivoting. False when the matrix is singular relative to its own scale.
bool solve(std::vector<double> a, std::vector<double> b, std::size_t n, std::vector<double> & x)
{
  double scale = 0.0;
  for (const double v : a) {scale = std::max(scale, std::abs(v));}
  if (scale < 1e-300) {return false;}
  for (std::size_t col = 0; col < n; ++col) {
    std::size_t piv = col;
    for (std::size_t r = col + 1; r < n; ++r) {
      if (std::abs(a[r * n + col]) > std::abs(a[piv * n + col])) {piv = r;}
    }
    if (std::abs(a[piv * n + col]) < 1e-9 * scale) {return false;}
    if (piv != col) {
      for (std::size_t c = 0; c < n; ++c) {std::swap(a[col * n + c], a[piv * n + c]);}
      std::swap(b[col], b[piv]);
    }
    for (std::size_t r = col + 1; r < n; ++r) {
      const double f = a[r * n + col] / a[col * n + col];
      for (std::size_t c = col; c < n; ++c) {a[r * n + c] -= f * a[col * n + c];}
      b[r] -= f * b[col];
    }
  }
  x.assign(n, 0.0);
  for (std::size_t i = n; i-- > 0;) {
    double s = b[i];
    for (std::size_t c = i + 1; c < n; ++c) {s -= a[i * n + c] * x[c];}
    x[i] = s / a[i * n + i];
  }
  return true;
}

// Least squares via the normal equations for rows `rows` (each row has n coefficients) and targets y.
bool leastSquares(
  const std::vector<std::vector<double>> & rows, const std::vector<double> & y, std::size_t n,
  std::vector<double> & x)
{
  std::vector<double> ata(n * n, 0.0), aty(n, 0.0);
  for (std::size_t k = 0; k < rows.size(); ++k) {
    for (std::size_t i = 0; i < n; ++i) {
      aty[i] += rows[k][i] * y[k];
      for (std::size_t j = 0; j < n; ++j) {ata[i * n + j] += rows[k][i] * rows[k][j];}
    }
  }
  return solve(ata, aty, n, x);
}
}  // namespace

namespace
{
constexpr double kMinPoseSpreadRad = 0.17;   // about 10 degrees between the most different poses
constexpr double kMinPayloadMassKg = 1e-3;   // below this a negative fit is noise around zero payload
}  // namespace

CalibrationResult identifyPayload(
  const std::vector<CalibrationSample> & samples, const Vec3 & gravity_world, double sign,
  double max_condition)
{
  CalibrationResult out;
  out.samples = samples.size();
  if (samples.size() < 4) {
    out.message = "need at least 4 static poses, got " + std::to_string(samples.size());
    return out;
  }
  // Gravity must point in clearly different directions in the sensor frame: the largest angle
  // between any two samples has to exceed a physical minimum, else mass, centre of gravity and bias
  // are only separated by noise (the normal equations alone pass such sets).
  std::vector<Vec3> dirs;
  for (const auto & s : samples) {dirs.push_back(rotateInverse(s.r_world_sensor, gravity_world));}
  const auto dot = [](const Vec3 & a, const Vec3 & b) {return a.x * b.x + a.y * b.y + a.z * b.z;};
  double max_angle = 0.0;
  for (std::size_t i = 0; i < dirs.size(); ++i) {
    for (std::size_t j = i + 1; j < dirs.size(); ++j) {
      const double ni = std::sqrt(dot(dirs[i], dirs[i])), nj = std::sqrt(dot(dirs[j], dirs[j]));
      if (ni < 1e-12 || nj < 1e-12) {continue;}
      max_angle = std::max(max_angle, std::acos(std::clamp(dot(dirs[i], dirs[j]) / (ni * nj), -1.0, 1.0)));
    }
  }
  if (max_angle < kMinPoseSpreadRad) {
    out.message = "the poses do not excite the fit: gravity directions in the sensor frame span only " +
      std::to_string(max_angle * 180.0 / M_PI) + " deg, need at least " +
      std::to_string(kMinPoseSpreadRad * 180.0 / M_PI) + " deg";
    return out;
  }
  {
    std::vector<Mat3> rs;
    for (const auto & s : samples) {rs.push_back(s.r_world_sensor);}
    out.condition_number = regressorConditionNumber(rs, gravity_world);
    if (out.condition_number > max_condition) {
      out.message = "the poses do not excite the fit: regressor condition number " +
        std::to_string(out.condition_number) + " exceeds " + std::to_string(max_condition) +
        " (spread the orientations: more poses, rotation about at least two axes)";
      return out;
    }
  }
  // y = sign * raw. Force:  F = m g_s + b_F        unknowns [m, bFx, bFy, bFz]
  //                 Torque: T = u x g_s + b_T      unknowns [u = m c (3), bTx, bTy, bTz]
  std::vector<std::vector<double>> frows, trows;
  std::vector<double> fy, ty;
  for (const auto & s : samples) {
    const Vec3 g = rotateInverse(s.r_world_sensor, gravity_world);
    const Vec3 f = sign * force(s.raw), t = sign * torque(s.raw);
    frows.push_back({g.x, 1, 0, 0}); fy.push_back(f.x);
    frows.push_back({g.y, 0, 1, 0}); fy.push_back(f.y);
    frows.push_back({g.z, 0, 0, 1}); fy.push_back(f.z);
    trows.push_back({0, g.z, -g.y, 1, 0, 0}); ty.push_back(t.x);
    trows.push_back({-g.z, 0, g.x, 0, 1, 0}); ty.push_back(t.y);
    trows.push_back({g.y, -g.x, 0, 0, 0, 1}); ty.push_back(t.z);
  }
  std::vector<double> th_f, th_t;
  if (!leastSquares(frows, fy, 4, th_f) || !leastSquares(trows, ty, 6, th_t)) {
    out.message = "the poses do not excite the fit: gravity must point in clearly different directions "
      "in the sensor frame (rotate about at least two non-parallel axes)";
    return out;
  }
  double sf = 0.0, st = 0.0;
  for (std::size_t k = 0; k < frows.size(); ++k) {
    double e = -fy[k];
    for (std::size_t i = 0; i < 4; ++i) {e += frows[k][i] * th_f[i];}
    sf += e * e;
    double e2 = -ty[k];
    for (std::size_t i = 0; i < 6; ++i) {e2 += trows[k][i] * th_t[i];}
    st += e2 * e2;
  }
  out.rms_force_n = std::sqrt(sf / static_cast<double>(frows.size()));
  out.rms_torque_nm = std::sqrt(st / static_cast<double>(trows.size()));
  if (th_f[0] < -kMinPayloadMassKg) {
    out.message = "negative mass (" + std::to_string(th_f[0]) +
      " kg): the sensor sign is probably wrong, repeat with sign = -1";
    return out;
  }
  out.payload.mass_kg = th_f[0];
  out.bias = make({th_f[1], th_f[2], th_f[3]}, {th_t[3], th_t[4], th_t[5]});
  if (std::abs(th_f[0]) > 1e-6) {
    out.payload.cog = {th_t[0] / th_f[0], th_t[1] / th_f[0], th_t[2] / th_f[0]};
  }
  out.ok = true;
  return out;
}

}  // namespace force_core
