#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <random>
#include <vector>

#include "core_types/plane_touch_calibration.hpp"
#include "core_types/tool_footprint.hpp"
#include "core_types/tool_profile.hpp"

using core_types::Pose;
using core_types::Vec3;

namespace
{
constexpr double kDeg = M_PI / 180.0;

struct Quat
{
  double x, y, z, w;
};

Quat axisAngle(double ax, double ay, double az, double angle)
{
  const double n = std::sqrt(ax * ax + ay * ay + az * az);
  const double s = std::sin(angle / 2) / n;
  return {ax * s, ay * s, az * s, std::cos(angle / 2)};
}

Quat mul(const Quat & a, const Quat & b)
{
  return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

Pose toPose(const Quat & q, const Vec3 & p)
{
  Pose o;
  o.x = p[0]; o.y = p[1]; o.z = p[2];
  o.qx = q.x; o.qy = q.y; o.qz = q.z; o.qw = q.w;
  return o;
}

std::array<double, 9> matrix(const Quat & q)
{
  std::array<double, 9> r;
  core_types::detail::quatToMatrix(toPose(q, {0, 0, 0}), r);
  return r;
}

// Plane z = 0 (outward normal +z), tool pointing down. Touches at tilt cones around the downward axis.
struct Scene
{
  Vec3 tip{0.01, -0.02, 0.18};
  double max_tilt_deg{50.0};
  double noise_m{0.0005};
  unsigned seed{7};
  int n{40};
  Vec3 normal{0, 0, 1};
};

std::vector<Pose> simulate(const Scene & sc, bool vary_about_both = true)
{
  std::mt19937 rng(sc.seed);
  std::uniform_real_distribution<double> ang(-sc.max_tilt_deg * kDeg, sc.max_tilt_deg * kDeg);
  std::uniform_real_distribution<double> pos(-0.3, 0.3);
  std::normal_distribution<double> g(0.0, sc.noise_m);
  // Rotation taking flange z to -normal for normal = +z: 180 deg about x. Other normals: general.
  const Quat down = axisAngle(1, 0, 0, M_PI);
  std::vector<Pose> poses;
  for (int i = 0; i < sc.n; ++i) {
    const double a = ang(rng);
    const double b = vary_about_both ? ang(rng) : 0.0;
    const Quat q = mul(mul(axisAngle(1, 0, 0, a), axisAngle(0, 1, 0, b)), down);
    const auto r = matrix(q);
    const Vec3 rt = core_types::detail::mulMat(r, sc.tip);
    const Vec3 on_plane{pos(rng), pos(rng), 0.0};
    Vec3 p{on_plane[0] - rt[0] + g(rng), on_plane[1] - rt[1] + g(rng), on_plane[2] - rt[2] + g(rng)};
    poses.push_back(toPose(q, p));
  }
  return poses;
}

double dist(const Vec3 & a, const Vec3 & b)
{
  return core_types::detail::norm3(core_types::detail::sub3(a, b));
}
}  // namespace

TEST(PlaneTouch, RecoversTipOffsetWithNoise)
{
  Scene sc;
  const auto res = core_types::planeTouchCalibration(simulate(sc), {});
  ASSERT_TRUE(res.ok) << res.error;
  EXPECT_LT(dist(res.tcp_xyz, sc.tip), 0.0015);
  EXPECT_NEAR(res.normal[2], 1.0, 1e-2);
  EXPECT_NEAR(res.plane_offset_m, 0.0, 0.002);
  EXPECT_EQ(res.touches_used, 40u);
}

TEST(PlaneTouch, ExactWithoutNoise)
{
  Scene sc;
  sc.noise_m = 0.0;
  const auto res = core_types::planeTouchCalibration(simulate(sc), {});
  ASSERT_TRUE(res.ok) << res.error;
  EXPECT_LT(dist(res.tcp_xyz, sc.tip), 1e-8);
  EXPECT_LT(res.rms_m, 1e-9);
}

TEST(PlaneTouch, ReportsResidualsAndCondition)
{
  Scene sc;
  const auto res = core_types::planeTouchCalibration(simulate(sc), {});
  ASSERT_TRUE(res.ok) << res.error;
  EXPECT_GT(res.rms_m, 1e-5);
  EXPECT_LT(res.rms_m, 0.002);
  EXPECT_GE(res.max_residual_m, res.rms_m);
  EXPECT_GT(res.condition_number, 1.0);
  EXPECT_LT(res.condition_number, 1e4);
  EXPECT_GT(res.orientation_variation_deg, 20.0);
}

TEST(PlaneTouch, SeedSweepMeanAndWorstCase)
{
  // 0.5 mm per-axis noise, 40 touches, tool tilted up to 50 deg about both axes. The worst seed is
  // a few millimetres (the offset along the normal is weakly observable), the mean is well under 1 mm.
  double sum = 0.0, worst = 0.0;
  for (unsigned seed = 1; seed <= 30; ++seed) {
    Scene sc;
    sc.seed = seed;
    const auto res = core_types::planeTouchCalibration(simulate(sc), {});
    ASSERT_TRUE(res.ok) << "seed " << seed << ": " << res.error;
    const double e = dist(res.tcp_xyz, sc.tip);
    sum += e;
    worst = std::max(worst, e);
  }
  EXPECT_LT(sum / 30.0, 0.001);
  EXPECT_LT(worst, 0.003);
}

TEST(PlaneTouch, MoreTouchesReduceTheError)
{
  double few = 0.0, many = 0.0;
  for (unsigned seed = 1; seed <= 20; ++seed) {
    Scene a;
    a.seed = seed;
    a.n = 12;
    Scene b = a;
    b.n = 80;
    few += dist(core_types::planeTouchCalibration(simulate(a), {}).tcp_xyz, a.tip);
    many += dist(core_types::planeTouchCalibration(simulate(b), {}).tcp_xyz, a.tip);
  }
  EXPECT_LT(many, few);
}

TEST(PlaneTouch, KnownNormalRecoversTip)
{
  Scene sc;
  core_types::PlaneTouchOptions opt;
  opt.known_normal = Vec3{0, 0, 2.0};  // not unit on purpose
  const auto res = core_types::planeTouchCalibration(simulate(sc), {}, opt);
  ASSERT_TRUE(res.ok) << res.error;
  EXPECT_LT(dist(res.tcp_xyz, sc.tip), 0.0015);
  EXPECT_NEAR(res.normal[2], 1.0, 1e-12);
}

TEST(PlaneTouch, KnownNormalNeedsFewerTouches)
{
  Scene sc;
  sc.n = 5;
  sc.noise_m = 0.0;
  core_types::PlaneTouchOptions opt;
  opt.known_normal = Vec3{0, 0, 1};
  const auto ok = core_types::planeTouchCalibration(simulate(sc), {}, opt);
  EXPECT_TRUE(ok.ok) << ok.error;
  const auto unknown = core_types::planeTouchCalibration(simulate(sc), {});
  EXPECT_FALSE(unknown.ok);
}

TEST(PlaneTouch, SmallButSufficientVariationIsAcceptedWithLargerError)
{
  Scene sc;
  sc.max_tilt_deg = 15.0;  // total spread above the 20 degree limit, accuracy is worse
  const auto res = core_types::planeTouchCalibration(simulate(sc), {});
  ASSERT_TRUE(res.ok) << res.error;
  EXPECT_GE(res.orientation_variation_deg, 20.0);
  EXPECT_LT(dist(res.tcp_xyz, sc.tip), 0.01);
}

TEST(PlaneTouch, RejectsLowOrientationVariation)
{
  Scene sc;
  sc.max_tilt_deg = 3.0;  // far below 20 deg total
  const auto res = core_types::planeTouchCalibration(simulate(sc), {});
  EXPECT_FALSE(res.ok);
  EXPECT_LT(res.orientation_variation_deg, 20.0);
  EXPECT_NE(res.error.find("orientation variation"), std::string::npos);
}

TEST(PlaneTouch, RejectsLowVariationWithKnownNormal)
{
  Scene sc;
  sc.max_tilt_deg = 3.0;
  core_types::PlaneTouchOptions opt;
  opt.known_normal = Vec3{0, 0, 1};
  EXPECT_FALSE(core_types::planeTouchCalibration(simulate(sc), {}, opt).ok);
}

TEST(PlaneTouch, RejectsRotationAboutOneAxis)
{
  Scene sc;
  sc.max_tilt_deg = 35.0;
  const auto res = core_types::planeTouchCalibration(simulate(sc, false), {});
  EXPECT_FALSE(res.ok);  // variation is large but about one axis only: ill-conditioned
  EXPECT_FALSE(res.error.empty());
}

TEST(PlaneTouch, VariationThresholdIsAnOption)
{
  Scene sc;
  sc.max_tilt_deg = 6.0;
  sc.noise_m = 0.0;
  core_types::PlaneTouchOptions opt;
  opt.min_orientation_variation_deg = 5.0;
  opt.known_normal = Vec3{0, 0, 1};
  const auto res = core_types::planeTouchCalibration(simulate(sc), {}, opt);
  EXPECT_TRUE(res.ok) << res.error;
}

TEST(PlaneTouch, DropsTouchesWithoutForceConfirmation)
{
  Scene sc;
  sc.noise_m = 0.0;
  auto poses = simulate(sc);
  std::vector<bool> ok(poses.size(), true);
  // Corrupt two touches (tip nowhere near the plane) and mark them unconfirmed.
  poses[2].z += 0.05;
  poses[9].z -= 0.04;
  ok[2] = false;
  ok[9] = false;
  const auto res = core_types::planeTouchCalibration(poses, ok);
  ASSERT_TRUE(res.ok) << res.error;
  EXPECT_EQ(res.touches_used, poses.size() - 2);
  EXPECT_LT(dist(res.tcp_xyz, sc.tip), 1e-7);
  // Without the flags the corrupted touches spoil the fit.
  const auto bad = core_types::planeTouchCalibration(poses, {});
  EXPECT_GT(bad.rms_m, 1e-3);
}

TEST(PlaneTouch, TooFewConfirmedTouches)
{
  Scene sc;
  const auto poses = simulate(sc);
  std::vector<bool> ok(poses.size(), false);
  ok[0] = ok[1] = true;
  const auto res = core_types::planeTouchCalibration(poses, ok);
  EXPECT_FALSE(res.ok);
  EXPECT_NE(res.error.find("at least"), std::string::npos);
}

TEST(PlaneTouch, FlagSizeMismatchIsAnError)
{
  Scene sc;
  const auto res = core_types::planeTouchCalibration(simulate(sc), std::vector<bool>(3, true));
  EXPECT_FALSE(res.ok);
}

TEST(PlaneTouch, RejectsZeroQuaternionAndNonFinitePose)
{
  Scene sc;
  auto poses = simulate(sc);
  poses[1].qx = poses[1].qy = poses[1].qz = poses[1].qw = 0.0;
  EXPECT_FALSE(core_types::planeTouchCalibration(poses, {}).ok);
  poses = simulate(sc);
  poses[0].x = std::nan("");
  EXPECT_FALSE(core_types::planeTouchCalibration(poses, {}).ok);
}

TEST(PlaneTouch, RejectsZeroKnownNormal)
{
  Scene sc;
  core_types::PlaneTouchOptions opt;
  opt.known_normal = Vec3{0, 0, 0};
  EXPECT_FALSE(core_types::planeTouchCalibration(simulate(sc), {}, opt).ok);
}

TEST(PlaneTouch, StartingGuessFarFromTruthStillConverges)
{
  Scene sc;
  core_types::PlaneTouchOptions opt;
  opt.initial_tcp_xyz = {0.0, 0.0, 0.0};
  const auto res = core_types::planeTouchCalibration(simulate(sc), {}, opt);
  ASSERT_TRUE(res.ok) << res.error;
  EXPECT_LT(dist(res.tcp_xyz, sc.tip), 0.0015);
}

TEST(PlaneTouch, TiltedPlaneNormalIsRecovered)
{
  // Rotate the whole scene about y by 10 degrees: tool poses and plane both tilt.
  Scene sc;
  sc.noise_m = 0.0;
  auto poses = simulate(sc);
  const Quat rot = axisAngle(0, 1, 0, 10.0 * kDeg);
  const auto rm = matrix(rot);
  for (auto & p : poses) {
    const Quat q = mul(rot, Quat{p.qx, p.qy, p.qz, p.qw});
    const Vec3 np = core_types::detail::mulMat(rm, Vec3{p.x, p.y, p.z});
    p = toPose(q, np);
  }
  const auto res = core_types::planeTouchCalibration(poses, {});
  ASSERT_TRUE(res.ok) << res.error;
  EXPECT_LT(dist(res.tcp_xyz, sc.tip), 1e-7);
  const Vec3 expect = core_types::detail::mulMat(rm, Vec3{0, 0, 1});
  EXPECT_LT(dist(res.normal, expect), 1e-7);
}

TEST(PlaneTouch, PivotCalibrationStillWorksAfterSolverExtraction)
{
  // The shared Gauss-Jordan solver: a known 3x3 system.
  double m[3][4] = {{2, 1, 0, 3}, {1, 3, 1, 8}, {0, 1, 4, 9}};
  double x[3];
  ASSERT_TRUE(core_types::detail::solveAugmented<3>(m, x));
  EXPECT_NEAR(2 * x[0] + x[1], 3, 1e-12);
  EXPECT_NEAR(x[0] + 3 * x[1] + x[2], 8, 1e-12);
  EXPECT_NEAR(x[1] + 4 * x[2], 9, 1e-12);
  double sing[2][3] = {{1, 2, 3}, {2, 4, 6}};
  double xs[2];
  EXPECT_FALSE(core_types::detail::solveAugmented<2>(sing, xs));
}

// ---- tool tilt fit

namespace
{
// Tool with a mounting tilt `err` about tool-x: true axis = R_i Rx(err) ez, normal is its opposite.
std::vector<core_types::ToolTiltTouch> tiltTouches(
  const Vec3 & err_axis, double err_deg, bool with_pose, double noise_deg = 0.0, unsigned seed = 3)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> ang(-25 * kDeg, 25 * kDeg);
  std::normal_distribution<double> g(0.0, noise_deg * kDeg);
  std::vector<core_types::ToolTiltTouch> out;
  const Quat down = axisAngle(1, 0, 0, M_PI);
  const Quat err = axisAngle(err_axis[0], err_axis[1], err_axis[2], err_deg * kDeg);
  for (int i = 0; i < 8; ++i) {
    const Quat q = mul(mul(axisAngle(1, 0, 0, ang(rng)), axisAngle(0, 1, 0, ang(rng))), down);
    const auto r = matrix(q);
    core_types::ToolTiltTouch t;
    t.commanded_axis = core_types::detail::mulMat(r, Vec3{0, 0, 1});
    const Vec3 true_axis = core_types::detail::mulMat(matrix(mul(q, err)), Vec3{0, 0, 1});
    Vec3 n = core_types::detail::scale3(true_axis, -1.0);
    n[0] += g(rng); n[1] += g(rng); n[2] += g(rng);
    t.measured_normal = core_types::detail::normalized3(n);
    if (with_pose) {
      t.tool_pose = toPose(q, {0, 0, 0});
    }
    out.push_back(t);
  }
  return out;
}
}  // namespace

TEST(ToolTilt, RecoversTwoDegreesAboutToolX)
{
  const auto fit = core_types::fitToolTilt(tiltTouches({1, 0, 0}, 2.0, true));
  ASSERT_TRUE(fit.ok) << fit.error;
  EXPECT_TRUE(fit.in_tool_frame);
  EXPECT_NEAR(fit.about_x_deg, 2.0, 0.3);
  EXPECT_NEAR(fit.about_y_deg, 0.0, 0.3);
  EXPECT_NEAR(fit.tilt_error_deg, 2.0, 0.3);
}

TEST(ToolTilt, RecoversTenDegreesAboutToolY)
{
  const auto fit = core_types::fitToolTilt(tiltTouches({0, 1, 0}, 10.0, true));
  ASSERT_TRUE(fit.ok) << fit.error;
  EXPECT_NEAR(fit.about_y_deg, 10.0, 0.3);
  EXPECT_NEAR(fit.about_x_deg, 0.0, 0.3);
}

TEST(ToolTilt, RecoversMixedAxisTilt)
{
  const auto fit = core_types::fitToolTilt(tiltTouches({1, 1, 0}, 10.0, true));
  ASSERT_TRUE(fit.ok) << fit.error;
  EXPECT_NEAR(fit.tilt_error_deg, 10.0, 0.3);
  EXPECT_NEAR(fit.about_x_deg, 10.0 / std::sqrt(2.0), 0.3);
  EXPECT_NEAR(fit.about_y_deg, 10.0 / std::sqrt(2.0), 0.3);
}

TEST(ToolTilt, NegativeTiltHasNegativeSign)
{
  const auto fit = core_types::fitToolTilt(tiltTouches({1, 0, 0}, -4.0, true));
  ASSERT_TRUE(fit.ok) << fit.error;
  EXPECT_NEAR(fit.about_x_deg, -4.0, 0.3);
}

TEST(ToolTilt, RollIsReportedUnobservable)
{
  const auto fit = core_types::fitToolTilt(tiltTouches({1, 0, 0}, 2.0, true));
  ASSERT_TRUE(fit.ok);
  EXPECT_FALSE(fit.roll_observable);
  EXPECT_DOUBLE_EQ(fit.correction_rotvec[2], 0.0);
}

TEST(ToolTilt, RollAboutAxisDoesNotChangeTheEstimate)
{
  // Same tilt, but the commanded axis comes with a roll about itself: result is identical.
  const auto a = core_types::fitToolTilt(tiltTouches({1, 0, 0}, 5.0, false));
  ASSERT_TRUE(a.ok);
  EXPECT_FALSE(a.roll_observable);
  EXPECT_NEAR(a.tilt_error_deg, 5.0, 0.3);
}

TEST(ToolTilt, BaseFrameModeWithoutPoses)
{
  const auto fit = core_types::fitToolTilt(tiltTouches({1, 0, 0}, 3.0, false));
  ASSERT_TRUE(fit.ok) << fit.error;
  EXPECT_FALSE(fit.in_tool_frame);
  EXPECT_GT(fit.tilt_error_deg, 0.5);
  // The per-touch base-frame corrections differ (the error is fixed in the tool, not the base).
  EXPECT_GT(fit.spread_deg, 0.1);
}

TEST(ToolTilt, ToolFrameSpreadIsSmallForAFixedError)
{
  const auto fit = core_types::fitToolTilt(tiltTouches({1, 0, 0}, 3.0, true));
  ASSERT_TRUE(fit.ok);
  EXPECT_LT(fit.spread_deg, 0.05);
}

TEST(ToolTilt, NormalNoiseKeepsAccuracy)
{
  const auto fit = core_types::fitToolTilt(tiltTouches({0, 1, 0}, 10.0, true, 0.3));
  ASSERT_TRUE(fit.ok) << fit.error;
  EXPECT_NEAR(fit.about_y_deg, 10.0, 0.3);
  EXPECT_GT(fit.spread_deg, 0.0);
}

TEST(ToolTilt, ZeroErrorGivesZero)
{
  const auto fit = core_types::fitToolTilt(tiltTouches({1, 0, 0}, 0.0, true));
  ASSERT_TRUE(fit.ok);
  EXPECT_LT(fit.tilt_error_deg, 1e-6);
}

TEST(ToolTilt, RejectsEmptyAndDegenerateInput)
{
  EXPECT_FALSE(core_types::fitToolTilt({}).ok);
  auto t = tiltTouches({1, 0, 0}, 2.0, true);
  EXPECT_FALSE(core_types::fitToolTilt(t, 20).ok);
  t[0].measured_normal = {0, 0, 0};
  EXPECT_FALSE(core_types::fitToolTilt(t).ok);
}

TEST(ToolTilt, RejectsAxisPointingAwayFromSurface)
{
  core_types::ToolTiltTouch t;
  t.commanded_axis = {0, 0, 1};
  t.measured_normal = {0, 0, 1};  // inward axis is -normal = -z, opposite of the commanded +z
  EXPECT_FALSE(core_types::fitToolTilt({t}).ok);
}

TEST(ToolTilt, FitFromPlaneTouchNormal)
{
  // Chain: plane touch gives the normal; commanded axes are all -z (tool down), plane tilted 2 deg.
  Scene sc;
  sc.noise_m = 0.0;
  auto poses = simulate(sc);
  const Quat rot = axisAngle(0, 1, 0, 2.0 * kDeg);
  const auto rm = matrix(rot);
  for (auto & p : poses) {
    const Vec3 np = core_types::detail::mulMat(rm, Vec3{p.x, p.y, p.z});
    p = toPose(mul(rot, Quat{p.qx, p.qy, p.qz, p.qw}), np);
  }
  const auto plane = core_types::planeTouchCalibration(poses, {});
  ASSERT_TRUE(plane.ok) << plane.error;
  core_types::ToolTiltTouch t;
  t.commanded_axis = {0, 0, -1};
  t.measured_normal = plane.normal;
  const auto fit = core_types::fitToolTilt({t});
  ASSERT_TRUE(fit.ok);
  EXPECT_NEAR(fit.tilt_error_deg, 2.0, 0.05);
}

// ---- footprint

TEST(ToolFootprint, TiltZeroIsIdentity)
{
  EXPECT_DOUBLE_EQ(core_types::tiltedSwathAcrossStroke(0.2, 0.1, 0.0, 33.0), 0.2);
}

TEST(ToolFootprint, TiltAlongStrokeLeavesSwath)
{
  EXPECT_NEAR(core_types::tiltedSwathAcrossStroke(0.2, 0.1, 40.0, 0.0), 0.2, 1e-12);
  EXPECT_NEAR(core_types::tiltedSwathAcrossStroke(0.2, 0.1, 40.0, 180.0), 0.2, 1e-12);
}

TEST(ToolFootprint, TiltAcrossStrokeElongates)
{
  EXPECT_NEAR(core_types::tiltedSwathAcrossStroke(0.2, 0.1, 30.0, 90.0), 0.2 / std::cos(30 * kDeg), 1e-12);
  EXPECT_NEAR(core_types::tiltedSwathAcrossStroke(0.2, 0.0, 60.0, 90.0), 0.4, 1e-12);
}

TEST(ToolFootprint, ObliqueDirectionIsBetween)
{
  const double w = core_types::tiltedSwathAcrossStroke(0.2, 0.0, 45.0, 45.0);
  EXPECT_GT(w, 0.2);
  EXPECT_LT(w, 0.2 / std::cos(45 * kDeg));
}

TEST(ToolFootprint, PadLengthAddsAtOblique)
{
  const double no_len = core_types::tiltedSwathAcrossStroke(0.2, 0.0, 45.0, 45.0);
  const double with_len = core_types::tiltedSwathAcrossStroke(0.2, 0.3, 45.0, 45.0);
  EXPECT_GT(with_len, no_len);
}

TEST(ToolFootprint, RejectsBadInput)
{
  EXPECT_THROW(core_types::tiltedSwathAcrossStroke(-0.1, 0, 10, 0), std::invalid_argument);
  EXPECT_THROW(core_types::tiltedSwathAcrossStroke(0.1, 0, 90, 0), std::invalid_argument);
  EXPECT_THROW(core_types::tiltedSwathAcrossStroke(0.1, 0, -1, 0), std::invalid_argument);
  EXPECT_THROW(core_types::tiltedSwathAcrossStroke(0.1, 0, 10, std::nan("")), std::invalid_argument);
}

TEST(ToolGeometry, TiltDefaultsAreZero)
{
  core_types::ToolGeometry g;
  EXPECT_EQ(g.tilt_deg, 0.0);
  EXPECT_EQ(g.tilt_axis, (std::array<double, 3>{0, 0, 0}));
}
