// Fixed tool tilt: wall pose lift, composition with the edge tilt, swath projection, loading.

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include <Eigen/Dense>

#include "coverage_planner/core/tool_profile_loading.hpp"
#include "coverage_planner/core/wall_basis.hpp"
#include "coverage_planner/core/wall_pose_lift.hpp"
#include "ros_adapters/tool_profile_yaml_loader.hpp"

using namespace coverage_planner;  // NOLINT

namespace
{
constexpr double kDeg = M_PI / 180.0;

WallPlane makeWall()
{
  // basis_u = world +X, basis_v = world +Z, normal_outward = world -Y.
  WallPlane wall;
  wall.id = 0;
  wall.pose.header.frame_id = "map";
  wall.pose.pose = encodeWallBasis(
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0),
    Eigen::Vector3d(0.0, -1.0, 0.0));
  wall.u_min = 0.0;
  wall.u_max = 2.0;
  wall.v_min = 0.0;
  wall.v_max = 2.0;
  return wall;
}

Path flatPath(double u, double v)
{
  PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = u;
  p.pose.position.z = v;
  p.pose.orientation.w = 1.0;
  Path path;
  path.poses.push_back(p);
  return path;
}

Eigen::Matrix3d rotationOf(const Pose & pose)
{
  return Eigen::Quaterniond(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
         .toRotationMatrix();
}

ToolTask contactTask(double tilt_deg = 0.0, std::array<double, 3> axis = {1, 0, 0})
{
  ToolTask t;
  t.contact = true;
  t.tool_tilt_deg = tilt_deg;
  t.tool_tilt_axis = axis;
  return t;
}

ToolTask sprayTask(double standoff, double edge_tilt, double tilt_deg = 0.0,
  std::array<double, 3> axis = {1, 0, 0})
{
  ToolTask t;
  t.contact = false;
  t.standoff_m = standoff;
  t.tool_normal_policy = "tilt_at_edges";
  t.max_edge_tilt_deg = edge_tilt;
  t.tool_tilt_deg = tilt_deg;
  t.tool_tilt_axis = axis;
  return t;
}

double angleBetween(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  return std::acos(std::clamp(a.normalized().dot(b.normalized()), -1.0, 1.0)) / kDeg;
}
}  // namespace

// ---- lift

TEST(ToolTiltLift, ZeroTiltIsBitIdenticalToUntiltedLift)
{
  const auto wall = makeWall();
  ToolTask plain;
  plain.contact = true;
  ToolTask zero = contactTask(0.0, {0.3, 0.9, 0.0});  // a stale axis must not matter at tilt 0
  Path flat = flatPath(0.7, 1.1);
  const auto a = liftCoveragePathPose(flat, wall, plain);
  const auto b = liftCoveragePathPose(flat, wall, zero);
  ASSERT_EQ(a.poses.size(), 1u);
  EXPECT_EQ(std::memcmp(&a.poses[0].pose, &b.poses[0].pose, sizeof(Pose)), 0);
}

TEST(ToolTiltLift, ZeroTiltIdenticalForEdgeTiltToo)
{
  const auto wall = makeWall();
  Path flat = flatPath(1.0, 0.05);
  const auto a = liftCoveragePathPose(flat, wall, sprayTask(0.2, 30.0));
  const auto b = liftCoveragePathPose(flat, wall, sprayTask(0.2, 30.0, 0.0, {0, 1, 0}));
  EXPECT_EQ(std::memcmp(&a.poses[0].pose, &b.poses[0].pose, sizeof(Pose)), 0);
}

TEST(ToolTiltLift, TiltAboutUAxisTiltsTheToolAxisByThatAngle)
{
  const auto wall = makeWall();
  const auto lifted = liftCoveragePathPose(flatPath(1.0, 1.0), wall, contactTask(25.0, {1, 0, 0}));
  const Eigen::Matrix3d r = rotationOf(lifted.poses[0].pose);
  const Eigen::Vector3d z = r.col(2);
  EXPECT_NEAR(angleBetween(z, Eigen::Vector3d(0, 1, 0)), 25.0, 1e-9);
  // Tilting about the wall's u axis moves z within the (normal, v) plane: no world-X component.
  EXPECT_NEAR(z.x(), 0.0, 1e-12);
  EXPECT_GT(std::fabs(z.z()), 0.3);
}

TEST(ToolTiltLift, TiltAboutVAxisTiltsInTheNormalUPlane)
{
  const auto wall = makeWall();
  const auto lifted = liftCoveragePathPose(flatPath(1.0, 1.0), wall, contactTask(25.0, {0, 1, 0}));
  const Eigen::Vector3d z = rotationOf(lifted.poses[0].pose).col(2);
  EXPECT_NEAR(angleBetween(z, Eigen::Vector3d(0, 1, 0)), 25.0, 1e-9);
  EXPECT_NEAR(z.z(), 0.0, 1e-12);
  EXPECT_GT(std::fabs(z.x()), 0.3);
}

TEST(ToolTiltLift, OppositeAxesTiltOppositeWays)
{
  const auto wall = makeWall();
  const Eigen::Vector3d zp = rotationOf(
    liftCoveragePathPose(flatPath(1, 1), wall, contactTask(20.0, {1, 0, 0})).poses[0].pose).col(2);
  const Eigen::Vector3d zn = rotationOf(
    liftCoveragePathPose(flatPath(1, 1), wall, contactTask(20.0, {-1, 0, 0})).poses[0].pose).col(2);
  EXPECT_NEAR(zp.z(), -zn.z(), 1e-12);
  EXPECT_NEAR(zp.y(), zn.y(), 1e-12);
}

TEST(ToolTiltLift, NegativeTiltEqualsPositiveAboutOppositeAxis)
{
  const auto wall = makeWall();
  const auto a = liftCoveragePathPose(flatPath(1, 1), wall, contactTask(-20.0, {1, 0, 0}));
  const auto b = liftCoveragePathPose(flatPath(1, 1), wall, contactTask(20.0, {-1, 0, 0}));
  EXPECT_LT((rotationOf(a.poses[0].pose) - rotationOf(b.poses[0].pose)).norm(), 1e-12);
}

TEST(ToolTiltLift, ObliqueAxisAndUnnormalisedAxis)
{
  const auto wall = makeWall();
  const auto lifted = liftCoveragePathPose(flatPath(1, 1), wall, contactTask(15.0, {3, 4, 0}));
  const Eigen::Vector3d z = rotationOf(lifted.poses[0].pose).col(2);
  EXPECT_NEAR(angleBetween(z, Eigen::Vector3d(0, 1, 0)), 15.0, 1e-9);
}

TEST(ToolTiltLift, FrameStaysOrthonormalRightHanded)
{
  const auto wall = makeWall();
  const auto lifted = liftCoveragePathPose(flatPath(1, 1), wall, contactTask(40.0, {1, 1, 0}));
  const Eigen::Matrix3d r = rotationOf(lifted.poses[0].pose);
  EXPECT_NEAR((r * r.transpose() - Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(r.determinant(), 1.0, 1e-12);
}

TEST(ToolTiltLift, PositionStaysAtContactPointAndStandoffUnchanged)
{
  const auto wall = makeWall();
  const auto c = liftCoveragePathPose(flatPath(1.0, 1.0), wall, contactTask(30.0, {1, 0, 0})).poses[0];
  EXPECT_NEAR(c.pose.position.x, 1.0, 1e-12);
  EXPECT_NEAR(c.pose.position.y, 0.0, 1e-12);
  EXPECT_NEAR(c.pose.position.z, 1.0, 1e-12);
  const auto s = liftCoveragePathPose(flatPath(1.0, 1.0), wall, sprayTask(0.2, 0.0, 30.0)).poses[0];
  EXPECT_NEAR(s.pose.position.y, -0.2, 1e-12);  // standoff along the surface normal, not the tilted axis
}

TEST(ToolTiltLift, EveryPoseOfAPathGetsTheSameTilt)
{
  const auto wall = makeWall();
  Path flat = flatPath(0.5, 0.5);
  flat.poses.push_back(flatPath(1.5, 1.2).poses[0]);
  const auto lifted = liftCoveragePathPose(flat, wall, contactTask(18.0, {1, 0, 0}));
  ASSERT_EQ(lifted.poses.size(), 2u);
  EXPECT_LT(
    (rotationOf(lifted.poses[0].pose) - rotationOf(lifted.poses[1].pose)).norm(), 1e-12);
}

TEST(ToolTiltLift, ToolFrameYawRotatesTheTiltAxisWithTheTool)
{
  const auto wall = makeWall();
  ToolTask t = contactTask(20.0, {1, 0, 0});
  t.tool_frame_yaw_deg = 90.0;  // contact-frame x now points along the wall's v: tilt about v
  const Eigen::Vector3d z = rotationOf(liftCoveragePathPose(flatPath(1, 1), wall, t).poses[0].pose).col(2);
  EXPECT_NEAR(angleBetween(z, Eigen::Vector3d(0, 1, 0)), 20.0, 1e-9);
  EXPECT_NEAR(z.z(), 0.0, 1e-12);
  EXPECT_GT(std::fabs(z.x()), 0.2);
}

TEST(ToolTiltLift, YawWithTiltEqualsYawedFrameThenTilt)
{
  // Physical definition: R = Rot(axis_world, tilt) * (yawed nominal frame).
  const auto wall = makeWall();
  ToolTask t = contactTask(20.0, {1, 0, 0});
  t.tool_frame_yaw_deg = 35.0;
  const Eigen::Matrix3d got = rotationOf(liftCoveragePathPose(flatPath(1, 1), wall, t).poses[0].pose);
  ToolTask plain;
  plain.contact = true;
  plain.tool_frame_yaw_deg = 35.0;
  const Eigen::Matrix3d f0 = rotationOf(liftCoveragePathPose(flatPath(1, 1), wall, plain).poses[0].pose);
  const Eigen::Vector3d axis_world = f0.col(0);  // contact-frame x after yaw
  const Eigen::Matrix3d expected = Eigen::AngleAxisd(20.0 * kDeg, axis_world).toRotationMatrix() * f0;
  EXPECT_LT((got - expected).norm(), 1e-12);
}

TEST(ToolTiltLift, EdgeTiltComposesAfterTheIntendedTilt)
{
  const auto wall = makeWall();
  // Far from every edge: only the intended tilt (20 deg about u).
  const Eigen::Vector3d z_mid = rotationOf(
    liftCoveragePathPose(flatPath(1.0, 1.0), wall, sprayTask(0.2, 30.0, 20.0, {1, 0, 0})).poses[0].pose).col(2);
  EXPECT_NEAR(angleBetween(z_mid, Eigen::Vector3d(0, 1, 0)), 20.0, 1e-9);
  // At the v_min edge (distance 0 < margin): edge tilt adds on top.
  const auto edge = liftCoveragePathPose(flatPath(1.0, 0.0), wall, sprayTask(0.2, 30.0, 20.0, {1, 0, 0}));
  const Eigen::Vector3d z_edge = rotationOf(edge.poses[0].pose).col(2);
  // The edge tilt is a 30 degree rotation of the already tilted axis toward -v.
  EXPECT_NEAR(angleBetween(z_edge, z_mid), 30.0, 1e-9);
  const auto no_fixed = liftCoveragePathPose(flatPath(1.0, 0.0), wall, sprayTask(0.2, 30.0));
  const Eigen::Vector3d z_edge_only = rotationOf(no_fixed.poses[0].pose).col(2);
  EXPECT_NEAR(angleBetween(z_edge_only, Eigen::Vector3d(0, 1, 0)), 30.0, 1e-9);
}

TEST(ToolTiltLift, TiltAboutUPlusEdgeTiltTowardVAddsAnglesInThePlane)
{
  // Tilt about u moves z in the (normal, v) plane toward world -Z; the v_min edge tilt also goes
  // toward -Z, so the angles to the normal add: 20 + 30 = 50 degrees.
  const auto wall = makeWall();
  const Eigen::Vector3d z = rotationOf(
    liftCoveragePathPose(flatPath(1.0, 0.0), wall, sprayTask(0.2, 30.0, 20.0, {-1, 0, 0})).poses[0].pose).col(2);
  const double a = angleBetween(z, Eigen::Vector3d(0, 1, 0));
  EXPECT_TRUE(std::fabs(a - 50.0) < 1e-9 || std::fabs(a - 10.0) < 1e-9) << a;
}

TEST(ToolTiltLift, EdgeTiltAwayFromFixedTiltSubtracts)
{
  const auto wall = makeWall();
  const double a1 = angleBetween(
    rotationOf(liftCoveragePathPose(flatPath(1.0, 0.0), wall, sprayTask(0.2, 30.0, 20.0, {1, 0, 0})).poses[0].pose).col(2),
    Eigen::Vector3d(0, 1, 0));
  const double a2 = angleBetween(
    rotationOf(liftCoveragePathPose(flatPath(1.0, 0.0), wall, sprayTask(0.2, 30.0, 20.0, {-1, 0, 0})).poses[0].pose).col(2),
    Eigen::Vector3d(0, 1, 0));
  EXPECT_NEAR(std::max(a1, a2), 50.0, 1e-9);
  EXPECT_NEAR(std::min(a1, a2), 10.0, 1e-9);
}

TEST(ToolTiltLift, ContactToolNeverGetsEdgeTiltButKeepsFixedTilt)
{
  const auto wall = makeWall();
  ToolTask t = contactTask(15.0, {1, 0, 0});
  t.tool_normal_policy = "tilt_at_edges";
  t.max_edge_tilt_deg = 30.0;
  const Eigen::Vector3d z = rotationOf(liftCoveragePathPose(flatPath(1.0, 0.0), wall, t).poses[0].pose).col(2);
  EXPECT_NEAR(angleBetween(z, Eigen::Vector3d(0, 1, 0)), 15.0, 1e-9);
}

TEST(ToolTiltLift, LargeTiltStaysFinite)
{
  const auto wall = makeWall();
  const auto lifted = liftCoveragePathPose(flatPath(1, 0.0), wall, sprayTask(0.2, 40.0, 79.0, {1, 0, 0}));
  const auto & q = lifted.poses[0].pose.orientation;
  EXPECT_TRUE(std::isfinite(q.x + q.y + q.z + q.w));
  EXPECT_NEAR(std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w), 1.0, 1e-12);
}

// ---- swath

namespace
{
ToolCoverageProfile padTool(double tilt_deg, std::array<double, 3> axis, const std::string & sweep,
  double pad_length = 0.0)
{
  ToolCoverageProfile t;
  t.name = "pad";
  t.task.contact = true;
  t.task.target_force_n = 10.0;
  t.task.tool_tilt_deg = tilt_deg;
  t.task.tool_tilt_axis = axis;
  t.coverage.sweep_direction = sweep;
  t.coverage.contact_area.pad_width_m = 0.2;
  t.coverage.contact_area.pad_length_m = pad_length;
  t.coverage.contact_area.reference_force_n = 10.0;
  return t;
}
}  // namespace

TEST(ToolTiltSwath, ZeroTiltLeavesTheModelWidth)
{
  EXPECT_DOUBLE_EQ(toolSwath(padTool(0.0, {1, 0, 0}, "vertical")), 0.2);
}

TEST(ToolTiltSwath, TiltAlongStrokeKeepsTheSwath)
{
  // Vertical strokes run along v. Tilt about u (x axis) tilts toward v = along the stroke.
  EXPECT_NEAR(toolSwath(padTool(40.0, {1, 0, 0}, "vertical")), 0.2, 1e-12);
  // Horizontal strokes run along u. Tilt about v (y axis) tilts toward u = along the stroke.
  EXPECT_NEAR(toolSwath(padTool(40.0, {0, 1, 0}, "horizontal")), 0.2, 1e-12);
}

TEST(ToolTiltSwath, TiltAcrossStrokeWidensBySecant)
{
  const double expect = 0.2 / std::cos(40.0 * kDeg);
  EXPECT_NEAR(toolSwath(padTool(40.0, {0, 1, 0}, "vertical")), expect, 1e-12);
  EXPECT_NEAR(toolSwath(padTool(40.0, {1, 0, 0}, "horizontal")), expect, 1e-12);
}

TEST(ToolTiltSwath, ToolFrameYawSwapsAlongAndAcross)
{
  auto t = padTool(40.0, {1, 0, 0}, "vertical");
  t.task.tool_frame_yaw_deg = 90.0;  // tilt axis turns from u to v: now across the vertical stroke
  EXPECT_NEAR(toolSwath(t), 0.2 / std::cos(40.0 * kDeg), 1e-12);
}

TEST(ToolTiltSwath, ObliqueAxisIsBetweenTheExtremes)
{
  const double w = toolSwath(padTool(40.0, {1, 1, 0}, "vertical"));
  EXPECT_GT(w, 0.2);
  EXPECT_LT(w, 0.2 / std::cos(40.0 * kDeg));
}

TEST(ToolTiltSwath, NegativeTiltGivesTheSameSwath)
{
  EXPECT_NEAR(
    toolSwath(padTool(-30.0, {0, 1, 0}, "vertical")), toolSwath(padTool(30.0, {0, 1, 0}, "vertical")),
    1e-12);
}

TEST(ToolTiltSwath, SpacingFollowsTheProjectedSwath)
{
  auto t = padTool(60.0, {0, 1, 0}, "vertical");
  t.coverage.overlap_pct = 25.0;
  EXPECT_NEAR(toolLineSpacing(t), 0.4 * 0.75, 1e-12);
}

TEST(ToolTiltSwath, PadLengthMattersOnlyAtAnObliqueTilt)
{
  EXPECT_NEAR(toolSwath(padTool(40.0, {0, 1, 0}, "vertical", 0.3)), 0.2 / std::cos(40.0 * kDeg), 1e-12);
  EXPECT_GT(toolSwath(padTool(40.0, {1, 1, 0}, "vertical", 0.3)), toolSwath(padTool(40.0, {1, 1, 0}, "vertical", 0.0)));
}

TEST(ToolTiltSwath, FixedSwathToolIsUnchangedByTilt)
{
  ToolCoverageProfile t;
  t.task.contact = true;
  t.task.tool_tilt_deg = 40.0;
  t.task.tool_tilt_axis = {0, 1, 0};
  t.coverage.swath_m = 0.25;
  EXPECT_DOUBLE_EQ(toolSwath(t), 0.25);
}

TEST(ToolTiltSwath, SprayFootprintIsProjectedToo)
{
  ToolCoverageProfile t;
  t.task.contact = false;
  t.task.standoff_m = 0.3;
  t.coverage.spray_half_angle_deg = 20.0;
  t.coverage.effective_width_fraction = 0.8;
  t.coverage.sweep_direction = "horizontal";
  const double base = toolSwath(t);
  t.task.tool_tilt_deg = 30.0;
  t.task.tool_tilt_axis = {1, 0, 0};  // tilt toward v, across the horizontal stroke
  EXPECT_NEAR(toolSwath(t), base / std::cos(30.0 * kDeg), 1e-12);
  t.task.tool_tilt_axis = {0, 1, 0};  // tilt toward u, along the stroke
  EXPECT_NEAR(toolSwath(t), base, 1e-12);
}

TEST(ToolTiltSwath, DirectionHelperMatchesTheConvention)
{
  auto t = padTool(10.0, {1, 0, 0}, "vertical");
  EXPECT_NEAR(std::fmod(std::fabs(tiltDirectionFromStrokeDeg(t)), 180.0), 0.0, 1e-9);
  t.coverage.sweep_direction = "horizontal";
  EXPECT_NEAR(std::fmod(std::fabs(tiltDirectionFromStrokeDeg(t)), 180.0), 90.0, 1e-9);
}

// ---- loading

TEST(ToolTiltLoading, CoverageProfileCarriesTheGeometryTilt)
{
  core_types::ToolProfile p;
  p.name = "t";
  p.task.contact = true;
  p.geometry.tilt_deg = 25.0;
  p.geometry.tilt_axis = {0, 1, 0};
  const auto c = toCoverageProfile(p);
  EXPECT_DOUBLE_EQ(c.task.tool_tilt_deg, 25.0);
  EXPECT_EQ(c.task.tool_tilt_axis, (std::array<double, 3>{0, 1, 0}));
}

TEST(ToolTiltLoading, ShippedProfilesHaveNoTilt)
{
  for (const char * name : {"contact_wiper", "spray_gun"}) {
    const auto full = ros_adapters::loadToolProfile(TOOL_PROFILES_PATH, name);
    EXPECT_DOUBLE_EQ(full.geometry.tilt_deg, 0.0) << name;
    const auto c = toCoverageProfile(full);
    EXPECT_DOUBLE_EQ(c.task.tool_tilt_deg, 0.0) << name;
  }
}

TEST(ToolTiltLoading, ShippedProfileSwathsAreUnchanged)
{
  auto wiper = toCoverageProfile(ros_adapters::loadToolProfile(TOOL_PROFILES_PATH, "contact_wiper"));
  applyToolOverrides(wiper, {});
  EXPECT_NEAR(toolSwath(wiper), 0.20, 1e-12);
  auto gun = toCoverageProfile(ros_adapters::loadToolProfile(TOOL_PROFILES_PATH, "spray_gun"));
  applyToolOverrides(gun, {});
  EXPECT_NEAR(toolSwath(gun), 0.8 * 2.0 * 0.30 * std::tan(20.0 * kDeg), 1e-9);
}
