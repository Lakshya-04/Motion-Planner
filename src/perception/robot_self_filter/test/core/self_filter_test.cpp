#include <gtest/gtest.h>

#include <chrono>
#include <random>

#include "robot_self_filter/core/self_filter.hpp"

using namespace robot_self_filter;  // NOLINT

namespace
{
// One 0.4 m cube link "body" at the origin and one 0.1 x 0.1 x 0.5 m "arm" link at (1, 0, 0).
std::vector<LinkShapes> twoLinks()
{
  Shape box;
  box.kind = Shape::Kind::kBox;
  box.size = Vec3(0.4, 0.4, 0.4);
  Shape arm;
  arm.kind = Shape::Kind::kBox;
  arm.size = Vec3(0.1, 0.1, 0.5);
  return {{"body", {box}}, {"arm", {arm}}};
}

LinkPoses poses()
{
  LinkPoses p;
  p["body"] = Pose::Identity();
  Pose a = Pose::Identity();
  a.translation() = Vec3(1.0, 0.0, 0.0);
  p["arm"] = a;
  return p;
}

scene_model::PointCloud cloud(std::vector<Vec3> pts)
{
  scene_model::PointCloud c;
  c.points = std::move(pts);
  return c;
}

SelfFilterConfig padded(double pad)
{
  SelfFilterConfig c;
  c.padding.sphere_radius = pad;
  c.padding.box = Vec3::Constant(pad);
  c.padding.cylinder_radius = pad;
  c.padding.cylinder_height = pad;
  return c;
}

SelfFilterResult run(const SelfFilter & f, const scene_model::PointCloud & c, SelfFilterInput in = {})
{
  in.cloud = &c;
  if (in.link_poses.empty()) {in.link_poses = poses();}
  return f.filter(in);
}
}  // namespace

TEST(SelfFilter, RemovesPointsInsideAndKeepsPointsOutside)
{
  const SelfFilter f(twoLinks(), padded(0.0));
  const auto c = cloud({Vec3(0, 0, 0), Vec3(0.19, 0.19, 0.19), Vec3(0.21, 0, 0), Vec3(3, 3, 3),
    Vec3(1.0, 0.0, 0.2), Vec3(1.0, 0.0, 0.3)});
  const auto r = run(f, c);
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_EQ(r.removed, (std::vector<bool>{true, true, false, false, true, false}));
  EXPECT_EQ(r.removed_count, 3U);
  EXPECT_EQ(r.filtered.points.size(), 3U);
  EXPECT_EQ(r.removed_per_link.at("body"), 2U);
  EXPECT_EQ(r.removed_per_link.at("arm"), 1U);
}

TEST(SelfFilter, PaddingGrowsTheRemovedRegion)
{
  const auto c = cloud({Vec3(0.25, 0, 0)});  // 5 cm outside the cube
  EXPECT_EQ(run(SelfFilter(twoLinks(), padded(0.0)), c).removed_count, 0U);
  EXPECT_EQ(run(SelfFilter(twoLinks(), padded(0.04)), c).removed_count, 0U);
  EXPECT_EQ(run(SelfFilter(twoLinks(), padded(0.06)), c).removed_count, 1U);
}

TEST(SelfFilter, PerLinkPaddingOverridesTheDefault)
{
  SelfFilterConfig cfg = padded(0.0);
  Padding big;
  big.box = Vec3::Constant(0.2);
  cfg.link_padding["arm"] = big;
  const SelfFilter f(twoLinks(), cfg);
  const auto c = cloud({Vec3(1.15, 0, 0), Vec3(0.35, 0, 0)});  // 10 cm from the arm; 15 cm from the body
  const auto r = run(f, c);
  EXPECT_EQ(r.removed, (std::vector<bool>{true, false}));
}

TEST(SelfFilter, OnlyConfiguredLinksCountAsSelf)
{
  SelfFilterConfig cfg = padded(0.0);
  cfg.links = {"arm"};
  const auto c = cloud({Vec3(0, 0, 0), Vec3(1, 0, 0)});
  const auto r = run(SelfFilter(twoLinks(), cfg), c);
  EXPECT_EQ(r.removed, (std::vector<bool>{false, true}));
}

TEST(SelfFilter, LabelsAndColorsStayAlignedWithTheKeptPoints)
{
  auto c = cloud({Vec3(0, 0, 0), Vec3(5, 0, 0), Vec3(6, 0, 0)});
  c.labels = {10, 20, 30};
  c.colors = {{1, 1, 1}, {2, 2, 2}, {3, 3, 3}};
  const auto r = run(SelfFilter(twoLinks(), padded(0.0)), c);
  ASSERT_EQ(r.filtered.points.size(), 2U);
  EXPECT_EQ(r.filtered.labels, (std::vector<uint32_t>{20, 30}));
  EXPECT_EQ(r.filtered.colors[0][0], 2);
  EXPECT_EQ(r.filtered.colors[1][0], 3);
}

TEST(SelfFilter, TheCloudFrameIsTransformedIntoTheRobotFrame)
{
  // The cloud is in a sensor frame offset by +2 m in x from the robot root.
  SelfFilterInput in;
  in.cloud_to_root = Pose::Identity();
  in.cloud_to_root.translation() = Vec3(2.0, 0.0, 0.0);
  const auto c = cloud({Vec3(-2.0, 0, 0), Vec3(0.0, 0, 0)});  // root (0,0,0) is inside the body
  const auto r = run(SelfFilter(twoLinks(), padded(0.0)), c, in);
  EXPECT_EQ(r.removed, (std::vector<bool>{true, false}));
}

TEST(SelfFilter, MovingALinkMovesTheFilteredRegion)
{
  const SelfFilter f(twoLinks(), padded(0.0));
  const auto c = cloud({Vec3(1.0, 0, 0), Vec3(1.0, 0, 1.0)});
  SelfFilterInput in;
  in.link_poses = poses();
  EXPECT_EQ(run(f, c, in).removed, (std::vector<bool>{true, false}));
  in.link_poses["arm"].translation() = Vec3(1.0, 0.0, 1.0);  // the arm moved up 1 m
  EXPECT_EQ(run(f, c, in).removed, (std::vector<bool>{false, true}));
}

TEST(SelfFilter, MinSensorDistanceDropsPointsNearTheSensor)
{
  SelfFilterConfig cfg = padded(0.0);
  cfg.min_sensor_distance = 0.5;
  SelfFilterInput in;
  in.sensor_origin = Vec3(5, 5, 5);
  const auto c = cloud({Vec3(5.1, 5, 5), Vec3(7, 5, 5)});
  const auto r = run(SelfFilter(twoLinks(), cfg), c, in);
  EXPECT_EQ(r.removed, (std::vector<bool>{true, false}));
  EXPECT_EQ(r.removed_per_link.at("<min_distance>"), 1U);
}

TEST(SelfFilter, ShadowRemovesPointsHiddenBehindALinkButNotInFrontOfIt)
{
  // Sensor at x = -3 looking along +x; the body cube is at the origin. A point at x = +3 is hidden
  // behind it; a point at x = -1 is in front of it; a point at x = +3 off to the side is visible.
  const auto c = cloud({Vec3(3, 0, 0), Vec3(-1, 0, 0), Vec3(3, 2, 0)});
  SelfFilterInput in;
  in.sensor_origin = Vec3(-3, 0, 0);

  SelfFilterConfig off = padded(0.0);
  EXPECT_EQ(run(SelfFilter(twoLinks(), off), c, in).removed_count, 0U);  // off by default

  SelfFilterConfig on = padded(0.0);
  on.shadow = true;
  const auto r = run(SelfFilter(twoLinks(), on), c, in);
  EXPECT_EQ(r.removed, (std::vector<bool>{true, false, false}));
}

TEST(SelfFilter, ShadowNeedsTheSensorOrigin)
{
  SelfFilterConfig cfg = padded(0.0);
  cfg.shadow = true;
  const auto c = cloud({Vec3(0, 0, 0)});
  const auto r = run(SelfFilter(twoLinks(), cfg), c);
  EXPECT_FALSE(r.success);
  EXPECT_NE(r.message.find("sensor origin"), std::string::npos);
}

TEST(SelfFilter, ContactModeUsesReducedPaddingOnTheContactLinkOnly)
{
  SelfFilterConfig cfg = padded(0.1);
  cfg.contact_link = "arm";
  cfg.contact_padding = 0.0;
  const SelfFilter f(twoLinks(), cfg);
  // 6 cm beside the arm's 10 cm wide body, and 6 cm outside the cube (cube padding stays on).
  const auto c = cloud({Vec3(1.11, 0, 0), Vec3(0.26, 0, 0)});
  SelfFilterInput normal;
  EXPECT_EQ(run(f, c, normal).removed, (std::vector<bool>{true, true}));
  SelfFilterInput touching;
  touching.contact_mode = true;
  EXPECT_EQ(run(f, c, touching).removed, (std::vector<bool>{false, true}));
}

TEST(SelfFilter, SpeedPaddingGrowsWithLinkSpeed)
{
  SelfFilterConfig cfg = padded(0.0);
  cfg.speed_padding_gain = 0.2;  // 20 cm per m/s
  const SelfFilter f(twoLinks(), cfg);
  const auto c = cloud({Vec3(0.3, 0, 0)});  // 10 cm outside the cube
  SelfFilterInput still;
  EXPECT_EQ(run(f, c, still).removed_count, 0U);
  SelfFilterInput moving;
  moving.link_speed["body"] = 1.0;
  EXPECT_EQ(run(f, c, moving).removed_count, 1U);
}

TEST(SelfFilter, MissingPosesAndCloudAreReported)
{
  const SelfFilter f(twoLinks(), padded(0.0));
  SelfFilterInput none;
  EXPECT_FALSE(f.filter(none).success);
  const auto c = cloud({Vec3(0, 0, 0)});
  SelfFilterInput partial;
  partial.cloud = &c;
  partial.link_poses["body"] = Pose::Identity();  // no pose for "arm"
  const auto r = f.filter(partial);
  EXPECT_FALSE(r.success);
  EXPECT_NE(r.message.find("arm"), std::string::npos);
  EXPECT_TRUE(run(f, scene_model::PointCloud{}).success);  // an empty cloud is fine
}

TEST(SelfFilter, CylinderAndSphereShapes)
{
  Shape cyl;
  cyl.kind = Shape::Kind::kCylinder;
  cyl.radius = 0.1;
  cyl.height = 0.4;
  Shape ball;
  ball.kind = Shape::Kind::kSphere;
  ball.radius = 0.2;
  ball.local.translation() = Vec3(1, 0, 0);
  const SelfFilter f({{"c", {cyl, ball}}}, padded(0.0));
  SelfFilterInput in;
  in.link_poses["c"] = Pose::Identity();
  const auto c = cloud({Vec3(0.09, 0, 0.19), Vec3(0.11, 0, 0), Vec3(0, 0, 0.21), Vec3(1.15, 0, 0), Vec3(1.25, 0, 0)});
  const auto r = f.filter([&] {in.cloud = &c; return in;}());
  EXPECT_EQ(r.removed, (std::vector<bool>{true, false, false, true, false}));
}

TEST(SelfFilter, GeometryHelpersAgreeWithTheFilter)
{
  Shape box;
  box.kind = Shape::Kind::kBox;
  box.size = Vec3(1, 1, 1);
  EXPECT_TRUE(pointInShape(box, Pose::Identity(), Vec3(0.49, 0, 0), 0, Vec3::Zero(), 0));
  EXPECT_FALSE(pointInShape(box, Pose::Identity(), Vec3(0.51, 0, 0), 0, Vec3::Zero(), 0));
  EXPECT_TRUE(segmentTouchesShape(box, Pose::Identity(), Vec3(-2, 0, 0), Vec3(2, 0, 0), 0, Vec3::Zero(), 0));
  EXPECT_FALSE(segmentTouchesShape(box, Pose::Identity(), Vec3(-2, 2, 0), Vec3(2, 2, 0), 0, Vec3::Zero(), 0));
  EXPECT_FALSE(segmentTouchesShape(box, Pose::Identity(), Vec3(-2, 0, 0), Vec3(-1, 0, 0), 0, Vec3::Zero(), 0));
}

TEST(SelfFilter, FilteringALargeCloudIsFast)
{
  std::vector<LinkShapes> shapes;
  LinkPoses p;
  for (int i = 0; i < 15; ++i) {
    Shape s;
    s.kind = i % 2 ? Shape::Kind::kBox : Shape::Kind::kCylinder;
    s.size = Vec3(0.2, 0.2, 0.5);
    s.radius = 0.1;
    s.height = 0.5;
    shapes.push_back({"l" + std::to_string(i), {s}});
    Pose pose = Pose::Identity();
    pose.translation() = Vec3(0.1 * i, 0, 0.3);
    p["l" + std::to_string(i)] = pose;
  }
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> u(-3, 3);
  scene_model::PointCloud c;
  for (int i = 0; i < 200000; ++i) {c.points.emplace_back(u(rng), u(rng), u(rng));}
  const SelfFilter f(shapes, padded(0.02));
  SelfFilterInput in;
  in.cloud = &c;
  in.link_poses = p;
  const auto t0 = std::chrono::steady_clock::now();
  const auto r = f.filter(in);
  const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  ASSERT_TRUE(r.success);
  EXPECT_LT(s, 1.0) << "200k points x 15 shapes took " << s << " s";
  EXPECT_GT(r.removed_count, 0U);
}
