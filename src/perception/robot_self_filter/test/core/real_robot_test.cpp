#include <gtest/gtest.h>

#include <fstream>
#include <random>
#include <sstream>

#include "robot_self_filter/core/robot_model.hpp"
#include "robot_self_filter/core/self_filter.hpp"

using namespace robot_self_filter;  // NOLINT

namespace
{
std::string readFile(const std::string & path)
{
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// The rendered generic mobile manipulator (primitives only).
const UrdfRobot & robot()
{
  static const UrdfRobot r = UrdfRobot::fromUrdf(readFile(SELF_FILTER_TEST_DATA "/mobile_manipulator_generic.urdf"));
  return r;
}

JointPositions home()
{
  return {{"lift_joint", 0.4}, {"arm_joint1", 0.0}, {"arm_joint2", -1.5708}, {"arm_joint3", 1.5708},
    {"arm_joint4", -1.5708}, {"arm_joint5", -1.5708}, {"arm_joint6", 0.0}};
}

// Random points on the surface of every collision shape, in the root frame.
std::vector<Vec3> surfacePoints(const LinkPoses & poses, int per_shape, uint32_t seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(-1.0, 1.0), a(0.0, 2.0 * M_PI);
  std::vector<Vec3> out;
  for (const auto & ls : robot().shapes()) {
    for (const auto & s : ls.shapes) {
      const Pose T = poses.at(ls.link) * s.local;
      for (int i = 0; i < per_shape; ++i) {
        Vec3 q;
        if (s.kind == Shape::Kind::kBox) {
          q = Vec3(u(rng), u(rng), u(rng)).cwiseProduct(0.5 * s.size);
          const int face = i % 3;
          q[face] = (i % 2 ? 1.0 : -1.0) * 0.5 * s.size[face];
        } else if (s.kind == Shape::Kind::kCylinder) {
          const double ang = a(rng);
          q = Vec3(s.radius * std::cos(ang), s.radius * std::sin(ang), 0.5 * s.height * u(rng));
        } else {
          Vec3 d(u(rng), u(rng), u(rng));
          q = d.normalized() * s.radius;
        }
        out.push_back(T * q);
      }
    }
  }
  return out;
}

double distanceToNearestShape(const Vec3 & p, const LinkPoses & poses)
{
  // Conservative: distance to the nearest shape centre minus its bounding radius.
  double best = 1e9;
  for (const auto & ls : robot().shapes()) {
    for (const auto & s : ls.shapes) {
      const Pose T = poses.at(ls.link) * s.local;
      double bound = s.kind == Shape::Kind::kBox ? 0.5 * s.size.norm() :
        s.kind == Shape::Kind::kSphere ? s.radius : std::hypot(s.radius, 0.5 * s.height);
      best = std::min(best, (p - T.translation()).norm() - bound);
    }
  }
  return best;
}
}  // namespace

TEST(RealRobot, ParsesTheDescriptionAndFindsAllLinkShapes)
{
  EXPECT_EQ(robot().rootLink(), "base_footprint");
  EXPECT_TRUE(robot().skippedMeshes().empty());  // primitives only
  size_t shapes = 0;
  for (const auto & ls : robot().shapes()) {shapes += ls.shapes.size();}
  EXPECT_GE(shapes, 12U);
  EXPECT_TRUE(robot().hasLink("tcp_link"));
  EXPECT_EQ(robot().jointNames().size(), 9U);  // lift, 2 telescoping stages, 6 arm joints
}

TEST(RealRobot, ForwardKinematicsPlacesTheTcpWhereTheMapSaysIt)
{
  // The same pose the reachability tests use for this arm: the tcp at the home configuration.
  const auto poses = robot().linkPoses(home());
  const Vec3 tcp = poses.at("tcp_link").translation();
  const Vec3 base = poses.at("arm_base_link").translation();
  EXPECT_NEAR(base.z() - poses.at("lift_base_link").translation().z(), 0.8 + 0.4 + 0.02 + 0.0, 0.01);
  EXPECT_LT((tcp - base).norm(), 1.7);  // within the arm's reach of its base
}

TEST(RealRobot, PointsOnTheRobotsSurfaceAreRemovedAndFarPointsAreKept)
{
  const auto poses = robot().linkPoses(home());
  SelfFilterConfig cfg;
  cfg.padding.sphere_radius = 0.02;
  cfg.padding.box = Vec3::Constant(0.02);
  cfg.padding.cylinder_radius = 0.02;
  cfg.padding.cylinder_height = 0.04;
  const SelfFilter f(robot().shapes(), cfg);

  scene_model::PointCloud cloud;
  cloud.points = surfacePoints(poses, 400, 3);
  const size_t surface_count = cloud.points.size();
  std::mt19937 rng(8);
  std::uniform_real_distribution<double> u(-4.0, 4.0);
  size_t far = 0;
  while (far < 5000) {
    const Vec3 p(u(rng), u(rng), 0.0 + 3.0 * std::abs(u(rng)) / 4.0);
    if (distanceToNearestShape(p, poses) > 0.3) {
      cloud.points.push_back(p);
      ++far;
    }
  }
  SelfFilterInput in;
  in.cloud = &cloud;
  in.link_poses = poses;
  const auto r = f.filter(in);
  ASSERT_TRUE(r.success) << r.message;

  size_t surface_removed = 0, far_removed = 0;
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    if (r.removed[i]) {(i < surface_count ? surface_removed : far_removed)++;}
  }
  EXPECT_GT(static_cast<double>(surface_removed) / surface_count, 0.995) << "robot points left in the cloud";
  EXPECT_EQ(far_removed, 0U) << "environment points removed";
}

TEST(RealRobot, TheFilteredRegionFollowsTheLift)
{
  const SelfFilter f(robot().shapes(), SelfFilterConfig{});
  auto low = home();
  low["lift_joint"] = 0.0;
  auto high = home();
  high["lift_joint"] = 1.2;
  // A point on the arm's tool at the high pose lies in free space at the low pose.
  const Vec3 tool_high = robot().linkPoses(high).at("tool_mount_link").translation();
  scene_model::PointCloud c;
  c.points = {tool_high};
  SelfFilterInput in;
  in.cloud = &c;
  in.link_poses = robot().linkPoses(high);
  EXPECT_EQ(f.filter(in).removed_count, 1U);
  in.link_poses = robot().linkPoses(low);
  EXPECT_EQ(f.filter(in).removed_count, 0U);
}
