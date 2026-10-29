#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Dense>

#include "coverage_planner/coverage_pass_expander.hpp"
#include "coverage_planner/wall_basis.hpp"
#include "coverage_planner/wall_pose_lift.hpp"

namespace
{

coverage_planner_msgs::msg::WallPlane makeWall()
{
  // origin at world (0,0,0); basis_u = world +X, basis_v = world +Z, normal_outward = basis_u x
  // basis_v = world -Y (matches the right-handed convention
  // generate_wall_occupancy_grid_action_server.cpp uses: col(2) = col(0) x col(1)).
  coverage_planner_msgs::msg::WallPlane wall;
  wall.id = 0;
  wall.pose.header.frame_id = "map";
  wall.pose.pose = coverage_planner::encodeWallBasis(
    Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0),
    Eigen::Vector3d(0.0, -1.0, 0.0));
  wall.u_min = 0.0;
  wall.u_max = 2.0;
  wall.v_min = 0.0;
  wall.v_max = 2.0;
  return wall;
}

geometry_msgs::msg::PoseStamped makeFlatPose(double u, double v)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = u;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = v;
  pose.pose.orientation.w = 1.0;
  return pose;
}

Eigen::Vector3d zAxisOf(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  return q.toRotationMatrix().col(2);
}

coverage_planner::ToolTask makeContactTask()
{
  coverage_planner::ToolTask task;
  task.contact = true;
  task.standoff_m = 0.0;
  task.tool_normal_policy = "constant";
  return task;
}

coverage_planner::ToolTask makeNonContactTask(double standoff_m, double max_edge_tilt_deg)
{
  coverage_planner::ToolTask task;
  task.contact = false;
  task.standoff_m = standoff_m;
  task.tool_normal_policy = "tilt_at_edges";
  task.max_edge_tilt_deg = max_edge_tilt_deg;
  return task;
}

}  // namespace

TEST(WallPoseLift, ContactToolKeepsOnSurfaceWithZAxisIntoWall)
{
  const auto wall = makeWall();
  nav_msgs::msg::Path flat;
  flat.poses.push_back(makeFlatPose(1.0, 1.0));

  const auto lifted = coverage_planner::liftCoveragePathPose(flat, wall, makeContactTask());

  ASSERT_EQ(lifted.poses.size(), 1u);
  const auto & pose = lifted.poses[0].pose;
  EXPECT_NEAR(pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(pose.position.y, 0.0, 1e-9);
  EXPECT_NEAR(pose.position.z, 1.0, 1e-9);

  const Eigen::Vector3d z_axis = zAxisOf(pose);
  // TCP Z must be anti-parallel to normal_outward=(0,-1,0), i.e. (0,1,0).
  EXPECT_NEAR(z_axis.x(), 0.0, 1e-9);
  EXPECT_NEAR(z_axis.y(), 1.0, 1e-9);
  EXPECT_NEAR(z_axis.z(), 0.0, 1e-9);
}

TEST(WallPoseLift, NonContactToolOffsetsAlongNormalByStandoff)
{
  const auto wall = makeWall();
  nav_msgs::msg::Path flat;
  flat.poses.push_back(makeFlatPose(1.0, 1.0));

  const auto lifted =
    coverage_planner::liftCoveragePathPose(flat, wall, makeNonContactTask(0.2, 0.0));

  ASSERT_EQ(lifted.poses.size(), 1u);
  const auto & pose = lifted.poses[0].pose;
  // Offset along normal_outward=(0,-1,0) by 0.2 -> y = -0.2.
  EXPECT_NEAR(pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(pose.position.y, -0.2, 1e-9);
  EXPECT_NEAR(pose.position.z, 1.0, 1e-9);
}

TEST(WallPoseLift, TiltAtEdgesTiltsTowardNearestBoundaryWithinMargin)
{
  const auto wall = makeWall();
  nav_msgs::msg::Path flat;
  // v=0.05 is within the 0.2 m margin of the v_min=0.0 boundary; u=1.0 is far from both u edges.
  flat.poses.push_back(makeFlatPose(1.0, 0.05));

  const auto lifted =
    coverage_planner::liftCoveragePathPose(flat, wall, makeNonContactTask(0.2, 30.0));

  ASSERT_EQ(lifted.poses.size(), 1u);
  const Eigen::Vector3d z_axis = zAxisOf(lifted.poses[0].pose);

  // Tilting toward the v_min edge means extending reach in -basis_v = world -Z: the Z axis must
  // pick up a negative world-Z component while still mostly pointing into the wall (+Y).
  EXPECT_LT(z_axis.z(), -1e-6);
  EXPECT_GT(z_axis.y(), 0.5);
  EXPECT_NEAR(z_axis.x(), 0.0, 1e-9);
  EXPECT_NEAR(z_axis.norm(), 1.0, 1e-9);
}

TEST(WallPoseLift, NoTiltFarFromAnyBoundary)
{
  const auto wall = makeWall();
  nav_msgs::msg::Path flat;
  flat.poses.push_back(makeFlatPose(1.0, 1.0));  // dead center, 1.0 m from every edge.

  const auto lifted =
    coverage_planner::liftCoveragePathPose(flat, wall, makeNonContactTask(0.2, 30.0));

  ASSERT_EQ(lifted.poses.size(), 1u);
  const Eigen::Vector3d z_axis = zAxisOf(lifted.poses[0].pose);
  EXPECT_NEAR(z_axis.x(), 0.0, 1e-9);
  EXPECT_NEAR(z_axis.y(), 1.0, 1e-9);
  EXPECT_NEAR(z_axis.z(), 0.0, 1e-9);
}

TEST(WallPoseLift, ContactToolNeverTiltsEvenIfPolicySaysTiltAtEdges)
{
  const auto wall = makeWall();
  nav_msgs::msg::Path flat;
  flat.poses.push_back(makeFlatPose(1.0, 0.01));  // right at the v_min edge.

  coverage_planner::ToolTask task;
  task.contact = true;  // hard safety exclusion must win regardless of the policy string below.
  task.standoff_m = 0.0;
  task.tool_normal_policy = "tilt_at_edges";
  task.max_edge_tilt_deg = 30.0;

  const auto lifted = coverage_planner::liftCoveragePathPose(flat, wall, task);

  ASSERT_EQ(lifted.poses.size(), 1u);
  const Eigen::Vector3d z_axis = zAxisOf(lifted.poses[0].pose);
  EXPECT_NEAR(z_axis.y(), 1.0, 1e-9);
  EXPECT_NEAR(z_axis.z(), 0.0, 1e-9);
}

namespace
{

std::vector<coverage_planner_msgs::msg::MotionPlanSegment> makeFlatSegments()
{
  coverage_planner_msgs::msg::MotionPlanSegment strip;
  strip.label = "strip_0";
  strip.start_index = 0;
  strip.end_index = 2;
  strip.frame_id = "map";
  strip.mode = "cartesian";
  strip.planner = "cartesian";

  coverage_planner_msgs::msg::MotionPlanSegment connector;
  connector.label = "connector_0";
  connector.start_index = 3;
  connector.end_index = 4;
  connector.frame_id = "map";
  connector.mode = "fsp";
  connector.planner = "ompl";
  connector.requires_retraction = true;

  return {strip, connector};
}

nav_msgs::msg::Path makeFlatPathForSegments()
{
  nav_msgs::msg::Path path;
  path.poses.push_back(makeFlatPose(0.0, 0.5));
  path.poses.push_back(makeFlatPose(0.5, 0.5));
  path.poses.push_back(makeFlatPose(1.0, 0.5));
  path.poses.push_back(makeFlatPose(1.2, 0.6));
  path.poses.push_back(makeFlatPose(1.4, 0.7));
  return path;
}

coverage_planner::ToolCoverageProfile makeSinglePassProfile()
{
  coverage_planner::ToolCoverageProfile profile;
  profile.task.contact = true;
  profile.task.standoff_m = 0.0;
  profile.task.tool_normal_policy = "constant";
  profile.coverage.passes = {coverage_planner::PassVariant{}};
  return profile;
}

coverage_planner::ToolCoverageProfile makeDoublePassProfile()
{
  coverage_planner::ToolCoverageProfile profile = makeSinglePassProfile();
  profile.coverage.passes = {
    coverage_planner::PassVariant{},
    coverage_planner::PassVariant{0.0, true},
  };
  return profile;
}

}  // namespace

TEST(CoveragePassExpander, SinglePassProfileDoesNotDuplicateOrRenameStrip)
{
  const auto wall = makeWall();
  const auto expanded = coverage_planner::expandCoveragePasses(
    makeFlatPathForSegments(), makeFlatSegments(), wall, makeSinglePassProfile());

  ASSERT_EQ(expanded.segments.size(), 2u);
  EXPECT_EQ(expanded.segments[0].label, "strip_0");
  EXPECT_EQ(expanded.segments[0].mode, "cartesian");
  EXPECT_TRUE(expanded.segments[0].tags.empty());
  EXPECT_EQ(expanded.segments[1].label, "connector_0");
  EXPECT_EQ(expanded.segments[1].mode, "fsp");

  // 3 strip poses + 2 connector poses, none duplicated.
  EXPECT_EQ(expanded.path.poses.size(), 5u);
}

TEST(CoveragePassExpander, DoublePassProfileDuplicatesOnlyCartesianSegments)
{
  const auto wall = makeWall();
  const auto expanded = coverage_planner::expandCoveragePasses(
    makeFlatPathForSegments(), makeFlatSegments(), wall, makeDoublePassProfile());

  // strip_0 duplicated into strip_0_pass0 + strip_0_pass1; connector_0 stays single.
  ASSERT_EQ(expanded.segments.size(), 3u);
  EXPECT_EQ(expanded.segments[0].label, "strip_0_pass0");
  EXPECT_TRUE(expanded.segments[0].tags.empty());
  EXPECT_EQ(expanded.segments[1].label, "strip_0_pass1");
  ASSERT_FALSE(expanded.segments[1].tags.empty());
  EXPECT_EQ(expanded.segments[1].tags[0], "second_pass");
  EXPECT_EQ(expanded.segments[2].label, "connector_0");
  EXPECT_EQ(expanded.segments[2].mode, "fsp");

  // 3 strip poses x 2 passes + 2 connector poses = 8.
  EXPECT_EQ(expanded.path.poses.size(), 8u);
}

TEST(CoveragePassExpander, ReversedPassActuallyReversesWaypointOrder)
{
  const auto wall = makeWall();
  const auto expanded = coverage_planner::expandCoveragePasses(
    makeFlatPathForSegments(), makeFlatSegments(), wall, makeDoublePassProfile());

  const auto & first_pass = expanded.segments[0];
  const auto & second_pass = expanded.segments[1];

  const auto & first_pass_start = expanded.path.poses[first_pass.start_index].pose.position;
  const auto & second_pass_start = expanded.path.poses[second_pass.start_index].pose.position;
  const auto & second_pass_end = expanded.path.poses[second_pass.end_index].pose.position;

  // Original strip order (in u): 0.0 -> 0.5 -> 1.0. Reversed second pass must start where the
  // first pass ended (u=1.0) and end where the first pass started (u=0.0).
  EXPECT_NEAR(first_pass_start.x, 0.0, 1e-9);
  EXPECT_NEAR(second_pass_start.x, 1.0, 1e-9);
  EXPECT_NEAR(second_pass_end.x, 0.0, 1e-9);
}
