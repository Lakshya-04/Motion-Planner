// A real client talking to a stand-in cuRobo planner node (same process, real ROS services).
#include <gtest/gtest.h>

#include <cstdlib>
#include <cstring>
#include <memory>
#include <sstream>
#include <thread>

#include <curobo_bridge_msgs/srv/plan_joint.hpp>
#include <curobo_bridge_msgs/srv/plan_pose.hpp>
#include <curobo_bridge_msgs/srv/set_world.hpp>
#include <rclcpp/rclcpp.hpp>

#include "curobo_planner/curobo_planner.hpp"

using curobo_bridge_msgs::srv::PlanJoint;
using curobo_bridge_msgs::srv::PlanPose;
using curobo_bridge_msgs::srv::SetWorld;
namespace mp = motion_planner;

namespace
{
class StubServer
{
public:
  StubServer()
  {
    node = std::make_shared<rclcpp::Node>("curobo_planner");
    joint = node->create_service<PlanJoint>(
      "~/plan_joint", [this](const PlanJoint::Request::SharedPtr q, PlanJoint::Response::SharedPtr r) {
        last_joint = *q;
        if (q->goal.size() != 2) {
          r->failure_reason_code = "invalid_request";
          r->failure_stage = "precheck";
          r->message = "bad goal";
          return;
        }
        if (q->goal[0] > 5.0) {
          r->failure_reason_code = "no_path";
          r->failure_stage = "planning";
          return;
        }
        r->success = true;
        r->failure_reason_code = "none";
        r->planning_time_s = 0.5;
        r->trajectory.joint_names = {"j1", "j2"};
        for (int i = 0; i < 3; ++i) {
          trajectory_msgs::msg::JointTrajectoryPoint p;
          p.positions = {q->start[0] + i * 0.1, q->start[1]};
          p.time_from_start.nanosec = static_cast<uint32_t>(i * 100000000);
          r->trajectory.points.push_back(p);
        }
      });
    pose = node->create_service<PlanPose>(
      "~/plan_pose", [this](const PlanPose::Request::SharedPtr q, PlanPose::Response::SharedPtr r) {
        last_pose = *q;
        r->success = true;
        r->trajectory.joint_names = {"j1", "j2"};
        r->trajectory.points.resize(1);
      });
    world = node->create_service<SetWorld>(
      "~/set_world", [this](const SetWorld::Request::SharedPtr q, SetWorld::Response::SharedPtr r) {
        last_world = *q;
        r->success = q->boxes.size() < 5;
        r->box_count = static_cast<uint32_t>(q->boxes.size());
        if (!r->success) {r->message = "too many boxes";}
      });
  }

  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Service<PlanJoint>::SharedPtr joint;
  rclcpp::Service<PlanPose>::SharedPtr pose;
  rclcpp::Service<SetWorld>::SharedPtr world;
  PlanJoint::Request last_joint;
  PlanPose::Request last_pose;
  SetWorld::Request last_world;
};

struct Fixture
{
  Fixture()
  {
    server = std::make_unique<StubServer>();
    client_node = std::make_shared<rclcpp::Node>("curobo_client");
    exec.add_node(server->node);
    exec.add_node(client_node);
    spinner = std::thread([this] {exec.spin();});
    curobo_planner::Options o;
    o.wait_for_service_sec = 5.0;
    o.timeout_sec = 5.0;
    planner = std::make_unique<curobo_planner::CuroboClientPlanner>(client_node, o);
  }
  ~Fixture()
  {
    exec.cancel();
    spinner.join();
  }

  std::unique_ptr<StubServer> server;
  std::shared_ptr<rclcpp::Node> client_node;
  rclcpp::executors::SingleThreadedExecutor exec;
  std::thread spinner;
  std::unique_ptr<curobo_planner::CuroboClientPlanner> planner;
};

mp::PlanningRequest jointRequest(double goal0)
{
  mp::PlanningRequest r;
  r.start_joint_state = {0.0, 0.2};
  r.target_joint_state = {goal0, 0.2};
  return r;
}
}  // namespace

TEST(CuroboClient, JointPlanBecomesATimedTrajectory)
{
  Fixture f;
  const auto r = f.planner->plan(jointRequest(1.0));
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_EQ(f.planner->name(), "curobo");
  EXPECT_EQ(r.trajectory.joint_names, (std::vector<std::string>{"j1", "j2"}));
  ASSERT_EQ(r.trajectory.points.size(), 3U);
  EXPECT_NEAR(r.trajectory.points[2].time_from_start_sec, 0.2, 1e-9);
  EXPECT_NEAR(r.trajectory.points[2].positions[0], 0.2, 1e-9);
  EXPECT_EQ(r.planning_time_ms, 500U);
  EXPECT_DOUBLE_EQ(r.path_fraction, 1.0);
}

TEST(CuroboClient, ServerFailuresKeepCodeAndStage)
{
  Fixture f;
  auto r = f.planner->plan(jointRequest(9.0));
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.failure_reason_code, "no_path");
  EXPECT_EQ(r.failure_stage, mp::FailureStage::kPlanning);

  mp::PlanningRequest bad = jointRequest(1.0);
  bad.target_joint_state = {1.0, 2.0, 3.0};
  r = f.planner->plan(bad);
  EXPECT_EQ(r.failure_reason_code, "invalid_request");
  EXPECT_EQ(r.failure_stage, mp::FailureStage::kPrecheck);
}

TEST(CuroboClient, PoseGoalSendsPositionAndQuaternion)
{
  Fixture f;
  mp::PlanningRequest r;
  r.start_joint_state = {0.0, 0.0};
  r.target_poses = {core_types::Pose{0.4, 0.1, 0.3, 0.1, 0.2, 0.3, 0.9}};
  const auto res = f.planner->plan(r);
  ASSERT_TRUE(res.success) << res.message;
  const auto & g = f.server->last_pose.goal;
  EXPECT_DOUBLE_EQ(g.position.x, 0.4);
  EXPECT_DOUBLE_EQ(g.orientation.x, 0.1);
  EXPECT_DOUBLE_EQ(g.orientation.w, 0.9);
}

TEST(CuroboClient, RejectsRequestsItCannotServe)
{
  Fixture f;
  mp::PlanningRequest none;
  EXPECT_EQ(f.planner->plan(none).failure_reason_code, "invalid_request");
  mp::PlanningRequest ambiguous;
  ambiguous.start_joint_state = {0.0, 0.0};
  ambiguous.target_poses = {core_types::Pose{}, core_types::Pose{}};
  EXPECT_EQ(f.planner->plan(ambiguous).failure_reason_code, "invalid_request");
  EXPECT_EQ(f.planner->ik(mp::IkRequest{}).failure_reason_code, "unsupported");
  EXPECT_EQ(f.planner->collision(mp::CollisionRequest{}).failure_reason_code, "unsupported");
}

TEST(CuroboClient, WorldCarriesBoxesAndTransformedPoints)
{
  Fixture f;
  mp::WorldGeometryUpdate w;
  w.frame_id = "base";
  mp::WorldGeometryBox b;
  b.id = "wall";
  b.center_x = 1.0;
  b.size_y = 2.0;
  b.orientation_w = 1.0;
  w.boxes.push_back(b);
  w.points = {{1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  w.points_origin.x = 10.0;   // points are in a frame whose origin is at x = 10 in `base`
  std::string message;
  ASSERT_TRUE(f.planner->setWorld(w, &message)) << message;
  const auto & got = f.server->last_world;
  ASSERT_EQ(got.boxes.size(), 1U);
  EXPECT_EQ(got.boxes[0].id, "wall");
  EXPECT_DOUBLE_EQ(got.boxes[0].size[1], 2.0);
  ASSERT_EQ(got.points.width, 2U);
  float xyz[3];
  std::memcpy(xyz, &got.points.data[12], 12);
  EXPECT_NEAR(xyz[0], 12.0F, 1e-5);   // second point (2,0,0) + origin (10,0,0)

  EXPECT_TRUE(f.planner->clearWorld());
  EXPECT_TRUE(f.server->last_world.boxes.empty());

  mp::WorldGeometryUpdate many;
  many.boxes.assign(6, b);
  EXPECT_FALSE(f.planner->setWorld(many, &message));
  EXPECT_EQ(message, "too many boxes");
}

TEST(CuroboClient, MissingServerIsReportedNotHung)
{
  auto node = std::make_shared<rclcpp::Node>("lonely_client");
  curobo_planner::Options o;
  o.node_name = "/nobody_home";
  o.wait_for_service_sec = 0.3;
  curobo_planner::CuroboClientPlanner planner(node, o);
  const auto r = planner.plan(jointRequest(1.0));
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.failure_reason_code, "unavailable");
}

// End to end against a real `curobo_planner_node` running cuRobo on the GPU. Only runs when
// CUROBO_START holds the robot's start joint values (space separated) and the node is up.
TEST(CuroboClientIntegration, PlansOnTheRealNode)
{
  const char * env = std::getenv("CUROBO_START");
  if (!env) {
    GTEST_SKIP() << "CUROBO_START not set";
  }
  std::vector<double> start;
  std::istringstream in(env);
  for (double v; in >> v;) {start.push_back(v);}
  auto node = std::make_shared<rclcpp::Node>("integration_client");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec] {exec.spin();});
  curobo_planner::Options o;
  o.wait_for_service_sec = 120.0;   // the node compiles CUDA kernels on first start
  o.timeout_sec = 120.0;
  curobo_planner::CuroboClientPlanner planner(node, o);

  mp::PlanningRequest r;
  r.start_joint_state = start;
  r.target_joint_state = start;
  for (auto & v : r.target_joint_state) {v += 0.2;}
  const auto res = planner.plan(r);
  mp::PlanningResult blocked;
  std::string message;
  mp::WorldGeometryUpdate w;
  mp::WorldGeometryBox slab;
  slab.id = "slab";
  slab.center_z = 0.4;
  slab.size_x = slab.size_y = slab.size_z = 4.0;   // swallows the robot
  slab.orientation_w = 1.0;
  w.boxes.push_back(slab);
  const bool world_ok = planner.setWorld(w, &message);
  blocked = planner.plan(r);
  planner.clearWorld();
  exec.cancel();
  spinner.join();

  ASSERT_TRUE(res.success) << res.failure_reason_code << ": " << res.message;
  EXPECT_EQ(res.trajectory.joint_names.size(), start.size());
  EXPECT_GT(res.trajectory.points.size(), 2U);
  for (size_t j = 0; j < start.size(); ++j) {
    EXPECT_NEAR(res.trajectory.points.back().positions[j], r.target_joint_state[j], 2e-2);
  }
  ASSERT_TRUE(world_ok) << message;
  EXPECT_FALSE(blocked.success);
  EXPECT_EQ(blocked.failure_reason_code, "no_path");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
