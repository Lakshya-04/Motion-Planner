// ROS-level tests for wall_coverage_service_node: ~/start and ~/stop, and the full chain
// (wall scene -> coverage_planner_node service -> lifted plan on ~/coverage_plan).
// The two node sources are compiled into this test with main() renamed away (see CMakeLists).
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include "coverage_planner/ros/coverage_planner_node.hpp"
#include "coverage_planner/ros/wall_coverage_service_node.hpp"

using namespace std::chrono_literals;
using Trigger = std_srvs::srv::Trigger;

#ifndef WALL_SCENE_SAMPLE_PATH
#error "WALL_SCENE_SAMPLE_PATH must be provided by CMake (-D)"
#endif

namespace
{

class WallCoverageServiceNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void start(const std::vector<rclcpp::Parameter> & params, bool with_planner)
  {
    rclcpp::NodeOptions o;
    o.parameter_overrides(params);
    node_ = std::make_shared<coverage_planner::WallCoverageServiceNode>(o);
    client_node_ = std::make_shared<rclcpp::Node>("wall_service_test_client");
    start_ = client_node_->create_client<Trigger>("/wall_coverage_service_node/start");
    stop_ = client_node_->create_client<Trigger>("/wall_coverage_service_node/stop");
    exec_.add_node(node_);
    exec_.add_node(client_node_);
    if (with_planner) {
      planner_ = std::make_shared<coverage_planner::CoveragePlannerNode>();
      exec_.add_node(planner_);
    }
    spinner_ = std::thread([this]() { exec_.spin(); });
    ASSERT_TRUE(start_->wait_for_service(5s));
    ASSERT_TRUE(stop_->wait_for_service(5s));
  }

  void TearDown() override
  {
    exec_.cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
  }

  Trigger::Response::SharedPtr trigger(const rclcpp::Client<Trigger>::SharedPtr & c)
  {
    auto fut = c->async_send_request(std::make_shared<Trigger::Request>());
    if (fut.wait_for(20s) != std::future_status::ready) {
      return nullptr;
    }
    return fut.get();
  }

  std::shared_ptr<coverage_planner::WallCoverageServiceNode> node_;
  std::shared_ptr<coverage_planner::CoveragePlannerNode> planner_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<Trigger>::SharedPtr start_, stop_;
  rclcpp::executors::MultiThreadedExecutor exec_{rclcpp::ExecutorOptions(), 3};
  std::thread spinner_;
};

}  // namespace

TEST_F(WallCoverageServiceNodeTest, StartWithoutSceneParameterFails)
{
  start({}, false);
  const auto res = trigger(start_);
  ASSERT_TRUE(res);
  EXPECT_FALSE(res->success);
  EXPECT_NE(res->message.find("wall_scene_json"), std::string::npos);
  // A failed start must release the busy flag: a second call fails the same way, not "in progress".
  const auto again = trigger(start_);
  ASSERT_TRUE(again);
  EXPECT_EQ(again->message, res->message);
}

TEST_F(WallCoverageServiceNodeTest, StopWhenIdleSucceeds)
{
  start({}, false);
  const auto res = trigger(stop_);
  ASSERT_TRUE(res);
  EXPECT_TRUE(res->success);
  EXPECT_EQ(res->message, "nothing was running");
}

TEST_F(WallCoverageServiceNodeTest, UnknownWallIdFails)
{
  start(
    {rclcpp::Parameter("wall_scene_json", WALL_SCENE_SAMPLE_PATH),
     rclcpp::Parameter("wall_id", 999)},
    false);
  const auto res = trigger(start_);
  ASSERT_TRUE(res);
  EXPECT_FALSE(res->success);
  EXPECT_FALSE(res->message.empty());
}

TEST_F(WallCoverageServiceNodeTest, StartPublishesLiftedPlan)
{
  start(
    {rclcpp::Parameter("wall_scene_json", WALL_SCENE_SAMPLE_PATH), rclcpp::Parameter("wall_id", 0),
     rclcpp::Parameter("resolution", 0.05)},
    true);
  std::atomic<bool> got{false};
  coverage_planner_msgs::msg::CoveragePlan plan;
  std::mutex m;
  auto sub = client_node_->create_subscription<coverage_planner_msgs::msg::CoveragePlan>(
    "/wall_coverage_service_node/coverage_plan", rclcpp::QoS(1),
    [&](coverage_planner_msgs::msg::CoveragePlan::SharedPtr p) {
      std::lock_guard<std::mutex> l(m);
      plan = *p;
      got = true;
    });
  std::this_thread::sleep_for(500ms);
  const auto res = trigger(start_);
  ASSERT_TRUE(res);
  ASSERT_TRUE(res->success) << res->message;
  for (int i = 0; i < 300 && !got; ++i) {
    std::this_thread::sleep_for(100ms);
  }
  ASSERT_TRUE(got) << "no plan published";
  std::lock_guard<std::mutex> l(m);
  EXPECT_TRUE(plan.success) << plan.message;
  EXPECT_FALSE(plan.path.poses.empty());
  EXPECT_FALSE(plan.segments.empty());
}
