// ROS-level tests for coverage_planner_node: the node class is compiled in directly (its main()
// is renamed away) and driven through its real GenerateCoveragePath service in one process.
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#define main coverage_planner_node_main
#include "../../src/ros/coverage_planner_node.cpp"
#undef main

using namespace std::chrono_literals;
using coverage_planner_msgs::srv::GenerateCoveragePath;

namespace
{

nav_msgs::msg::OccupancyGrid freeGrid(unsigned w, unsigned h, float res)
{
  nav_msgs::msg::OccupancyGrid g;
  g.header.frame_id = "map";
  g.info.resolution = res;
  g.info.width = w;
  g.info.height = h;
  g.info.origin.orientation.w = 1.0;
  g.data.assign(static_cast<size_t>(w) * h, 0);
  return g;
}

class CoveragePlannerNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<coverage_planner::CoveragePlannerNode>();
    client_node_ = std::make_shared<rclcpp::Node>("coverage_planner_node_test_client");
    client_ = client_node_->create_client<GenerateCoveragePath>(
      "/coverage_planner_node/generate_coverage_path");
    exec_.add_node(node_);
    exec_.add_node(client_node_);
    spinner_ = std::thread([this]() { exec_.spin(); });
    ASSERT_TRUE(client_->wait_for_service(5s));
  }

  void TearDown() override
  {
    exec_.cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
  }

  GenerateCoveragePath::Response::SharedPtr call(const GenerateCoveragePath::Request::SharedPtr & r)
  {
    auto fut = client_->async_send_request(r);
    if (fut.wait_for(10s) != std::future_status::ready) {
      return nullptr;
    }
    return fut.get();
  }

  std::shared_ptr<coverage_planner::CoveragePlannerNode> node_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<GenerateCoveragePath>::SharedPtr client_;
  rclcpp::executors::MultiThreadedExecutor exec_{rclcpp::ExecutorOptions(), 2};
  std::thread spinner_;
};

}  // namespace

TEST_F(CoveragePlannerNodeTest, NoMapFailsWithMessage)
{
  auto req = std::make_shared<GenerateCoveragePath::Request>();
  const auto res = call(req);
  ASSERT_TRUE(res);
  EXPECT_FALSE(res->success);
  EXPECT_NE(res->message.find("No map"), std::string::npos);
  EXPECT_TRUE(res->segments.empty());
}

TEST_F(CoveragePlannerNodeTest, UnknownPatternListsAvailable)
{
  auto req = std::make_shared<GenerateCoveragePath::Request>();
  req->map = freeGrid(40, 40, 0.05F);
  req->pattern = "no_such_pattern";
  const auto res = call(req);
  ASSERT_TRUE(res);
  EXPECT_FALSE(res->success);
  EXPECT_NE(res->message.find("Unknown coverage pattern"), std::string::npos);
  EXPECT_NE(res->message.find("boustrophedon"), std::string::npos);
}

TEST_F(CoveragePlannerNodeTest, RequestMapGivesPathAndSegments)
{
  auto req = std::make_shared<GenerateCoveragePath::Request>();
  req->map = freeGrid(60, 60, 0.05F);  // 3 m x 3 m, all free
  req->swath = 0.4F;
  req->step = 0.4F;
  req->obstacle_inflation_radius = 0.01F;
  req->requires_contact = true;
  const auto res = call(req);
  ASSERT_TRUE(res);
  ASSERT_TRUE(res->success) << res->message;
  EXPECT_EQ(res->pattern_used, "boustrophedon");
  EXPECT_GT(res->num_waypoints, 2);
  EXPECT_EQ(static_cast<size_t>(res->num_waypoints), res->path.poses.size());
  EXPECT_GT(res->total_distance, 1.0F);
  EXPECT_EQ(res->path.header.frame_id, "map");
  ASSERT_FALSE(res->segments.empty());
  EXPECT_EQ(res->segments.size(), res->segment_labels.size());
  for (const auto & s : res->segments) {
    EXPECT_TRUE(s.requires_contact);
  }
}

TEST_F(CoveragePlannerNodeTest, SubscribedMapIsUsedWhenRequestHasNone)
{
  auto pub = client_node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)));
  const auto map = freeGrid(40, 40, 0.05F);
  bool ok = false;
  for (int i = 0; i < 50 && !ok; ++i) {
    pub->publish(map);
    std::this_thread::sleep_for(100ms);
    auto req = std::make_shared<GenerateCoveragePath::Request>();
    req->obstacle_inflation_radius = 0.01F;
    const auto res = call(req);
    ASSERT_TRUE(res);
    ok = res->success;
  }
  EXPECT_TRUE(ok) << "service never used the subscribed map";
}

TEST_F(CoveragePlannerNodeTest, SuccessPublishesPathMarkersAndDiagnostics)
{
  std::atomic<int> paths{0}, markers{0}, diags{0};
  auto s1 = client_node_->create_subscription<nav_msgs::msg::Path>(
    "/coverage_planner_node/coverage_path", 10, [&](nav_msgs::msg::Path::SharedPtr) { ++paths; });
  auto s2 = client_node_->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/coverage_planner_node/coverage_markers", 10,
    [&](visualization_msgs::msg::MarkerArray::SharedPtr) { ++markers; });
  auto s3 = client_node_->create_subscription<std_msgs::msg::String>(
    "/coverage_planner_node/coverage_diagnostics", 10, [&](std_msgs::msg::String::SharedPtr m) {
      if (m->data.find("pattern=boustrophedon") != std::string::npos) {
        ++diags;
      }
    });
  std::this_thread::sleep_for(500ms);  // discovery
  auto req = std::make_shared<GenerateCoveragePath::Request>();
  req->map = freeGrid(40, 40, 0.05F);
  req->obstacle_inflation_radius = 0.01F;
  const auto res = call(req);
  ASSERT_TRUE(res);
  ASSERT_TRUE(res->success) << res->message;
  for (int i = 0; i < 50 && (paths == 0 || markers == 0 || diags == 0); ++i) {
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_GE(paths.load(), 1);
  EXPECT_GE(markers.load(), 1);
  EXPECT_GE(diags.load(), 1);
}
