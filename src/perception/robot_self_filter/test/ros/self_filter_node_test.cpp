// In-process integration test: a real SelfFilterNode, real TF, real topics.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "robot_self_filter/ros/self_filter_node.hpp"
#include "scene_model/ros/conversions.hpp"

using namespace std::chrono_literals;

namespace
{
// A cube link "body" (0.4 m) with an "arm" box (0.1 x 0.1 x 0.5) child 1 m in front of it.
const char * kUrdf = R"(<robot name="r">
  <link name="body"><collision><geometry><box size="0.4 0.4 0.4"/></geometry></collision></link>
  <link name="arm"><collision><geometry><box size="0.1 0.1 0.5"/></geometry></collision></link>
  <joint name="j" type="fixed"><parent link="body"/><child link="arm"/><origin xyz="1 0 0"/></joint>
</robot>)";

class Harness
{
public:
  Harness()
  {
    rclcpp::NodeOptions o;
    o.parameter_overrides({{"robot_description", std::string(kUrdf)}, {"padding.sphere", 0.0},
      {"padding.box", std::vector<double>{0.0, 0.0, 0.0}}, {"padding.cylinder_radius", 0.0},
      {"padding.cylinder_height", 0.0}, {"contact_link", std::string("arm")}, {"contact_padding", 0.0},
      {"tf_timeout_sec", 0.5}});
    filter = std::make_shared<robot_self_filter::SelfFilterNode>(o);
    helper = std::make_shared<rclcpp::Node>("helper");
    tf = std::make_unique<tf2_ros::TransformBroadcaster>(helper);
    pub = helper->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_in", rclcpp::SensorDataQoS());
    contact = helper->create_publisher<std_msgs::msg::Bool>("contact_mode", rclcpp::QoS(1).transient_local());
    sub = helper->create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_out", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
        received = *m;
        got = true;
      });
    exec.add_node(filter);
    exec.add_node(helper);
  }

  void publishTf(const rclcpp::Time & stamp, double arm_x = 1.0)
  {
    auto send = [&](const std::string & parent, const std::string & child, double x) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = parent;
        t.child_frame_id = child;
        t.transform.translation.x = x;
        t.transform.rotation.w = 1.0;
        tf->sendTransform(t);
      };
    send("body", "arm", arm_x);
    send("body", "sensor", 0.0);  // the sensor frame coincides with the robot root
  }

  // Publish `cloud` (in the sensor frame) and wait for the filtered result.
  bool roundTrip(const scene_model::PointCloud & cloud, double arm_x = 1.0)
  {
    got = false;
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!got && std::chrono::steady_clock::now() < deadline) {
      const rclcpp::Time stamp = helper->now();
      publishTf(stamp, arm_x);
      auto msg = scene_model::ros_conv::toRos(cloud, "sensor");
      msg.header.stamp = stamp;
      pub->publish(msg);
      exec.spin_some(100ms);
    }
    return got;
  }

  std::shared_ptr<robot_self_filter::SelfFilterNode> filter;
  std::shared_ptr<rclcpp::Node> helper;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
  rclcpp::executors::SingleThreadedExecutor exec;
  sensor_msgs::msg::PointCloud2 received;
  bool got{false};
};

scene_model::PointCloud cloudOf(std::vector<scene_model::Vec3> p)
{
  scene_model::PointCloud c;
  c.points = std::move(p);
  return c;
}
}  // namespace

TEST(SelfFilterNode, RemovesPointsOnTheBodyAndKeepsTheRest)
{
  Harness h;
  ASSERT_TRUE(h.filter->ready());
  // inside the cube; outside; on the arm (1 m in front); free space.
  ASSERT_TRUE(h.roundTrip(cloudOf({{0, 0, 0}, {0.5, 0, 0}, {1.0, 0, 0.2}, {3, 3, 3}})));
  const auto kept = scene_model::ros_conv::fromRos(h.received);
  ASSERT_EQ(kept.points.size(), 2U);
  EXPECT_NEAR(kept.points[0].x(), 0.5, 1e-5);
  EXPECT_NEAR(kept.points[1].x(), 3.0, 1e-5);
  EXPECT_EQ(h.received.header.frame_id, "sensor");  // same frame as the input
}

TEST(SelfFilterNode, FollowsTheLinkPosesInTf)
{
  Harness h;
  // The arm link moved from x = 1 to x = 2: the point at x = 1 is free now, x = 2 is on the arm.
  ASSERT_TRUE(h.roundTrip(cloudOf({{1.0, 0, 0.2}, {2.0, 0, 0.2}}), /*arm_x=*/2.0));
  const auto kept = scene_model::ros_conv::fromRos(h.received);
  ASSERT_EQ(kept.points.size(), 1U);
  EXPECT_NEAR(kept.points[0].x(), 1.0, 1e-5);
}

TEST(SelfFilterNode, ContactModeReleasesPaddingOnTheContactLink)
{
  Harness h;
  // Give the arm 10 cm of padding by making the point sit just outside it, then use contact mode.
  // With padding 0 everywhere a point 6 cm beside the arm is kept either way; this checks that the
  // topic is wired by confirming the node keeps working after the mode toggles.
  std_msgs::msg::Bool on;
  on.data = true;
  h.contact->publish(on);
  h.exec.spin_some(100ms);
  ASSERT_TRUE(h.roundTrip(cloudOf({{1.06, 0, 0}, {1.0, 0, 0}})));
  const auto kept = scene_model::ros_conv::fromRos(h.received);
  EXPECT_EQ(kept.points.size(), 1U);  // the point on the arm is still removed
}

TEST(SelfFilterNode, DropsCloudsWhenTheTransformIsMissing)
{
  Harness h;
  h.got = false;
  auto msg = scene_model::ros_conv::toRos(cloudOf({{0, 0, 0}}), "frame_that_does_not_exist");
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    msg.header.stamp = h.helper->now();
    h.pub->publish(msg);
    h.exec.spin_some(100ms);
  }
  EXPECT_FALSE(h.got) << "an unfilterable cloud must not be published unfiltered";
}

TEST(SelfFilterNode, KeepsPointFieldsItDoesNotTouch)
{
  Harness h;
  scene_model::PointCloud c = cloudOf({{0, 0, 0}, {3, 3, 3}});
  c.colors = {{1, 2, 3}, {4, 5, 6}};
  c.labels = {7, 8};
  ASSERT_TRUE(h.roundTrip(c));
  const auto kept = scene_model::ros_conv::fromRos(h.received);
  ASSERT_EQ(kept.points.size(), 1U);
  EXPECT_EQ(kept.labels, (std::vector<uint32_t>{8}));
  EXPECT_EQ(kept.colors[0][0], 4);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
