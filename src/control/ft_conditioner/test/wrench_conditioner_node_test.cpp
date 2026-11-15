// In-process test: a real WrenchConditionerNode, real topics, real TF.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "ft_conditioner/wrench_conditioner_node.hpp"
#include "force_core/wrench_conditioning.hpp"

using namespace std::chrono_literals;
using force_core::Mat3;
using force_core::Vec3;
using geometry_msgs::msg::WrenchStamped;

namespace
{
constexpr double kPi = 3.14159265358979323846;

Mat3 rotY(double a)
{
  Mat3 r;
  r.m = {std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a)};
  return r;
}
Mat3 rotX(double a)
{
  Mat3 r;
  r.m = {1, 0, 0, 0, std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a)};
  return r;
}
Mat3 mul(const Mat3 & a, const Mat3 & b)
{
  Mat3 r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double s = 0;
      for (int k = 0; k < 3; ++k) {s += a.at(i, k) * b.at(k, j);}
      r.m[3 * i + j] = s;
    }
  }
  return r;
}
void quat(const Mat3 & r, geometry_msgs::msg::Quaternion & q)
{
  const double tr = r.at(0, 0) + r.at(1, 1) + r.at(2, 2);
  const double w = std::sqrt(std::max(0.0, 1.0 + tr)) / 2.0;
  q.w = w;
  q.x = (r.at(2, 1) - r.at(1, 2)) / (4 * w);
  q.y = (r.at(0, 2) - r.at(2, 0)) / (4 * w);
  q.z = (r.at(1, 0) - r.at(0, 1)) / (4 * w);
}

core_types::Wrench w(double fx, double fy, double fz, double tx, double ty, double tz)
{
  core_types::Wrench x;
  x.force_x = fx; x.force_y = fy; x.force_z = fz; x.torque_x = tx; x.torque_y = ty; x.torque_z = tz;
  return x;
}

// Truth: tool origin 0.2 m beyond the sensor along its z, tool axes turned 90 degrees about y.
force_core::FrameOffset truthOffset()
{
  force_core::FrameOffset o;
  o.p = {0.0, 0.0, 0.2};
  o.R = rotY(kPi / 2);
  return o;
}
const force_core::Payload kPayload{1.2, {0.0, 0.01, 0.05}};
const Vec3 kGravity{0.0, 0.0, -9.80665};

struct Rig
{
  explicit Rig(std::vector<rclcpp::Parameter> extra = {}, bool offset_from_tf = true)
  {
    rclcpp::NodeOptions o;
    std::vector<rclcpp::Parameter> p = {
      {"input_topic", "/ft/raw"}, {"output_topic", "/ft/conditioned"}, {"world_frame", "world"},
      {"tool_frame", "tool"}, {"payload_mass_kg", kPayload.mass_kg},
      {"payload_cog_xyz", std::vector<double>{kPayload.cog.x, kPayload.cog.y, kPayload.cog.z}},
      {"tool_in_sensor_from_tf", offset_from_tf}};
    if (!offset_from_tf) {
      p.emplace_back("tool_in_sensor_xyz", std::vector<double>{0.0, 0.0, 0.2});
      p.emplace_back("tool_in_sensor_rpy", std::vector<double>{0.0, kPi / 2, 0.0});
    }
    for (auto & e : extra) {p.push_back(e);}
    o.parameter_overrides(p);
    node = std::make_shared<ft_conditioner::WrenchConditionerNode>(o);
    driver = std::make_shared<rclcpp::Node>("driver");
    tf = std::make_unique<tf2_ros::TransformBroadcaster>(driver);
    raw_pub = driver->create_publisher<WrenchStamped>("/ft/raw", rclcpp::SensorDataQoS());
    sub = driver->create_subscription<WrenchStamped>(
      "/ft/conditioned", rclcpp::SensorDataQoS(), [this](const WrenchStamped::SharedPtr m) {last = *m; ++received;});
    exec.add_node(node);
    exec.add_node(driver);
  }

  void spinFor(std::chrono::milliseconds d)
  {
    const auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end) {exec.spin_some(10ms);}
  }

  void publishTf(const Mat3 & r_world_sensor, bool with_tool)
  {
    std::vector<geometry_msgs::msg::TransformStamped> v;
    geometry_msgs::msg::TransformStamped a;
    a.header.stamp = driver->now(); a.header.frame_id = "world"; a.child_frame_id = "sensor";
    a.transform.translation.z = 1.0;
    quat(r_world_sensor, a.transform.rotation);
    v.push_back(a);
    if (with_tool) {
      const auto o = truthOffset();
      geometry_msgs::msg::TransformStamped b;
      b.header.stamp = a.header.stamp; b.header.frame_id = "sensor"; b.child_frame_id = "tool";
      b.transform.translation.x = o.p.x; b.transform.translation.y = o.p.y; b.transform.translation.z = o.p.z;
      quat(o.R, b.transform.rotation);
      v.push_back(b);
    }
    tf->sendTransform(v);
  }

  void publishRaw(const core_types::Wrench & x)
  {
    WrenchStamped m;
    m.header.stamp = driver->now();
    m.header.frame_id = "sensor";
    m.wrench.force.x = x.force_x; m.wrench.force.y = x.force_y; m.wrench.force.z = x.force_z;
    m.wrench.torque.x = x.torque_x; m.wrench.torque.y = x.torque_y; m.wrench.torque.z = x.torque_z;
    raw_pub->publish(m);
  }

  // Hold one pose and one reading until an output within `tol` of `expect` arrives.
  bool settle(const Mat3 & r, bool with_tool, const core_types::Wrench & raw, const core_types::Wrench & expect,
    double tol = 1e-6, std::chrono::milliseconds limit = 4000ms)
  {
    const auto end = std::chrono::steady_clock::now() + limit;
    while (std::chrono::steady_clock::now() < end) {
      publishTf(r, with_tool);
      publishRaw(raw);
      spinFor(40ms);
      if (received > 0 && close(last.wrench, expect, tol)) {return true;}
    }
    return false;
  }

  static bool close(const geometry_msgs::msg::Wrench & a, const core_types::Wrench & b, double tol)
  {
    return std::abs(a.force.x - b.force_x) < tol && std::abs(a.force.y - b.force_y) < tol &&
           std::abs(a.force.z - b.force_z) < tol && std::abs(a.torque.x - b.torque_x) < tol &&
           std::abs(a.torque.y - b.torque_y) < tol && std::abs(a.torque.z - b.torque_z) < tol;
  }

  bool call(const std::string & name, std::string & message)
  {
    auto client = driver->create_client<std_srvs::srv::Trigger>(name);
    if (!client->wait_for_service(3s)) {message = "no service"; return false;}
    auto fut = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (exec.spin_until_future_complete(fut, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
      message = "timeout"; return false;
    }
    const auto res = fut.get();
    message = res->message;
    return res->success;
  }

  std::shared_ptr<ft_conditioner::WrenchConditionerNode> node;
  std::shared_ptr<rclcpp::Node> driver;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf;
  rclcpp::Publisher<WrenchStamped>::SharedPtr raw_pub;
  rclcpp::Subscription<WrenchStamped>::SharedPtr sub;
  rclcpp::executors::SingleThreadedExecutor exec;
  WrenchStamped last;
  std::size_t received{0};
};

// What the sensor reads for a contact wrench at the tool point (tool axes) in sensor orientation r.
core_types::Wrench rawFor(const core_types::Wrench & contact, const Mat3 & r)
{
  const auto o = truthOffset();
  const Vec3 f_t{contact.force_x, contact.force_y, contact.force_z};
  const Vec3 f_s = force_core::rotate(o.R, f_t);
  const Vec3 load_t = force_core::cross(o.p, f_s);
  const auto g = force_core::gravityWrench(kPayload, r, kGravity);
  return w(f_s.x + g.force_x, f_s.y + g.force_y, f_s.z + g.force_z,
    load_t.x + g.torque_x, load_t.y + g.torque_y, load_t.z + g.torque_z);
}
}  // namespace

TEST(WrenchConditionerNode, RecoversTheContactAtAnOffsetToolInAnyOrientation)
{
  Rig rig;
  for (const double a : {0.0, 0.8, -1.3}) {
    const Mat3 r = mul(rotX(a), rotY(0.4 * a + 0.2));
    const auto contact = w(1.0, -2.0, -15.0, 0.0, 0.0, 0.0);   // pressing in, some friction
    EXPECT_TRUE(rig.settle(r, true, rawFor(contact, r), contact)) << "orientation " << a;
  }
  EXPECT_EQ(rig.last.header.frame_id, "tool");
}

TEST(WrenchConditionerNode, FreeSpaceReadsZeroAfterCompensation)
{
  Rig rig;
  const Mat3 r = mul(rotX(0.6), rotY(-0.9));
  EXPECT_TRUE(rig.settle(r, true, rawFor(w(0, 0, 0, 0, 0, 0), r), w(0, 0, 0, 0, 0, 0)));
}

TEST(WrenchConditionerNode, ToolOffsetFromParametersWhenThereIsNoTfFrame)
{
  Rig rig({}, /*offset_from_tf=*/false);
  const Mat3 r = rotX(0.5);
  const auto contact = w(0.5, 1.5, -10.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(rig.settle(r, /*with_tool=*/false, rawFor(contact, r), contact));
}

TEST(WrenchConditionerNode, DropsAWrenchWithoutTfInsteadOfGuessing)
{
  Rig rig;
  rig.publishRaw(w(1, 2, 3, 0, 0, 0));
  rig.spinFor(300ms);
  EXPECT_EQ(rig.received, 0U);
  EXPECT_GE(rig.node->dropped(), 1U);
}

TEST(WrenchConditionerNode, DropsAWrenchWhoseNewestTransformIsTooOld)
{
  Rig rig;     // max_tf_age default 0.1 s
  const Mat3 r = rotX(0.5);
  ASSERT_TRUE(rig.settle(r, true, rawFor(w(0, 0, -5, 0, 0, 0), r), w(0, 0, -5, 0, 0, 0)));
  // the TF publisher stalls, the sensor keeps publishing
  rig.spinFor(400ms);
  const std::size_t before = rig.received;
  for (int i = 0; i < 10; ++i) {
    rig.publishRaw(rawFor(w(0, 0, -5, 0, 0, 0), r));
    rig.spinFor(30ms);
  }
  EXPECT_EQ(rig.received, before);
  EXPECT_GE(rig.node->staleTf(), 1U);
}

TEST(WrenchConditionerNode, CapturePoseRefusesAStaleInputAndAveragesTheWindow)
{
  Rig rig;
  const Mat3 r = rotX(0.4);
  std::string msg;
  EXPECT_FALSE(rig.call("/wrench_conditioner/capture_pose", msg));    // nothing received yet
  // a noisy reading: alternate +-0.2 N around the true free-space wrench
  const auto base = rawFor(w(0, 0, 0, 0, 0, 0), r);
  for (int i = 0; i < 20; ++i) {
    auto x = base;
    x.force_x += (i % 2 ? 0.2 : -0.2);
    rig.publishTf(r, true);
    rig.publishRaw(x);
    rig.spinFor(20ms);
  }
  ASSERT_TRUE(rig.call("/wrench_conditioner/capture_pose", msg)) << msg;
  const auto got = rig.node->calibrationSamples();
  ASSERT_EQ(got.size(), 1U);
  EXPECT_NEAR(got[0].raw.force_x, base.force_x, 0.05);                // averaged, not one +-0.2 sample
  // the input stops: a later capture must fail instead of reusing the old reading
  rig.spinFor(600ms);
  EXPECT_FALSE(rig.call("/wrench_conditioner/capture_pose", msg));
  EXPECT_NE(msg.find("stale"), std::string::npos) << msg;
  EXPECT_EQ(rig.node->calibrationSamples().size(), 1U);
}

TEST(TfFreshEnough, LimitsTheAgeAndTreatsStaticAsFresh)
{
  using ft_conditioner::tfFreshEnough;
  EXPECT_TRUE(tfFreshEnough(1'000'000'000, 950'000'000, 0.1));
  EXPECT_FALSE(tfFreshEnough(1'000'000'000, 800'000'000, 0.1));
  EXPECT_TRUE(tfFreshEnough(1'000'000'000, 1'200'000'000, 0.1));   // transform newer than the wrench
  EXPECT_TRUE(tfFreshEnough(1'000'000'000, 0, 0.1));               // static transform
  EXPECT_TRUE(tfFreshEnough(1'000'000'000, 1, 0.0));               // limit off
}

TEST(WrenchConditionerNode, IdentifiesPayloadAndBiasFromCapturedPoses)
{
  // start with no payload known, a bias present on the sensor
  Rig rig({rclcpp::Parameter("payload_mass_kg", 0.0),
      rclcpp::Parameter("payload_cog_xyz", std::vector<double>{0.0, 0.0, 0.0})});
  const auto bias = w(0.3, -0.2, 0.5, 0.01, -0.02, 0.005);
  std::string msg;
  EXPECT_FALSE(rig.call("/wrench_conditioner/identify", msg));      // nothing captured yet
  EXPECT_NE(msg.find("at least 4"), std::string::npos) << msg;
  for (const double a : {0.0, 0.9, 1.8, 2.7, -1.0, -2.0}) {
    const Mat3 r = mul(rotX(a), rotY(0.6 * a - 0.5));
    const auto g = force_core::gravityWrench(kPayload, r, kGravity);
    const auto raw = w(g.force_x + bias.force_x, g.force_y + bias.force_y, g.force_z + bias.force_z,
      g.torque_x + bias.torque_x, g.torque_y + bias.torque_y, g.torque_z + bias.torque_z);
    // wait until the node has seen THIS pose (its orientation is part of the sample)
    for (int i = 0; i < 20; ++i) {
      rig.publishTf(r, true);
      rig.publishRaw(raw);
      rig.spinFor(30ms);
    }
    ASSERT_TRUE(rig.call("/wrench_conditioner/capture_pose", msg)) << msg;
  }
  ASSERT_TRUE(rig.call("/wrench_conditioner/identify", msg)) << msg;
  const auto cfg = rig.node->config();
  EXPECT_NEAR(cfg.payload.mass_kg, kPayload.mass_kg, 1e-6);
  EXPECT_NEAR(cfg.payload.cog.y, kPayload.cog.y, 1e-6);
  EXPECT_NEAR(cfg.payload.cog.z, kPayload.cog.z, 1e-6);
  EXPECT_NEAR(cfg.bias.force_z, bias.force_z, 1e-6);
  EXPECT_NE(msg.find("payload_mass_kg"), std::string::npos);      // a snippet to keep them
  // and it now conditions a free-space reading to zero, bias included
  const Mat3 r = mul(rotX(1.2), rotY(0.3));
  const auto g = force_core::gravityWrench(kPayload, r, kGravity);
  const auto raw = w(g.force_x + bias.force_x, g.force_y + bias.force_y, g.force_z + bias.force_z,
    g.torque_x + bias.torque_x, g.torque_y + bias.torque_y, g.torque_z + bias.torque_z);
  EXPECT_TRUE(rig.settle(r, true, raw, w(0, 0, 0, 0, 0, 0), 1e-5));
  ASSERT_TRUE(rig.call("/wrench_conditioner/clear_samples", msg));
  EXPECT_EQ(rig.node->samples(), 0U);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
