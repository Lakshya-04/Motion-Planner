// JtacControllerCompliancePort against a stand-in controller node: a plain rclcpp node named like
// the real controller that hosts the admittance.* parameters and the target_force / max_offset /
// measured_wrench topics. Pins which parameters and topics each CompliancePort call touches and
// the state-machine guards (tare/engage/disengage ordering, wrench staleness).
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include "compliant_trajectory_controller/jtac_controller_compliance_port.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
using compliant_trajectory_controller::JtacControllerCompliancePort;

namespace
{

constexpr const char * kName = "jtac_port_test_controller";

core_types::ComplianceConfig makeConfig()
{
  core_types::ComplianceConfig c;
  c.control_frame = "tool0";
  c.selected_axes = {false, false, true, false, false, false};
  c.mass = {2.0, 2.0, 2.0};
  c.stiffness = {400.0, 400.0, 400.0};
  c.damping = {40.0, 40.0, 40.0};  // ratio = 40 / (2 sqrt(2*400)) = 0.7071
  c.target_force_n = 7.5;
  c.max_offset_m = 0.03;
  return c;
}

class FakeController
{
public:
  FakeController()
  {
    node = std::make_shared<rclcpp::Node>(kName);
    node->declare_parameter("admittance.mass", std::vector<double>(6, 1.0));
    node->declare_parameter("admittance.stiffness", std::vector<double>(6, 1.0));
    node->declare_parameter("admittance.damping_ratio", std::vector<double>(6, 1.0));
    node->declare_parameter("control.frame.id", std::string("base"));
    node->declare_parameter("admittance.selected_axes", std::vector<bool>(6, true));
    target_sub = node->create_subscription<std_msgs::msg::Float64>(
      std::string("/") + kName + "/target_force_n", 10, [this](std_msgs::msg::Float64::SharedPtr m) {
        std::lock_guard<std::mutex> l(mtx);
        targets.push_back(m->data);
      });
    offset_sub = node->create_subscription<std_msgs::msg::Float64>(
      std::string("/") + kName + "/max_offset_m", 10, [this](std_msgs::msg::Float64::SharedPtr m) {
        std::lock_guard<std::mutex> l(mtx);
        offsets.push_back(m->data);
      });
    wrench_pub = node->create_publisher<geometry_msgs::msg::WrenchStamped>(
      std::string("/") + kName + "/measured_wrench", 10);
    exec.add_node(node);
    th = std::thread([this]() { exec.spin(); });
  }
  ~FakeController()
  {
    exec.cancel();
    th.join();
  }

  std::vector<double> getTargets()
  {
    std::lock_guard<std::mutex> l(mtx);
    return targets;
  }
  std::vector<double> getOffsets()
  {
    std::lock_guard<std::mutex> l(mtx);
    return offsets;
  }
  bool waitTargets(size_t n)
  {
    for (int i = 0; i < 50; ++i) {
      if (getTargets().size() >= n) {
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  void publishWrench(double fz, const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::WrenchStamped w;
    w.header.stamp = stamp;
    w.wrench.force.z = fz;
    wrench_pub->publish(w);
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor exec;
  std::thread th;
  std::mutex mtx;
  std::vector<double> targets, offsets;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_sub, offset_sub;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub;
};

class JtacCompliancePortTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  // The port starts its spin thread in the constructor and cancels the executor in the destructor;
  // a cancel that lands before the thread entered spin() is lost and the join never returns (a race
  // in the port itself, noted in the PR). Tests let the thread get going first.
  static void settle() { std::this_thread::sleep_for(300ms); }

  JtacControllerCompliancePort::Options options(double param_timeout = 3.0)
  {
    JtacControllerCompliancePort::Options o;
    o.controller_node_name = kName;
    o.parameter_set_timeout_s = param_timeout;
    o.wrench_staleness_timeout_s = 0.5;
    return o;
  }
};

}  // namespace

TEST_F(JtacCompliancePortTest, GuardsBeforeEngage)
{
  JtacControllerCompliancePort port(options());
  settle();
  std::string err;
  EXPECT_TRUE(port.tare(err));
  EXPECT_FALSE(port.disengage(err));
  EXPECT_EQ(err, "not engaged");
  EXPECT_FALSE(port.protectiveStop(err));
  const auto st = port.status();
  EXPECT_FALSE(st.engaged);
  EXPECT_FALSE(st.wrench_valid);
}

TEST_F(JtacCompliancePortTest, EngageFailsWithoutControllerParameterService)
{
  JtacControllerCompliancePort port(options(0.3));
  settle();
  std::string err;
  EXPECT_FALSE(port.engage(makeConfig(), err));
  EXPECT_NE(err.find("not available"), std::string::npos);
  EXPECT_FALSE(port.status().engaged);
}

TEST_F(JtacCompliancePortTest, EngageSetsParametersAndPublishesTargetAndOffset)
{
  FakeController ctl;
  JtacControllerCompliancePort port(options());
  std::this_thread::sleep_for(500ms);  // discovery
  std::string err;
  ASSERT_TRUE(port.engage(makeConfig(), err)) << err;
  EXPECT_TRUE(port.status().engaged);

  const auto mass = ctl.node->get_parameter("admittance.mass").as_double_array();
  ASSERT_EQ(mass.size(), 6U);
  EXPECT_DOUBLE_EQ(mass[0], 2.0);
  EXPECT_DOUBLE_EQ(mass[3], 0.5);  // rotational defaults
  const auto k = ctl.node->get_parameter("admittance.stiffness").as_double_array();
  EXPECT_DOUBLE_EQ(k[2], 400.0);
  EXPECT_DOUBLE_EQ(k[5], 50.0);
  const auto zeta = ctl.node->get_parameter("admittance.damping_ratio").as_double_array();
  EXPECT_NEAR(zeta[0], 0.70710678, 1e-6);
  EXPECT_DOUBLE_EQ(zeta[4], 1.0);
  EXPECT_EQ(ctl.node->get_parameter("control.frame.id").as_string(), "tool0");
  const auto axes = ctl.node->get_parameter("admittance.selected_axes").as_bool_array();
  EXPECT_EQ(axes, (std::vector<bool>{false, false, true, false, false, false}));

  ASSERT_TRUE(ctl.waitTargets(1));
  EXPECT_DOUBLE_EQ(ctl.getTargets().front(), 7.5);
  for (int i = 0; i < 20 && ctl.getOffsets().empty(); ++i) {
    std::this_thread::sleep_for(100ms);
  }
  ASSERT_FALSE(ctl.getOffsets().empty());
  EXPECT_DOUBLE_EQ(ctl.getOffsets().front(), 0.03);

  // Engage is the rigid->compliant switch: once only.
  EXPECT_FALSE(port.engage(makeConfig(), err));
  EXPECT_NE(err.find("already engaged"), std::string::npos);
  // tare is refused in contact.
  EXPECT_FALSE(port.tare(err));
}

TEST_F(JtacCompliancePortTest, DisengageDeselectsAllAxesAndZeroesTheTarget)
{
  FakeController ctl;
  JtacControllerCompliancePort port(options());
  std::this_thread::sleep_for(500ms);
  std::string err;
  ASSERT_TRUE(port.engage(makeConfig(), err)) << err;
  ASSERT_TRUE(port.setTargetForce(12.0, err));
  ASSERT_TRUE(ctl.waitTargets(2));
  EXPECT_DOUBLE_EQ(ctl.getTargets()[1], 12.0);

  ASSERT_TRUE(port.disengage(err)) << err;
  EXPECT_FALSE(port.status().engaged);
  const auto axes = ctl.node->get_parameter("admittance.selected_axes").as_bool_array();
  EXPECT_EQ(axes, std::vector<bool>(6, false));
  ASSERT_TRUE(ctl.waitTargets(3));
  EXPECT_DOUBLE_EQ(ctl.getTargets()[2], 0.0);
  // and a second disengage is refused
  EXPECT_FALSE(port.disengage(err));
}

TEST_F(JtacCompliancePortTest, ProtectiveStopUnloadsTheTargetForce)
{
  FakeController ctl;
  JtacControllerCompliancePort port(options());
  std::this_thread::sleep_for(500ms);
  std::string err;
  ASSERT_TRUE(port.engage(makeConfig(), err)) << err;
  ASSERT_TRUE(port.protectiveStop(err)) << err;
  ASSERT_TRUE(ctl.waitTargets(2));
  EXPECT_DOUBLE_EQ(ctl.getTargets()[1], 0.0);
  EXPECT_TRUE(port.status().engaged);  // a stop does not disengage; the caller does that
}

TEST_F(JtacCompliancePortTest, StatusReportsFreshWrenchOnlyWhileEngagedAndNotStale)
{
  FakeController ctl;
  JtacControllerCompliancePort port(options());
  std::this_thread::sleep_for(500ms);

  // Wrench arrives before engage: stored, but not valid out of contact.
  ctl.publishWrench(3.0, ctl.node->now());
  std::this_thread::sleep_for(300ms);
  EXPECT_FALSE(port.status().wrench_valid);

  std::string err;
  ASSERT_TRUE(port.engage(makeConfig(), err)) << err;
  ctl.publishWrench(4.5, ctl.node->now());
  bool valid = false;
  core_types::ComplianceStatus st;
  for (int i = 0; i < 30 && !valid; ++i) {
    std::this_thread::sleep_for(50ms);
    ctl.publishWrench(4.5, ctl.node->now());
    st = port.status();
    valid = st.wrench_valid;
  }
  ASSERT_TRUE(valid);
  EXPECT_DOUBLE_EQ(st.wrench.force_z, 4.5);
  EXPECT_GT(st.stamp_ns, 0U);

  // Stop publishing: the last sample goes stale after the 0.5 s timeout.
  std::this_thread::sleep_for(800ms);
  EXPECT_FALSE(port.status().wrench_valid);

  // A sample stamped in the future is also rejected.
  ctl.publishWrench(1.0, ctl.node->now() + rclcpp::Duration::from_seconds(10.0));
  std::this_thread::sleep_for(300ms);
  EXPECT_FALSE(port.status().wrench_valid);
}
