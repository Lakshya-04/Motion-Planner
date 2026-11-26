// Loads the controller with hand-made interfaces on the compliant_trajectory_controller demo URDF.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "cartesian_tracking_controller/cartesian_tracking_controller.hpp"
#include "controller_interface/controller_interface_params.hpp"
#include "diff_ik_core/tool_correction.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{
using cartesian_tracking_controller::CartesianTrackingController;
using std_msgs::msg::Float64MultiArray;

const std::vector<std::string> kJoints = {
  "lift_prismatic_joint", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
// away from a singularity (arm not stretched, wrist not straight)
const std::vector<double> kQ = {0.2, 0.3, 0.6, 1.0, 0.4, 0.8, 0.3};
const std::vector<double> kTool = {0.05, 0.02, 0.10, 0.1, 0.0, 0.2};   // x y z roll pitch yaw

std::string readUrdf()
{
  std::ifstream f(DEMO_URDF_PATH);
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}

diff_ik_core::ToolOffset toolOffset(const std::vector<double> & v)
{
  diff_ik_core::ToolOffset t;
  t.xyz = Eigen::Vector3d(v[0], v[1], v[2]);
  t.rpy = Eigen::Vector3d(v[3], v[4], v[5]);
  return t;
}

hardware_interface::InterfaceDescription describe(const std::string & joint, const std::string & type)
{
  hardware_interface::InterfaceInfo info;
  info.name = type;
  info.data_type = "double";
  info.size = 1;
  info.enable_limits = false;
  return hardware_interface::InterfaceDescription(joint, info);
}

// FK and Jacobian of the flange with the same KDL plugin the controller uses.
class Kin
{
public:
  Kin()
  {
    node_ = std::make_shared<rclcpp::Node>(
      "kin_probe", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).parameter_overrides(
        {rclcpp::Parameter("kinematics.base", "base_link"), rclcpp::Parameter("kinematics.tip", "link6"),
          rclcpp::Parameter("kinematics.alpha", 0.0005)}));
    loader_ = std::make_unique<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
      "kinematics_interface", "kinematics_interface::KinematicsInterface");
    ik_.reset(loader_->createUnmanagedInstance("kinematics_interface_kdl/KinematicsInterfaceKDL"));
    EXPECT_TRUE(ik_->initialize(readUrdf(), node_->get_node_parameters_interface(), "kinematics"));
  }
  Eigen::Isometry3d flange(const std::vector<double> & q)
  {
    Eigen::Isometry3d t;
    EXPECT_TRUE(ik_->calculate_link_transform(Eigen::Map<const Eigen::VectorXd>(q.data(), 7), "link6", t));
    return t;
  }
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(const std::vector<double> & q)
  {
    Eigen::Matrix<double, 6, Eigen::Dynamic> j(6, 7);
    EXPECT_TRUE(ik_->calculate_jacobian(Eigen::Map<const Eigen::VectorXd>(q.data(), 7), "link6", j));
    return j;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> loader_;
  std::unique_ptr<kinematics_interface::KinematicsInterface> ik_;
};

class Rig
{
public:
  explicit Rig(
    const std::string & name, const std::vector<double> & tool = kTool, bool position_cmd = false,
    const std::vector<rclcpp::Parameter> & extra = {})
  : name_(name), q_(kQ)
  {
    std::vector<std::string> cmd_types = {"velocity"};
    if (position_cmd) {
      cmd_types.push_back("position");
    }
    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("joints", kJoints),
      rclcpp::Parameter("command_interfaces", cmd_types),
      rclcpp::Parameter("kinematics.base", "base_link"),
      rclcpp::Parameter("kinematics.tip", "link6"),
      rclcpp::Parameter("tool.xyz", std::vector<double>(tool.begin(), tool.begin() + 3)),
      rclcpp::Parameter("tool.rpy", std::vector<double>(tool.begin() + 3, tool.end())),
      rclcpp::Parameter("diagnostics_rate", 1000.0),
      rclcpp::Parameter("reference_timeout", 0.5)};
    params.insert(params.end(), extra.begin(), extra.end());

    controller_interface::ControllerInterfaceParams p;
    p.controller_name = name;
    p.robot_description = readUrdf();
    p.update_rate = 100;
    p.controller_manager_update_rate = 100;
    p.node_options = rclcpp::NodeOptions().parameter_overrides(params);
    ctrl_ = std::make_unique<CartesianTrackingController>();
    init_ok_ = ctrl_->init(p) == controller_interface::return_type::OK;
    cmd_types_ = cmd_types;

    test_node_ = std::make_shared<rclcpp::Node>(name + "_test");
    pose_pub_ = test_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/" + name + "/reference_pose", 10);
    twist_pub_ = test_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/" + name + "/reference_twist", 10);
    tool_pub_ = test_node_->create_publisher<Float64MultiArray>("/" + name + "/tool_offset", 10);
    diag_sub_ = test_node_->create_subscription<Float64MultiArray>(
      "/" + name + "/diagnostics", 10, [this](const Float64MultiArray::SharedPtr m) {
        diag_ = m->data;
        diag_labels_.clear();
        for (const auto & d : m->layout.dim) {
          diag_labels_.push_back(d.label);
        }
      });
  }

  bool configure()
  {
    if (!init_ok_) {
      return false;
    }
    if (ctrl_->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      return false;
    }
    exec_.add_node(ctrl_->get_node()->get_node_base_interface());
    exec_.add_node(test_node_);
    return true;
  }

  void assign()
  {
    std::vector<hardware_interface::LoanedCommandInterface> cmds;
    std::vector<hardware_interface::LoanedStateInterface> states;
    for (size_t j = 0; j < kJoints.size(); ++j) {
      for (const auto & t : cmd_types_) {
        auto c = std::make_shared<hardware_interface::CommandInterface>(describe(kJoints[j], t));
        cmd_[kJoints[j] + "/" + t] = c;
        cmds.emplace_back(c, [] {});
      }
      auto s = std::make_shared<hardware_interface::StateInterface>(describe(kJoints[j], "position"));
      (void)s->set_value(q_[j]);
      state_.push_back(s);
      states.emplace_back(s);
    }
    ctrl_->assign_interfaces(std::move(cmds), std::move(states));
  }

  bool activate()
  {
    return ctrl_->get_node()->activate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  void setQ(size_t j, double v) {(void)state_[j]->set_value(v);}

  void pose(const Eigen::Isometry3d & t, double x_override = 0.0, bool nan = false)
  {
    geometry_msgs::msg::PoseStamped m;
    m.pose.position.x = nan ? std::nan("") : t.translation().x() + x_override;
    m.pose.position.y = t.translation().y();
    m.pose.position.z = t.translation().z();
    const Eigen::Quaterniond q(t.rotation());
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();
    pose_pub_->publish(m);
  }
  void twist(double vx, double vy, double vz)
  {
    geometry_msgs::msg::TwistStamped m;
    m.twist.linear.x = vx;
    m.twist.linear.y = vy;
    m.twist.linear.z = vz;
    twist_pub_->publish(m);
  }
  void toolOffset(const std::vector<double> & v)
  {
    Float64MultiArray m;
    m.data = v;
    tool_pub_->publish(m);
  }

  void settle()
  {
    for (int i = 0; i < 10; ++i) {
      exec_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // one control cycle, 10 ms apart; returns the velocity commands
  std::vector<double> step()
  {
    settle();
    t_ += 0.01;
    const rclcpp::Time now(static_cast<int64_t>(t_ * 1e9), RCL_ROS_TIME);
    EXPECT_EQ(
      ctrl_->update(now, rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
    settle();
    return command("velocity");
  }
  void skip(double seconds) {t_ += seconds;}

  std::vector<double> command(const std::string & type)
  {
    std::vector<double> v;
    for (const auto & j : kJoints) {
      v.push_back(cmd_.at(j + "/" + type)->get_optional().value_or(std::nan("")));
    }
    return v;
  }

  CartesianTrackingController & ctrl() {return *ctrl_;}
  const std::vector<double> & diag() const {return diag_;}
  const std::vector<std::string> & diagLabels() const {return diag_labels_;}
  std::vector<double> q() const {return q_;}

private:
  std::string name_;
  std::vector<double> q_;
  std::vector<std::string> cmd_types_;
  bool init_ok_ = false;
  double t_ = 1.0;
  std::unique_ptr<CartesianTrackingController> ctrl_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr tool_pub_;
  rclcpp::Subscription<Float64MultiArray>::SharedPtr diag_sub_;
  std::map<std::string, hardware_interface::CommandInterface::SharedPtr> cmd_;
  std::vector<hardware_interface::StateInterface::SharedPtr> state_;
  std::vector<double> diag_;
  std::vector<std::string> diag_labels_;
};

Eigen::Isometry3d contactPose(Kin & kin, const std::vector<double> & tool)
{
  return kin.flange(kQ) * toolOffset(tool).transform();
}

Eigen::Matrix<double, 6, 1> contactTwist(Kin & kin, const std::vector<double> & qd)
{
  const auto jf = kin.jacobian(kQ);
  const auto jc = diff_ik_core::shiftJacobianToTool(jf, kin.flange(kQ).rotation(), toolOffset(kTool));
  return jc * Eigen::Map<const Eigen::VectorXd>(qd.data(), 7);
}

double norm(const std::vector<double> & v)
{
  double s = 0.0;
  for (double x : v) {
    s += x * x;
  }
  return std::sqrt(s);
}
}  // namespace

TEST(CartesianTrackingController, ConfiguresAndActivates)
{
  Rig r("ctc_a");
  ASSERT_TRUE(r.configure());
  const auto cmd = r.ctrl().command_interface_configuration();
  ASSERT_EQ(cmd.names.size(), 7u);
  EXPECT_EQ(cmd.names.front(), "lift_prismatic_joint/velocity");
  EXPECT_EQ(r.ctrl().state_interface_configuration().names.size(), 7u);
  EXPECT_TRUE(r.ctrl().is_chainable());
  r.assign();
  ASSERT_TRUE(r.activate());
  // nothing received yet: holds and says so
  EXPECT_EQ(norm(r.step()), 0.0);
  ASSERT_EQ(r.diag().size(), 6u);
  EXPECT_EQ(r.diag()[0], 1.0);
}

TEST(CartesianTrackingController, RejectsWrongJointOrder)
{
  std::vector<std::string> swapped = kJoints;
  std::swap(swapped[1], swapped[2]);
  Rig r("ctc_bad", kTool, false, {rclcpp::Parameter("joints", swapped)});
  EXPECT_FALSE(r.configure());
}

TEST(CartesianTrackingController, ConstantTwistMovesJointsAndFillsDiagnostics)
{
  Kin kin;
  Rig r("ctc_b");
  ASSERT_TRUE(r.configure());
  r.assign();
  ASSERT_TRUE(r.activate());
  r.pose(contactPose(kin, kTool));
  r.twist(0.03, 0.0, 0.0);
  const auto qd = r.step();
  EXPECT_GT(norm(qd), 1e-4);

  // the joint velocities make the contact point move along +x and nothing else
  const auto v = contactTwist(kin, qd);
  EXPECT_NEAR(v[0], 0.03, 1e-4);
  EXPECT_NEAR(v[1], 0.0, 1e-4);
  EXPECT_NEAR(v[2], 0.0, 1e-4);
  EXPECT_NEAR(v.tail<3>().norm(), 0.0, 1e-4);

  const auto & d = r.diag();
  ASSERT_EQ(d.size(), 6u);
  EXPECT_EQ(d[0], 0.0);                 // status Ok
  EXPECT_NEAR(d[1], 0.0, 1e-9);         // reference is the current pose
  EXPECT_NEAR(d[2], 0.0, 1e-9);
  EXPECT_GT(d[3], 0.0);                 // sigma_min
  EXPECT_NEAR(d[4], 1.0, 1e-9);         // twist not limited
  EXPECT_GT(d[5], 0.0);                 // joint limit margin
  ASSERT_EQ(r.diagLabels().size(), 6u);
  EXPECT_EQ(r.diagLabels()[3], "sigma_min");

  // the opposite twist gives the opposite commands
  r.twist(-0.03, 0.0, 0.0);
  const auto qd2 = r.step();
  for (size_t i = 0; i < qd.size(); ++i) {
    EXPECT_NEAR(qd2[i], -qd[i], 1e-6);
  }
}

TEST(CartesianTrackingController, WrongToolOffsetGivesOtherCommandsAndHotSwapFixesIt)
{
  Kin kin;
  const std::vector<double> none = {0, 0, 0, 0, 0, 0};
  Rig good("ctc_good", kTool);
  Rig wrong("ctc_wrong", none);
  for (auto * r : {&good, &wrong}) {
    ASSERT_TRUE(r->configure());
    r->assign();
    ASSERT_TRUE(r->activate());
    // wanted contact pose is 2 cm off in x, plus a feedforward twist
    r->pose(contactPose(kin, kTool), 0.02);
    r->twist(0.0, 0.02, 0.0);
  }
  const auto qd_good = good.step();
  const auto qd_wrong = wrong.step();
  ASSERT_GT(norm(qd_good), 1e-4);
  double diff = 0.0;
  for (size_t i = 0; i < qd_good.size(); ++i) {
    diff += std::abs(qd_good[i] - qd_wrong[i]);
  }
  EXPECT_GT(diff, 1e-3);
  // with the offset right the error is the 2 cm we put in; with it wrong it is not
  EXPECT_NEAR(good.diag()[1], 0.02, 1e-9);
  EXPECT_GT(std::abs(wrong.diag()[1] - 0.02), 1e-3);

  // the right offset arrives on the topic, the next cycle uses it
  wrong.toolOffset(kTool);
  wrong.settle();
  const auto qd_swapped = wrong.step();
  for (size_t i = 0; i < qd_good.size(); ++i) {
    EXPECT_NEAR(qd_swapped[i], qd_good[i], 1e-9);
  }
  EXPECT_NEAR(wrong.diag()[1], 0.02, 1e-9);
}

TEST(CartesianTrackingController, BadReferenceHolds)
{
  Kin kin;
  Rig r("ctc_d", kTool, true);   // velocity and position commands
  ASSERT_TRUE(r.configure());
  r.assign();
  ASSERT_TRUE(r.activate());

  r.pose(contactPose(kin, kTool), 0.02);
  r.twist(0.0, 0.02, 0.0);
  const auto moving = r.step();
  ASSERT_GT(norm(moving), 1e-4);
  EXPECT_EQ(r.diag()[0], 0.0);
  // position command is the integrated next q, not the measured one
  const auto pos = r.command("position");
  EXPECT_GT(std::abs(pos[1] - kQ[1]) + std::abs(pos[2] - kQ[2]) + std::abs(pos[3] - kQ[3]), 0.0);

  // NaN position: zero velocity, position held at the measured q, status 3
  r.pose(contactPose(kin, kTool), 0.0, true);
  const auto held = r.step();
  EXPECT_EQ(norm(held), 0.0);
  EXPECT_EQ(r.diag()[0], 3.0);
  EXPECT_TRUE(std::isnan(r.diag()[1]));
  const auto hold_pos = r.command("position");
  for (size_t j = 0; j < kQ.size(); ++j) {
    EXPECT_DOUBLE_EQ(hold_pos[j], kQ[j]);
  }

  // a good reference resumes
  r.pose(contactPose(kin, kTool), 0.02);
  EXPECT_GT(norm(r.step()), 1e-4);
  EXPECT_EQ(r.diag()[0], 0.0);

  // NaN feedforward twist holds too
  r.twist(std::nan(""), 0.0, 0.0);
  EXPECT_EQ(norm(r.step()), 0.0);
  EXPECT_EQ(r.diag()[0], 3.0);
}

TEST(CartesianTrackingController, StaleReferenceHolds)
{
  Kin kin;
  Rig r("ctc_e");
  ASSERT_TRUE(r.configure());
  r.assign();
  ASSERT_TRUE(r.activate());
  r.pose(contactPose(kin, kTool), 0.02);
  EXPECT_GT(norm(r.step()), 1e-4);
  r.skip(0.6);   // timeout is 0.5 s
  EXPECT_EQ(norm(r.step()), 0.0);
  EXPECT_EQ(r.diag()[0], 2.0);
}

TEST(CartesianTrackingController, ChainedModeTakesReferenceFromInterfaces)
{
  Kin kin;
  Rig topic("ctc_t");
  Rig chained("ctc_c");
  for (auto * r : {&topic, &chained}) {
    ASSERT_TRUE(r->configure());
  }
  const auto refs = chained.ctrl().export_reference_interfaces();
  ASSERT_EQ(refs.size(), 13u);
  ASSERT_TRUE(chained.ctrl().set_chained_mode(true));
  for (auto * r : {&topic, &chained}) {
    r->assign();
    ASSERT_TRUE(r->activate());
  }

  const auto pose = contactPose(kin, kTool);
  topic.pose(pose, 0.02);
  topic.twist(0.0, 0.02, 0.0);

  const Eigen::Quaterniond q(pose.rotation());
  const double vals[13] = {
    pose.translation().x() + 0.02, pose.translation().y(), pose.translation().z(),
    q.x(), q.y(), q.z(), q.w(), 0.0, 0.02, 0.0, 0.0, 0.0, 0.0};
  for (size_t i = 0; i < 13; ++i) {
    (void)refs[i]->set_value(vals[i]);
  }
  const auto a = topic.step();
  const auto b = chained.step();
  ASSERT_GT(norm(a), 1e-4);
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_NEAR(a[i], b[i], 1e-9);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}
