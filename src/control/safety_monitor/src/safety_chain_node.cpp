/**
 * @file safety_chain_node.cpp
 * @brief Opt-in ROS 2 wrapper of safety_monitor::SafetyChain. NOT A CERTIFIED SAFETY FUNCTION.
 * @details Subscribes to the conditioned wrench, joint states, an operator e-stop and a controller
 *          fault topic, steps the chain at `rate_hz`, publishes the state (transient_local) and, in
 *          HOLD or ESTOP, a Bool stop request and a zero Twist for the base. `~/reset` is the only way
 *          out of a latched state. All decisions are in the ROS-free SafetyChain and
 *          safety_chain_node_core; this file only moves messages. Not compiled or run when written.
 *          See docs/safety_chain.md.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#include <array>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ros_adapters/tool_profile_yaml_loader.hpp"
#include "safety_monitor/safety_chain.hpp"
#include "safety_monitor/safety_chain_node_core.hpp"

namespace safety_monitor
{

class SafetyChainNode : public rclcpp::Node
{
public:
  SafetyChainNode()
  : rclcpp::Node("safety_chain_node")
  {
    ChainParams p;
    loadThresholds(p);
    p.ft_stale_s = declare_parameter<double>("ft_stale_s", p.ft_stale_s);
    p.joint_stale_s = declare_parameter<double>("joint_stale_s", p.joint_stale_s);
    p.require_ft = declare_parameter<bool>("require_ft", p.require_ft);
    p.require_joint_state = declare_parameter<bool>("require_joint_state", p.require_joint_state);
    p.hold_auto_clear = declare_parameter<bool>("hold_auto_clear", p.hold_auto_clear);
    p.hold_clear_time_s = declare_parameter<double>("hold_clear_time_s", p.hold_clear_time_s);
    for (std::size_t i = 0; i < p.severity.size(); ++i) {
      const auto cause = static_cast<ChainCause>(i);
      const std::string key = std::string("severity.") + toString(cause);
      const std::string text = declare_parameter<std::string>(key, toString(p.severity[i]));
      if (!parseSeverity(text, p.severity[i])) {
        throw std::runtime_error(key + ": '" + text + "' is not none, hold or estop");
      }
    }
    chain_ = std::make_unique<SafetyChain>(p);

    const double rate_hz = declare_parameter<double>("rate_hz", 50.0);
    if (!(rate_hz > 0.0)) {throw std::runtime_error("rate_hz must be > 0");}
    const auto wrench_topic = declare_parameter<std::string>(
      "wrench_topic", "/ft_conditioner/wrench_conditioned");
    const auto joint_topic = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    const auto estop_topic = declare_parameter<std::string>("estop_topic", "/safety_chain/estop");
    const auto fault_topic = declare_parameter<std::string>(
      "controller_fault_topic", "/safety_chain/controller_fault");
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto stop_topic = declare_parameter<std::string>("arm_stop_topic", "/safety_chain/arm_stop");
    const auto state_topic = declare_parameter<std::string>("state_topic", "/safety_chain/state");

    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      wrench_topic, sensor_qos,
      [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr m) {onWrench(*m);});
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_topic, sensor_qos,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr) {chain_->updateJointState(nowS());});
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      estop_topic, rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {chain_->setOperatorEstop(m->data);});
    fault_sub_ = create_subscription<std_msgs::msg::Bool>(
      fault_topic, rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {chain_->setControllerFault(m->data);});

    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic, latched);
    stop_pub_ = create_publisher<std_msgs::msg::Bool>(stop_topic, latched);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::QoS(10));

    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {onReset(*res);});

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz), [this]() {onTimer();});
    publishAll(true);   // start OK so a late subscriber sees a state at once
    RCLCPP_WARN(get_logger(),
      "safety_chain_node is NOT a certified safety function; a hardware e-stop must not depend on it");
  }

private:
  // Per-tool thresholds: from the tool profile, each overridable; with no profile both are needed.
  void loadThresholds(ChainParams & p)
  {
    const auto file = declare_parameter<std::string>("tool_profiles_file", "");
    const auto tool = declare_parameter<std::string>("tool_name", "");
    // A negative override means "take the value from the profile".
    double force = declare_parameter<double>("force_abort_n", -1.0);
    double torque = declare_parameter<double>("torque_abort_nm", -1.0);
    if (!file.empty() || !tool.empty()) {
      const auto profile = ros_adapters::loadToolProfile(file, tool);   // throws on a bad file or name
      if (force < 0.0) {force = profile.task.overload_abort_n;}
      if (torque < 0.0) {torque = profile.task.tangential_torque_abort_nm;}
      RCLCPP_INFO(get_logger(), "tool '%s': force_abort_n %.1f, torque_abort_nm %.1f",
        tool.c_str(), force, torque);
    }
    if (force < 0.0 || torque < 0.0) {
      throw std::runtime_error(
              "set tool_profiles_file and tool_name, or both force_abort_n and torque_abort_nm "
              "(0 disables a check); refusing to start with unknown thresholds");
    }
    p.force_abort_n = force;
    p.torque_abort_nm = torque;
  }

  double nowS() {return now().seconds();}

  void onWrench(const geometry_msgs::msg::WrenchStamped & m)
  {
    double f = 0.0, t = 0.0;
    wrenchMagnitudes(
      m.wrench.force.x, m.wrench.force.y, m.wrench.force.z,
      m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z, f, t);
    // Receive time, not the header stamp: the staleness check then cannot be fooled by a stamp
    // that is old, zero or from another clock.
    chain_->updateFt(f, t, nowS());
  }

  void onTimer()
  {
    const double t = nowS();
    const ChainOutputs out = chain_->step(t);
    publishAll(false, out);
    if (out.base_stop) {cmd_vel_pub_->publish(geometry_msgs::msg::Twist());}   // all zero
  }

  void onReset(std_srvs::srv::Trigger::Response & res)
  {
    const double t = nowS();
    if (chain_->state() == ChainState::kOk) {
      res.success = false;
      res.message = "already OK, nothing to reset";
      return;
    }
    res.success = chain_->reset(t);
    if (res.success) {
      res.message = "reset accepted";
      publishAll(true);
    } else {
      const std::string why = refusalReasons(chain_->events(), t);
      res.message = "reset refused: " + (why.empty() ? std::string("cause still active") : why);
    }
    RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
  }

  // Publishes the state text and the arm stop only when they change (both are latched topics).
  void publishAll(bool force, const ChainOutputs & out = ChainOutputs{})
  {
    const ChainOutputs o = force ? chain_->outputs() : out;
    std_msgs::msg::String s;
    s.data = formatState(o.state, activeCauseNames(chain_->events(), o.state));
    if (s.data != last_state_text_) {
      last_state_text_ = s.data;
      state_pub_->publish(s);
      RCLCPP_WARN_EXPRESSION(get_logger(), o.state != ChainState::kOk, "safety chain: %s",
        s.data.c_str());
    }
    if (!stop_published_ || o.arm_stop != last_arm_stop_) {
      stop_published_ = true;
      last_arm_stop_ = o.arm_stop;
      std_msgs::msg::Bool b;
      b.data = o.arm_stop;
      stop_pub_->publish(b);
    }
  }

  std::unique_ptr<SafetyChain> chain_;
  std::string last_state_text_;
  bool last_arm_stop_{false}, stop_published_{false};
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_, fault_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace safety_monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<safety_monitor::SafetyChainNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("safety_chain_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
