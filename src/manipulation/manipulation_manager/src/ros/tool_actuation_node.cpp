/**
 * @file tool_actuation_node.cpp
 * @brief Thin ROS wrapper around core/tool_actuation.hpp (opt-in; the manager does not start it).
 * @details Parameters: tool_command, tool_off_lead_s, tool_on_at_speed_ratio, response_timeout_s, retry_s,
 *          pause_keeps_state, rate_hz, valve_topic (/tool/valve), echo_topic (/tool/valve_state, empty =
 *          no echo).
 *          Topics in: `~/window` Float64MultiArray [on_s, end_s] (trajectory time; starts a window),
 *          `~/trajectory_time` Float64, `~/paused` Bool, `~/speed_ratio` Float64, `~/cancel` Bool (true),
 *          `~/fault` Bool (true), echo Bool. Out: valve Bool (reliable, transient local) and
 *          `~/status` String (`<RESULT> on_latency_s off_latency_s`).
 *          The valve is forced off when the node shuts down.
 */
#include <chrono>
#include <memory>
#include <string>

#include "manipulation_manager/core/tool_actuation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace manipulation_manager;  // NOLINT

class ToolActuationNode : public rclcpp::Node
{
public:
  ToolActuationNode()
  : Node("tool_actuation")
  {
    ToolActuationConfig c;
    c.tool_command = declare_parameter<std::string>("tool_command", "valve");
    c.off_lead_s = declare_parameter<double>("tool_off_lead_s", 0.0);
    c.on_at_speed_ratio = declare_parameter<double>("tool_on_at_speed_ratio", 0.0);
    c.response_timeout_s = declare_parameter<double>("response_timeout_s", 1.0);
    c.retry_s = declare_parameter<double>("retry_s", 0.25);
    c.pause_keeps_state = declare_parameter<bool>("pause_keeps_state", false);
    exec_ = std::make_unique<ToolActuation>(c);

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    pub_ = create_publisher<std_msgs::msg::Bool>(
      declare_parameter<std::string>("valve_topic", "/tool/valve"), qos);
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);
    const auto echo = declare_parameter<std::string>("echo_topic", "/tool/valve_state");
    if (!echo.empty()) {
      echo_sub_ = create_subscription<std_msgs::msg::Bool>(
        echo, qos, [this](std_msgs::msg::Bool::SharedPtr m) {echo_valid_ = true; echo_on_ = m->data;});
    }
    window_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/window", 10, [this](std_msgs::msg::Float64MultiArray::SharedPtr m) {
        if (m->data.size() >= 2) {
          exec_->start(m->data[0], m->data[1]);
          active_ = true;
        }
      });
    time_sub_ = create_subscription<std_msgs::msg::Float64>(
      "~/trajectory_time", 10, [this](std_msgs::msg::Float64::SharedPtr m) {traj_s_ = m->data;});
    speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      "~/speed_ratio", 10, [this](std_msgs::msg::Float64::SharedPtr m) {speed_ = m->data;});
    pause_sub_ = create_subscription<std_msgs::msg::Bool>(
      "~/paused", 10, [this](std_msgs::msg::Bool::SharedPtr m) {paused_ = m->data;});
    cancel_sub_ = create_subscription<std_msgs::msg::Bool>(
      "~/cancel", 10, [this](std_msgs::msg::Bool::SharedPtr m) {if (m->data) {exec_->cancel();}});
    fault_sub_ = create_subscription<std_msgs::msg::Bool>(
      "~/fault", 10, [this](std_msgs::msg::Bool::SharedPtr m) {if (m->data) {exec_->fault();}});

    const double hz = declare_parameter<double>("rate_hz", 50.0);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / hz), [this]() {step();});
    publish(false);  // closed at start, explicitly
  }

  ~ToolActuationNode() override
  {
    try {
      publish(false);  // forced off on every exit path
    } catch (...) {
    }
  }

private:
  void publish(bool on)
  {
    std_msgs::msg::Bool m;
    m.data = on;
    pub_->publish(m);
  }

  void step()
  {
    if (!active_) {
      return;
    }
    const double now = get_clock()->now().seconds();
    const auto o = exec_->tick(traj_s_, now, paused_, speed_, echo_valid_, echo_on_);
    if (o.send) {
      publish(o.command);
    }
    if (o.result != last_result_) {
      last_result_ = o.result;
      std_msgs::msg::String s;
      s.data = std::string(toString(o.result)) + " " + std::to_string(exec_->onLatencyS()) + " " +
        std::to_string(exec_->offLatencyS());
      status_pub_->publish(s);
      if (o.result != ToolResult::RUNNING) {
        RCLCPP_INFO(get_logger(), "tool window finished: %s", s.data.c_str());
        active_ = false;
      }
    }
  }

  std::unique_ptr<ToolActuation> exec_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr echo_sub_, pause_sub_, cancel_sub_, fault_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr window_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr time_sub_, speed_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double traj_s_{0.0}, speed_{1.0};
  bool paused_{false}, echo_valid_{false}, echo_on_{false}, active_{false};
  ToolResult last_result_{ToolResult::RUNNING};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ToolActuationNode>());
  rclcpp::shutdown();
  return 0;
}
