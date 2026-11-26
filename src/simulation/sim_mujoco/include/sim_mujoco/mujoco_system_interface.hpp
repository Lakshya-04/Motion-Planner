/**
 * @file mujoco_system_interface.hpp
 * @brief ros2_control SystemInterface that steps a MuJoCo model (hardware:=mujoco).
 * @details Same joint interfaces as the jtac_mock hardware: position and velocity command; position,
 *          velocity and effort state. The tool force/torque is exported as the sensor named in the
 *          <ros2_control> block (`ft_sensor`, state interfaces force.x .. torque.z), which is the
 *          shape ft_bridge_hardware and the JTAC / admittance controllers read.
 *
 *          The sensor values are RAW (see mujoco_sim.hpp): the load of everything beyond the wrist
 *          site `ft_sensor` (tool, weight, inertia) in the sensor frame, nothing compensated. The same
 *          wrench is published as geometry_msgs/WrenchStamped on `sim_mujoco/ft_raw` (frame
 *          `ft_frame_id`), and the ground-truth contact wrench on the tool, at the TCP in the tcp frame,
 *          on `sim_mujoco/ft_ground_truth` (frame `tcp_frame_id`) for evaluation only.
 *
 *          Hardware parameters (xacro: <param name="..."> inside <hardware>):
 *            mjcf          path of the MJCF (default: environment variable SIM_MUJOCO_MJCF)
 *            ft_site       MJCF site carrying the force/torque sensors (default "ft_sensor")
 *            tcp_site      MJCF site of the TCP for the ground truth (default "tcp_site")
 *            ft_frame_id   frame_id of the raw topic (default "tool_mount_link")
 *            tcp_frame_id  frame_id of the ground-truth topic (default "tcp_link")
 *            depth_frame_id frame_id of the point cloud (default "depth_sensor_link")
 *            lidar_frame_id frame_id of the laser scan (default "lidar_link")
 *            ft_tare       "true": subtract the reading at activation (default "false")
 *
 *          Planar base: when the MJCF has the base joints (launch base_motion:=true), a
 *          geometry_msgs/Twist on `cmd_vel` (body frame) moves the base, stopping after
 *          `cmd_vel_timeout` seconds without a message (hardware parameter, default 0.5); the plugin
 *          publishes the base pose in the odom frame (geometry_msgs/PoseStamped, frame
 *          `odom_frame_id`, default "odom") on `sim_mujoco/base_pose` and the body-frame velocity
 *          (TwistStamped) on `sim_mujoco/base_twist`; the `base_odom.py` node turns them into
 *          nav_msgs/Odometry on `/odom` and TF odom -> base_footprint (so the plugin needs only
 *          geometry_msgs). Without the joints none of this exists.
 *
 *          Depth sensor: when the MJCF has one (launch depth_sensor:=true) the plugin publishes the
 *          ray-cast scan as sensor_msgs/PointCloud2 (xyz float32, dense, sensor frame x forward) on
 *          `depth_sensor/points`; the TF to the sensor frame is the launch file's static transform.
 *
 *          Lidar: when the MJCF has one (launch lidar:=true) the plugin publishes the 2D ray-cast scan as
 *          sensor_msgs/LaserScan on `scan` (frame `lidar_frame_id`); the TF is the launch file's static transform.
 *
 *          Non-idealities (default off, nothing changes unless set): environment variables
 *          SIM_MUJOCO_FT_{NOISE_DENSITY,BIAS_WALK,LOWPASS_HZ,DELAY_MS,QUANTUM} (the raw F/T, force and
 *          torque channels alike), SIM_MUJOCO_JOINT_{POS_NOISE,VEL_NOISE} (std, rad and rad/s),
 *          SIM_MUJOCO_CONTROL_DELAY_MS (command latency) and SIM_MUJOCO_NOISE_SEED; the model is in
 *          signal_model.hpp and docs/sim_vs_real.md. The ground-truth topic is never touched.
 *
 *          Threading: read() and write() run on the controller manager thread; the model is stepped
 *          in write() by the controller period, so simulated time follows the update rate.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "sim_mujoco/mujoco_sim.hpp"
#include "sim_mujoco/signal_model.hpp"

namespace sim_mujoco
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MujocoSystemInterface : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::unique_ptr<MujocoSim> sim_;
  std::string sensor_name_;
  bool ft_tare_{false};
  std::string ft_frame_id_;
  std::string tcp_frame_id_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr raw_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr truth_pub_;

  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr base_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr base_twist_pub_;
  std::string odom_frame_id_;
  double cmd_vel_timeout_{0.5};
  rclcpp::Time last_cmd_time_;
  std::array<double, 3> cmd_vel_{};
  void publishBase(const rclcpp::Time & stamp);

  // Depth sensor (launch depth_sensor:=true): sensor_msgs/PointCloud2 `depth_sensor/points` in
  // `depth_frame_id` (hardware parameter, default "depth_sensor_link"), at the MJCF's rate.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_pub_;
  std::string depth_frame_id_;
  double next_depth_time_{0.0};
  void publishDepth(const rclcpp::Time & stamp);

  // Lidar (launch lidar:=true): sensor_msgs/LaserScan `scan` in `lidar_frame_id` (hardware parameter,
  // default "lidar_link"), at the MJCF's rate.
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_;
  std::string lidar_frame_id_;
  double next_lidar_time_{0.0};
  void publishLidar(const rclcpp::Time & stamp);

  // Opt-in non-idealities (signal_model.hpp), built on the first read() when the period is known.
  struct Noise
  {
    SignalConfig ft, pos, vel, cmd;
    bool any{false};
    std::vector<SignalModel> ft_m, pos_m, vel_m, cmd_pos_m, cmd_vel_m;
  } noise_;
  bool noise_built_{false};
  void buildNoise(double dt);

  std::vector<double> initial_position_;
  std::vector<double> state_position_;
  std::vector<double> state_velocity_;
  std::vector<double> state_effort_;
  std::vector<double> command_position_;
  std::vector<double> command_velocity_;
  std::array<double, 6> ft_{};
};

}  // namespace sim_mujoco
