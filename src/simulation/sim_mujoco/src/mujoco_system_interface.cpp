#include "sim_mujoco/mujoco_system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "gravity_core/urdf_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace sim_mujoco
{

namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("MujocoSystemInterface");
constexpr double kMaxAdvanceS = 0.05;

std::string param(const hardware_interface::HardwareInfo & info, const std::string & key,
  const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(key);
  return (it == info.hardware_parameters.end() || it->second.empty()) ? fallback : it->second;
}

double envd(const char * name, double fallback = 0.0)
{
  const char * v = std::getenv(name);
  return (v && *v) ? std::atof(v) : fallback;
}
}  // namespace

void MujocoSystemInterface::buildNoise(double dt)
{
  noise_.ft = {envd("SIM_MUJOCO_FT_NOISE_DENSITY"), envd("SIM_MUJOCO_FT_BIAS_WALK"),
    envd("SIM_MUJOCO_FT_LOWPASS_HZ"), envd("SIM_MUJOCO_FT_DELAY_MS") * 1e-3,
    envd("SIM_MUJOCO_FT_QUANTUM"), static_cast<uint64_t>(envd("SIM_MUJOCO_NOISE_SEED", 1.0))};
  noise_.pos.noise_density = envd("SIM_MUJOCO_JOINT_POS_NOISE") * std::sqrt(2.0 * dt);  // std per sample
  noise_.vel.noise_density = envd("SIM_MUJOCO_JOINT_VEL_NOISE") * std::sqrt(2.0 * dt);
  noise_.cmd.delay_s = envd("SIM_MUJOCO_CONTROL_DELAY_MS") * 1e-3;
  noise_.pos.seed = noise_.ft.seed + 1;
  noise_.vel.seed = noise_.ft.seed + 2;
  const size_t n = state_position_.size();
  for (size_t k = 0; k < 6; ++k) {
    auto c = noise_.ft;
    c.seed += 100 + k;
    noise_.ft_m.emplace_back(c, dt);
  }
  for (size_t i = 0; i < n; ++i) {
    auto p = noise_.pos, v = noise_.vel;
    p.seed += 1000 + i;
    v.seed += 2000 + i;
    noise_.pos_m.emplace_back(p, dt);
    noise_.vel_m.emplace_back(v, dt);
    noise_.cmd_pos_m.emplace_back(noise_.cmd, dt);
    noise_.cmd_vel_m.emplace_back(noise_.cmd, dt);
  }
  noise_.any = noise_.ft.enabled() || noise_.pos.enabled() || noise_.vel.enabled() || noise_.cmd.enabled();
  if (noise_.any) {
    RCLCPP_WARN(kLogger,
      "Sensor non-idealities ON: ft density %.3g bias walk %.3g lowpass %.3g Hz delay %.1f ms quantum %.3g; "
      "joint noise pos %.3g vel %.3g; control delay %.1f ms (period %.4f s)",
      noise_.ft.noise_density, noise_.ft.bias_walk, noise_.ft.lowpass_hz, noise_.ft.delay_s * 1e3,
      noise_.ft.quantum, envd("SIM_MUJOCO_JOINT_POS_NOISE"), envd("SIM_MUJOCO_JOINT_VEL_NOISE"),
      noise_.cmd.delay_s * 1e3, dt);
  }
  noise_built_ = true;
}

CallbackReturn MujocoSystemInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const char * env = std::getenv("SIM_MUJOCO_MJCF");
  const std::string mjcf = param(info_, "mjcf", env ? env : "");
  if (mjcf.empty()) {
    RCLCPP_FATAL(kLogger, "No MJCF: set the 'mjcf' hardware parameter or SIM_MUJOCO_MJCF.");
    return CallbackReturn::ERROR;
  }
  ft_tare_ = param(info_, "ft_tare", "false") == "true";
  ft_frame_id_ = param(info_, "ft_frame_id", "tool_mount_link");
  tcp_frame_id_ = param(info_, "tcp_frame_id", "tcp_link");

  if (info_.sensors.size() > 1) {
    RCLCPP_FATAL(kLogger, "At most one <sensor> (the tool F/T) is supported.");
    return CallbackReturn::ERROR;
  }
  sensor_name_ = info_.sensors.empty() ? "" : info_.sensors[0].name;

  std::vector<std::string> joints;
  for (const auto & joint : info_.joints) {
    joints.push_back(joint.name);
    double initial = 0.0;
    for (const auto & state : joint.state_interfaces) {
      if (state.name == "position" && !state.initial_value.empty()) {
        try {
          initial = std::stod(state.initial_value);
        } catch (const std::exception &) {
          RCLCPP_FATAL(kLogger, "Joint %s: initial_value '%s' is not a number", joint.name.c_str(),
            state.initial_value.c_str());
          return CallbackReturn::ERROR;
        }
      }
    }
    initial_position_.push_back(initial);
  }
  try {
    sim_ = std::make_unique<MujocoSim>(mjcf, joints, param(info_, "ft_site", "ft_sensor"),
      param(info_, "tcp_site", "tcp_site"));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(kLogger, "%s", e.what());
    return CallbackReturn::ERROR;
  }
  if (param(info_, "gravity_feedforward", "false") == "true") {
    try {
      gravity_ = std::make_unique<gravity_core::GravityModel>(
        gravity_core::loadUrdf(info_.original_xml, joints));
      const std::string link = param(info_, "gravity_payload_link", "");
      if (!link.empty()) {
        std::istringstream cog(param(info_, "gravity_payload_cog", "0 0 0"));
        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        cog >> c[0] >> c[1] >> c[2];
        gravity_->setPayload(link, std::stod(param(info_, "gravity_payload_mass", "0")), c);
      }
    } catch (const std::exception & e) {
      RCLCPP_FATAL(kLogger, "gravity_feedforward: %s", e.what());
      return CallbackReturn::ERROR;
    }
    gravity_q_.setZero(joints.size());
    gravity_tau_.setZero(joints.size());
    if (sim_->hasGravcomp()) {
      RCLCPP_WARN(kLogger, "gravity_feedforward is on but the MJCF still has gravcomp bodies: "
        "gravity is compensated twice. Generate with gravcomp=False (launch gravity_comp:=core).");
    }
    RCLCPP_INFO(kLogger, "Gravity feed-forward from gravity_core on %zu joints", joints.size());
  }
  const size_t n = joints.size();
  state_position_.assign(n, 0.0);
  state_velocity_.assign(n, 0.0);
  state_effort_.assign(n, 0.0);
  command_position_.assign(n, std::numeric_limits<double>::quiet_NaN());
  command_velocity_.assign(n, std::numeric_limits<double>::quiet_NaN());
  node_ = rclcpp::Node::make_shared("mujoco_system_" + info_.name);
  raw_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "sim_mujoco/ft_raw", rclcpp::SensorDataQoS());
  truth_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "sim_mujoco/ft_ground_truth", rclcpp::SensorDataQoS());
  if (sim_->hasDepth()) {
    depth_frame_id_ = param(info_, "depth_frame_id", "depth_sensor_link");
    depth_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "depth_sensor/points", rclcpp::SensorDataQoS());
    RCLCPP_INFO(kLogger, "Depth sensor %dx%d, %.0f x %.0f deg, %.1f Hz -> depth_sensor/points (frame %s)",
      sim_->depthConfig().width, sim_->depthConfig().height, sim_->depthConfig().hfov * 57.29578,
      sim_->depthConfig().vfov * 57.29578, sim_->depthConfig().rate_hz, depth_frame_id_.c_str());
  }
  if (sim_->hasLidar()) {
    lidar_frame_id_ = param(info_, "lidar_frame_id", "lidar_link");
    lidar_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
    const auto & lc = sim_->lidarConfig();
    RCLCPP_INFO(kLogger, "Lidar %zu rays, %.0f to %.0f deg, %.2f to %.1f m, %.1f Hz -> scan (frame %s)",
      sim_->lidarRayCount(), lc.angle_min * 57.29578, lc.angle_max * 57.29578, lc.range_min, lc.range_max,
      lc.rate_hz, lidar_frame_id_.c_str());
  }
  if (sim_->hasBase()) {
    odom_frame_id_ = param(info_, "odom_frame_id", "odom");
    cmd_vel_timeout_ = std::stod(param(info_, "cmd_vel_timeout", "0.5"));
    last_cmd_time_ = node_->now();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SystemDefaultsQoS(), [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        cmd_vel_ = {m->linear.x, m->linear.y, m->angular.z};
        last_cmd_time_ = node_->now();
      });
    base_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "sim_mujoco/base_pose", rclcpp::SensorDataQoS());
    base_twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "sim_mujoco/base_twist", rclcpp::SensorDataQoS());
    RCLCPP_INFO(kLogger, "Planar base: cmd_vel -> sim_mujoco/base_pose (frame %s)",
      odom_frame_id_.c_str());
  }
  RCLCPP_INFO(
    kLogger, "MuJoCo %s: '%s', %zu joints, timestep %.4f s, F/T sensor '%s' (tare %s)",
    mj_versionString(), mjcf.c_str(), n, sim_->timestep(), sensor_name_.c_str(),
    ft_tare_ ? "on" : "off");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & name = info_.joints[i].name;
    out.emplace_back(name, hardware_interface::HW_IF_POSITION, &state_position_[i]);
    out.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &state_velocity_[i]);
    out.emplace_back(name, hardware_interface::HW_IF_EFFORT, &state_effort_[i]);
  }
  if (!sensor_name_.empty()) {
    static const char * const names[6] = {"force.x", "force.y", "force.z",
      "torque.x", "torque.y", "torque.z"};
    for (size_t k = 0; k < 6; ++k) {
      out.emplace_back(sensor_name_, names[k], &ft_[k]);
    }
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> MujocoSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & name = info_.joints[i].name;
    out.emplace_back(name, hardware_interface::HW_IF_POSITION, &command_position_[i]);
    out.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &command_velocity_[i]);
  }
  return out;
}

CallbackReturn MujocoSystemInterface::on_activate(const rclcpp_lifecycle::State &)
{
  sim_->setJointPositions(initial_position_);
  sim_->advance(0.0);
  for (size_t i = 0; i < initial_position_.size(); ++i) {
    state_position_[i] = sim_->position(i);
    state_velocity_[i] = 0.0;
    state_effort_[i] = 0.0;
    // hold the start pose until a controller commands something else
    command_position_[i] = sim_->position(i);
    command_velocity_[i] = 0.0;
  }
  if (ft_tare_) {
    sim_->tareWrench();
  } else {
    sim_->clearTare();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MujocoSystemInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (sim_) {
    RCLCPP_INFO(kLogger, "UNEXPECTED_CONTACT_SUMMARY %s", sim_->unexpectedContactSummary().c_str());
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MujocoSystemInterface::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!noise_built_ && period.seconds() > 0.0) {
    buildNoise(period.seconds());
  }
  for (size_t i = 0; i < state_position_.size(); ++i) {
    state_position_[i] = sim_->position(i);
    state_velocity_[i] = sim_->velocity(i);
    state_effort_[i] = sim_->effort(i);
    if (noise_.any) {
      state_position_[i] = noise_.pos_m[i].step(state_position_[i]);
      state_velocity_[i] = noise_.vel_m[i].step(state_velocity_[i]);
    }
  }
  ft_ = sim_->wrench();
  if (noise_.any) {
    for (size_t k = 0; k < 6; ++k) {
      ft_[k] = noise_.ft_m[k].step(ft_[k]);
    }
  }
  const auto stamp = node_->now();
  auto fill = [&stamp](const std::string & frame, const Wrench & w) {
      geometry_msgs::msg::WrenchStamped m;
      m.header.stamp = stamp;
      m.header.frame_id = frame;
      m.wrench.force.x = w[0];
      m.wrench.force.y = w[1];
      m.wrench.force.z = w[2];
      m.wrench.torque.x = w[3];
      m.wrench.torque.y = w[4];
      m.wrench.torque.z = w[5];
      return m;
    };
  if (sim_->hasBase()) {
    executor_->spin_some(std::chrono::nanoseconds(0));
    publishBase(stamp);
  }
  if (sim_->hasDepth() && sim_->time() >= next_depth_time_) {
    next_depth_time_ = sim_->time() + 1.0 / sim_->depthConfig().rate_hz;
    publishDepth(stamp);
  }
  if (sim_->hasLidar() && sim_->time() >= next_lidar_time_) {
    next_lidar_time_ = sim_->time() + 1.0 / sim_->lidarConfig().rate_hz;
    publishLidar(stamp);
  }
  raw_pub_->publish(fill(ft_frame_id_, ft_));
  const Wrench truth = sim_->groundTruthWrench();
  truth_pub_->publish(fill(tcp_frame_id_, truth));
  {
    std::vector<std::string> fresh;
    sim_->scanUnexpectedContacts(fresh);
    for (const auto & s : fresh) {
      RCLCPP_WARN(kLogger, "UNEXPECTED_CONTACT t=%.3f %s", sim_->time(), s.c_str());
    }
  }
  static const bool diag_contacts = std::getenv("SIM_DIAG_CONTACTS") != nullptr;
  if (diag_contacts && std::abs(truth[2]) > 15.0) {
    RCLCPP_WARN(kLogger, "DIAG t=%.3f truth=[%.1f %.1f %.1f] raw=[%.1f %.1f %.1f] contacts: %s", sim_->time(),
      truth[0], truth[1], truth[2], ft_[0], ft_[1], ft_[2], sim_->describeToolContacts().c_str());
  }
  return hardware_interface::return_type::OK;
}

void MujocoSystemInterface::publishDepth(const rclcpp::Time & stamp)
{
  const auto points = sim_->depthScan();
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = depth_frame_id_;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 12;
  cloud.row_step = cloud.point_step * cloud.width;
  const char * const names[3] = {"x", "y", "z"};
  for (uint32_t k = 0; k < 3; ++k) {
    sensor_msgs::msg::PointField f;
    f.name = names[k];
    f.offset = 4 * k;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    cloud.fields.push_back(f);
  }
  cloud.data.resize(points.size() * 12);
  if (!points.empty()) {
    std::memcpy(cloud.data.data(), points.data(), points.size() * 12);
  }
  depth_pub_->publish(cloud);
}

void MujocoSystemInterface::publishLidar(const rclcpp::Time & stamp)
{
  const auto & lc = sim_->lidarConfig();
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = stamp;
  scan.header.frame_id = lidar_frame_id_;
  scan.ranges = sim_->lidarScan();
  scan.angle_min = static_cast<float>(lc.angle_min);
  scan.angle_increment = static_cast<float>(lc.angle_increment);
  scan.angle_max = static_cast<float>(lc.angle_min + (scan.ranges.size() - 1) * lc.angle_increment);
  scan.scan_time = static_cast<float>(1.0 / lc.rate_hz);
  scan.time_increment = 0.0f;  // all rays are cast at one instant
  scan.range_min = static_cast<float>(lc.range_min);
  scan.range_max = static_cast<float>(lc.range_max);
  lidar_pub_->publish(scan);
}

void MujocoSystemInterface::publishBase(const rclcpp::Time & stamp)
{
  const auto p = sim_->basePose();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = odom_frame_id_;
  pose.pose.position.x = p.x;
  pose.pose.position.y = p.y;
  pose.pose.orientation.z = std::sin(p.yaw / 2.0);
  pose.pose.orientation.w = std::cos(p.yaw / 2.0);
  base_pose_pub_->publish(pose);
  geometry_msgs::msg::TwistStamped twist;
  twist.header = pose.header;
  twist.twist.linear.x = p.vx;
  twist.twist.linear.y = p.vy;
  twist.twist.angular.z = p.wz;
  base_twist_pub_->publish(twist);
}

hardware_interface::return_type MujocoSystemInterface::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  for (size_t i = 0; i < command_position_.size(); ++i) {
    if (noise_.any) {
      sim_->setCommand(i, noise_.cmd_pos_m[i].step(command_position_[i]),
        noise_.cmd_vel_m[i].step(command_velocity_[i]));
    } else {
      sim_->setCommand(i, command_position_[i], command_velocity_[i]);
    }
  }
  if (gravity_) {
    // Measured positions, like a real arm's feed-forward; skipped until the first read() filled them.
    bool finite = true;
    for (size_t i = 0; i < state_position_.size(); ++i) {
      gravity_q_[i] = state_position_[i];
      finite = finite && std::isfinite(gravity_q_[i]);
    }
    if (finite) {
      gravity_->compute(gravity_q_, gravity_tau_);
      for (size_t i = 0; i < state_position_.size(); ++i) {
        sim_->setFeedforward(i, gravity_tau_[i]);
      }
    }
  }
  if (sim_->hasBase()) {
    const bool fresh = (node_->now() - last_cmd_time_).seconds() <= cmd_vel_timeout_;
    sim_->setBaseVelocity(fresh ? cmd_vel_[0] : 0.0, fresh ? cmd_vel_[1] : 0.0,
      fresh ? cmd_vel_[2] : 0.0);
  }
  // A long stall must not become hundreds of mj_step in one write(): clamp (5 periods at 100 Hz).
  const double dt = std::min(period.seconds(), kMaxAdvanceS);
  if (period.seconds() > dt) {
    RCLCPP_WARN_THROTTLE(kLogger, *node_->get_clock(), 5000, "write() period %.3f s clamped to %.3f s",
      period.seconds(), dt);
  }
  sim_->advance(dt);
  return hardware_interface::return_type::OK;
}

}  // namespace sim_mujoco

PLUGINLIB_EXPORT_CLASS(sim_mujoco::MujocoSystemInterface, hardware_interface::SystemInterface)
