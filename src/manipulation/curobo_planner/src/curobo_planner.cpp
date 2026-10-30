#include "curobo_planner/curobo_planner.hpp"

#include <chrono>
#include <cstring>
#include <future>

#include <Eigen/Geometry>
#include <curobo_bridge_msgs/msg/box.hpp>
#include <curobo_bridge_msgs/srv/plan_joint.hpp>
#include <curobo_bridge_msgs/srv/plan_pose.hpp>
#include <curobo_bridge_msgs/srv/set_world.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "core_types/null_logger.hpp"

namespace curobo_planner
{
namespace mp = motion_planner;
using curobo_bridge_msgs::srv::PlanJoint;
using curobo_bridge_msgs::srv::PlanPose;
using curobo_bridge_msgs::srv::SetWorld;

namespace
{
mp::PlanningResult fail(const std::string & code, const std::string & message, mp::FailureStage stage)
{
  mp::PlanningResult r;
  r.status = "failed";
  r.message = message;
  r.failure_reason_code = code;
  r.failure_stage = stage;
  return r;
}

mp::FailureStage stageFrom(const std::string & s)
{
  return s == "planning" ? mp::FailureStage::kPlanning : mp::FailureStage::kPrecheck;
}

template<typename Response>
mp::PlanningResult toResult(const Response & res)
{
  if (!res.success) {
    return fail(res.failure_reason_code, res.message, stageFrom(res.failure_stage));
  }
  mp::PlanningResult r;
  r.success = true;
  r.status = "success";
  r.path_fraction = 1.0;
  r.planning_time_ms = static_cast<uint64_t>(res.planning_time_s * 1000.0);
  r.trajectory.joint_names = res.trajectory.joint_names;
  for (const auto & p : res.trajectory.points) {
    core_types::TrajectoryPoint tp;
    tp.positions = p.positions;
    tp.time_from_start_sec = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9;
    r.trajectory.points.push_back(std::move(tp));
  }
  return r;
}

Eigen::Isometry3d toIso(const core_types::Pose & p)
{
  Eigen::Quaterniond q(p.qw, p.qx, p.qy, p.qz);
  q = q.norm() < 1e-9 ? Eigen::Quaterniond::Identity() : q.normalized();
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = q.toRotationMatrix();
  t.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  return t;
}
}  // namespace

struct CuroboClientPlanner::Impl
{
  rclcpp::Node::SharedPtr node;
  Options options;
  std::shared_ptr<core_types::Logger> logger;
  rclcpp::Client<PlanJoint>::SharedPtr plan_joint;
  rclcpp::Client<PlanPose>::SharedPtr plan_pose;
  rclcpp::Client<SetWorld>::SharedPtr set_world;

  template<typename Srv>
  typename Srv::Response::SharedPtr call(
    typename rclcpp::Client<Srv>::SharedPtr client, typename Srv::Request::SharedPtr request,
    std::string & error)
  {
    if (!client->wait_for_service(std::chrono::duration<double>(options.wait_for_service_sec))) {
      error = "cuRobo planner service is not available";
      return nullptr;
    }
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(options.timeout_sec)) != std::future_status::ready) {
      client->remove_pending_request(future);
      error = "cuRobo planner call timed out";
      return nullptr;
    }
    return future.get();
  }
};

CuroboClientPlanner::CuroboClientPlanner(
  rclcpp::Node::SharedPtr node, const Options & options, std::shared_ptr<core_types::Logger> logger)
: impl_(std::make_unique<Impl>())
{
  impl_->node = std::move(node);
  impl_->options = options;
  impl_->logger = logger ? std::move(logger) : std::make_shared<core_types::NullLogger>();
  impl_->plan_joint = impl_->node->create_client<PlanJoint>(options.node_name + "/plan_joint");
  impl_->plan_pose = impl_->node->create_client<PlanPose>(options.node_name + "/plan_pose");
  impl_->set_world = impl_->node->create_client<SetWorld>(options.node_name + "/set_world");
}

CuroboClientPlanner::~CuroboClientPlanner() = default;

mp::PlanningResult CuroboClientPlanner::plan(const mp::PlanningRequest & request)
{
  if (request.start_joint_state.empty()) {
    return fail("invalid_request", "start_joint_state is required", mp::FailureStage::kPrecheck);
  }
  std::string error;
  if (!request.target_joint_state.empty()) {
    auto req = std::make_shared<PlanJoint::Request>();
    req->start = request.start_joint_state;
    req->goal = request.target_joint_state;
    auto res = impl_->call<PlanJoint>(impl_->plan_joint, req, error);
    return res ? toResult(*res) : fail("unavailable", error, mp::FailureStage::kPrecheck);
  }
  if (request.target_poses.size() == 1) {
    const auto & p = request.target_poses.front();
    auto req = std::make_shared<PlanPose::Request>();
    req->start = request.start_joint_state;
    req->goal.position.x = p.x;
    req->goal.position.y = p.y;
    req->goal.position.z = p.z;
    req->goal.orientation.x = p.qx;
    req->goal.orientation.y = p.qy;
    req->goal.orientation.z = p.qz;
    req->goal.orientation.w = p.qw;
    auto res = impl_->call<PlanPose>(impl_->plan_pose, req, error);
    return res ? toResult(*res) : fail("unavailable", error, mp::FailureStage::kPrecheck);
  }
  return fail(
    "invalid_request", "need target_joint_state, or exactly one target pose", mp::FailureStage::kPrecheck);
}

bool CuroboClientPlanner::setWorld(const mp::WorldGeometryUpdate & world, std::string * message)
{
  auto req = std::make_shared<SetWorld::Request>();
  for (const auto & b : world.boxes) {
    curobo_bridge_msgs::msg::Box box;
    box.id = b.id;
    box.pose.position.x = b.center_x;
    box.pose.position.y = b.center_y;
    box.pose.position.z = b.center_z;
    box.pose.orientation.x = b.orientation_x;
    box.pose.orientation.y = b.orientation_y;
    box.pose.orientation.z = b.orientation_z;
    box.pose.orientation.w = b.orientation_w;
    box.size = {b.size_x, b.size_y, b.size_z};
    req->boxes.push_back(box);
  }
  if (!world.points.empty()) {
    const Eigen::Isometry3d T = toIso(world.points_origin);
    auto & cloud = req->points;
    cloud.header.frame_id = world.frame_id;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(world.points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    for (const char * n : {"x", "y", "z"}) {
      sensor_msgs::msg::PointField f;
      f.name = n;
      f.offset = static_cast<uint32_t>(cloud.fields.size()) * 4;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count = 1;
      cloud.fields.push_back(f);
    }
    cloud.data.resize(cloud.row_step);
    for (size_t i = 0; i < world.points.size(); ++i) {
      const Eigen::Vector3d w = T * Eigen::Vector3d(world.points[i].x, world.points[i].y, world.points[i].z);
      const float xyz[3] = {static_cast<float>(w.x()), static_cast<float>(w.y()), static_cast<float>(w.z())};
      std::memcpy(&cloud.data[i * 12], xyz, 12);
    }
  }
  std::string error;
  auto res = impl_->call<SetWorld>(impl_->set_world, req, error);
  if (!res) {
    if (message) {*message = error;}
    return false;
  }
  if (!res->success && message) {*message = res->message;}
  return res->success;
}

bool CuroboClientPlanner::clearWorld(std::string * message)
{
  return setWorld(mp::WorldGeometryUpdate{}, message);
}

mp::IkResult CuroboClientPlanner::ik(const mp::IkRequest &)
{
  mp::IkResult r;
  r.message = "the cuRobo client does not expose IK; use an IK backend";
  r.failure_reason_code = "unsupported";
  r.failure_stage = mp::FailureStage::kPrecheck;
  return r;
}

mp::CollisionResult CuroboClientPlanner::collision(const mp::CollisionRequest &)
{
  mp::CollisionResult r;
  r.message = "the cuRobo client does not expose collision checks";
  r.failure_reason_code = "unsupported";
  r.failure_stage = mp::FailureStage::kPrecheck;
  return r;
}

}  // namespace curobo_planner
