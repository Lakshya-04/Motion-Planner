#include "robot_self_filter/ros/self_filter_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "scene_model/core/io.hpp"
#include "scene_model/ros/conversions.hpp"

namespace robot_self_filter
{
namespace
{

Pose toPose(const geometry_msgs::msg::TransformStamped & t)
{
  Pose T = Pose::Identity();
  T.translation() = Vec3(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
  Eigen::Quaterniond q(t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y,
    t.transform.rotation.z);
  q.normalize();
  T.linear() = q.toRotationMatrix();
  return T;
}

// package://pkg/path and file:// URIs of mesh collision shapes -> bounding box.
std::optional<MeshBounds> meshBounds(const std::string & uri)
{
  std::string path;
  const std::string pkg = "package://";
  if (uri.rfind(pkg, 0) == 0) {
    const auto slash = uri.find('/', pkg.size());
    if (slash == std::string::npos) {
      return std::nullopt;
    }
    try {
      path = ament_index_cpp::get_package_share_directory(uri.substr(pkg.size(), slash - pkg.size())) +
        uri.substr(slash);
    } catch (const std::exception &) {
      return std::nullopt;
    }
  } else if (uri.rfind("file://", 0) == 0) {
    path = uri.substr(7);
  } else {
    return std::nullopt;
  }
  try {
    const auto mesh = scene_model::loadMesh(path);  // .stl / .obj; others throw -> skipped
    MeshBounds b;
    b.min = Vec3::Constant(1e18);
    b.max = Vec3::Constant(-1e18);
    for (const auto & v : mesh.vertices) {
      b.min = b.min.cwiseMin(v);
      b.max = b.max.cwiseMax(v);
    }
    if (mesh.vertices.empty()) {
      return std::nullopt;
    }
    return b;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

}  // namespace

SelfFilterNode::SelfFilterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("self_filter", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  declare_parameter<std::string>("robot_description", "");
  root_link_param_ = declare_parameter<std::string>("root_link", "");
  tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.1);

  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", rclcpp::SensorDataQoS());
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_in", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {onCloud(msg);});
  contact_sub_ = create_subscription<std_msgs::msg::Bool>(
    "contact_mode", rclcpp::QoS(1).transient_local(),
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {contact_mode_ = msg->data;});

  const std::string urdf = get_parameter("robot_description").as_string();
  if (!urdf.empty()) {
    loadRobot(urdf);
  } else {
    description_sub_ = create_subscription<std_msgs::msg::String>(
      "robot_description", rclcpp::QoS(1).transient_local().reliable(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        if (!ready()) {loadRobot(msg->data);}
      });
  }
}

SelfFilterConfig SelfFilterNode::readConfig()
{
  SelfFilterConfig c;
  c.padding.sphere_radius = declare_parameter<double>("padding.sphere", 0.02);
  const auto box = declare_parameter<std::vector<double>>("padding.box", {0.02, 0.02, 0.02});
  if (box.size() == 3) {
    c.padding.box = Vec3(box[0], box[1], box[2]);
  }
  c.padding.cylinder_radius = declare_parameter<double>("padding.cylinder_radius", 0.02);
  c.padding.cylinder_height = declare_parameter<double>("padding.cylinder_height", 0.02);

  for (const auto & link : declare_parameter<std::vector<std::string>>("padding_overrides", std::vector<std::string>{})) {
    Padding p = c.padding;
    const std::string base = "padding_overrides." + link + ".";
    p.sphere_radius = declare_parameter<double>(base + "sphere", p.sphere_radius);
    const auto b = declare_parameter<std::vector<double>>(base + "box", {p.box.x(), p.box.y(), p.box.z()});
    if (b.size() == 3) {
      p.box = Vec3(b[0], b[1], b[2]);
    }
    p.cylinder_radius = declare_parameter<double>(base + "cylinder_radius", p.cylinder_radius);
    p.cylinder_height = declare_parameter<double>(base + "cylinder_height", p.cylinder_height);
    c.link_padding[link] = p;
  }
  for (const auto & l : declare_parameter<std::vector<std::string>>("links", std::vector<std::string>{})) {
    c.links.insert(l);
  }
  c.min_sensor_distance = declare_parameter<double>("min_sensor_distance", 0.0);
  c.shadow = declare_parameter<bool>("shadow", false);
  c.contact_link = declare_parameter<std::string>("contact_link", "");
  c.contact_padding = declare_parameter<double>("contact_padding", 0.0);
  return c;
}

void SelfFilterNode::loadRobot(const std::string & urdf)
{
  try {
    robot_ = UrdfRobot::fromUrdf(urdf, meshBounds);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "cannot use the robot description: %s", e.what());
    return;
  }
  for (const auto & m : robot_->skippedMeshes()) {
    RCLCPP_WARN(get_logger(), "mesh collision shape not filtered (unsupported or unresolved): %s", m.c_str());
  }
  root_link_ = root_link_param_.empty() ? robot_->rootLink() : root_link_param_;
  filter_.emplace(robot_->shapes(), readConfig());
  RCLCPP_INFO(get_logger(), "self filter ready (root '%s', %zu links with shapes)", root_link_.c_str(),
    robot_->shapes().size());
}

void SelfFilterNode::onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  if (!ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no robot description yet: dropping clouds");
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp);
  const auto timeout = rclcpp::Duration::from_seconds(tf_timeout_sec_);

  SelfFilterInput in;
  scene_model::PointCloud cloud;
  std::vector<size_t> source;
  try {
    cloud = scene_model::ros_conv::fromRos(*msg, source);
    in.cloud_to_root = toPose(tf_buffer_.lookupTransform(root_link_, msg->header.frame_id, stamp, timeout));
    in.sensor_origin = in.cloud_to_root.translation();
    for (const auto & ls : robot_->shapes()) {
      const auto & wanted = filter_->config().links;
      if (ls.shapes.empty() || (!wanted.empty() && !wanted.count(ls.link))) {
        continue;
      }
      in.link_poses[ls.link] = (ls.link == root_link_) ? Pose::Identity() :
        toPose(tf_buffer_.lookupTransform(root_link_, ls.link, stamp, timeout));
    }
  } catch (const std::exception & e) {
    // Publishing the cloud unfiltered would put the robot into every map: drop it instead.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "dropping a cloud: %s", e.what());
    return;
  }
  in.cloud = &cloud;
  in.contact_mode = contact_mode_;

  const auto result = filter_->filter(in);
  if (!result.success) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "dropping a cloud: %s", result.message.c_str());
    return;
  }
  // Map the mask back onto the message (points with a non-finite coordinate are kept as they are).
  std::vector<bool> removed(static_cast<size_t>(msg->width) * msg->height, false);
  for (size_t k = 0; k < source.size(); ++k) {
    removed[source[k]] = result.removed[k];
  }
  cloud_pub_->publish(scene_model::ros_conv::keepPoints(*msg, removed));
}

}  // namespace robot_self_filter
