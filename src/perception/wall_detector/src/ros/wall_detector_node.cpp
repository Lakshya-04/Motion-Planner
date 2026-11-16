// Wall detector node: PointCloud2 (any frame with TF) -> wall plane -> wall_scene.json for the coverage planner.
//
// Collects `accumulate_frames` clouds, transforms them to `target_frame`, fits the dominant plane
// (scene_model RANSAC + PCA), writes `output_dir/wall_scene.json` + `walls.pcd`, publishes the same JSON
// on ~/wall_scene (std_msgs/String, transient local) and, if asked, a thin box at the wall as a MoveIt
// collision object. Repeats for every further batch.
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "scene_model/ros/conversions.hpp"
#include "wall_detector/core/wall_detection.hpp"

namespace wall_detector
{

class WallDetectorNode : public rclcpp::Node
{
public:
  WallDetectorNode()
  : rclcpp::Node("wall_detector"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/depth_sensor/points");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_footprint");
    output_dir_ = declare_parameter<std::string>("output_dir", "/tmp/wall_detector");
    wall_id_ = static_cast<uint32_t>(declare_parameter<int64_t>("wall_id", 0));
    accumulate_ = static_cast<size_t>(declare_parameter<int64_t>("accumulate_frames", 5));
    params_.plane.inlier_threshold = declare_parameter<double>("inlier_threshold", 0.01);
    params_.plane.ransac_iterations = static_cast<int>(declare_parameter<int64_t>("ransac_iterations", 300));
    params_.plane.min_inliers = static_cast<size_t>(declare_parameter<int64_t>("min_inliers", 200));
    params_.max_range_m = declare_parameter<double>("max_range_m", 0.0);
    params_.min_inlier_fraction = declare_parameter<double>("min_inlier_fraction", 0.3);
    publish_collision_ = declare_parameter<bool>("publish_collision_object", false);
    collision_thickness_ = declare_parameter<double>("collision_thickness_m", 0.05);
    collision_id_ = declare_parameter<std::string>("collision_object_id", "detected_wall");

    scene_pub_ = create_publisher<std_msgs::msg::String>("~/wall_scene", rclcpp::QoS(1).transient_local());
    if (publish_collision_) {
      collision_pub_ = create_publisher<moveit_msgs::msg::CollisionObject>(
        "/collision_object", rclcpp::QoS(10).reliable());
    }
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {onCloud(*m);});
    RCLCPP_INFO(get_logger(), "wall_detector: %s -> %s in frame %s, %zu frames per fit, output %s",
      cloud_topic_.c_str(), "wall_scene.json", target_frame_.c_str(), accumulate_, output_dir_.c_str());
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        target_frame_, msg.header.frame_id, msg.header.stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException &) {
      try {
        tf = tf_buffer_.lookupTransform(target_frame_, msg.header.frame_id, tf2::TimePointZero);
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no TF %s -> %s: %s",
          msg.header.frame_id.c_str(), target_frame_.c_str(), e.what());
        return;
      }
    }
    const scene_model::Transform pose = tf2::transformToEigen(tf);
    const auto cloud = scene_model::ros_conv::fromRos(msg);
    auto points = transformPoints(cloud.points, pose);
    batch_.insert(batch_.end(), points.begin(), points.end());
    viewpoint_ = pose.translation();  // the sensor origin in the target frame (last one of the batch)
    if (++frames_ < accumulate_) {
      return;
    }
    frames_ = 0;
    std::vector<scene_model::Vec3> all;
    all.swap(batch_);
    const auto det = detectWall(all, viewpoint_, params_);
    if (!det) {
      RCLCPP_WARN(get_logger(), "no wall in %zu points (need a plane with >= %zu inliers and %.0f%% of the points)",
        all.size(), params_.plane.min_inliers, params_.min_inlier_fraction * 100.0);
      return;
    }
    const std::string source = "wall_detector from " + cloud_topic_;
    try {
      const auto path = writeWallScene(
        output_dir_, det->fit, det->inlier_points, wall_id_, target_frame_, source);
      RCLCPP_INFO(get_logger(),
        "wall: n = (%.4f, %.4f, %.4f), d = %.4f, %zu of %zu points, rms %.2f mm, u [%.2f, %.2f] v [%.2f, %.2f] -> %s",
        det->fit.normal.x(), det->fit.normal.y(), det->fit.normal.z(),
        -det->fit.normal.dot(det->fit.origin), det->fit.inliers.size(), det->num_points,
        det->fit.rms * 1000.0, det->fit.u_min, det->fit.u_max, det->fit.v_min, det->fit.v_max, path.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "cannot write the wall scene: %s", e.what());
      return;
    }
    std_msgs::msg::String text;
    text.data = wallSceneJson(det->fit, wall_id_, target_frame_, source).dump();
    scene_pub_->publish(text);
    if (publish_collision_) {
      collision_pub_->publish(collisionObject(det->fit));
    }
  }

  moveit_msgs::msg::CollisionObject collisionObject(const scene_model::PlaneFit & fit) const
  {
    // Box in the plane frame (x = u, y = v, z = normal toward the robot): the measured face is the
    // front, the box extends collision_thickness behind it.
    moveit_msgs::msg::CollisionObject o;
    o.header.frame_id = target_frame_;
    o.header.stamp = now();
    o.id = collision_id_;
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {fit.u_max - fit.u_min, fit.v_max - fit.v_min, collision_thickness_};
    scene_model::Transform pose = fit.pose();
    pose.translate(scene_model::Vec3(
        0.5 * (fit.u_min + fit.u_max), 0.5 * (fit.v_min + fit.v_max), -0.5 * collision_thickness_));
    o.primitives.push_back(box);
    o.primitive_poses.push_back(scene_model::ros_conv::toRos(pose));
    o.operation = moveit_msgs::msg::CollisionObject::ADD;
    return o;
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string cloud_topic_, target_frame_, output_dir_, collision_id_;
  uint32_t wall_id_{0};
  size_t accumulate_{5};
  size_t frames_{0};
  bool publish_collision_{false};
  double collision_thickness_{0.05};
  WallDetectionParams params_;
  std::vector<scene_model::Vec3> batch_;
  scene_model::Vec3 viewpoint_{scene_model::Vec3::Zero()};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr scene_pub_;
  rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr collision_pub_;
};

}  // namespace wall_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wall_detector::WallDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
