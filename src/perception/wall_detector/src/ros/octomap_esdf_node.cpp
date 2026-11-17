// Octomap -> distance field for the planners' set_esdf (docs/curobo_v2_notes.md).
//
// Polls MoveIt's /get_planning_scene for the octomap (the live one built by the occupancy map monitor,
// `live_octomap:=true`), rasterises the occupied leaves into a fixed window and takes scene_model's exact
// Euclidean distance transform. Publishes
//   ~/esdf       std_msgs/Float32MultiArray   nx*ny*nz distances, x-major (index = ix*ny*nz + iy*nz + iz), the
//                                             layout of curobo_bridge_msgs/srv/SetEsdf; metres, + outside
//   ~/esdf_info  std_msgs/Float64MultiArray   [voxel_size, cx, cy, cz, nx, ny, nz], centre in the octomap frame
// only when the octomap changed (checked by its data size and a hash).
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap_with_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "wall_detector/core/octree_esdf.hpp"

namespace wall_detector
{

class OctomapEsdfNode : public rclcpp::Node
{
public:
  OctomapEsdfNode()
  : rclcpp::Node("octomap_esdf")
  {
    const auto c = declare_parameter<std::vector<double>>("center", {0.8, 0.0, 1.0});
    const auto d = declare_parameter<std::vector<double>>("dims", {1.6, 1.6, 1.6});
    window_.center = Eigen::Vector3d(c.at(0), c.at(1), c.at(2));
    window_.dims = Eigen::Vector3d(d.at(0), d.at(1), d.at(2));
    window_.voxel_size = declare_parameter<double>("voxel_size", 0.05);
    const double period = 1.0 / declare_parameter<double>("rate_hz", 1.0);
    client_ = create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    esdf_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("~/esdf", rclcpp::QoS(1).transient_local());
    info_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/esdf_info", rclcpp::QoS(1).transient_local());
    timer_ = create_wall_timer(std::chrono::duration<double>(period), [this]() {poll();});
  }

private:
  void poll()
  {
    if (pending_ || !client_->service_is_ready()) {
      return;
    }
    auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    req->components.components = moveit_msgs::msg::PlanningSceneComponents::OCTOMAP;
    pending_ = true;
    client_->async_send_request(
      req, [this](rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedFuture f) {
        pending_ = false;
        onScene(f.get()->scene.world.octomap);
      });
  }

  void onScene(const octomap_msgs::msg::OctomapWithPose & m)
  {
    if (m.octomap.data.empty()) {
      return;
    }
    const size_t key = std::hash<std::string>{}(
      std::string(reinterpret_cast<const char *>(m.octomap.data.data()), m.octomap.data.size()));
    if (key == last_key_) {
      return;
    }
    std::unique_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::msgToMap(m.octomap));
    auto * tree = dynamic_cast<octomap::OcTree *>(abstract.get());
    if (tree == nullptr) {
      RCLCPP_WARN(get_logger(), "octomap is not an OcTree");
      return;
    }
    last_key_ = key;
    Eigen::Isometry3d origin;
    tf2::fromMsg(m.origin, origin);
    // The window is given in the planning frame; the tree sits at `origin` in it.
    EsdfWindow w = window_;
    w.center = origin.inverse() * window_.center;
    const auto t0 = now();
    const auto esdf = octreeToEsdf(*tree, w);
    // Float32MultiArray layout: x-major means x outermost, as in index = ix*ny*nz + iy*nz + iz.
    std_msgs::msg::Float32MultiArray out;
    out.data.resize(esdf.distance.size());
    for (int i = 0; i < esdf.nx; ++i) {
      for (int j = 0; j < esdf.ny; ++j) {
        for (int k = 0; k < esdf.nz; ++k) {
          out.data[(static_cast<size_t>(i) * esdf.ny + j) * esdf.nz + k] = esdf.distance[esdf.index(i, j, k)];
        }
      }
    }
    std_msgs::msg::Float64MultiArray info;
    info.data = {esdf.voxel_size, window_.center.x(), window_.center.y(), window_.center.z(),
      static_cast<double>(esdf.nx), static_cast<double>(esdf.ny), static_cast<double>(esdf.nz)};
    esdf_pub_->publish(out);
    info_pub_->publish(info);
    RCLCPP_INFO(get_logger(), "ESDF %dx%dx%d at %.3f m, %zu octomap bytes, %.1f ms", esdf.nx, esdf.ny, esdf.nz,
      esdf.voxel_size, m.octomap.data.size(), (now() - t0).seconds() * 1e3);
  }

  EsdfWindow window_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr esdf_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool pending_{false};
  size_t last_key_{0};
};

}  // namespace wall_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wall_detector::OctomapEsdfNode>());
  rclcpp::shutdown();
  return 0;
}
