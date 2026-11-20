#ifndef COVERAGE_PLANNER__COVERAGE_PLANNER_NODE_HPP_
#define COVERAGE_PLANNER__COVERAGE_PLANNER_NODE_HPP_

#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>
#include "coverage_planner/core/coverage_planner_base.hpp"
#include "coverage_planner_msgs/srv/generate_coverage_path.hpp"

namespace coverage_planner
{

// The interfaces live in coverage_planner_msgs; alias them into this package's own
// namespace so `msg::`/`srv::` still mean what they always did here.
namespace srv = coverage_planner_msgs::srv;

/**
 * @brief Main ROS 2 node for coverage path planning
 *
 * Subscribes to /map and provides a service to generate coverage paths.
 * Uses a factory pattern to support multiple planning strategies.
 */
class CoveragePlannerNode : public rclcpp::Node
{
public:
  explicit CoveragePlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Map callback - caches the latest map
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * @brief Service callback - generates coverage path
   */
  void generateCoveragePathCallback(
    const std::shared_ptr<srv::GenerateCoveragePath::Request> request,
    std::shared_ptr<srv::GenerateCoveragePath::Response> response);

  /**
   * @brief Declare and load all parameters
   */
  void declareParameters();

  /**
   * @brief Build planner params from node parameters and service request
   */
  CoveragePlannerParams buildPlannerParams(
    const std::shared_ptr<srv::GenerateCoveragePath::Request> & request);

  /**
   * @brief Publish waypoint markers for visualization
   */
  void publishWaypointMarkers(const nav_msgs::msg::Path & path);

  /**
   * @brief Publish planner diagnostics summary
   */
  void publishDiagnostics(
    const CoveragePlannerResult & result, const std::string & pattern,
    const nav_msgs::msg::OccupancyGrid & map);

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  // Services
  rclcpp::Service<srv::GenerateCoveragePath>::SharedPtr coverage_service_;

  // Publishers (for visualization)
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;

  // Cached map
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  std::mutex map_mutex_;

  // Node parameters (defaults)
  std::string coverage_pattern_;
  double swath_;
  double step_;
  std::string sweep_direction_;
  std::string start_corner_;
  bool add_strip_connectors_;
  double obstacle_inflation_radius_;
  bool allow_unknown_;
  bool plan_all_components_;
  int min_component_cells_;
  bool optimize_strip_order_;
  bool components_in_sweep_order_;
  bool fill_gaps_;
  double min_stroke_length_m_;
  double edge_zone_m_;
  double edge_spacing_scale_;
  std::string decomposition_;
  std::string map_frame_;
  std::string map_topic_;
};

}  // namespace coverage_planner

#endif  // COVERAGE_PLANNER__COVERAGE_PLANNER_NODE_HPP_
