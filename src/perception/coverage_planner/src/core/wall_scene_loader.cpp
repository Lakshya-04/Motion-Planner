#include "coverage_planner/core/wall_scene_loader.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include <Eigen/Dense>

#include "coverage_planner/core/wall_basis.hpp"

namespace coverage_planner
{


namespace
{

Eigen::Vector3d vector3dFromArray(const nlohmann::json & array)
{
  return Eigen::Vector3d(
    array.at(0).get<double>(), array.at(1).get<double>(), array.at(2).get<double>());
}

WallPlane parseWall(const nlohmann::json & wall_node, const std::string & frame_id)
{
  WallPlane wall;
  wall.id = wall_node.at("id").get<uint32_t>();
  wall.pose.header.frame_id = frame_id;
  wall.pose.pose = encodeWallBasis(
    vector3dFromArray(wall_node.at("origin_xyz")), vector3dFromArray(wall_node.at("basis_u")),
    vector3dFromArray(wall_node.at("basis_v")), vector3dFromArray(wall_node.at("normal_outward")));

  const auto & u_range = wall_node.at("u_range_m");
  wall.u_min = u_range.at(0).get<double>();
  wall.u_max = u_range.at(1).get<double>();

  const auto & v_range = wall_node.at("v_range_m");
  wall.v_min = v_range.at(0).get<double>();
  wall.v_max = v_range.at(1).get<double>();

  wall.plane_rms_mm = wall_node.at("plane_rms_mm").get<double>();
  wall.spread_p95_mm = wall_node.at("spread_p95_mm").get<double>();
  wall.spread_p99_mm = wall_node.at("spread_p99_mm").get<double>();
  return wall;
}

ObstacleRegion parseObstacle(const nlohmann::json & obstacle_node)
{
  ObstacleRegion obstacle;
  obstacle.id = obstacle_node.at("id").get<uint32_t>();
  obstacle.parent_wall = obstacle_node.at("parent_wall").get<uint32_t>();

  const auto & uv_on_wall = obstacle_node.at("uv_on_wall");
  const auto & u = uv_on_wall.at("u");
  const auto & v = uv_on_wall.at("v");
  obstacle.u_min = u.at(0).get<double>();
  obstacle.u_max = u.at(1).get<double>();
  obstacle.v_min = v.at(0).get<double>();
  obstacle.v_max = v.at(1).get<double>();
  return obstacle;
}

}  // namespace

WallSceneQuery loadWallScene(
  const std::string & json_path, uint32_t wall_id, const WallFlatnessThresholds & thresholds)
{
  std::ifstream file(json_path);
  if (!file.is_open()) {
    throw std::runtime_error("wall_scene_loader: could not open " + json_path);
  }

  nlohmann::json scene;
  try {
    file >> scene;
  } catch (const nlohmann::json::parse_error & e) {
    throw std::runtime_error("wall_scene_loader: failed to parse " + json_path + ": " + e.what());
  }

  const std::string frame_id = scene.value("frame_id", "");

  const nlohmann::json * wall_node = nullptr;
  for (const auto & candidate : scene.at("walls")) {
    if (candidate.at("id").get<uint32_t>() == wall_id) {
      wall_node = &candidate;
      break;
    }
  }
  if (wall_node == nullptr) {
    throw std::runtime_error(
      "wall_scene_loader: no wall with id " + std::to_string(wall_id) + " in " + json_path);
  }

  WallSceneQuery result;
  result.wall = parseWall(*wall_node, frame_id);

  const bool exceeds_flatness = result.wall.plane_rms_mm > thresholds.max_plane_rms_mm ||
                                result.wall.spread_p95_mm > thresholds.max_spread_p95_mm ||
                                result.wall.spread_p99_mm > thresholds.max_spread_p99_mm;

  if (exceeds_flatness) {
    std::ostringstream message;
    message << "wall_scene_loader: wall " << wall_id
            << " exceeds flatness thresholds (plane_rms_mm=" << result.wall.plane_rms_mm
            << " spread_p95_mm=" << result.wall.spread_p95_mm
            << " spread_p99_mm=" << result.wall.spread_p99_mm
            << ", thresholds: rms<=" << thresholds.max_plane_rms_mm
            << " p95<=" << thresholds.max_spread_p95_mm << " p99<=" << thresholds.max_spread_p99_mm
            << ") -- this wall may not be flat enough for 2D coverage planning";
    if (!thresholds.allow_override) {
      throw std::runtime_error(message.str());
    }
    result.flatness_warning = message.str();
  }

  if (scene.contains("obstacles")) {
    for (const auto & obstacle_node : scene.at("obstacles")) {
      if (obstacle_node.at("parent_wall").get<uint32_t>() == wall_id) {
        result.obstacles.push_back(parseObstacle(obstacle_node));
      }
    }
  }

  return result;
}

}  // namespace coverage_planner
