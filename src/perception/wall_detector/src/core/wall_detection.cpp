#include "wall_detector/core/wall_detection.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "scene_model/core/io.hpp"

namespace wall_detector
{

using scene_model::Vec3;

std::optional<WallDetection> detectWall(
  const std::vector<Vec3> & points, const Vec3 & viewpoint, const WallDetectionParams & params)
{
  std::vector<Vec3> kept;
  kept.reserve(points.size());
  for (const auto & p : points) {
    if (!p.allFinite()) {
      continue;
    }
    if (params.max_range_m > 0.0 && (p - viewpoint).norm() > params.max_range_m) {
      continue;
    }
    kept.push_back(p);
  }
  if (kept.size() < params.plane.min_inliers) {
    return std::nullopt;
  }
  Vec3 mean = Vec3::Zero();
  for (const auto & p : kept) {
    mean += p;
  }
  mean /= static_cast<double>(kept.size());

  scene_model::PlaneFitParams plane = params.plane;
  if (plane.normal_hint.isZero()) {
    plane.normal_hint = viewpoint - mean;  // toward the sensor
  }
  auto fit = scene_model::fitPlane(kept, plane);
  if (!fit) {
    return std::nullopt;
  }
  WallDetection out;
  out.num_points = kept.size();
  out.inlier_fraction = static_cast<double>(fit->inliers.size()) / static_cast<double>(kept.size());
  if (out.inlier_fraction < params.min_inlier_fraction) {
    return std::nullopt;
  }
  out.inlier_points.reserve(fit->inliers.size());
  for (const auto i : fit->inliers) {
    out.inlier_points.push_back(kept[i]);
  }
  out.fit = std::move(*fit);
  return out;
}

std::vector<Vec3> transformPoints(
  const std::vector<Vec3> & points, const scene_model::Transform & transform)
{
  std::vector<Vec3> out;
  out.reserve(points.size());
  for (const auto & p : points) {
    out.push_back(transform * p);
  }
  return out;
}

nlohmann::json wallSceneJson(
  const scene_model::PlaneFit & fit, uint32_t wall_id, const std::string & frame_id,
  const std::string & source)
{
  auto vec = [](const Vec3 & v) {return nlohmann::json::array({v.x(), v.y(), v.z()});};
  nlohmann::json wall;
  wall["id"] = wall_id;
  wall["plane_abcd"] = {fit.normal.x(), fit.normal.y(), fit.normal.z(), -fit.normal.dot(fit.origin)};
  wall["origin_xyz"] = vec(fit.origin);
  wall["basis_u"] = vec(fit.u);
  wall["basis_v"] = vec(fit.v);
  wall["normal_outward"] = vec(fit.normal);
  wall["u_range_m"] = {fit.u_min, fit.u_max};
  wall["v_range_m"] = {fit.v_min, fit.v_max};
  wall["plane_rms_mm"] = fit.rms * 1000.0;
  wall["spread_p95_mm"] = fit.spread_p95 * 1000.0;
  wall["spread_p99_mm"] = fit.spread_p99 * 1000.0;
  wall["num_inliers"] = fit.inliers.size();

  nlohmann::json scene;
  scene["frame_id"] = frame_id;
  scene["note"] = "Detected by wall_detector from a live point cloud; wall pose in frame_id. "
    "walls.pcd (label = wall id) holds the inlier points. Obstacles are not detected (empty).";
  scene["source"] = source;
  scene["units"] = "meters";
  scene["walls"] = nlohmann::json::array({wall});
  scene["obstacles"] = nlohmann::json::array();
  return scene;
}

namespace
{
void replaceFile(const std::filesystem::path & tmp, const std::filesystem::path & dst)
{
  std::error_code ec;
  std::filesystem::rename(tmp, dst, ec);
  if (ec) {
    throw std::runtime_error("cannot move " + tmp.string() + " to " + dst.string() + ": " + ec.message());
  }
}
}  // namespace

std::string writeWallScene(
  const std::string & dir, const scene_model::PlaneFit & fit, const std::vector<Vec3> & points,
  uint32_t wall_id, const std::string & frame_id, const std::string & source)
{
  const std::filesystem::path out(dir);
  std::filesystem::create_directories(out);

  scene_model::PointCloud cloud;
  cloud.points = points;
  cloud.labels.assign(points.size(), wall_id);
  const auto pcd_tmp = out / "walls.pcd.tmp";
  scene_model::savePcdAscii(pcd_tmp.string(), cloud);
  replaceFile(pcd_tmp, out / "walls.pcd");

  const auto json_tmp = out / "wall_scene.json.tmp";
  {
    std::ofstream f(json_tmp);
    if (!f) {
      throw std::runtime_error("cannot write " + json_tmp.string());
    }
    f << wallSceneJson(fit, wall_id, frame_id, source).dump(2) << "\n";
  }
  replaceFile(json_tmp, out / "wall_scene.json");
  return (out / "wall_scene.json").string();
}

}  // namespace wall_detector
