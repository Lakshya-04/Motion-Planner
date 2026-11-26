// prior_map_to_wall_scene: converts a prior map (.obj, .ply or prior_surfaces .json) to the
// wall_scene.json the coverage planner reads. ROS-free. See docs/prior_map.md.
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include "wall_detector/core/prior_map.hpp"

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "usage: prior_map_to_wall_scene IN.{obj,ply,json} OUT.json [--frame F] [--scale S]"
                 " [--robot X Y Z]\n";
    return 2;
  }
  wall_detector::PriorMapParams params;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--frame" && i + 1 < argc) {
      params.frame_id = argv[++i];
    } else if (a == "--scale" && i + 1 < argc) {
      params.mesh_scale = std::atof(argv[++i]);
    } else if (a == "--robot" && i + 3 < argc) {
      params.robot_position = wall_detector::Vec3(
        std::atof(argv[i + 1]), std::atof(argv[i + 2]), std::atof(argv[i + 3]));
      i += 3;
    } else {
      std::cerr << "unknown or incomplete option " << a << "\n";
      return 2;
    }
  }
  try {
    const auto map = wall_detector::loadPriorMap(argv[1], params);
    std::ofstream out(argv[2]);
    if (!out) {
      std::cerr << "cannot write " << argv[2] << "\n";
      return 1;
    }
    out << wall_detector::priorMapToWallSceneJson(map, argv[1]).dump(2) << "\n";
    for (const auto & w : map.warnings) {
      std::cerr << "warning: " << w << "\n";
    }
    std::cout << map.surfaces.size() << " surface(s), frame " << map.frame_id << " -> " << argv[2] << "\n";
  } catch (const std::exception & e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
