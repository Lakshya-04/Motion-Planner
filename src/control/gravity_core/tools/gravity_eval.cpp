// gravity_eval robot.urdf joint1,joint2,... [payload_link mass cx cy cz] < poses
// Reads one pose per line (one value per joint) and prints the gravity torques, one line per pose.
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "gravity_core/urdf_loader.hpp"

int main(int argc, char** argv) {
  if (argc != 3 && argc != 8) {
    std::cerr << "usage: gravity_eval robot.urdf j1,j2,... [payload_link mass cx cy cz] < poses\n";
    return 2;
  }
  std::ifstream f(argv[1]);
  std::stringstream xml;
  xml << f.rdbuf();
  std::vector<std::string> joints;
  std::stringstream names(argv[2]);
  for (std::string n; std::getline(names, n, ',');) joints.push_back(n);
  try {
    auto model = gravity_core::loadUrdf(xml.str(), joints);
    if (argc == 8) {
      model.setPayload(argv[3], std::stod(argv[4]),
                       Eigen::Vector3d(std::stod(argv[5]), std::stod(argv[6]), std::stod(argv[7])));
    }
    Eigen::VectorXd q(model.dof()), tau;
    std::cout << std::setprecision(12);
    for (std::string line; std::getline(std::cin, line);) {
      std::stringstream in(line);
      for (size_t i = 0; i < model.dof(); ++i) in >> q[i];
      model.compute(q, tau);
      for (Eigen::Index i = 0; i < tau.size(); ++i) std::cout << tau[i] << (i + 1 < tau.size() ? " " : "\n");
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
  return 0;
}
