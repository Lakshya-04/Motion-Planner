#pragma once

#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace gravity_core {

enum class JointType { Revolute, Prismatic, Fixed };

struct Link {
  std::string name;
  double mass = 0.0;
  Eigen::Vector3d com = Eigen::Vector3d::Zero();  // in the link frame
};

// Joint k joins links[parent] to links[child]; links and joints are ordered parents first.
struct Joint {
  std::string name;
  JointType type = JointType::Fixed;
  int parent = 0;
  int child = 0;
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();  // parent link -> joint frame
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();            // in the joint frame
  int mimic = -1;                                             // joint index it follows, or -1
  double mimic_multiplier = 1.0;
  double mimic_offset = 0.0;
};

// Torque (N m, or N for prismatic) each actuated joint must supply to hold a pose still.
// Needs masses and centres of mass only; inertia tensors do not matter for a static pose.
class GravityModel {
 public:
  // `actuated` names the joints that get an output; mimic joints fold into the joint they follow.
  GravityModel(std::vector<Link> links, std::vector<Joint> joints,
               const std::vector<std::string>& actuated);

  // Gravity in the root link frame. Default is a level base.
  void setGravity(const Eigen::Vector3d& g) { gravity_ = g; }

  // One point mass on a link, replacing the earlier payload. Mass 0 clears it.
  void setPayload(const std::string& link, double mass, const Eigen::Vector3d& com_in_link);

  size_t dof() const { return actuated_.size(); }
  const std::vector<std::string>& actuatedNames() const { return actuated_names_; }

  // q has one entry per actuated joint. Does not allocate once tau has the right size.
  void compute(const Eigen::VectorXd& q, Eigen::VectorXd& tau);

 private:
  std::vector<Link> links_;
  std::vector<Joint> joints_;
  std::vector<int> actuated_;  // joint indices
  std::vector<std::string> actuated_names_;
  Eigen::Vector3d gravity_{0.0, 0.0, -9.81};
  int payload_link_ = -1;
  double payload_mass_ = 0.0;
  Eigen::Vector3d payload_com_ = Eigen::Vector3d::Zero();

  // Per joint: which actuated joint drives it, and q_joint = scale * q_driver + offset.
  std::vector<int> driver_;
  std::vector<double> scale_;
  std::vector<double> offset_;

  std::vector<Eigen::Isometry3d> link_t_;
  std::vector<double> mass_sum_;
  std::vector<Eigen::Vector3d> moment_sum_;
  std::vector<Eigen::Vector3d> axis_w_;
  std::vector<Eigen::Vector3d> pivot_w_;
  std::vector<double> tau_full_;
};

}  // namespace gravity_core
