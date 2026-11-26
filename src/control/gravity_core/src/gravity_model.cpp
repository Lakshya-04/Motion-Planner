#include "gravity_core/gravity_model.hpp"

#include <algorithm>
#include <stdexcept>

namespace gravity_core {

GravityModel::GravityModel(std::vector<Link> links, std::vector<Joint> joints,
                           const std::vector<std::string>& actuated)
    : links_(std::move(links)), joints_(std::move(joints)) {
  const size_t nj = joints_.size();
  for (const auto& j : joints_) {
    const int n = static_cast<int>(links_.size());
    if (j.parent < 0 || j.child < 0 || j.parent >= n || j.child >= n || j.parent >= j.child) {
      throw std::runtime_error("gravity_core: joint " + j.name + " is out of order or out of range");
    }
  }
  for (const auto& name : actuated) {
    int found = -1;
    for (size_t k = 0; k < nj; ++k) {
      if (joints_[k].name == name) found = static_cast<int>(k);
    }
    if (found < 0) throw std::runtime_error("gravity_core: unknown joint " + name);
    if (joints_[found].type == JointType::Fixed) throw std::runtime_error("gravity_core: fixed joint " + name);
    actuated_.push_back(found);
    actuated_names_.push_back(name);
  }

  // Follow mimic chains to the actuated joint that drives each joint.
  driver_.assign(nj, -1);
  scale_.assign(nj, 1.0);
  offset_.assign(nj, 0.0);
  for (size_t k = 0; k < nj; ++k) {
    int cur = static_cast<int>(k);
    double s = 1.0;
    double o = 0.0;
    for (size_t hops = 0; hops <= nj; ++hops) {
      const auto it = std::find(actuated_.begin(), actuated_.end(), cur);
      if (it != actuated_.end()) {
        driver_[k] = static_cast<int>(it - actuated_.begin());
        scale_[k] = s;
        offset_[k] = o;
        break;
      }
      if (joints_[cur].mimic < 0) break;
      o += s * joints_[cur].mimic_offset;
      s *= joints_[cur].mimic_multiplier;
      cur = joints_[cur].mimic;
    }
  }

  link_t_.assign(links_.size(), Eigen::Isometry3d::Identity());
  mass_sum_.assign(links_.size(), 0.0);
  moment_sum_.assign(links_.size(), Eigen::Vector3d::Zero());
  axis_w_.assign(nj, Eigen::Vector3d::Zero());
  pivot_w_.assign(nj, Eigen::Vector3d::Zero());
  tau_full_.assign(nj, 0.0);
}

void GravityModel::setPayload(const std::string& link, double mass, const Eigen::Vector3d& com) {
  if (mass == 0.0) {
    payload_link_ = -1;
    payload_mass_ = 0.0;
    return;
  }
  for (size_t i = 0; i < links_.size(); ++i) {
    if (links_[i].name == link) {
      payload_link_ = static_cast<int>(i);
      payload_mass_ = mass;
      payload_com_ = com;
      return;
    }
  }
  throw std::runtime_error("gravity_core: unknown payload link " + link);
}

void GravityModel::compute(const Eigen::VectorXd& q, Eigen::VectorXd& tau) {
  if (q.size() != static_cast<Eigen::Index>(actuated_.size())) {
    throw std::runtime_error("gravity_core: q has the wrong size");
  }
  if (tau.size() != q.size()) tau.resize(q.size());
  tau.setZero();

  // Link poses, joint axes and pivots in the root frame.
  link_t_[0].setIdentity();
  for (size_t k = 0; k < joints_.size(); ++k) {
    const auto& j = joints_[k];
    const Eigen::Isometry3d tj = link_t_[j.parent] * j.origin;
    const Eigen::Vector3d axis = j.axis.normalized();
    const double qk = driver_[k] >= 0 ? scale_[k] * q[driver_[k]] + offset_[k] : 0.0;
    Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
    if (j.type == JointType::Revolute) {
      motion.linear() = Eigen::AngleAxisd(qk, axis).toRotationMatrix();
    } else if (j.type == JointType::Prismatic) {
      motion.translation() = axis * qk;
    }
    link_t_[j.child] = tj * motion;
    axis_w_[k] = tj.linear() * axis;
    pivot_w_[k] = tj.translation();
  }

  for (size_t i = 0; i < links_.size(); ++i) {
    mass_sum_[i] = links_[i].mass;
    moment_sum_[i] = links_[i].mass * (link_t_[i] * links_[i].com);
  }
  if (payload_link_ >= 0) {
    mass_sum_[payload_link_] += payload_mass_;
    moment_sum_[payload_link_] += payload_mass_ * (link_t_[payload_link_] * payload_com_);
  }

  // Leaves up: a joint holds everything below it, so sum the subtree before using it.
  for (size_t k = joints_.size(); k-- > 0;) {
    const auto& j = joints_[k];
    const double m = mass_sum_[j.child];
    const Eigen::Vector3d s = moment_sum_[j.child];
    double t = 0.0;
    if (j.type == JointType::Revolute) {
      t = -gravity_.dot(axis_w_[k].cross(s - m * pivot_w_[k]));
    } else if (j.type == JointType::Prismatic) {
      t = -gravity_.dot(m * axis_w_[k]);
    }
    tau_full_[k] = t;
    mass_sum_[j.parent] += m;
    moment_sum_[j.parent] += s;
  }

  for (size_t k = 0; k < joints_.size(); ++k) {
    if (driver_[k] >= 0) tau[driver_[k]] += scale_[k] * tau_full_[k];
  }
}

}  // namespace gravity_core
