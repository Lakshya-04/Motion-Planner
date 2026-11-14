#include "sim_mujoco/mujoco_sim.hpp"

#include <cmath>
#include <stdexcept>

namespace sim_mujoco
{

namespace
{
// Multiplies MuJoCo's site force / torque to give "environment on tool". Verified against a wall
// push in test_mujoco_sim.cpp (SignOfWallReaction).
constexpr double kSensorSign = -1.0;
}  // namespace

MujocoSim::MujocoSim(
  const std::string & mjcf_path, const std::vector<std::string> & joints,
  const std::string & ft_site)
{
  char error[1000] = "";
  model_ = mj_loadXML(mjcf_path.c_str(), nullptr, error, sizeof(error));
  if (!model_) {
    throw std::runtime_error("MuJoCo could not load '" + mjcf_path + "': " + error);
  }
  data_ = mj_makeData(model_);
  if (!data_) {
    mj_deleteModel(model_);
    throw std::runtime_error("mj_makeData failed");
  }

  try {
    for (const auto & name : joints) {
      const int j = mj_name2id(model_, mjOBJ_JOINT, name.c_str());
      if (j < 0) {
        throw std::runtime_error("joint '" + name + "' is not in the MJCF");
      }
      int act = -1;
      for (int a = 0; a < model_->nu; ++a) {
        if (model_->actuator_trntype[a] == mjTRN_JOINT && model_->actuator_trnid[2 * a] == j) {
          act = a;
          break;
        }
      }
      if (act < 0) {
        throw std::runtime_error("joint '" + name + "' has no actuator in the MJCF");
      }
      const double kp = model_->actuator_gainprm[act * mjNGAIN];
      const double kv = -model_->actuator_biasprm[act * mjNBIAS + 2];
      if (!(kp > 0.0) || model_->actuator_gaintype[act] != mjGAIN_FIXED ||
        model_->actuator_biastype[act] != mjBIAS_AFFINE)
      {
        throw std::runtime_error("actuator of joint '" + name + "' is not a position actuator");
      }
      Joint joint;
      joint.qpos_adr = model_->jnt_qposadr[j];
      joint.dof_adr = model_->jnt_dofadr[j];
      joint.actuator = act;
      joint.kv_over_kp = kv / kp;
      joint.target = data_->qpos[joint.qpos_adr];
      joint_.push_back(joint);
    }
    velocity_command_.assign(joint_.size(), 0.0);

    const int site = mj_name2id(model_, mjOBJ_SITE, ft_site.c_str());
    if (site < 0) {
      throw std::runtime_error("F/T site '" + ft_site + "' is not in the MJCF");
    }
    for (int s = 0; s < model_->nsensor; ++s) {
      if (model_->sensor_objtype[s] != mjOBJ_SITE || model_->sensor_objid[s] != site) {
        continue;
      }
      if (model_->sensor_type[s] == mjSENS_FORCE) {
        force_adr_ = model_->sensor_adr[s];
      } else if (model_->sensor_type[s] == mjSENS_TORQUE) {
        torque_adr_ = model_->sensor_adr[s];
      }
    }
    if (force_adr_ < 0 || torque_adr_ < 0) {
      throw std::runtime_error("site '" + ft_site + "' needs a force and a torque sensor");
    }
  } catch (...) {
    mj_deleteData(data_);
    mj_deleteModel(model_);
    throw;
  }
  mj_forward(model_, data_);
}

MujocoSim::~MujocoSim()
{
  mj_deleteData(data_);
  mj_deleteModel(model_);
}

void MujocoSim::applyMimic()
{
  // Joint equality constraints q1 = c0 + c1 q2 + ...: put the follower on its constraint so a
  // new pose does not start with a violated constraint.
  for (int e = 0; e < model_->neq; ++e) {
    if (model_->eq_type[e] != mjEQ_JOINT) {
      continue;
    }
    const int j1 = model_->eq_obj1id[e];
    const int j2 = model_->eq_obj2id[e];
    if (j2 < 0) {
      continue;
    }
    const mjtNum * c = model_->eq_data + e * mjNEQDATA;
    const double x = data_->qpos[model_->jnt_qposadr[j2]] - model_->qpos0[model_->jnt_qposadr[j2]];
    double y = 0.0;
    double xp = 1.0;
    for (int k = 0; k < 5; ++k) {
      y += c[k] * xp;
      xp *= x;
    }
    data_->qpos[model_->jnt_qposadr[j1]] = model_->qpos0[model_->jnt_qposadr[j1]] + y;
  }
}

void MujocoSim::setJointPositions(const std::vector<double> & q)
{
  if (q.size() != joint_.size()) {
    throw std::invalid_argument("setJointPositions: wrong number of joints");
  }
  for (size_t i = 0; i < q.size(); ++i) {
    data_->qpos[joint_[i].qpos_adr] = q[i];
    data_->qvel[joint_[i].dof_adr] = 0.0;
    joint_[i].target = q[i];
    velocity_command_[i] = 0.0;
  }
  applyMimic();
  mj_forward(model_, data_);
}

void MujocoSim::setCommand(size_t i, double position, double velocity)
{
  if (std::isfinite(position)) {
    joint_[i].target = position;
  }
  velocity_command_[i] = std::isfinite(velocity) ? velocity : 0.0;
}

int MujocoSim::advance(double dt)
{
  carry_ += dt;
  const double h = model_->opt.timestep;
  const int n = static_cast<int>(std::floor(carry_ / h + 1e-9));
  carry_ -= n * h;
  for (int s = 0; s < n; ++s) {
    for (size_t i = 0; i < joint_.size(); ++i) {
      data_->ctrl[joint_[i].actuator] = joint_[i].target + joint_[i].kv_over_kp * velocity_command_[i];
    }
    mj_step(model_, data_);
  }
  return n;
}

Wrench MujocoSim::rawWrench() const
{
  Wrench w{};
  for (int k = 0; k < 3; ++k) {
    w[k] = kSensorSign * data_->sensordata[force_adr_ + k];
    w[3 + k] = kSensorSign * data_->sensordata[torque_adr_ + k];
  }
  return w;
}

Wrench MujocoSim::wrench() const
{
  Wrench w = rawWrench();
  for (size_t k = 0; k < w.size(); ++k) {
    w[k] -= tare_[k];
  }
  return w;
}

void MujocoSim::tareWrench()
{
  tare_ = rawWrench();
}

}  // namespace sim_mujoco
