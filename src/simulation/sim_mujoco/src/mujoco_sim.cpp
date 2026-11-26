#include "sim_mujoco/mujoco_sim.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <limits>
#include <string>
#include <stdexcept>

namespace sim_mujoco
{

namespace
{
// Multiplies MuJoCo's site force / torque to give the load the tool side puts on the sensor
// (F_env + weight). Measured with a known weight and a known push in test_mujoco_sim.cpp.
constexpr double kSensorSign = -1.0;
constexpr double kMaxBaseLagM = 0.05;
constexpr double kMaxBaseLagRad = 0.05;
}  // namespace

MujocoSim::MujocoSim(
  const std::string & mjcf_path, const std::vector<std::string> & joints,
  const std::string & ft_site, const std::string & tcp_site)
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
      joint.kp = kp;
      joint.target = data_->qpos[joint.qpos_adr];
      joint_.push_back(joint);
    }
    velocity_command_.assign(joint_.size(), 0.0);

    const int site = mj_name2id(model_, mjOBJ_SITE, ft_site.c_str());
    if (site < 0) {
      throw std::runtime_error("F/T site '" + ft_site + "' is not in the MJCF");
    }
    tcp_site_ = mj_name2id(model_, mjOBJ_SITE, tcp_site.c_str());
    if (tcp_site_ < 0) {
      throw std::runtime_error("TCP site '" + tcp_site + "' is not in the MJCF");
    }
    // tool subtree = the F/T site's body and everything below it
    tool_body_.assign(model_->nbody, 0);
    for (int b = 0; b < model_->nbody; ++b) {
      for (int k = b; k > 0; k = model_->body_parentid[k]) {
        if (k == model_->site_bodyid[site]) {
          tool_body_[b] = 1;
          break;
        }
      }
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
    initBase();
    initDepth();
    initLidar();
  } catch (...) {
    mj_deleteData(data_);
    mj_deleteModel(model_);
    throw;
  }
  mj_forward(model_, data_);
}

void MujocoSim::initBase()
{
  static const char * const names[3] = {"base_x_joint", "base_y_joint", "base_yaw_joint"};
  int found = 0;
  for (int k = 0; k < 3; ++k) {
    found += mj_name2id(model_, mjOBJ_JOINT, names[k]) >= 0 ? 1 : 0;
  }
  if (found == 0) {
    return;
  }
  if (found != 3) {
    throw std::runtime_error("MJCF has some but not all of base_x_joint, base_y_joint, base_yaw_joint");
  }
  for (int k = 0; k < 3; ++k) {
    const int j = mj_name2id(model_, mjOBJ_JOINT, names[k]);
    int act = -1;
    for (int a = 0; a < model_->nu; ++a) {
      if (model_->actuator_trntype[a] == mjTRN_JOINT && model_->actuator_trnid[2 * a] == j) {
        act = a;
      }
    }
    if (act < 0) {
      throw std::runtime_error(std::string("joint '") + names[k] + "' has no actuator");
    }
    const double kp = model_->actuator_gainprm[act * mjNGAIN];
    base_.qpos_adr[k] = model_->jnt_qposadr[j];
    base_.dof_adr[k] = model_->jnt_dofadr[j];
    base_.actuator[k] = act;
    base_.kv_over_kp[k] = kp > 0.0 ? -model_->actuator_biasprm[act * mjNBIAS + 2] / kp : 0.0;
    base_.target[k] = data_->qpos[base_.qpos_adr[k]];
  }
  auto numeric = [this](const char * name, int index, double fallback) {
      const int id = mj_name2id(model_, mjOBJ_NUMERIC, name);
      return (id >= 0 && index < model_->numeric_size[id]) ?
             model_->numeric_data[model_->numeric_adr[id] + index] : fallback;
    };
  base_.diff = numeric("sim_mujoco_base_diff", 0, 0.0) > 0.5;
  base_.max_linear = numeric("sim_mujoco_base_max_vel", 0, 0.5);
  base_.max_angular = numeric("sim_mujoco_base_max_vel", 1, 1.0);
  base_.present = true;
}

void MujocoSim::initDepth()
{
  depth_.site = mj_name2id(model_, mjOBJ_SITE, "depth_sensor");
  const int id = mj_name2id(model_, mjOBJ_NUMERIC, "sim_mujoco_depth");
  if (depth_.site < 0 && id < 0) {
    return;
  }
  if (depth_.site < 0 || id < 0 || model_->numeric_size[id] < 8) {
    throw std::runtime_error(
      "depth sensor needs the site 'depth_sensor' and the 8-value numeric 'sim_mujoco_depth'");
  }
  const mjtNum * d = model_->numeric_data + model_->numeric_adr[id];
  DepthConfig & c = depth_.cfg;
  c.width = static_cast<int>(d[0]);
  c.height = static_cast<int>(d[1]);
  c.hfov = d[2];
  c.vfov = d[3];
  c.min_range = d[4];
  c.max_range = d[5];
  c.rate_hz = d[6];
  c.noise_std = d[7];
  if (c.width < 1 || c.height < 1 || !(c.hfov > 0.0 && c.hfov < M_PI) ||
    !(c.vfov > 0.0 && c.vfov < M_PI) || !(c.max_range > c.min_range) || !(c.rate_hz > 0.0))
  {
    throw std::runtime_error("sim_mujoco_depth: invalid configuration");
  }
  // pinhole grid of pixel centres: first column is +y (left), first row is +z (up)
  const size_t n = static_cast<size_t>(c.width) * static_cast<size_t>(c.height);
  depth_.dirs.resize(3 * n);
  for (int r = 0; r < c.height; ++r) {
    for (int k = 0; k < c.width; ++k) {
      const double sy = 1.0 - 2.0 * (k + 0.5) / c.width;
      const double sz = 1.0 - 2.0 * (r + 0.5) / c.height;
      const double v[3] = {1.0, sy * std::tan(c.hfov / 2.0), sz * std::tan(c.vfov / 2.0)};
      const double norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
      const size_t i = static_cast<size_t>(r) * c.width + k;
      for (int a = 0; a < 3; ++a) {
        depth_.dirs[3 * i + a] = v[a] / norm;
      }
    }
  }
  depth_.world_dirs.resize(3 * n);
  depth_.dist.resize(n);
  depth_.geomid.resize(n);
  depth_.present = true;
}

std::vector<std::array<float, 3>> MujocoSim::depthScan()
{
  std::vector<std::array<float, 3>> points;
  if (!depth_.present) {
    return points;
  }
  const size_t n = depth_.dist.size();
  const mjtNum * origin = data_->site_xpos + 3 * depth_.site;
  const mjtNum * rot = data_->site_xmat + 9 * depth_.site;  // row-major, columns are the site axes
  for (size_t i = 0; i < n; ++i) {
    mju_mulMatVec(&depth_.world_dirs[3 * i], rot, &depth_.dirs[3 * i], 3, 3);
  }
  // the scene's geoms only (the generator puts them in group 1); the robot is not scanned
  const mjtByte groups[mjNGROUP] = {0, 1, 0, 0, 0, 0};
  mj_multiRay(model_, data_, origin, depth_.world_dirs.data(), groups, 1, -1, depth_.geomid.data(),
    depth_.dist.data(), nullptr, static_cast<int>(n), depth_.cfg.max_range);
  std::normal_distribution<double> noise(0.0, std::max(depth_.cfg.noise_std, 1e-12));
  points.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    double range = depth_.dist[i];
    if (range < 0.0) {
      continue;
    }
    if (depth_.cfg.noise_std > 0.0) {
      range += noise(depth_.rng);
    }
    if (range < depth_.cfg.min_range || range > depth_.cfg.max_range) {
      continue;
    }
    points.push_back({static_cast<float>(range * depth_.dirs[3 * i]),
        static_cast<float>(range * depth_.dirs[3 * i + 1]),
        static_cast<float>(range * depth_.dirs[3 * i + 2])});
  }
  return points;
}

void MujocoSim::initLidar()
{
  lidar_.site = mj_name2id(model_, mjOBJ_SITE, "lidar");
  const int id = mj_name2id(model_, mjOBJ_NUMERIC, "sim_mujoco_lidar");
  if (lidar_.site < 0 && id < 0) {
    return;
  }
  if (lidar_.site < 0 || id < 0 || model_->numeric_size[id] < 9) {
    throw std::runtime_error(
      "lidar needs the site 'lidar' and the 9-value numeric 'sim_mujoco_lidar'");
  }
  const mjtNum * d = model_->numeric_data + model_->numeric_adr[id];
  LidarConfig & c = lidar_.cfg;
  c.angle_min = d[0];
  c.angle_max = d[1];
  c.angle_increment = d[2];
  c.range_min = d[3];
  c.range_max = d[4];
  c.rate_hz = d[5];
  c.noise_std = d[6];
  c.dropout = d[7];
  c.seed = static_cast<unsigned>(d[8]);
  if (!(c.angle_increment > 0.0) || !(c.angle_max >= c.angle_min) || !(c.range_min >= 0.0) ||
    !(c.range_max > c.range_min) || !(c.rate_hz > 0.0) || !(c.noise_std >= 0.0) ||
    !(c.dropout >= 0.0 && c.dropout <= 1.0))
  {
    throw std::runtime_error("sim_mujoco_lidar: invalid configuration");
  }
  // same count rule as sim_mujoco.lidar.ray_count (a thousandth of a step absorbs rounding in the angles)
  const double span = (c.angle_max - c.angle_min) / c.angle_increment;
  if (span > 1.0e5) {
    throw std::runtime_error("sim_mujoco_lidar: more than 100000 rays");
  }
  const size_t n = static_cast<size_t>(std::floor(span + 1e-3)) + 1;
  lidar_.dirs.resize(3 * n);
  for (size_t i = 0; i < n; ++i) {
    const double a = c.angle_min + static_cast<double>(i) * c.angle_increment;
    lidar_.dirs[3 * i] = std::cos(a);
    lidar_.dirs[3 * i + 1] = std::sin(a);
    lidar_.dirs[3 * i + 2] = 0.0;
  }
  lidar_.world_dirs.resize(3 * n);
  lidar_.dist.resize(n);
  lidar_.geomid.resize(n);
  lidar_.rng.seed(c.seed);
  lidar_.present = true;
}

std::vector<float> MujocoSim::lidarScan()
{
  if (!lidar_.present) {
    return {};
  }
  const size_t n = lidar_.dist.size();
  const mjtNum * origin = data_->site_xpos + 3 * lidar_.site;
  const mjtNum * rot = data_->site_xmat + 9 * lidar_.site;
  for (size_t i = 0; i < n; ++i) {
    mju_mulMatVec(&lidar_.world_dirs[3 * i], rot, &lidar_.dirs[3 * i], 3, 3);
  }
  // the scene's geoms only (the generator puts them in group 1); the robot is not scanned
  const mjtByte groups[mjNGROUP] = {0, 1, 0, 0, 0, 0};
  mj_multiRay(model_, data_, origin, lidar_.world_dirs.data(), groups, 1, -1, lidar_.geomid.data(),
    lidar_.dist.data(), nullptr, static_cast<int>(n), lidar_.cfg.range_max);
  const LidarConfig & c = lidar_.cfg;
  std::normal_distribution<double> noise(0.0, std::max(c.noise_std, 1e-12));
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  const float inf = std::numeric_limits<float>::infinity();
  std::vector<float> ranges(n, inf);
  for (size_t i = 0; i < n; ++i) {
    double range = lidar_.dist[i];
    if (range < 0.0) {
      continue;
    }
    if (c.dropout > 0.0 && uniform(lidar_.rng) < c.dropout) {
      ranges[i] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }
    if (c.noise_std > 0.0) {
      range += noise(lidar_.rng);
    }
    ranges[i] = range < c.range_min ? -inf : (range > c.range_max ? inf : static_cast<float>(range));
  }
  return ranges;
}

void MujocoSim::setBaseVelocity(double vx, double vy, double wz)
{
  if (!base_.present) {
    return;
  }
  auto fin = [](double v) {return std::isfinite(v) ? v : 0.0;};
  vx = fin(vx);
  vy = base_.diff ? 0.0 : fin(vy);
  wz = fin(wz);
  const double lin = std::hypot(vx, vy);
  if (lin > base_.max_linear) {
    vx *= base_.max_linear / lin;
    vy *= base_.max_linear / lin;
  }
  wz = std::clamp(wz, -base_.max_angular, base_.max_angular);
  base_.cmd[0] = vx;
  base_.cmd[1] = vy;
  base_.cmd[2] = wz;
}

MujocoSim::BasePose MujocoSim::basePose() const
{
  BasePose p{};
  if (!base_.present) {
    return p;
  }
  p.x = data_->qpos[base_.qpos_adr[0]];
  p.y = data_->qpos[base_.qpos_adr[1]];
  p.yaw = data_->qpos[base_.qpos_adr[2]];
  const double wx = data_->qvel[base_.dof_adr[0]];
  const double wy = data_->qvel[base_.dof_adr[1]];
  p.vx = std::cos(p.yaw) * wx + std::sin(p.yaw) * wy;
  p.vy = -std::sin(p.yaw) * wx + std::cos(p.yaw) * wy;
  p.wz = data_->qvel[base_.dof_adr[2]];
  return p;
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

void MujocoSim::setFeedforward(size_t i, double tau)
{
  joint_[i].feedforward = std::isfinite(tau) ? tau : 0.0;
}

bool MujocoSim::hasGravcomp() const
{
  for (int b = 0; b < model_->nbody; ++b) {
    if (model_->body_gravcomp[b] != 0.0) {
      return true;
    }
  }
  return false;
}

int MujocoSim::advance(double dt)
{
  carry_ += dt;
  const double h = model_->opt.timestep;
  const int n = static_cast<int>(std::floor(carry_ / h + 1e-9));
  carry_ -= n * h;
  for (int s = 0; s < n; ++s) {
    for (size_t i = 0; i < joint_.size(); ++i) {
      data_->ctrl[joint_[i].actuator] = joint_[i].target + joint_[i].kv_over_kp * velocity_command_[i] +
        joint_[i].feedforward / joint_[i].kp;
    }
    if (base_.present) {
      // Integrate the set point along the commanded body velocity (zero command = hold position),
      // but never let it run more than kMaxBaseLag ahead of a blocked base (no wind-up).
      const double c = std::cos(base_.target[2]);
      const double sn = std::sin(base_.target[2]);
      const double world[3] = {c * base_.cmd[0] - sn * base_.cmd[1],
        sn * base_.cmd[0] + c * base_.cmd[1], base_.cmd[2]};
      for (int k = 0; k < 3; ++k) {
        const double q = data_->qpos[base_.qpos_adr[k]];
        const double lag = k < 2 ? kMaxBaseLagM : kMaxBaseLagRad;
        base_.target[k] = std::clamp(base_.target[k] + world[k] * h, q - lag, q + lag);
        data_->ctrl[base_.actuator[k]] = base_.target[k] + base_.kv_over_kp[k] * world[k];
      }
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

Wrench MujocoSim::groundTruthWrench() const
{
  const mjtNum * origin = data_->site_xpos + 3 * tcp_site_;
  const mjtNum * rot = data_->site_xmat + 9 * tcp_site_;  // columns are the site axes in world
  double f_w[3] = {0, 0, 0};
  double t_w[3] = {0, 0, 0};
  for (int i = 0; i < data_->ncon; ++i) {
    const mjContact & c = data_->contact[i];
    if (c.exclude != 0) {
      continue;
    }
    const bool a = tool_body_[model_->geom_bodyid[c.geom1]];
    const bool b = tool_body_[model_->geom_bodyid[c.geom2]];
    if (a == b) {
      continue;  // not a tool/environment contact
    }
    mjtNum local[6];
    mj_contactForce(model_, data_, i, local);
    // contact frame rows are its axes; force acts on geom2 along +normal, on geom1 along -normal
    mjtNum fw[3], tw[3];
    mju_mulMatTVec(fw, c.frame, local, 3, 3);
    mju_mulMatTVec(tw, c.frame, local + 3, 3, 3);
    const double sign = b ? 1.0 : -1.0;  // want the force ON the tool geom
    const double r[3] = {c.pos[0] - origin[0], c.pos[1] - origin[1], c.pos[2] - origin[2]};
    const double f[3] = {sign * fw[0], sign * fw[1], sign * fw[2]};
    for (int k = 0; k < 3; ++k) {
      f_w[k] += f[k];
      t_w[k] += sign * tw[k];
    }
    t_w[0] += r[1] * f[2] - r[2] * f[1];
    t_w[1] += r[2] * f[0] - r[0] * f[2];
    t_w[2] += r[0] * f[1] - r[1] * f[0];
  }
  Wrench out{};
  for (int k = 0; k < 3; ++k) {
    for (int j = 0; j < 3; ++j) {
      out[k] += rot[3 * j + k] * f_w[j];       // R^T f
      out[3 + k] += rot[3 * j + k] * t_w[j];   // R^T t
    }
  }
  return out;
}

void MujocoSim::scanUnexpectedContacts(std::vector<std::string> & new_pairs)
{
  auto body_name = [&](int g) {
      const int b = model_->geom_bodyid[g];
      const char * bn = mj_id2name(model_, mjOBJ_BODY, b);
      return std::string(bn ? bn : "?");
    };
  std::set<std::string> counted;
  for (int i = 0; i < data_->ncon; ++i) {
    const mjContact & c = data_->contact[i];
    if (c.exclude != 0 || c.dist >= 0.0) {
      continue;
    }
    const int b1 = model_->geom_bodyid[c.geom1];
    const int b2 = model_->geom_bodyid[c.geom2];
    const bool world1 = (b1 == 0);
    const bool world2 = (b2 == 0);
    std::string kind;
    if (!world1 && !world2) {
      kind = "self";
    } else if (world1 != world2) {
      const int robot_body = world1 ? b2 : b1;
      if (tool_body_[robot_body]) {
        continue;  // the tool on the wall: the intended contact
      }
      kind = "link_world";
    } else {
      continue;  // world against world
    }
    std::string n1 = body_name(c.geom1);
    std::string n2 = body_name(c.geom2);
    if (n2 < n1) {
      std::swap(n1, n2);
    }
    const std::string key = n1 + " <-> " + n2;
    auto & stat = unexpected_[key];
    if (stat.steps == 0) {
      stat.kind = kind;
      char buf[48];
      std::snprintf(buf, sizeof buf, " [%s] depth=%.4f", kind.c_str(), -c.dist);
      new_pairs.push_back(key + buf);
    }
    if (counted.insert(key).second) {  // box-box gives several contacts per pair, count the pair once per scan
      ++stat.steps;
    }
    stat.depth = std::max(stat.depth, -static_cast<double>(c.dist));
  }
}

std::string MujocoSim::unexpectedContactSummary() const
{
  if (unexpected_.empty()) {
    return "none";
  }
  std::string out;
  for (const auto & kv : unexpected_) {
    char buf[96];
    std::snprintf(buf, sizeof buf, " [%s] steps=%ld max_depth=%.4f m; ", kv.second.kind.c_str(), kv.second.steps,
      kv.second.depth);
    out += kv.first + buf;
  }
  return out;
}

std::string MujocoSim::describeToolContacts() const
{
  std::string out;
  for (int i = 0; i < data_->ncon; ++i) {
    const mjContact & c = data_->contact[i];
    if (c.exclude != 0) {
      continue;
    }
    const bool a = tool_body_[model_->geom_bodyid[c.geom1]];
    const bool b = tool_body_[model_->geom_bodyid[c.geom2]];
    if (a == b) {
      continue;
    }
    mjtNum local[6];
    mj_contactForce(model_, data_, i, local);
    auto nm = [&](int g) {
        const char * gn = mj_id2name(model_, mjOBJ_GEOM, g);
        const char * bn = mj_id2name(model_, mjOBJ_BODY, model_->geom_bodyid[g]);
        return std::string(gn ? gn : "?") + "@" + (bn ? bn : "?");
      };
    char buf[64];
    std::snprintf(buf, sizeof buf, " fn=%.1f dist=%.4f", local[0], c.dist);
    out += nm(c.geom1) + " <-> " + nm(c.geom2) + buf + "; ";
  }
  return out;
}

}  // namespace sim_mujoco
