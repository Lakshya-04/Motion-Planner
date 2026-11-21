// Test-only generic serial chain from standard DH rows (no library code depends on it).
#pragma once

#include <Eigen/Dense>

#include <cmath>
#include <vector>

namespace chain_test
{

struct Joint
{
  double a, alpha, d, theta_off;
  bool prismatic;
};

class Chain
{
public:
  explicit Chain(std::vector<Joint> j)
  : joints_(std::move(j)) {}
  int dof() const {return static_cast<int>(joints_.size());}

  static Eigen::Isometry3d dh(const Joint & j, double q)
  {
    const double th = j.prismatic ? j.theta_off : j.theta_off + q;
    const double d = j.prismatic ? j.d + q : j.d;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(Eigen::AngleAxisd(th, Eigen::Vector3d::UnitZ()));
    T.translate(Eigen::Vector3d(0, 0, d));
    T.translate(Eigen::Vector3d(j.a, 0, 0));
    T.rotate(Eigen::AngleAxisd(j.alpha, Eigen::Vector3d::UnitX()));
    return T;
  }

  Eigen::Isometry3d fk(const Eigen::VectorXd & q) const
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    for (int i = 0; i < dof(); ++i) {T = T * dh(joints_[static_cast<size_t>(i)], q(i));}
    return T;
  }

  // Geometric Jacobian in the base frame (linear rows 0-2, angular rows 3-5).
  Eigen::MatrixXd jacobian(const Eigen::VectorXd & q) const
  {
    Eigen::MatrixXd J(6, dof());
    std::vector<Eigen::Vector3d> z(static_cast<size_t>(dof())), p(static_cast<size_t>(dof()));
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    for (int i = 0; i < dof(); ++i) {
      z[static_cast<size_t>(i)] = T.linear().col(2);
      p[static_cast<size_t>(i)] = T.translation();
      T = T * dh(joints_[static_cast<size_t>(i)], q(i));
    }
    const Eigen::Vector3d pe = T.translation();
    for (int i = 0; i < dof(); ++i) {
      const auto zi = z[static_cast<size_t>(i)];
      if (joints_[static_cast<size_t>(i)].prismatic) {
        J.block<3, 1>(0, i) = zi;
        J.block<3, 1>(3, i).setZero();
      } else {
        J.block<3, 1>(0, i) = zi.cross(pe - p[static_cast<size_t>(i)]);
        J.block<3, 1>(3, i) = zi;
      }
    }
    return J;
  }

  // Finite-difference Jacobian, used to validate jacobian().
  Eigen::MatrixXd numericJacobian(const Eigen::VectorXd & q, double h = 1e-6) const
  {
    Eigen::MatrixXd J(6, dof());
    const Eigen::Isometry3d T0 = fk(q);
    for (int i = 0; i < dof(); ++i) {
      Eigen::VectorXd qp = q;
      qp(i) += h;
      const Eigen::Isometry3d T1 = fk(qp);
      J.block<3, 1>(0, i) = (T1.translation() - T0.translation()) / h;
      const Eigen::AngleAxisd r(T1.linear() * T0.linear().transpose());
      J.block<3, 1>(3, i) = r.axis() * r.angle() / h;
    }
    return J;
  }

private:
  std::vector<Joint> joints_;
};

inline Chain planar(const std::vector<double> & lengths)
{
  std::vector<Joint> j;
  for (double l : lengths) {j.push_back({l, 0.0, 0.0, 0.0, false});}
  return Chain(j);
}

inline Chain arm6()
{
  const double h = M_PI / 2;
  return Chain({{0, h, 0.1625, 0, false}, {-0.425, 0, 0, 0, false}, {-0.3922, 0, 0, 0, false},
      {0, h, 0.1333, 0, false}, {0, -h, 0.0997, 0, false}, {0, 0, 0.0996, 0, false}});
}

inline Chain arm7()
{
  const double h = M_PI / 2;
  return Chain({{0, -h, 0.30, 0, false}, {0, h, 0, 0, false}, {0.05, -h, 0.30, 0, false},
      {-0.05, h, 0, 0, false}, {0, -h, 0.30, 0, false}, {0.08, h, 0, 0, false},
      {0, 0, 0.12, 0, false}});
}

inline Chain liftArm6()
{
  std::vector<Joint> j = {{0, 0, 0.0, 0, true}};
  const double h = M_PI / 2;
  const std::vector<Joint> r = {{0, h, 0.1625, 0, false}, {-0.425, 0, 0, 0, false},
    {-0.3922, 0, 0, 0, false}, {0, h, 0.1333, 0, false}, {0, -h, 0.0997, 0, false},
    {0, 0, 0.0996, 0, false}};
  j.insert(j.end(), r.begin(), r.end());
  return Chain(j);
}

inline Eigen::VectorXd vec(std::initializer_list<double> v)
{
  Eigen::VectorXd o(static_cast<Eigen::Index>(v.size()));
  Eigen::Index i = 0;
  for (double x : v) {o(i++) = x;}
  return o;
}

}  // namespace chain_test
