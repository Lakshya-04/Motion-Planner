/**
 * @file distance_source.hpp
 * @brief The DistanceSource port, for "signed distance and gradient to the nearest obstacle" plus three
 *        implementations: static primitives, a dense voxel ESDF grid and moving capsules.
 * @details Distances are signed: positive outside, negative inside the obstacle. The gradient is
 *          the unit direction of increasing distance (pointing away from the obstacle). A source
 *          may answer "no information" (`valid == false`, e.g. a point outside the ESDF grid or an
 *          empty source); the monitor then ignores it for that point.
 *
 *          `query(p, t_ahead)` asks for the obstacle `t_ahead` seconds in the future: static
 *          sources ignore it, the moving-capsule source advances the capsules along their
 *          velocity (constant-velocity prediction). `stamp()` is the time of the last data update;
 *          sources that never change report +infinity (never stale), others the real stamp, and a
 *          negative or NaN stamp is an invalid input to the monitor.
 *
 *          DistanceSource is the seam for adapters over the existing planner and world models (see
 *          the README, "Reuse of the planner models"); no motion_planner or MoveIt dependency here.
 *
 *          The ESDF grid here is self-contained and independent of scene_model; an adapter from
 *          scene_model's ESDF (`computeEsdf`) is a later step.
 *          NOT A CERTIFIED SAFETY FUNCTION, see the package README.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <Eigen/Dense>
#include <limits>
#include <string>
#include <vector>

#include "safety_monitor/sphere_model.hpp"

namespace safety_monitor
{

/// What kind of thing a source represents; selects the margins the monitor applies.
enum class SourceKind {kStatic, kPerson};

struct DistanceQuery
{
  bool valid{false};
  double distance{std::numeric_limits<double>::infinity()};  ///< signed, metres
  Vec3 gradient{Vec3::UnitZ()};                              ///< unit, away from the obstacle
  Vec3 velocity{Vec3::Zero()};                               ///< obstacle velocity at the nearest point
};

class DistanceSource
{
public:
  virtual ~DistanceSource() = default;
  virtual DistanceQuery query(const Vec3 & p, double t_ahead) const = 0;
  virtual SourceKind kind() const {return SourceKind::kStatic;}
  /// True when this source is the intended work surface: contact-exempt links skip it.
  virtual bool isWorkSurface() const {return false;}
  virtual double stamp() const {return std::numeric_limits<double>::infinity();}
};

/// Static primitives: spheres, oriented boxes, half-spaces. The distance is the minimum over all.
class StaticPrimitiveSource : public DistanceSource
{
public:
  explicit StaticPrimitiveSource(bool work_surface = false)
  : work_surface_(work_surface) {}

  void addSphere(const Vec3 & center, double radius);
  /// Box with the given pose (box frame in world) and half extents.
  void addBox(const Eigen::Isometry3d & pose, const Vec3 & half_extents);
  /// Half-space {x : n . x <= offset} is the obstacle (solid below the plane); n is normalised.
  void addPlane(const Vec3 & normal, double offset);

  size_t size() const {return prims_.size();}
  DistanceQuery query(const Vec3 & p, double t_ahead) const override;
  bool isWorkSurface() const override {return work_surface_;}

private:
  struct Prim
  {
    enum Type {kSphere, kBox, kPlane} type;
    Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
    Vec3 a{Vec3::Zero()};  // sphere centre unused (in pose), box half extents, plane normal
    double s{0.0};         // sphere radius, plane offset
  };
  std::vector<Prim> prims_;
  bool work_surface_;
};

/// Dense signed-distance grid with trilinear interpolation. `values[(z*ny + y)*nx + x]` is the
/// signed distance at the voxel centre `origin + voxel * (x, y, z)`. Points outside the box
/// spanned by the first and last voxel centres return `valid == false`.
class EsdfGrid : public DistanceSource
{
public:
  EsdfGrid() = default;
  EsdfGrid(const Vec3 & origin, double voxel, int nx, int ny, int nz);

  /// Fills every voxel from a function of the voxel centre (tests, synthetic maps).
  template<typename F>
  void fill(F && f)
  {
    for (int z = 0; z < nz_; ++z) {
      for (int y = 0; y < ny_; ++y) {
        for (int x = 0; x < nx_; ++x) {
          values_[index(x, y, z)] = static_cast<float>(f(center(x, y, z)));
        }
      }
    }
  }
  void set(int x, int y, int z, float v) {values_[index(x, y, z)] = v;}
  Vec3 center(int x, int y, int z) const {return origin_ + voxel_ * Vec3(x, y, z);}
  /// Time of the last data update (seconds); negative = never set (the monitor fails safe).
  void setStamp(double s) {stamp_ = s;}
  void setDynamic(bool d) {dynamic_ = d;}
  bool empty() const {return values_.empty();}

  DistanceQuery query(const Vec3 & p, double t_ahead) const override;
  double stamp() const override
  {
    return dynamic_ ? stamp_ : std::numeric_limits<double>::infinity();
  }

private:
  size_t index(int x, int y, int z) const
  {
    return (static_cast<size_t>(z) * ny_ + y) * nx_ + x;
  }
  Vec3 origin_{Vec3::Zero()};
  double voxel_{1.0};
  int nx_{0}, ny_{0}, nz_{0};
  std::vector<float> values_;
  double stamp_{-1.0};
  bool dynamic_{false};
};

/// A capsule: segment a-b with a radius and a world velocity.
struct MovingCapsule
{
  Vec3 a{Vec3::Zero()};
  Vec3 b{Vec3::Zero()};
  double radius{0.0};
  Vec3 velocity{Vec3::Zero()};
};

/// A "person" proxy: a list of moving capsules, kind kPerson, predicted at constant velocity.
class MovingCapsuleSource : public DistanceSource
{
public:
  /// Replaces the capsules and stamps the update (seconds).
  void update(const std::vector<MovingCapsule> & capsules, double stamp)
  {
    capsules_ = capsules;
    stamp_ = stamp;
  }
  size_t size() const {return capsules_.size();}
  DistanceQuery query(const Vec3 & p, double t_ahead) const override;
  SourceKind kind() const override {return SourceKind::kPerson;}
  double stamp() const override {return stamp_;}

private:
  std::vector<MovingCapsule> capsules_;
  double stamp_{-1.0};  // never updated: invalid until update() is called
};

}  // namespace safety_monitor
