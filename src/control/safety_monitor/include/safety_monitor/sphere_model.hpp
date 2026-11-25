/**
 * @file sphere_model.hpp
 * @brief The robot as spheres: per-link sphere lists, runtime tool/payload spheres, the
 *        allowed-collision table and the contact-exempt link set.
 * @details The caller supplies one world transform per link (from its own forward kinematics);
 *          this header depends on Eigen only. Self-collision pairs are all link pairs except
 *          (a) a link with itself, (b) parent-child pairs, (c) pairs explicitly allowed (the SRDF
 *          allowed-collision matrix is what a deployment feeds in). Spheres attached at run time
 *          (tool, payload) belong to a link and are tracked by a tag so they can be detached.
 *          `contact_exempt` links (the tool body on its intended work surface) are skipped by the
 *          monitor against obstacle sources flagged as the work surface, and only those.
 *          NOT A CERTIFIED SAFETY FUNCTION, see the package README.
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */

#pragma once

#include <Eigen/Dense>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace safety_monitor
{

using Vec3 = Eigen::Vector3d;

/// A sphere in its link frame.
struct Sphere
{
  Vec3 center{Vec3::Zero()};
  double radius{0.0};
};

/// A sphere in the world frame, with the link and the index inside the link it came from.
struct WorldSphere
{
  Vec3 center{Vec3::Zero()};
  double radius{0.0};
  int link{-1};
  int index_in_link{-1};
};

class SphereModel
{
public:
  /// Adds a link; `parent` is the index of its parent link or -1. Returns the link index
  /// (transforms passed to worldSpheres() are ordered by this index). Throws on a duplicate name
  /// or a parent index that does not exist yet, or a sphere with a non-finite centre or a
  /// negative or non-finite radius (std::invalid_argument).
  int addLink(const std::string & name, int parent, const std::vector<Sphere> & spheres);

  /// Marks a link pair as allowed to touch (no distance check). Throws on an unknown name.
  void allowPair(const std::string & a, const std::string & b);

  /// Marks a link as contact-exempt against the work surface. Throws on an unknown name.
  void setContactExempt(const std::string & link, bool exempt = true);

  /// Attaches a tool/payload sphere to a link under `tag` (several spheres may share a tag). Throws
  /// std::invalid_argument on an unknown link or an invalid sphere (see addLink()).
  void attachSphere(const std::string & link, const std::string & tag, const Sphere & sphere);
  /// Removes every sphere attached under `tag`; returns how many were removed.
  int detachSpheres(const std::string & tag);

  int linkCount() const {return static_cast<int>(links_.size());}
  int linkIndex(const std::string & name) const;   ///< -1 when unknown
  const std::string & linkName(int i) const {return links_.at(i).name;}
  bool isContactExempt(int link) const {return links_.at(link).exempt;}
  int sphereCount() const;

  /// True when the link pair is checked for self-collision.
  bool pairChecked(int a, int b) const;
  /// All checked link pairs (a < b).
  std::vector<std::pair<int, int>> selfPairs() const;

  /// Writes the world spheres of every link, ordered by link then by sphere (model spheres first,
  /// then attached ones in attach order). `transforms` must hold one entry per link. Returns false
  /// (output cleared) on a size mismatch.
  bool worldSpheres(
    const std::vector<Eigen::Isometry3d> & transforms, std::vector<WorldSphere> & out) const;

private:
  struct Attached
  {
    std::string tag;
    Sphere sphere;
  };
  struct Link
  {
    std::string name;
    int parent{-1};
    bool exempt{false};
    std::vector<Sphere> spheres;
    std::vector<Attached> attached;
  };
  std::vector<Link> links_;
  std::set<std::pair<int, int>> allowed_;
};

/// Linear and angular velocity of a link frame, both in the world frame, linear part at the link
/// origin.
struct LinkTwist
{
  Vec3 linear{Vec3::Zero()};
  Vec3 angular{Vec3::Zero()};
};

/// World velocity of every sphere from per-link twists (same ordering as worldSpheres()):
/// v = v_link + w x (c - origin). Returns false on a size mismatch.
bool sphereVelocities(
  const SphereModel & model, const std::vector<Eigen::Isometry3d> & transforms,
  const std::vector<LinkTwist> & twists, std::vector<Vec3> & out);

}  // namespace safety_monitor
