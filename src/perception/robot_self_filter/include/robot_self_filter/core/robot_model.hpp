// A robot as the self filter needs it: the collision shapes of each link and the kinematics to
// place them. Parsed from URDF text with tinyxml2, so it needs neither ROS nor MoveIt and behaves
// the same on every distro.
#pragma once

#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace robot_self_filter
{

using Vec3 = Eigen::Vector3d;
using Pose = Eigen::Isometry3d;
using JointPositions = std::map<std::string, double>;
using LinkPoses = std::map<std::string, Pose>;

struct Shape
{
  enum class Kind {kBox, kSphere, kCylinder};
  Kind kind{Kind::kBox};
  Vec3 size{Vec3::Zero()};  ///< Box: full extents.
  double radius{0.0};       ///< Sphere / cylinder.
  double height{0.0};       ///< Cylinder, along local z.
  Pose local{Pose::Identity()};  ///< Shape frame in the link frame.
};

struct LinkShapes
{
  std::string link;
  std::vector<Shape> shapes;
};

/// Axis-aligned bounds of a mesh file, used to stand in for a mesh collision shape (a box: it
/// over-approximates, which is the safe direction for filtering). Return nullopt if unknown.
struct MeshBounds
{
  Vec3 min{Vec3::Zero()};
  Vec3 max{Vec3::Zero()};
};
using MeshBoundsFn = std::function<std::optional<MeshBounds>(const std::string & uri)>;

class UrdfRobot
{
public:
  /// Parses URDF text. Mesh collision shapes become their bounding box when `mesh_bounds`
  /// resolves them (scaled by the mesh's scale attribute); otherwise they are skipped and listed
  /// in skippedMeshes(). Throws std::runtime_error for malformed URDF or unsupported joints
  /// (floating, planar).
  static UrdfRobot fromUrdf(const std::string & urdf_xml, const MeshBoundsFn & mesh_bounds = nullptr);

  const std::string & rootLink() const {return root_;}
  const std::vector<LinkShapes> & shapes() const {return shapes_;}
  const std::vector<std::string> & skippedMeshes() const {return skipped_meshes_;}
  const std::vector<std::string> & jointNames() const;
  bool hasLink(const std::string & name) const {return link_index_.count(name) > 0;}

  /// Pose of every link in the root link frame. Joints missing from `q` are at 0; mimic joints
  /// follow their master.
  LinkPoses linkPoses(const JointPositions & q) const;

private:
  struct Joint
  {
    std::string name, parent, child;
    enum class Type {kFixed, kRevolute, kContinuous, kPrismatic} type{Type::kFixed};
    Pose origin{Pose::Identity()};
    Vec3 axis{Vec3::UnitX()};
    std::string mimic_joint;
    double mimic_multiplier{1.0};
    double mimic_offset{0.0};
  };

  std::string root_;
  std::vector<LinkShapes> shapes_;
  std::vector<std::string> skipped_meshes_;
  std::vector<Joint> joints_;          // ordered parent before child
  std::map<std::string, int> link_index_;
  mutable std::vector<std::string> joint_names_;
};

}  // namespace robot_self_filter
