#pragma once

#include <optional>
#include <string>
#include <vector>

#include "scene_model/core/types.hpp"

namespace scene_model
{

/// Container for everything known about the world, in one frame. Insertion order is kept so
/// output is deterministic.
class SceneModel
{
public:
  explicit SceneModel(std::string frame_id = "") : frame_id_(std::move(frame_id)) {}

  const std::string & frameId() const { return frame_id_; }
  void setFrameId(std::string frame_id) { frame_id_ = std::move(frame_id); }

  /// Adds an object. Returns false (and changes nothing) if the id is empty or already taken.
  bool add(SceneObject object);
  /// Adds or replaces by id.
  void upsert(SceneObject object);
  bool remove(const std::string & id);
  void clear() { objects_.clear(); }

  const SceneObject * find(const std::string & id) const;
  const std::vector<SceneObject> & objects() const { return objects_; }
  size_t size() const { return objects_.size(); }
  bool empty() const { return objects_.empty(); }

  /// Objects with the given role, in insertion order.
  std::vector<const SceneObject *> withRole(Role role) const;
  /// Drops every non-static object (e.g. before re-sensing the dynamic part of the world).
  void removeDynamic();

  /// Axis-aligned bounds of everything, in the scene frame (object poses applied; objects whose
  /// frame differs from the scene frame are assumed already expressed in it).
  Aabb bounds() const;

  /// Merge another scene in. Objects with a clashing id are renamed `<id>_<n>`.
  void merge(const SceneModel & other);

private:
  std::string frame_id_;
  std::vector<SceneObject> objects_;
};

/// Bounds of one object's geometry after applying its pose.
Aabb objectBounds(const SceneObject & object);

/// Human-readable type name of the geometry ("box", "mesh", ...).
const char * geometryName(const Geometry & geometry);

}  // namespace scene_model
