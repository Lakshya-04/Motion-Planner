#include "robot_self_filter/core/self_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot_self_filter
{
namespace
{

// Radial pad for cylinders uses `a` too, so per-kind fields are kept separate in Padding and
// merged here: sphere_radius pads spheres, cylinder_radius pads cylinders.
struct Active
{
  const Shape * shape;
  std::string link;
  Pose world_from_shape;
  Pose shape_from_world;
  double pad_sphere, pad_cyl_r, pad_cyl_h;
  Vec3 pad_box;
  Vec3 lo, hi;  // world AABB of the inflated shape
};

Vec3 halfExtents(const Shape & s, const Active & a)
{
  switch (s.kind) {
    case Shape::Kind::kBox: return 0.5 * s.size + a.pad_box;
    case Shape::Kind::kSphere: return Vec3::Constant(s.radius + a.pad_sphere);
    case Shape::Kind::kCylinder:
      return Vec3(s.radius + a.pad_cyl_r, s.radius + a.pad_cyl_r, 0.5 * (s.height + a.pad_cyl_h));
  }
  return Vec3::Zero();
}

bool insideLocal(const Shape & s, const Active & a, const Vec3 & q)
{
  switch (s.kind) {
    case Shape::Kind::kSphere: {
        const double r = s.radius + a.pad_sphere;
        return q.squaredNorm() <= r * r;
      }
    case Shape::Kind::kBox: {
        const Vec3 h = 0.5 * s.size + a.pad_box;
        return std::abs(q.x()) <= h.x() && std::abs(q.y()) <= h.y() && std::abs(q.z()) <= h.z();
      }
    case Shape::Kind::kCylinder: {
        const double r = s.radius + a.pad_cyl_r;
        return q.head<2>().squaredNorm() <= r * r && std::abs(q.z()) <= 0.5 * (s.height + a.pad_cyl_h);
      }
  }
  return false;
}

// Interval of t in [0,1] where a + t d lies in |x| <= h along one slab axis. Returns false if
// empty. Narrows [t0, t1].
bool slab(double a, double d, double h, double & t0, double & t1)
{
  if (std::abs(d) < 1e-12) {
    return std::abs(a) <= h;
  }
  double lo = (-h - a) / d, hi = (h - a) / d;
  if (lo > hi) {std::swap(lo, hi);}
  t0 = std::max(t0, lo);
  t1 = std::min(t1, hi);
  return t0 <= t1;
}

bool segmentLocal(const Shape & s, const Active & act, const Vec3 & a, const Vec3 & b)
{
  const Vec3 d = b - a;
  double t0 = 0.0, t1 = 1.0;
  switch (s.kind) {
    case Shape::Kind::kBox: {
        const Vec3 h = 0.5 * s.size + act.pad_box;
        return slab(a.x(), d.x(), h.x(), t0, t1) && slab(a.y(), d.y(), h.y(), t0, t1) &&
               slab(a.z(), d.z(), h.z(), t0, t1);
      }
    case Shape::Kind::kSphere: {
        const double r = s.radius + act.pad_sphere;
        const double A = d.squaredNorm(), B = 2.0 * a.dot(d), C = a.squaredNorm() - r * r;
        if (A < 1e-18) {return C <= 0.0;}
        const double disc = B * B - 4.0 * A * C;
        if (disc < 0.0) {return false;}
        const double sq = std::sqrt(disc);
        const double lo = (-B - sq) / (2.0 * A), hi = (-B + sq) / (2.0 * A);
        return std::max(lo, 0.0) <= std::min(hi, 1.0);
      }
    case Shape::Kind::kCylinder: {
        const double r = s.radius + act.pad_cyl_r;
        const double hz = 0.5 * (s.height + act.pad_cyl_h);
        if (!slab(a.z(), d.z(), hz, t0, t1)) {return false;}
        const double A = d.head<2>().squaredNorm(), B = 2.0 * a.head<2>().dot(d.head<2>());
        const double C = a.head<2>().squaredNorm() - r * r;
        if (A < 1e-18) {return C <= 0.0;}
        const double disc = B * B - 4.0 * A * C;
        if (disc < 0.0) {return false;}
        const double sq = std::sqrt(disc);
        t0 = std::max(t0, (-B - sq) / (2.0 * A));
        t1 = std::min(t1, (-B + sq) / (2.0 * A));
        return t0 <= t1;
      }
  }
  return false;
}

Active makeActive(const Shape & s, const std::string & link, const Pose & link_pose,
  const Padding & pad, double extra)
{
  Active a;
  a.shape = &s;
  a.link = link;
  a.world_from_shape = link_pose * s.local;
  a.shape_from_world = a.world_from_shape.inverse();
  a.pad_sphere = pad.sphere_radius + extra;
  a.pad_cyl_r = pad.cylinder_radius + extra;
  a.pad_cyl_h = pad.cylinder_height + 2.0 * extra;
  a.pad_box = pad.box + Vec3::Constant(extra);
  // Conservative world AABB from the local half extents.
  const Vec3 h = halfExtents(s, a);
  const Vec3 c = a.world_from_shape.translation();
  const Vec3 ext = a.world_from_shape.linear().cwiseAbs() * h;
  a.lo = c - ext;
  a.hi = c + ext;
  return a;
}

}  // namespace

bool pointInShape(const Shape & s, const Pose & world_from_link, const Vec3 & p,
  double pad_a, const Vec3 & pad_box, double pad_h)
{
  Padding pad;
  pad.sphere_radius = pad_a;
  pad.cylinder_radius = pad_a;
  pad.box = pad_box;
  pad.cylinder_height = pad_h;
  const Active a = makeActive(s, "", world_from_link, pad, 0.0);
  return insideLocal(s, a, a.shape_from_world * p);
}

bool segmentTouchesShape(const Shape & s, const Pose & world_from_link, const Vec3 & a, const Vec3 & b,
  double pad_a, const Vec3 & pad_box, double pad_h)
{
  Padding pad;
  pad.sphere_radius = pad_a;
  pad.cylinder_radius = pad_a;
  pad.box = pad_box;
  pad.cylinder_height = pad_h;
  const Active act = makeActive(s, "", world_from_link, pad, 0.0);
  return segmentLocal(s, act, act.shape_from_world * a, act.shape_from_world * b);
}

SelfFilter::SelfFilter(std::vector<LinkShapes> shapes, SelfFilterConfig config)
: shapes_(std::move(shapes)), config_(std::move(config)) {}

SelfFilterResult SelfFilter::filter(const SelfFilterInput & in) const
{
  SelfFilterResult out;
  if (in.cloud == nullptr) {
    out.message = "no cloud";
    return out;
  }
  if ((config_.shadow || config_.min_sensor_distance > 0.0) && !in.sensor_origin) {
    out.message = "shadow / min_sensor_distance need the sensor origin in the root frame";
    return out;
  }

  // Shapes of the links that count as "self", placed at their current poses.
  std::vector<Active> active;
  for (const auto & ls : shapes_) {
    if (!config_.links.empty() && !config_.links.count(ls.link)) {
      continue;
    }
    if (ls.shapes.empty()) {
      continue;
    }
    const auto pose = in.link_poses.find(ls.link);
    if (pose == in.link_poses.end()) {
      out.message = "no pose for link '" + ls.link + "'";
      return out;
    }
    Padding pad = config_.padding;
    const auto over = config_.link_padding.find(ls.link);
    if (over != config_.link_padding.end()) {
      pad = over->second;
    }
    double extra = 0.0;
    const auto speed = in.link_speed.find(ls.link);
    if (speed != in.link_speed.end()) {
      extra = config_.speed_padding_gain * std::abs(speed->second);
    }
    if (in.contact_mode && !config_.contact_link.empty() && ls.link == config_.contact_link) {
      pad = Padding{config_.contact_padding, Vec3::Constant(config_.contact_padding),
        config_.contact_padding, config_.contact_padding};
      extra = 0.0;
    }
    for (const auto & shape : ls.shapes) {
      active.push_back(makeActive(shape, ls.link, pose->second, pad, extra));
    }
  }

  const auto & cloud = *in.cloud;
  const size_t n = cloud.points.size();
  out.removed.assign(n, false);
  const bool has_labels = cloud.labels.size() == n;
  const bool has_colors = cloud.colors.size() == n;

  for (size_t i = 0; i < n; ++i) {
    const Vec3 p = in.cloud_to_root * cloud.points[i];
    std::string reason;

    if (config_.min_sensor_distance > 0.0 && (p - *in.sensor_origin).norm() < config_.min_sensor_distance) {
      reason = "<min_distance>";
    }
    if (reason.empty()) {
      for (const auto & a : active) {
        if ((p.array() < a.lo.array()).any() || (p.array() > a.hi.array()).any()) {
          // Outside the AABB: not contained, but a shadow ray may still pass through the shape.
          if (!config_.shadow) {
            continue;
          }
        } else if (insideLocal(*a.shape, a, a.shape_from_world * p)) {
          reason = a.link;
          break;
        }
        if (config_.shadow &&
          segmentLocal(*a.shape, a, a.shape_from_world * *in.sensor_origin, a.shape_from_world * p))
        {
          reason = a.link;
          break;
        }
      }
    }

    if (!reason.empty()) {
      out.removed[i] = true;
      ++out.removed_count;
      ++out.removed_per_link[reason];
    } else {
      out.filtered.points.push_back(cloud.points[i]);
      if (has_labels) {out.filtered.labels.push_back(cloud.labels[i]);}
      if (has_colors) {out.filtered.colors.push_back(cloud.colors[i]);}
    }
  }
  out.success = true;
  out.message = "ok";
  return out;
}

}  // namespace robot_self_filter
