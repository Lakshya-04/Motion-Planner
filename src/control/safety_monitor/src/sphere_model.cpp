/**
 * @file sphere_model.cpp
 * @author Lakshya Agarwal <100lakshyaagarwal@gmail.com>
 */
#include "safety_monitor/sphere_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace safety_monitor
{

namespace
{
void checkSphere(const Sphere & s, const std::string & where)
{
  if (!s.center.allFinite() || !std::isfinite(s.radius) || s.radius < 0.0) {
    throw std::invalid_argument(
      "SphereModel: non-finite centre or negative/non-finite radius on " + where);
  }
}
}  // namespace

int SphereModel::addLink(const std::string & name, int parent, const std::vector<Sphere> & spheres)
{
  if (linkIndex(name) >= 0) {
    throw std::invalid_argument("SphereModel: duplicate link " + name);
  }
  if (parent < -1 || parent >= linkCount()) {
    throw std::invalid_argument("SphereModel: unknown parent of " + name);
  }
  for (const auto & s : spheres) {checkSphere(s, name);}
  Link l;
  l.name = name;
  l.parent = parent;
  l.spheres = spheres;
  links_.push_back(std::move(l));
  return linkCount() - 1;
}

int SphereModel::linkIndex(const std::string & name) const
{
  for (int i = 0; i < linkCount(); ++i) {
    if (links_[i].name == name) {return i;}
  }
  return -1;
}

void SphereModel::allowPair(const std::string & a, const std::string & b)
{
  const int ia = linkIndex(a);
  const int ib = linkIndex(b);
  if (ia < 0 || ib < 0) {
    throw std::invalid_argument("SphereModel: allowPair on unknown link");
  }
  allowed_.insert({std::min(ia, ib), std::max(ia, ib)});
}

void SphereModel::setContactExempt(const std::string & link, bool exempt)
{
  const int i = linkIndex(link);
  if (i < 0) {throw std::invalid_argument("SphereModel: unknown link " + link);}
  links_[i].exempt = exempt;
}

void SphereModel::attachSphere(const std::string & link, const std::string & tag, const Sphere & s)
{
  const int i = linkIndex(link);
  if (i < 0) {throw std::invalid_argument("SphereModel: unknown link " + link);}
  checkSphere(s, link + "/" + tag);
  links_[i].attached.push_back({tag, s});
}

int SphereModel::detachSpheres(const std::string & tag)
{
  int n = 0;
  for (auto & l : links_) {
    const auto before = l.attached.size();
    l.attached.erase(
      std::remove_if(
        l.attached.begin(), l.attached.end(), [&](const Attached & a) {return a.tag == tag;}),
      l.attached.end());
    n += static_cast<int>(before - l.attached.size());
  }
  return n;
}

int SphereModel::sphereCount() const
{
  int n = 0;
  for (const auto & l : links_) {n += static_cast<int>(l.spheres.size() + l.attached.size());}
  return n;
}

bool SphereModel::pairChecked(int a, int b) const
{
  if (a == b || a < 0 || b < 0 || a >= linkCount() || b >= linkCount()) {return false;}
  if (links_[a].parent == b || links_[b].parent == a) {return false;}
  return allowed_.count({std::min(a, b), std::max(a, b)}) == 0;
}

std::vector<std::pair<int, int>> SphereModel::selfPairs() const
{
  std::vector<std::pair<int, int>> out;
  for (int a = 0; a < linkCount(); ++a) {
    for (int b = a + 1; b < linkCount(); ++b) {
      if (pairChecked(a, b)) {out.emplace_back(a, b);}
    }
  }
  return out;
}

bool SphereModel::worldSpheres(
  const std::vector<Eigen::Isometry3d> & transforms, std::vector<WorldSphere> & out) const
{
  out.clear();
  if (static_cast<int>(transforms.size()) != linkCount()) {return false;}
  out.reserve(sphereCount());
  for (int i = 0; i < linkCount(); ++i) {
    int k = 0;
    for (const auto & s : links_[i].spheres) {
      out.push_back({transforms[i] * s.center, s.radius, i, k++});
    }
    for (const auto & a : links_[i].attached) {
      out.push_back({transforms[i] * a.sphere.center, a.sphere.radius, i, k++});
    }
  }
  return true;
}

bool sphereVelocities(
  const SphereModel & model, const std::vector<Eigen::Isometry3d> & transforms,
  const std::vector<LinkTwist> & twists, std::vector<Vec3> & out)
{
  out.clear();
  std::vector<WorldSphere> ws;
  if (!model.worldSpheres(transforms, ws) || static_cast<int>(twists.size()) != model.linkCount()) {
    return false;
  }
  out.reserve(ws.size());
  for (const auto & s : ws) {
    const auto & tw = twists[s.link];
    out.push_back(tw.linear + tw.angular.cross(s.center - transforms[s.link].translation()));
  }
  return true;
}

}  // namespace safety_monitor
