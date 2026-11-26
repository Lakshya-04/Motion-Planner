#include "wall_detector/core/prior_map.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "scene_model/core/io.hpp"
#include "wall_detector/core/wall_detection.hpp"

namespace wall_detector
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

[[noreturn]] void fail(const std::string & why)
{
  throw std::runtime_error("prior map: " + why);
}

std::string extensionOf(const std::string & path)
{
  const auto dot = path.find_last_of('.');
  std::string ext = dot == std::string::npos ? "" : path.substr(dot + 1);
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {return std::tolower(c);});
  return ext;
}

std::string readFile(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error(path + ": cannot open file");
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

Vec3 frameU(const Vec3 & normal, const Vec3 & up)
{
  Vec3 u = up.cross(normal);
  if (u.norm() < 1e-6) {
    u = Vec3::UnitX() - normal * normal.dot(Vec3::UnitX());
    if (u.norm() < 1e-6) {
      u = Vec3::UnitY() - normal * normal.dot(Vec3::UnitY());
    }
  }
  return u.normalized();
}

Vec3 vec3From(const nlohmann::json & j, const std::string & what)
{
  if (!j.is_array() || j.size() != 3) {
    fail(what + " must be a list of 3 numbers");
  }
  Vec3 v;
  for (int i = 0; i < 3; ++i) {
    if (!j[i].is_number()) {
      fail(what + " must hold numbers");
    }
    v[i] = j[i].get<double>();
    if (!std::isfinite(v[i])) {
      fail(what + " must be finite");
    }
  }
  return v;
}

std::vector<Vec3> polygonFrom(const nlohmann::json & j, const std::string & what)
{
  if (!j.is_array() || j.size() < 3) {
    fail(what + " needs at least 3 vertices");
  }
  std::vector<Vec3> poly;
  for (size_t i = 0; i < j.size(); ++i) {
    poly.push_back(vec3From(j[i], what + "[" + std::to_string(i) + "]"));
  }
  return poly;
}

/// Newell's method: area vector of a (planar) polygon; its direction follows the right-hand rule
/// of the vertex order.
Vec3 areaVector(const std::vector<Vec3> & p)
{
  Vec3 sum = Vec3::Zero();
  for (size_t i = 0; i < p.size(); ++i) {
    sum += p[i].cross(p[(i + 1) % p.size()]);
  }
  return 0.5 * sum;
}

Vec3 centroidOf(const std::vector<Vec3> & p)
{
  Vec3 c = Vec3::Zero();
  for (const auto & q : p) {c += q;}
  return c / static_cast<double>(p.size());
}

void orientToward(PriorSurface & s, const PriorMapParams & params, std::vector<std::string> & warnings)
{
  if (params.robot_position && (*params.robot_position - s.origin).dot(s.normal) < 0.0) {
    s.normal = -s.normal;
    warnings.push_back("surface '" + s.id + "': normal flipped to face the robot");
  }
}

}  // namespace

// ---------------------------------------------------------------------------------- JSON

PriorMap parsePriorSurfacesJson(const std::string & text, const PriorMapParams & params)
{
  nlohmann::json root;
  try {
    root = nlohmann::json::parse(text);
  } catch (const nlohmann::json::exception & e) {
    fail(std::string("cannot parse JSON: ") + e.what());
  }
  if (!root.is_object()) {
    fail("top level must be an object");
  }
  if (root.value("format", std::string("prior_surfaces")) != "prior_surfaces") {
    fail("'format' must be \"prior_surfaces\"");
  }
  if (root.contains("units") && root["units"] != "meters") {
    fail("'units' must be \"meters\" (or omitted)");
  }
  if (!root.contains("surfaces") || !root["surfaces"].is_array()) {
    fail("'surfaces' must be a list");
  }
  PriorMap map;
  map.frame_id = root.value("frame_id", params.frame_id);
  if (map.frame_id.empty()) {
    fail("frame_id is empty");
  }

  auto taken = [](const auto & items, const std::string & id) {
      return std::any_of(items.begin(), items.end(), [&](const auto & i) {return i.id == id;});
    };

  for (const auto & sj : root["surfaces"]) {
    if (!sj.is_object() || !sj.contains("id") || !sj["id"].is_string() || sj["id"] == "") {
      fail("every surface needs a non-empty string 'id'");
    }
    PriorSurface s;
    s.id = sj["id"].get<std::string>();
    if (taken(map.surfaces, s.id)) {
      fail("duplicate surface id '" + s.id + "'");
    }
    const std::string where = "surface '" + s.id + "'";
    if (!sj.contains("polygon")) {
      fail(where + " has no polygon");
    }
    s.polygon = polygonFrom(sj["polygon"], where + " polygon");
    const Vec3 area = areaVector(s.polygon);
    if (area.norm() < 1e-6) {
      fail(where + " polygon is degenerate (zero area)");
    }
    Vec3 normal = area.normalized();  // right-hand rule of the vertex order
    if (sj.contains("normal")) {
      const Vec3 given = vec3From(sj["normal"], where + " normal");
      if (given.norm() < 1e-9) {
        fail(where + " normal is zero");
      }
      if (std::abs(given.normalized().dot(normal)) < std::cos(params.normal_tol_deg * kPi / 180.0)) {
        fail(where + " normal does not match the polygon's plane");
      }
      normal = given.normalized().dot(normal) < 0.0 ? -normal : normal;
    }
    s.normal = normal;
    s.origin = centroidOf(s.polygon);
    for (const auto & p : s.polygon) {
      if (std::abs((p - s.origin).dot(s.normal)) > params.coplanar_tol_m) {
        fail(where + " polygon is not planar (vertex " +
          std::to_string((p - s.origin).dot(s.normal)) + " m off the plane)");
      }
    }
    orientToward(s, params, map.warnings);

    if (sj.contains("openings")) {
      if (!sj["openings"].is_array()) {
        fail(where + " openings must be a list");
      }
      const SurfaceFrame f = surfaceFrame(s, params.up);
      for (const auto & oj : sj["openings"]) {
        PriorOpening o;
        o.kind = oj.value("kind", std::string("opening"));
        if (!oj.is_object() || !oj.contains("polygon")) {
          fail(where + " opening has no polygon");
        }
        o.polygon = polygonFrom(oj["polygon"], where + " opening polygon");
        if (areaVector(o.polygon).norm() < 1e-6) {
          fail(where + " opening polygon is degenerate");
        }
        for (const auto & p : o.polygon) {
          const Vec3 d = p - f.origin;
          if (std::abs(d.dot(s.normal)) > params.coplanar_tol_m) {
            fail(where + " opening '" + o.kind + "' is not in the surface's plane");
          }
          const double pu = d.dot(f.u), pv = d.dot(f.v);
          const double tol = params.coplanar_tol_m;
          if (pu < f.u_min - tol || pu > f.u_max + tol || pv < f.v_min - tol || pv > f.v_max + tol) {
            fail(where + " opening '" + o.kind + "' extends outside the surface");
          }
        }
        s.openings.push_back(std::move(o));
      }
    }
    map.surfaces.push_back(std::move(s));
  }

  if (root.contains("obstacles")) {
    if (!root["obstacles"].is_array()) {
      fail("'obstacles' must be a list");
    }
    for (const auto & bj : root["obstacles"]) {
      if (!bj.is_object() || !bj.contains("id") || !bj["id"].is_string() || bj["id"] == "") {
        fail("every obstacle needs a non-empty string 'id'");
      }
      PriorBox b;
      b.id = bj["id"].get<std::string>();
      if (taken(map.obstacles, b.id)) {
        fail("duplicate obstacle id '" + b.id + "'");
      }
      if (!bj.contains("center") || !bj.contains("size")) {
        fail("obstacle '" + b.id + "' needs center and size");
      }
      b.center = vec3From(bj["center"], "obstacle center");
      b.size = vec3From(bj["size"], "obstacle size");
      if ((b.size.array() <= 0.0).any()) {
        fail("obstacle '" + b.id + "' size must be positive");
      }
      b.yaw_deg = bj.value("yaw_deg", 0.0);
      if (!std::isfinite(b.yaw_deg)) {
        fail("obstacle '" + b.id + "' yaw_deg must be finite");
      }
      map.obstacles.push_back(std::move(b));
    }
  }
  return map;
}

PriorMap loadPriorSurfacesJson(const std::string & path, const PriorMapParams & params)
{
  const std::string text = readFile(path);
  try {
    return parsePriorSurfacesJson(text, params);
  } catch (const std::exception & e) {
    throw std::runtime_error(path + ": " + e.what());
  }
}

// ---------------------------------------------------------------------------------- PLY

scene_model::Mesh parsePlyMesh(const std::string & text, const std::string & source)
{
  std::istringstream in(text);
  std::string line;
  auto bad = [&](const std::string & why) {fail("PLY " + source + ": " + why);};
  if (!std::getline(in, line) || line.substr(0, 3) != "ply") {
    bad("missing 'ply' magic line");
  }
  size_t n_vertex = 0, n_face = 0;
  int ix = -1, iy = -1, iz = -1, vertex_props = 0;
  bool ascii = false, in_vertex = false, in_face = false, header_done = false, face_list = false;
  while (std::getline(in, line)) {
    std::istringstream ls(line);
    std::string tag;
    ls >> tag;
    if (tag == "format") {
      std::string fmt;
      ls >> fmt;
      if (fmt != "ascii") {
        bad("only ASCII PLY is supported (format is '" + fmt + "')");
      }
      ascii = true;
    } else if (tag == "element") {
      std::string name;
      long long count = -1;
      ls >> name >> count;
      if (count < 0) {bad("bad element line: " + line);}
      in_vertex = name == "vertex";
      in_face = name == "face";
      if (in_vertex) {n_vertex = static_cast<size_t>(count);}
      if (in_face) {n_face = static_cast<size_t>(count);}
    } else if (tag == "property") {
      std::string type, name;
      ls >> type;
      if (in_vertex) {
        ls >> name;
        if (type == "list") {bad("list property on vertex");}
        if (name == "x") {ix = vertex_props;}
        if (name == "y") {iy = vertex_props;}
        if (name == "z") {iz = vertex_props;}
        ++vertex_props;
      } else if (in_face && type == "list") {
        face_list = true;
      }
    } else if (tag == "end_header") {
      header_done = true;
      break;
    }
  }
  if (!header_done) {bad("no end_header");}
  if (!ascii) {bad("no format line");}
  if (ix < 0 || iy < 0 || iz < 0) {bad("vertex has no x, y, z");}
  if (n_vertex == 0 || n_face == 0 || !face_list) {bad("needs vertices and faces with a vertex list");}

  scene_model::Mesh mesh;
  mesh.source_path = source;
  for (size_t i = 0; i < n_vertex; ++i) {
    if (!std::getline(in, line)) {bad("file ends inside the vertices");}
    std::istringstream ls(line);
    std::vector<double> vals;
    for (double d; ls >> d;) {vals.push_back(d);}
    if (static_cast<int>(vals.size()) < vertex_props) {bad("short vertex line: " + line);}
    const Vec3 p(vals[ix], vals[iy], vals[iz]);
    if (!p.allFinite()) {bad("non-finite vertex");}
    mesh.vertices.push_back(p);
  }
  for (size_t i = 0; i < n_face; ++i) {
    if (!std::getline(in, line)) {bad("file ends inside the faces");}
    std::istringstream ls(line);
    long long n = 0;
    if (!(ls >> n) || n < 3) {bad("face with fewer than 3 vertices: " + line);}
    std::vector<int> idx;
    for (long long k = 0; k < n; ++k) {
      long long v;
      if (!(ls >> v)) {bad("short face line: " + line);}
      if (v < 0 || v >= static_cast<long long>(n_vertex)) {bad("face index out of range");}
      idx.push_back(static_cast<int>(v));
    }
    for (size_t k = 1; k + 1 < idx.size(); ++k) {
      mesh.faces.push_back({idx[0], idx[k], idx[k + 1]});
    }
  }
  return mesh;
}

scene_model::Mesh loadPlyMesh(const std::string & path)
{
  const std::string text = readFile(path);
  try {
    return parsePlyMesh(text, path);
  } catch (const std::exception & e) {
    throw std::runtime_error(path + ": " + e.what());
  }
}

// ---------------------------------------------------------------------------------- mesh -> planes

SurfaceFrame surfaceFrame(const PriorSurface & surface, const Vec3 & up)
{
  SurfaceFrame f;
  f.origin = surface.origin;
  f.normal = surface.normal;
  f.u = frameU(surface.normal, up);
  f.v = surface.normal.cross(f.u);
  f.u_min = f.v_min = std::numeric_limits<double>::infinity();
  f.u_max = f.v_max = -std::numeric_limits<double>::infinity();
  for (const auto & p : surface.polygon) {
    const Vec3 d = p - f.origin;
    f.u_min = std::min(f.u_min, d.dot(f.u));
    f.u_max = std::max(f.u_max, d.dot(f.u));
    f.v_min = std::min(f.v_min, d.dot(f.v));
    f.v_max = std::max(f.v_max, d.dot(f.v));
  }
  return f;
}

PriorMap priorMapFromMesh(
  const scene_model::Mesh & mesh_in, const PriorMapParams & params, const std::string & id_prefix)
{
  if (mesh_in.faces.empty() || mesh_in.vertices.empty()) {
    fail("mesh has no faces");
  }
  if (!(params.mesh_scale > 0.0)) {
    fail("mesh_scale must be positive");
  }
  PriorMap map;
  map.frame_id = params.frame_id;

  // Weld vertices so triangles that share a corner share an index.
  const double q = std::max(params.weld_tol_m, 1e-9);
  std::map<std::array<long long, 3>, int> welded;
  std::vector<Vec3> verts;
  std::vector<int> remap(mesh_in.vertices.size());
  for (size_t i = 0; i < mesh_in.vertices.size(); ++i) {
    const Vec3 p = mesh_in.vertices[i] * params.mesh_scale;
    if (!p.allFinite()) {fail("mesh has a non-finite vertex");}
    const std::array<long long, 3> key{
      std::llround(p.x() / q), std::llround(p.y() / q), std::llround(p.z() / q)};
    const auto it = welded.find(key);
    if (it == welded.end()) {
      welded.emplace(key, static_cast<int>(verts.size()));
      remap[i] = static_cast<int>(verts.size());
      verts.push_back(p);
    } else {
      remap[i] = it->second;
    }
  }

  struct Cluster
  {
    Vec3 normal_sum{Vec3::Zero()};
    Vec3 normal{Vec3::Zero()};
    double offset{0.0};
    double area{0.0};
    std::vector<size_t> tris;
  };
  std::vector<std::array<int, 3>> tris;
  std::vector<Cluster> clusters;
  const double cos_tol = std::cos(params.normal_tol_deg * kPi / 180.0);
  for (const auto & f : mesh_in.faces) {
    for (int k : f) {
      if (k < 0 || k >= static_cast<int>(remap.size())) {fail("face index out of range");}
    }
    const std::array<int, 3> t{remap[f[0]], remap[f[1]], remap[f[2]]};
    const Vec3 cr = (verts[t[1]] - verts[t[0]]).cross(verts[t[2]] - verts[t[0]]);
    const double area = 0.5 * cr.norm();
    if (area < 1e-9) {continue;}  // degenerate sliver
    const Vec3 n = cr.normalized();
    const Vec3 c = (verts[t[0]] + verts[t[1]] + verts[t[2]]) / 3.0;
    Cluster * home = nullptr;
    for (auto & cl : clusters) {
      const double d = n.dot(cl.normal);
      if (std::abs(d) >= cos_tol && std::abs(c.dot(cl.normal) - cl.offset) <= params.plane_offset_tol_m) {
        home = &cl;
        break;
      }
    }
    if (home == nullptr) {
      clusters.push_back(Cluster{});
      home = &clusters.back();
      home->normal = n;
      home->offset = c.dot(n);
    }
    home->normal_sum += (n.dot(home->normal) < 0 ? -n : n) * area;
    home->area += area;
    home->tris.push_back(tris.size());
    tris.push_back(t);
  }

  // Connected components inside each plane cluster (triangles sharing a welded vertex).
  struct Piece
  {
    Vec3 normal;
    double area{0.0};
    std::vector<int> vertex_ids;
  };
  std::vector<Piece> pieces;
  for (const auto & cl : clusters) {
    std::vector<size_t> parent(cl.tris.size());
    std::iota(parent.begin(), parent.end(), 0);
    auto find = [&](size_t a) {
        while (parent[a] != a) {parent[a] = parent[parent[a]]; a = parent[a];}
        return a;
      };
    std::map<int, size_t> first_tri_of_vertex;
    for (size_t i = 0; i < cl.tris.size(); ++i) {
      for (int v : tris[cl.tris[i]]) {
        const auto it = first_tri_of_vertex.find(v);
        if (it == first_tri_of_vertex.end()) {
          first_tri_of_vertex.emplace(v, i);
        } else {
          parent[find(i)] = find(it->second);
        }
      }
    }
    std::map<size_t, Piece> comp;
    const Vec3 n = cl.normal_sum.normalized();
    for (size_t i = 0; i < cl.tris.size(); ++i) {
      Piece & p = comp[find(i)];
      p.normal = n;
      const auto & t = tris[cl.tris[i]];
      p.area += 0.5 * (verts[t[1]] - verts[t[0]]).cross(verts[t[2]] - verts[t[0]]).norm();
      for (int v : t) {p.vertex_ids.push_back(v);}
    }
    for (auto & kv : comp) {
      std::sort(kv.second.vertex_ids.begin(), kv.second.vertex_ids.end());
      kv.second.vertex_ids.erase(
        std::unique(kv.second.vertex_ids.begin(), kv.second.vertex_ids.end()),
        kv.second.vertex_ids.end());
      pieces.push_back(std::move(kv.second));
    }
  }
  std::stable_sort(pieces.begin(), pieces.end(), [](const Piece & a, const Piece & b) {return a.area > b.area;});

  int k = 0;
  for (const auto & p : pieces) {
    if (p.area < params.min_area_m2) {
      std::ostringstream w;
      w << "dropped a " << p.area << " m^2 plane (below min_area_m2)";
      map.warnings.push_back(w.str());
      continue;
    }
    const bool horizontal = std::abs(p.normal.dot(params.up.normalized())) > std::cos(10.0 * kPi / 180.0);
    if (horizontal && !params.include_horizontal) {
      map.warnings.push_back("dropped a horizontal plane (floor or ceiling)");
      continue;
    }
    Vec3 centroid = Vec3::Zero();
    for (int v : p.vertex_ids) {centroid += verts[v];}
    centroid /= static_cast<double>(p.vertex_ids.size());
    PriorSurface s;
    s.id = id_prefix + "_" + std::to_string(k++);
    s.normal = p.normal;
    s.origin = centroid;  // mean of the plane's vertices: on the plane to within the offset tolerance
    orientToward(s, params, map.warnings);
    const Vec3 u = frameU(s.normal, params.up);
    const Vec3 v = s.normal.cross(u);
    double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
    for (int id : p.vertex_ids) {
      const Vec3 d = verts[id] - s.origin;
      u0 = std::min(u0, d.dot(u)); u1 = std::max(u1, d.dot(u));
      v0 = std::min(v0, d.dot(v)); v1 = std::max(v1, d.dot(v));
    }
    s.polygon = {
      s.origin + u * u0 + v * v0, s.origin + u * u1 + v * v0,
      s.origin + u * u1 + v * v1, s.origin + u * u0 + v * v1};
    s.origin = centroidOf(s.polygon);
    map.surfaces.push_back(std::move(s));
  }
  if (map.surfaces.empty()) {
    fail("no plane left after filtering (see min_area_m2 / include_horizontal)");
  }
  return map;
}

PriorMap loadPriorMap(const std::string & path, const PriorMapParams & params)
{
  const std::string ext = extensionOf(path);
  try {
    if (ext == "json") {
      return loadPriorSurfacesJson(path, params);
    }
    if (ext == "obj") {
      return priorMapFromMesh(scene_model::loadObj(path), params);
    }
    if (ext == "ply") {
      return priorMapFromMesh(loadPlyMesh(path), params);
    }
  } catch (const std::exception & e) {
    const std::string what = e.what();
    // Keep one file name in the message.
    throw std::runtime_error(what.rfind(path, 0) == 0 ? what : path + ": " + what);
  }
  throw std::runtime_error(path + ": unsupported prior map extension '." + ext + "' (.obj, .ply, .json)");
}

PriorMap transformPriorMap(
  const PriorMap & map, const scene_model::Transform & T, const std::string & new_frame)
{
  if (new_frame.empty()) {
    fail("new frame is empty");
  }
  PriorMap out = map;
  out.frame_id = new_frame;
  for (auto & s : out.surfaces) {
    s.origin = T * s.origin;
    s.normal = (T.linear() * s.normal).normalized();
    for (auto & p : s.polygon) {p = T * p;}
    for (auto & o : s.openings) {
      for (auto & p : o.polygon) {p = T * p;}
    }
  }
  for (auto & b : out.obstacles) {
    b.center = T * b.center;
    // Only the yaw about +z is carried; a tilting transform is rejected rather than approximated.
    const Vec3 x = T.linear() * Vec3::UnitX();
    if (std::abs((T.linear() * Vec3::UnitZ()).z() - 1.0) > 1e-6) {
      fail("transformPriorMap: obstacle boxes need a transform that keeps +z (yaw and translation only)");
    }
    b.yaw_deg += std::atan2(x.y(), x.x()) * 180.0 / kPi;
  }
  return out;
}

// ---------------------------------------------------------------------------------- wall_scene.json

namespace
{

nlohmann::json vecJson(const Vec3 & v)
{
  return nlohmann::json::array({v.x(), v.y(), v.z()});
}

nlohmann::json wallJson(const SurfaceFrame & f, uint32_t id)
{
  nlohmann::json wall;
  wall["id"] = id;
  wall["plane_abcd"] = {f.normal.x(), f.normal.y(), f.normal.z(), -f.normal.dot(f.origin)};
  wall["origin_xyz"] = vecJson(f.origin);
  wall["basis_u"] = vecJson(f.u);
  wall["basis_v"] = vecJson(f.v);
  wall["normal_outward"] = vecJson(f.normal);
  wall["u_range_m"] = {f.u_min, f.u_max};
  wall["v_range_m"] = {f.v_min, f.v_max};
  wall["plane_rms_mm"] = 0.0;  // a model, not a measurement
  wall["spread_p95_mm"] = 0.0;
  wall["spread_p99_mm"] = 0.0;
  return wall;
}

nlohmann::json uvObstacle(
  uint32_t id, uint32_t wall, const std::string & kind, const std::string & name, double u0, double u1,
  double v0, double v1, double protrusion)
{
  nlohmann::json o;
  o["id"] = id;
  o["parent_wall"] = wall;
  o["kind"] = kind;
  o["name"] = name;
  o["protrusion_max_m"] = protrusion;
  o["uv_on_wall"] = {{"u", {u0, u1}}, {"v", {v0, v1}}};
  return o;
}

}  // namespace

nlohmann::json priorMapToWallSceneJson(
  const PriorMap & map, const std::string & source, const Vec3 & up, double max_standoff_m,
  double behind_tol_m)
{
  nlohmann::json walls = nlohmann::json::array();
  nlohmann::json obstacles = nlohmann::json::array();
  uint32_t next_obstacle = 0;
  for (size_t i = 0; i < map.surfaces.size(); ++i) {
    const auto & s = map.surfaces[i];
    const SurfaceFrame f = surfaceFrame(s, up);
    auto wall = wallJson(f, static_cast<uint32_t>(i));
    wall["name"] = s.id;
    walls.push_back(wall);
    for (const auto & o : s.openings) {
      double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
      for (const auto & p : o.polygon) {
        const Vec3 d = p - f.origin;
        u0 = std::min(u0, d.dot(f.u)); u1 = std::max(u1, d.dot(f.u));
        v0 = std::min(v0, d.dot(f.v)); v1 = std::max(v1, d.dot(f.v));
      }
      obstacles.push_back(uvObstacle(next_obstacle++, static_cast<uint32_t>(i), o.kind, s.id, u0, u1, v0, v1, 0.0));
    }
    for (const auto & b : map.obstacles) {
      const double yaw = b.yaw_deg * kPi / 180.0;
      const Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Vec3::UnitZ()).toRotationMatrix();
      double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300, n0 = 1e300, n1 = -1e300;
      for (int c = 0; c < 8; ++c) {
        const Vec3 corner(
          (c & 1 ? 0.5 : -0.5) * b.size.x(), (c & 2 ? 0.5 : -0.5) * b.size.y(),
          (c & 4 ? 0.5 : -0.5) * b.size.z());
        const Vec3 d = b.center + R * corner - f.origin;
        u0 = std::min(u0, d.dot(f.u)); u1 = std::max(u1, d.dot(f.u));
        v0 = std::min(v0, d.dot(f.v)); v1 = std::max(v1, d.dot(f.v));
        n0 = std::min(n0, d.dot(f.normal)); n1 = std::max(n1, d.dot(f.normal));
      }
      if (n0 > max_standoff_m || n1 < -behind_tol_m) {continue;}  // not near this wall
      u0 = std::max(u0, f.u_min); u1 = std::min(u1, f.u_max);
      v0 = std::max(v0, f.v_min); v1 = std::min(v1, f.v_max);
      if (u0 >= u1 || v0 >= v1) {continue;}  // no footprint on this wall
      obstacles.push_back(uvObstacle(
        next_obstacle++, static_cast<uint32_t>(i), "box", b.id, u0, u1, v0, v1, std::max(n1, 0.0)));
    }
  }
  nlohmann::json scene;
  scene["frame_id"] = map.frame_id;
  scene["note"] = "Built from a prior map (docs/prior_map.md): a model, not a measurement. Flatness "
    "figures are 0 by construction. Obstacle kinds: door / window / opening (flush, protrusion 0) and box.";
  scene["source"] = source;
  scene["units"] = "meters";
  scene["walls"] = walls;
  scene["obstacles"] = obstacles;
  scene["prior_map"] = true;
  scene["warnings"] = map.warnings;
  return scene;
}

// ---------------------------------------------------------------------------------- merge

MergeResult mergeWithScan(
  const PriorSurface & prior, const scene_model::PlaneFit & measured, const MergeParams & params)
{
  MergeResult r;
  r.plane = measured;  // never replaced
  const double dot = prior.normal.normalized().dot(measured.normal.normalized());
  r.normal_flipped = dot < 0.0;
  r.angle_deg = std::acos(std::min(1.0, std::abs(dot))) * 180.0 / kPi;
  r.offset_m = (measured.origin - prior.origin).dot(prior.normal.normalized());
  r.same_plane = !r.normal_flipped && r.angle_deg <= params.max_angle_deg &&
    std::abs(r.offset_m) <= params.max_offset_m;

  std::ostringstream m;
  m << "prior '" << prior.id << "' vs measured: offset " << r.offset_m * 1000.0 << " mm, angle "
    << r.angle_deg << " deg";
  if (r.normal_flipped) {
    m << ", normals point opposite ways";
  }
  if (!r.same_plane) {
    m << " -> NOT the same plane (limits " << params.max_offset_m * 1000.0 << " mm, "
      << params.max_angle_deg << " deg); measured pose kept, prior openings not carried over";
    r.message = m.str();
    return r;
  }
  m << " -> same plane; measured pose kept, " << prior.openings.size() << " prior opening(s) carried over";
  r.message = m.str();
  for (const auto & o : prior.openings) {
    UvPolygon uv;
    uv.kind = o.kind;
    for (const auto & p : o.polygon) {
      const Vec3 d = p - measured.origin;
      uv.uv.push_back({d.dot(measured.u), d.dot(measured.v)});
    }
    r.openings.push_back(std::move(uv));
  }
  return r;
}

nlohmann::json mergedWallSceneJson(
  const scene_model::PlaneFit & measured, const PriorSurface & prior, const MergeResult & merge,
  uint32_t wall_id, const std::string & frame_id, const std::string & source)
{
  nlohmann::json scene = wallSceneJson(measured, wall_id, frame_id, source);
  scene["note"] = "Measured wall (wall_detector) with the openings of a prior map; see prior_merge.";
  uint32_t next = 0;
  for (const auto & o : merge.openings) {
    double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
    for (const auto & p : o.uv) {
      u0 = std::min(u0, p[0]); u1 = std::max(u1, p[0]);
      v0 = std::min(v0, p[1]); v1 = std::max(v1, p[1]);
    }
    scene["obstacles"].push_back(uvObstacle(next++, wall_id, o.kind, prior.id, u0, u1, v0, v1, 0.0));
  }
  scene["walls"][0]["prior_merge"] = {
    {"prior_id", prior.id}, {"offset_m", merge.offset_m}, {"angle_deg", merge.angle_deg},
    {"same_plane", merge.same_plane}, {"message", merge.message}};
  return scene;
}

}  // namespace wall_detector
