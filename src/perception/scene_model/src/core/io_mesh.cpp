#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

#include "scene_model/core/io.hpp"

namespace scene_model
{
namespace
{

std::string extensionOf(const std::string & path)
{
  const auto dot = path.find_last_of('.');
  std::string ext = dot == std::string::npos ? "" : path.substr(dot + 1);
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {return std::tolower(c);});
  return ext;
}

[[noreturn]] void fail(const std::string & path, const std::string & why)
{
  throw std::runtime_error(path + ": " + why);
}

}  // namespace

Mesh loadObj(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    fail(path, "cannot open file");
  }
  Mesh mesh;
  mesh.source_path = path;
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    std::string tag;
    ss >> tag;
    if (tag == "v") {
      double x, y, z;
      if (!(ss >> x >> y >> z)) {
        fail(path, "bad vertex line: " + line);
      }
      mesh.vertices.emplace_back(x, y, z);
    } else if (tag == "f") {
      std::vector<int> idx;
      for (std::string tok; ss >> tok;) {
        const int raw = std::stoi(tok.substr(0, tok.find('/')));
        const int n = static_cast<int>(mesh.vertices.size());
        idx.push_back(raw < 0 ? n + raw : raw - 1);
      }
      if (idx.size() < 3) {
        fail(path, "face with fewer than 3 vertices: " + line);
      }
      for (size_t k = 1; k + 1 < idx.size(); ++k) {
        mesh.faces.push_back({idx[0], idx[k], idx[k + 1]});
      }
    }
  }
  for (const auto & f : mesh.faces) {
    for (int i : f) {
      if (i < 0 || i >= static_cast<int>(mesh.vertices.size())) {
        fail(path, "face index out of range");
      }
    }
  }
  return mesh;
}

Mesh loadStl(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    fail(path, "cannot open file");
  }
  const std::vector<char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

  Mesh mesh;
  mesh.source_path = path;
  std::map<std::array<float, 3>, int> index;
  auto addVertex = [&](float x, float y, float z) {
      const std::array<float, 3> key{x, y, z};
      const auto it = index.find(key);
      if (it != index.end()) {
        return it->second;
      }
      const int id = static_cast<int>(mesh.vertices.size());
      mesh.vertices.emplace_back(x, y, z);
      index.emplace(key, id);
      return id;
    };

  // Binary STL: 80-byte header, uint32 count, then 50 bytes per triangle. An ASCII file also
  // starts with "solid", so decide by checking whether the size matches the binary layout.
  bool binary = false;
  if (bytes.size() >= 84) {
    uint32_t count;
    std::memcpy(&count, bytes.data() + 80, 4);
    binary = bytes.size() == 84 + static_cast<size_t>(count) * 50;
  }
  if (binary) {
    uint32_t count;
    std::memcpy(&count, bytes.data() + 80, 4);
    for (uint32_t t = 0; t < count; ++t) {
      const char * tri = bytes.data() + 84 + static_cast<size_t>(t) * 50;
      std::array<int, 3> face{};
      for (int v = 0; v < 3; ++v) {
        float c[3];
        std::memcpy(c, tri + 12 + v * 12, 12);
        face[v] = addVertex(c[0], c[1], c[2]);
      }
      mesh.faces.push_back(face);
    }
  } else {
    std::istringstream ss(std::string(bytes.begin(), bytes.end()));
    std::string tok;
    std::vector<int> corner;
    while (ss >> tok) {
      if (tok == "vertex") {
        float x, y, z;
        if (!(ss >> x >> y >> z)) {
          fail(path, "bad vertex in ASCII STL");
        }
        corner.push_back(addVertex(x, y, z));
        if (corner.size() == 3) {
          mesh.faces.push_back({corner[0], corner[1], corner[2]});
          corner.clear();
        }
      }
    }
    if (!corner.empty()) {
      fail(path, "ASCII STL ends in the middle of a facet");
    }
  }
  if (mesh.faces.empty()) {
    fail(path, "STL has no triangles");
  }
  return mesh;
}

Mesh loadMesh(const std::string & path)
{
  const std::string ext = extensionOf(path);
  if (ext == "obj") {return loadObj(path);}
  if (ext == "stl") {return loadStl(path);}
  fail(path, "unsupported mesh extension '." + ext + "' (use .obj or .stl)");
}

}  // namespace scene_model
