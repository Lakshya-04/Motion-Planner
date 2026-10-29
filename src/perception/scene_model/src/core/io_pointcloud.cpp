#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
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

std::vector<char> readAll(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    fail(path, "cannot open file");
  }
  return std::vector<char>((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

template<typename T>
T readAs(const char * p)
{
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

// Reads one scalar of PCD/PLY type `type` ('F','U','I') and size `size` as a double.
double readScalar(const char * p, char type, int size)
{
  switch (type) {
    case 'F':
      return size == 4 ? readAs<float>(p) : readAs<double>(p);
    case 'U':
      switch (size) {
        case 1: return readAs<uint8_t>(p);
        case 2: return readAs<uint16_t>(p);
        case 4: return readAs<uint32_t>(p);
        default: return static_cast<double>(readAs<uint64_t>(p));
      }
    default:
      switch (size) {
        case 1: return readAs<int8_t>(p);
        case 2: return readAs<int16_t>(p);
        case 4: return readAs<int32_t>(p);
        default: return static_cast<double>(readAs<int64_t>(p));
      }
  }
}

struct PcdField
{
  std::string name;
  int size{4};
  char type{'F'};
  int count{1};
  int offset{0};
};

}  // namespace

PointCloud loadPcd(const std::string & path)
{
  const std::vector<char> bytes = readAll(path);
  size_t pos = 0;
  auto nextLine = [&]() {
      std::string line;
      while (pos < bytes.size() && bytes[pos] != '\n') {
        line.push_back(bytes[pos++]);
      }
      if (pos < bytes.size()) {
        ++pos;
      }
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      return line;
    };

  std::vector<std::string> names;
  std::vector<int> sizes, counts;
  std::vector<char> types;
  size_t num_points = 0;
  std::string data_mode;
  bool saw_data = false;
  while (pos < bytes.size() && !saw_data) {
    std::istringstream line(nextLine());
    std::string key;
    line >> key;
    if (key == "FIELDS") {
      for (std::string s; line >> s;) {names.push_back(s);}
    } else if (key == "SIZE") {
      for (int s; line >> s;) {sizes.push_back(s);}
    } else if (key == "TYPE") {
      for (std::string s; line >> s;) {types.push_back(s.empty() ? 'F' : s[0]);}
    } else if (key == "COUNT") {
      for (int s; line >> s;) {counts.push_back(s);}
    } else if (key == "POINTS") {
      line >> num_points;
    } else if (key == "DATA") {
      line >> data_mode;
      saw_data = true;
    }
  }
  if (!saw_data || names.empty() || sizes.size() != names.size() || types.size() != names.size()) {
    fail(path, "malformed PCD header");
  }
  if (counts.empty()) {
    counts.assign(names.size(), 1);
  }
  if (data_mode == "binary_compressed") {
    fail(path, "binary_compressed PCD is not supported (re-save as binary or ascii)");
  }

  std::vector<PcdField> fields;
  int offset = 0;
  for (size_t i = 0; i < names.size(); ++i) {
    fields.push_back({names[i], sizes[i], types[i], counts[i], offset});
    offset += sizes[i] * counts[i];
  }
  const int point_step = offset;
  auto fieldIndex = [&](const char * n) -> int {
      for (size_t i = 0; i < fields.size(); ++i) {
        if (fields[i].name == n) {return static_cast<int>(i);}
      }
      return -1;
    };
  const int ix = fieldIndex("x"), iy = fieldIndex("y"), iz = fieldIndex("z");
  if (ix < 0 || iy < 0 || iz < 0) {
    fail(path, "PCD has no x/y/z fields");
  }
  int irgb = fieldIndex("rgb");
  if (irgb < 0) {irgb = fieldIndex("rgba");}
  const int ilabel = fieldIndex("label");

  PointCloud cloud;
  cloud.points.reserve(num_points);
  auto addPoint = [&](double x, double y, double z, bool has_rgb, uint32_t rgb, bool has_label,
      uint32_t label) {
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        return;
      }
      cloud.points.emplace_back(x, y, z);
      if (has_rgb) {
        cloud.colors.push_back(
          {static_cast<uint8_t>((rgb >> 16) & 0xFF), static_cast<uint8_t>((rgb >> 8) & 0xFF),
            static_cast<uint8_t>(rgb & 0xFF)});
      }
      if (has_label) {
        cloud.labels.push_back(label);
      }
    };

  if (data_mode == "ascii") {
    for (size_t n = 0; n < num_points && pos < bytes.size(); ++n) {
      std::istringstream line(nextLine());
      std::vector<double> values;
      for (double v; line >> v;) {values.push_back(v);}
      // One value per field (COUNT 1) is the common case; skip multi-count fields otherwise.
      size_t col = 0;
      std::vector<size_t> field_col(fields.size());
      for (size_t i = 0; i < fields.size(); ++i) {
        field_col[i] = col;
        col += static_cast<size_t>(fields[i].count);
      }
      if (values.size() < col) {
        fail(path, "PCD ascii row has too few values");
      }
      uint32_t rgb = 0;
      bool has_rgb = false;
      if (irgb >= 0) {
        has_rgb = true;
        const double v = values[field_col[irgb]];
        if (fields[irgb].type == 'F') {
          const float f = static_cast<float>(v);
          std::memcpy(&rgb, &f, 4);
        } else {
          rgb = static_cast<uint32_t>(v);
        }
      }
      addPoint(
        values[field_col[ix]], values[field_col[iy]], values[field_col[iz]], has_rgb, rgb,
        ilabel >= 0, ilabel >= 0 ? static_cast<uint32_t>(values[field_col[ilabel]]) : 0);
    }
  } else if (data_mode == "binary") {
    if (bytes.size() - pos < num_points * static_cast<size_t>(point_step)) {
      fail(path, "binary PCD data is shorter than the header says");
    }
    for (size_t n = 0; n < num_points; ++n) {
      const char * row = bytes.data() + pos + n * static_cast<size_t>(point_step);
      auto get = [&](int i) {
          return readScalar(row + fields[i].offset, fields[i].type, fields[i].size);
        };
      uint32_t rgb = 0;
      if (irgb >= 0) {
        rgb = readAs<uint32_t>(row + fields[irgb].offset);
      }
      addPoint(
        get(ix), get(iy), get(iz), irgb >= 0, rgb, ilabel >= 0,
        ilabel >= 0 ? static_cast<uint32_t>(get(ilabel)) : 0);
    }
  } else {
    fail(path, "unknown PCD DATA mode '" + data_mode + "'");
  }
  return cloud;
}

PointCloud loadPly(const std::string & path)
{
  const std::vector<char> bytes = readAll(path);
  size_t pos = 0;
  auto nextLine = [&]() {
      std::string line;
      while (pos < bytes.size() && bytes[pos] != '\n') {
        line.push_back(bytes[pos++]);
      }
      if (pos < bytes.size()) {
        ++pos;
      }
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      return line;
    };
  if (nextLine() != "ply") {
    fail(path, "not a PLY file");
  }

  struct Prop
  {
    std::string name;
    char type;
    int size;
  };
  std::string format;
  size_t vertex_count = 0;
  bool in_vertex = false;
  std::vector<Prop> props;
  std::vector<std::pair<std::string, size_t>> later_elements;  // elements after vertex
  bool header_done = false;
  auto scalar = [&](const std::string & t) -> std::pair<char, int> {
      if (t == "char" || t == "int8") {return {'I', 1};}
      if (t == "uchar" || t == "uint8") {return {'U', 1};}
      if (t == "short" || t == "int16") {return {'I', 2};}
      if (t == "ushort" || t == "uint16") {return {'U', 2};}
      if (t == "int" || t == "int32") {return {'I', 4};}
      if (t == "uint" || t == "uint32") {return {'U', 4};}
      if (t == "float" || t == "float32") {return {'F', 4};}
      if (t == "double" || t == "float64") {return {'F', 8};}
      fail(path, "unsupported PLY property type '" + t + "'");
    };
  while (pos < bytes.size() && !header_done) {
    std::istringstream line(nextLine());
    std::string key;
    line >> key;
    if (key == "format") {
      line >> format;
    } else if (key == "element") {
      std::string name;
      size_t count = 0;
      line >> name >> count;
      in_vertex = (name == "vertex");
      if (in_vertex) {vertex_count = count;}
    } else if (key == "property" && in_vertex) {
      std::string t, name;
      line >> t >> name;
      if (t == "list") {
        fail(path, "list properties on the vertex element are not supported");
      }
      const auto [type, size] = scalar(t);
      props.push_back({name, type, size});
    } else if (key == "end_header") {
      header_done = true;
    }
  }
  if (!header_done || (vertex_count == 0 && props.empty())) {
    fail(path, "malformed PLY header");
  }
  auto propIndex = [&](const char * n) -> int {
      for (size_t i = 0; i < props.size(); ++i) {
        if (props[i].name == n) {return static_cast<int>(i);}
      }
      return -1;
    };
  const int ix = propIndex("x"), iy = propIndex("y"), iz = propIndex("z");
  if (ix < 0 || iy < 0 || iz < 0) {
    fail(path, "PLY has no x/y/z vertex properties");
  }
  const int ir = propIndex("red"), ig = propIndex("green"), ib = propIndex("blue");
  const bool has_color = ir >= 0 && ig >= 0 && ib >= 0;

  PointCloud cloud;
  cloud.points.reserve(vertex_count);
  auto add = [&](const std::vector<double> & v) {
      if (!std::isfinite(v[ix]) || !std::isfinite(v[iy]) || !std::isfinite(v[iz])) {
        return;
      }
      cloud.points.emplace_back(v[ix], v[iy], v[iz]);
      if (has_color) {
        cloud.colors.push_back(
          {static_cast<uint8_t>(v[ir]), static_cast<uint8_t>(v[ig]), static_cast<uint8_t>(v[ib])});
      }
    };
  if (format == "ascii") {
    for (size_t n = 0; n < vertex_count && pos < bytes.size(); ++n) {
      std::istringstream line(nextLine());
      std::vector<double> v;
      for (double d; line >> d;) {v.push_back(d);}
      if (v.size() < props.size()) {
        fail(path, "PLY ascii vertex row has too few values");
      }
      add(v);
    }
  } else if (format == "binary_little_endian") {
    size_t step = 0;
    for (const auto & p : props) {step += static_cast<size_t>(p.size);}
    if (bytes.size() - pos < vertex_count * step) {
      fail(path, "binary PLY data is shorter than the header says");
    }
    for (size_t n = 0; n < vertex_count; ++n) {
      const char * row = bytes.data() + pos + n * step;
      std::vector<double> v(props.size());
      size_t off = 0;
      for (size_t i = 0; i < props.size(); ++i) {
        v[i] = readScalar(row + off, props[i].type, props[i].size);
        off += static_cast<size_t>(props[i].size);
      }
      add(v);
    }
  } else {
    fail(path, "unsupported PLY format '" + format + "'");
  }
  return cloud;
}

PointCloud loadPointCloud(const std::string & path)
{
  const std::string ext = extensionOf(path);
  if (ext == "pcd") {return loadPcd(path);}
  if (ext == "ply") {return loadPly(path);}
  fail(path, "unsupported point-cloud extension '." + ext + "' (use .pcd or .ply)");
}

void savePcdAscii(const std::string & path, const PointCloud & cloud)
{
  std::ofstream out(path);
  if (!out) {
    fail(path, "cannot open file for writing");
  }
  const bool labels = cloud.labels.size() == cloud.points.size() && !cloud.points.empty();
  out << "# .PCD v0.7\nVERSION 0.7\n";
  out << (labels ? "FIELDS x y z label\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n" :
    "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n");
  out << "WIDTH " << cloud.points.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n";
  out << "POINTS " << cloud.points.size() << "\nDATA ascii\n";
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    const auto & p = cloud.points[i];
    out << p.x() << ' ' << p.y() << ' ' << p.z();
    if (labels) {out << ' ' << cloud.labels[i];}
    out << '\n';
  }
}

}  // namespace scene_model
