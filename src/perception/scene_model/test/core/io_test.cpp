#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>

#include "scene_model/core/io.hpp"

using namespace scene_model;  // NOLINT
namespace fs = std::filesystem;

namespace
{
class TempDir
{
public:
  TempDir()
  {
    path_ = fs::temp_directory_path() / ("scene_model_test_" + std::to_string(::getpid()) + "_" +
      std::to_string(counter_++));
    fs::create_directories(path_);
  }
  ~TempDir() {fs::remove_all(path_);}
  std::string file(const std::string & name) const {return (path_ / name).string();}
  void write(const std::string & name, const std::string & content) const
  {
    std::ofstream(file(name), std::ios::binary) << content;
  }
  void writeBytes(const std::string & name, const std::vector<char> & bytes) const
  {
    std::ofstream out(file(name), std::ios::binary);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  }
private:
  fs::path path_;
  static inline int counter_ = 0;
};

template<typename T>
void put(std::vector<char> & out, T v)
{
  const char * p = reinterpret_cast<const char *>(&v);
  out.insert(out.end(), p, p + sizeof(T));
}
}  // namespace

TEST(PointCloudIo, PcdAsciiRoundTripWithLabels)
{
  TempDir d;
  PointCloud in;
  in.points = {Vec3(1, 2, 3), Vec3(-1, 0.5, 4)};
  in.labels = {7, 9};
  savePcdAscii(d.file("a.pcd"), in);
  const PointCloud out = loadPointCloud(d.file("a.pcd"));
  ASSERT_EQ(out.points.size(), 2U);
  EXPECT_NEAR(out.points[1].y(), 0.5, 1e-6);
  ASSERT_EQ(out.labels.size(), 2U);
  EXPECT_EQ(out.labels[0], 7U);
  EXPECT_EQ(out.labels[1], 9U);
}

TEST(PointCloudIo, PcdBinaryWithRgbAndNanDropped)
{
  TempDir d;
  std::string header =
    "VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
    "WIDTH 3\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 3\nDATA binary\n";
  std::vector<char> bytes(header.begin(), header.end());
  auto point = [&](float x, float y, float z, uint32_t rgb) {
      put<float>(bytes, x); put<float>(bytes, y); put<float>(bytes, z); put<uint32_t>(bytes, rgb);
    };
  point(1.F, 2.F, 3.F, 0x00FF8001);
  point(std::nanf(""), 0.F, 0.F, 0);
  point(4.F, 5.F, 6.F, 0x00112233);
  d.writeBytes("b.pcd", bytes);
  const PointCloud c = loadPcd(d.file("b.pcd"));
  ASSERT_EQ(c.points.size(), 2U);  // NaN point dropped
  EXPECT_EQ(c.colors[0][0], 0xFF);
  EXPECT_EQ(c.colors[0][1], 0x80);
  EXPECT_EQ(c.colors[0][2], 0x01);
  EXPECT_NEAR(c.points[1].z(), 6.0, 1e-6);
}

TEST(PointCloudIo, PcdCompressedIsRejectedWithClearMessage)
{
  TempDir d;
  d.write("c.pcd",
    "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary_compressed\n");
  try {
    loadPcd(d.file("c.pcd"));
    FAIL() << "expected an exception";
  } catch (const std::runtime_error & e) {
    EXPECT_NE(std::string(e.what()).find("binary_compressed"), std::string::npos);
  }
}

TEST(PointCloudIo, TruncatedBinaryPcdThrows)
{
  TempDir d;
  std::string header =
    "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 5\nHEIGHT 1\nPOINTS 5\nDATA binary\n";
  std::vector<char> bytes(header.begin(), header.end());
  put<float>(bytes, 1.F);  // far too short
  d.writeBytes("t.pcd", bytes);
  EXPECT_THROW(loadPcd(d.file("t.pcd")), std::runtime_error);
}

TEST(PointCloudIo, PlyAsciiAndBinary)
{
  TempDir d;
  d.write("a.ply",
    "ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\nproperty float z\n"
    "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
    "1 2 3 10 20 30\n4 5 6 40 50 60\n");
  const PointCloud a = loadPly(d.file("a.ply"));
  ASSERT_EQ(a.points.size(), 2U);
  EXPECT_EQ(a.colors[1][2], 60);

  std::string header =
    "ply\nformat binary_little_endian 1.0\nelement vertex 2\nproperty float x\nproperty float y\n"
    "property float z\nend_header\n";
  std::vector<char> bytes(header.begin(), header.end());
  for (float v : {1.F, 2.F, 3.F, 7.F, 8.F, 9.F}) {put<float>(bytes, v);}
  d.writeBytes("b.ply", bytes);
  const PointCloud b = loadPointCloud(d.file("b.ply"));
  ASSERT_EQ(b.points.size(), 2U);
  EXPECT_NEAR(b.points[1].x(), 7.0, 1e-6);
}

TEST(PointCloudIo, UnknownExtensionAndMissingFile)
{
  EXPECT_THROW(loadPointCloud("/tmp/nope.xyz"), std::runtime_error);
  EXPECT_THROW(loadPcd("/tmp/definitely_missing_file.pcd"), std::runtime_error);
}

TEST(MeshIo, ObjQuadsAreTriangulatedAndNegativeIndicesWork)
{
  TempDir d;
  d.write("q.obj",
    "# quad\nv 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nvt 0 0\nvn 0 0 1\nf 1/1/1 2/1/1 3/1/1 4/1/1\n"
    "f -4 -3 -2\n");
  const Mesh m = loadMesh(d.file("q.obj"));
  EXPECT_EQ(m.vertices.size(), 4U);
  ASSERT_EQ(m.faces.size(), 3U);  // quad -> 2 triangles, plus one triangle
  EXPECT_EQ(m.faces[2], (std::array<int, 3>{0, 1, 2}));
  EXPECT_EQ(m.source_path, d.file("q.obj"));
}

TEST(MeshIo, ObjOutOfRangeFaceThrows)
{
  TempDir d;
  d.write("bad.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 9\n");
  EXPECT_THROW(loadObj(d.file("bad.obj")), std::runtime_error);
}

TEST(MeshIo, StlAsciiMergesSharedVertices)
{
  TempDir d;
  d.write("a.stl",
    "solid t\n facet normal 0 0 1\n  outer loop\n   vertex 0 0 0\n   vertex 1 0 0\n   vertex 0 1 0\n"
    "  endloop\n endfacet\n facet normal 0 0 1\n  outer loop\n   vertex 1 0 0\n   vertex 1 1 0\n"
    "   vertex 0 1 0\n  endloop\n endfacet\nendsolid t\n");
  const Mesh m = loadStl(d.file("a.stl"));
  EXPECT_EQ(m.faces.size(), 2U);
  EXPECT_EQ(m.vertices.size(), 4U);  // 6 corners, 2 shared pairs merged
}

TEST(MeshIo, StlBinary)
{
  TempDir d;
  std::vector<char> bytes(80, 0);
  put<uint32_t>(bytes, 1);
  for (float f : {0.F, 0.F, 1.F, 0.F, 0.F, 0.F, 1.F, 0.F, 0.F, 0.F, 1.F, 0.F}) {put<float>(bytes, f);}
  put<uint16_t>(bytes, 0);
  d.writeBytes("b.stl", bytes);
  const Mesh m = loadMesh(d.file("b.stl"));
  EXPECT_EQ(m.faces.size(), 1U);
  EXPECT_EQ(m.vertices.size(), 3U);
  EXPECT_NEAR(m.vertices[1].x(), 1.0, 1e-6);
}

TEST(SceneIo, NativeLayoutWithRolesAndFiles)
{
  TempDir d;
  d.write("m.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
  d.write("scene.yaml",
    "frame_id: map\nobjects:\n"
    "  - {id: wall, type: box, size: [4.0, 0.1, 3.0], role: contact_surface,\n"
    "     pose: {position: [0, 1, 1.5], orientation_xyzw: [0, 0, 0, 1]}}\n"
    "  - {id: ball, type: sphere, radius: 0.2, static: false}\n"
    "  - {id: part, type: mesh, file: m.obj, scale: 2.0}\n");
  const SceneModel s = loadSceneFile(d.file("scene.yaml"));
  EXPECT_EQ(s.frameId(), "map");
  ASSERT_EQ(s.size(), 3U);
  EXPECT_EQ(s.find("wall")->role, Role::kContactSurface);
  EXPECT_NEAR(s.find("wall")->pose.translation().z(), 1.5, 1e-12);
  EXPECT_FALSE(s.find("ball")->is_static);
  const Mesh & part = std::get<Mesh>(s.find("part")->geometry);
  EXPECT_NEAR(part.vertices[1].x(), 2.0, 1e-12);  // scale applied
}

TEST(SceneIo, CuroboStyleDictUsesWxyzPoses)
{
  const SceneModel s = loadSceneYaml(
    "cuboid:\n  table:\n    dims: [1.0, 0.6, 0.05]\n    pose: [0, 0, 0.4, 0.7071068, 0, 0, 0.7071068]\n"
    "sphere:\n  ball:\n    radius: 0.1\n    pose: [1, 2, 3, 1, 0, 0, 0]\n");
  ASSERT_EQ(s.size(), 2U);
  // 90 degrees about z: local x maps to world y.
  const Vec3 x_world = s.find("table")->pose.linear() * Vec3::UnitX();
  EXPECT_NEAR(x_world.y(), 1.0, 1e-6);
  EXPECT_NEAR(s.find("ball")->pose.translation().y(), 2.0, 1e-12);
}

TEST(SceneIo, ErrorsNameTheProblem)
{
  auto message = [](const std::string & text) {
      try {
        loadSceneYaml(text);
      } catch (const std::exception & e) {
        return std::string(e.what());
      }
      return std::string("no error");
    };
  EXPECT_NE(message("objects:\n  - {id: a, type: teapot}\n").find("unknown type 'teapot'"), std::string::npos);
  EXPECT_NE(message("objects:\n  - {id: a, type: box, size: [1, 2]}\n").find("list of 3"), std::string::npos);
  EXPECT_NE(message("objects:\n  - {id: a, type: sphere, radius: 1}\n  - {id: a, type: sphere, radius: 1}\n")
    .find("duplicate"), std::string::npos);
  EXPECT_NE(message("objects:\n  - {id: a, type: sphere, radius: 1, role: weird}\n").find("role"), std::string::npos);
  EXPECT_NE(message("[1, 2]").find("mapping"), std::string::npos);
}
