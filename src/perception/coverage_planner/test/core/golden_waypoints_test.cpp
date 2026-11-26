// Change detector: pins what the planner emits today (not that it is right). COVERAGE_GOLDEN_UPDATE=1 rewrites the files.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include "coverage_planner/core/coverage_pass_expander.hpp"
#include "coverage_planner/core/coverage_planner_base.hpp"
#include "coverage_planner/core/wall_basis.hpp"

#ifndef COVERAGE_GOLDEN_DIR
#error "COVERAGE_GOLDEN_DIR must be provided by the build (-D)"
#endif

using namespace coverage_planner;  // NOLINT

namespace
{

constexpr double kTol = 1e-9;
constexpr double kRes = 0.05;

OccupancyGrid makeGrid(int w, int h, const std::function<bool(int, int)> & blocked)
{
  OccupancyGrid g;
  g.header.frame_id = "wall";
  g.info.resolution = kRes;
  g.info.width = w;
  g.info.height = h;
  g.data.assign(static_cast<size_t>(w) * h, 0);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (blocked(x, y)) {g.data[static_cast<size_t>(y) * w + x] = 100;}
    }
  }
  return g;
}

CoveragePlannerParams baseParams()
{
  CoveragePlannerParams p{};
  p.swath = 0.2;
  p.step = 0.2;
  p.sweep_direction = "vertical";
  p.start_corner = "bottom_left";
  p.obstacle_inflation_radius = 0.0;
  p.allow_unknown = false;
  p.enable_boundary_pass = false;
  p.boundary_offset = 0.0;
  p.add_strip_connectors = false;
  p.pose_mode = "identity";
  p.map_frame = "wall";
  p.use_region = false;
  p.plan_all_components = true;
  p.min_component_cells = 1;
  p.optimize_strip_order = false;
  p.requires_contact = true;
  return p;
}

// Wall plane y = 0 in world, u = x, v = z.
WallPlane makeWall(double u_max, double v_max)
{
  WallPlane wall;
  wall.pose.pose = encodeWallBasis(
    Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, -1, 0));
  wall.u_max = u_max;
  wall.v_max = v_max;
  return wall;
}

struct Scene
{
  std::string name;
  std::string planner;
  OccupancyGrid grid;
  CoveragePlannerParams params;
  bool lift_and_expand;  // run the pass expander on the planner output
  bool double_pass;
};

std::string fmt(double v)
{
  if (std::fabs(v) < 5e-13) {v = 0.0;}  // no "-0.000000000000"
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.12f", v);
  return buf;
}

// One line per segment, then one per waypoint.
std::string render(const Path & path, const std::vector<MotionPlanSegment> & segs)
{
  std::ostringstream o;
  for (const auto & s : segs) {
    o << "seg " << s.label << " " << s.mode << " " << s.start_index << " " << s.end_index << "\n";
  }
  for (const auto & ps : path.poses) {
    const auto & p = ps.pose.position;
    const auto & q = ps.pose.orientation;
    o << "wp " << fmt(p.x) << " " << fmt(p.y) << " " << fmt(p.z) << " " << fmt(q.x) << " " << fmt(q.y) << " "
      << fmt(q.z) << " " << fmt(q.w) << "\n";
  }
  return o.str();
}

std::string runScene(const Scene & sc, size_t & n_wp)
{
  const auto planner = CoveragePlannerFactory::createPlanner(sc.planner);
  const auto r = planner->generatePath(sc.grid, sc.params);
  EXPECT_TRUE(r.success) << r.message;
  if (!sc.lift_and_expand) {
    n_wp = r.path.poses.size();
    return render(r.path, r.segments);
  }
  // Grid x, y -> wall u, v (world x, z), as the expander expects.
  Path flat = r.path;
  for (auto & ps : flat.poses) {
    ps.pose.position.z = ps.pose.position.y;
    ps.pose.position.y = 0.0;
  }
  ToolCoverageProfile prof;
  prof.task.contact = true;
  prof.task.standoff_m = 0.03;
  prof.task.tool_normal_policy = "constant";
  prof.coverage.passes = {PassVariant{}};
  if (sc.double_pass) {prof.coverage.passes.push_back(PassVariant{0.0, true});}
  const auto wall = makeWall(sc.grid.info.width * kRes, sc.grid.info.height * kRes);
  const auto e = expandCoveragePasses(flat, r.segments, wall, prof);
  n_wp = e.path.poses.size();
  return render(e.path, e.segments);
}

std::vector<std::string> lines(const std::string & s)
{
  std::vector<std::string> out;
  std::istringstream in(s);
  for (std::string l; std::getline(in, l);) {
    if (!l.empty() && l[0] != '#') {out.push_back(l);}
  }
  return out;
}

void compareLine(const std::string & want, const std::string & got, size_t idx)
{
  std::istringstream w(want), g(got);
  std::string wt, gt;
  w >> wt;
  g >> gt;
  ASSERT_EQ(wt, gt) << "line " << idx;
  if (wt == "seg") {
    EXPECT_EQ(want, got) << "line " << idx;
    return;
  }
  for (int i = 0; i < 7; ++i) {
    double a = 0, b = 0;
    w >> a;
    g >> b;
    EXPECT_NEAR(a, b, kTol) << "line " << idx << " field " << i;
  }
}

void checkGolden(const Scene & sc)
{
  size_t n_wp = 0;
  const std::string got = runScene(sc, n_wp);
  const std::string path = std::string(COVERAGE_GOLDEN_DIR) + "/" + sc.name + ".txt";
  if (std::getenv("COVERAGE_GOLDEN_UPDATE") != nullptr) {
    std::ofstream out(path);
    ASSERT_TRUE(out.good()) << path;
    out << "# golden for scene '" << sc.name << "' (" << sc.planner << "); seg <label> <mode> <start> <end>, "
        << "wp x y z qx qy qz qw\n"
        << got;
    return;
  }
  std::ifstream in(path);
  ASSERT_TRUE(in.good()) << "missing " << path << " (run with COVERAGE_GOLDEN_UPDATE=1)";
  std::stringstream buf;
  buf << in.rdbuf();
  const auto want = lines(buf.str());
  const auto have = lines(got);
  ASSERT_EQ(want.size(), have.size()) << sc.name;
  for (size_t i = 0; i < want.size(); ++i) {compareLine(want[i], have[i], i);}
  EXPECT_GT(n_wp, 0u);
}

}  // namespace

TEST(CoverageGolden, PlainRectangle)
{
  auto p = baseParams();
  checkGolden({"plain_rectangle", "boustrophedon", makeGrid(24, 16, [](int, int) {return false;}), p, false, false});
}

TEST(CoverageGolden, RectangularObstacle)
{
  auto p = baseParams();
  p.add_strip_connectors = true;
  const auto grid = makeGrid(30, 20, [](int x, int y) {return x >= 12 && x < 18 && y >= 6 && y < 14;});
  checkGolden({"rect_obstacle", "sweepline", grid, p, false, false});
}

TEST(CoverageGolden, DoublePassProfile)
{
  auto p = baseParams();
  p.add_strip_connectors = true;
  checkGolden({"double_pass", "boustrophedon", makeGrid(20, 12, [](int, int) {return false;}), p, true, true});
}
