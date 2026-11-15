#include <cstdio>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "manipulation_manager/ros/coverage/coverage_output_json_source.hpp"

namespace manipulation_manager
{
namespace
{
std::string writePlan(const std::string & name, const std::string & points)
{
  const std::string path = ::testing::TempDir() + name;
  std::ofstream f(path);
  f << R"({"final_lifted_and_expanded":{"path":{"poses":[)" << points << "]}}}";
  return path;
}
std::string pt(double x, double y, double z)
{
  return R"({"position":{"x":)" + std::to_string(x) + R"(,"y":)" + std::to_string(y) + R"(,"z":)" +
         std::to_string(z) + R"(},"orientation":{"x":0.5,"y":0.5,"z":0.5,"w":0.5}})";
}
}  // namespace

TEST(CoverageOutputJsonSourceTest, DefaultPlusYWallGroupsColumnsByXAscending)
{
  const auto path = writePlan("plus_y.json",
    pt(0.3, 1.0, 1.2) + "," + pt(0.3, 1.0, 1.4) + "," + pt(-0.1, 1.0, 1.2) + "," + pt(-0.1, 1.0, 1.4));
  CoverageOutputJsonSource src(path, "base_footprint", 0.0, 0.1, 1.0, 2.0);
  const auto plan = src.getCoveragePlan();
  ASSERT_EQ(plan.size(), 2U);
  EXPECT_NEAR(plan[0].poses.front().x, -0.1, 1e-6);
  EXPECT_NEAR(plan[1].poses.front().x, 0.3, 1e-6);
  EXPECT_NEAR(plan[0].poses.front().z, 1.2, 1e-6);
  EXPECT_NEAR(plan[0].poses.back().z, 1.4, 1e-6);
}

TEST(CoverageOutputJsonSourceTest, PlusXWallGroupsColumnsByYAndKeepsEachColumnsPlane)
{
  // Two columns at y = 0.17 and y = 0.01 on a wall at x = 0.98 (the sim wall); the old grouping by x
  // would merge them into one column.
  const auto path = writePlan("plus_x.json",
    pt(0.98, 0.17, 1.01) + "," + pt(0.98, 0.17, 1.29) + "," + pt(0.98, 0.01, 1.01) + "," + pt(0.98, 0.01, 1.29));
  CoverageOutputJsonSource src(
    path, "base_footprint", 0.0, 0.07, 1.0, 1.3, 0.02, WallDirection::from(1, 0, 0));
  const auto plan = src.getCoveragePlan();
  ASSERT_EQ(plan.size(), 2U);
  // ascending along the tangent (direction x up = (0, -1, 0)), i.e. y descending
  EXPECT_NEAR(plan[0].poses.front().y, 0.17, 1e-6);
  EXPECT_NEAR(plan[1].poses.front().y, 0.01, 1e-6);
  for (const auto & c : plan) {
    ASSERT_GE(c.poses.size(), 4U);
    for (const auto & p : c.poses) {
      EXPECT_NEAR(p.x, 0.98, 1e-6);
      EXPECT_NEAR(p.y, c.poses.front().y, 1e-9);
    }
    EXPECT_NEAR(c.target_eef_speed, 0.02, 1e-12);
  }
}

}  // namespace manipulation_manager
