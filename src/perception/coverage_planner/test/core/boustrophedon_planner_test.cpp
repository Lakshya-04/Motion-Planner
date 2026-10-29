#include <gtest/gtest.h>

#include "coverage_planner/boustrophedon_planner.hpp"

#include <algorithm>
#include <cmath>

namespace
{

nav_msgs::msg::OccupancyGrid makeGrid(int width, int height, const std::vector<int8_t> & data)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.width = static_cast<uint32_t>(width);
  grid.info.height = static_cast<uint32_t>(height);
  grid.info.resolution = 1.0f;
  grid.info.origin.orientation.w = 1.0;
  grid.data = data;
  return grid;
}

coverage_planner::CoveragePlannerParams defaultParams()
{
  coverage_planner::CoveragePlannerParams params;
  params.swath = 1.0;
  params.step = 1.0;
  params.sweep_direction = "horizontal";
  params.start_corner = "top_left";
  params.obstacle_inflation_radius = 0.0;
  params.allow_unknown = false;
  params.enable_boundary_pass = false;
  params.boundary_offset = 0.0;
  params.add_strip_connectors = false;
  params.pose_mode = "identity";
  params.map_frame = "map";
  params.use_region = false;
  params.plan_all_components = true;
  params.min_component_cells = 1;
  params.optimize_strip_order = false;
  return params;
}

}  // namespace

TEST(BoustrophedonPlannerTest, HorizontalSweepStopsAtCurrentFreeCellBeforeObstacle)
{
  // Row 0 has an internal obstacle at x=3; row 1 keeps free-space connected.
  auto grid = makeGrid(6, 2, {0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0});

  coverage_planner::BoustrophedonPlanner planner;
  auto params = defaultParams();
  auto result = planner.generatePath(grid, params);

  ASSERT_TRUE(result.success);
  ASSERT_GE(result.path.poses.size(), 2u);

  // First segment end should be at x=2 cell center -> world x=2.5.
  const double first_segment_end_x = result.path.poses[1].pose.position.x;
  EXPECT_NEAR(first_segment_end_x, 2.5, 1e-6);
}

TEST(BoustrophedonPlannerTest, HorizontalSweepLookaheadDoesNotWrapAcrossRows)
{
  auto grid = makeGrid(4, 2, {0, 0, 0, 0, 0, 0, 0, 0});

  coverage_planner::BoustrophedonPlanner planner;
  auto params = defaultParams();
  auto result = planner.generatePath(grid, params);

  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.path.poses.size(), 4u);

  std::vector<double> ys;
  ys.reserve(result.path.poses.size());
  for (const auto & ps : result.path.poses) {
    ys.push_back(ps.pose.position.y);
  }
  std::sort(ys.begin(), ys.end());
  ys.erase(
    std::unique(ys.begin(), ys.end(), [](double a, double b) { return std::abs(a - b) < 1e-9; }),
    ys.end());

  ASSERT_EQ(ys.size(), 2u);
  EXPECT_NEAR(ys[0], 0.5, 1e-6);
  EXPECT_NEAR(ys[1], 1.5, 1e-6);
}

TEST(BoustrophedonPlannerTest, BlockedConnectorRoutesAroundObstacleWhenFreeDetourExists)
{
  // Row 0 fully free; row 1 only has x=0 free. The direct line from row0's end to row1's start
  // crosses blocked cells, but a detour exists back along row 0 to (0,0) then down to (0,1) --
  // the BFS fallback must find and use it instead of rejecting the connector outright.
  auto grid = makeGrid(6, 2, {0, 0, 0, 0, 0, 0, 0, 100, 100, 100, 100, 100});

  coverage_planner::BoustrophedonPlanner planner;
  auto params = defaultParams();
  params.add_strip_connectors = true;
  auto result = planner.generatePath(grid, params);

  ASSERT_TRUE(result.success);
  // row0 segment (2) + routed connector through (4,0),(3,0),(2,0),(1,0),(0,0) (5) + row1's own
  // single-cell segment (2) = 9. render_end_index (not end_index -- that's the connector's own,
  // non-overlapping interior range) points one past its last interior pose, at the index row1's
  // start pose lands at once pushed -- no extra pose appended.
  EXPECT_EQ(result.path.poses.size(), 9u);

  const auto & connector = result.segments.at(1);
  ASSERT_EQ(connector.render_end_index, result.segments.at(2).start_index);
  ASSERT_LT(connector.end_index, connector.render_end_index);
}

TEST(BoustrophedonPlannerTest, ConnectorStillRejectedWhenNoFreeRouteExistsAtAll)
{
  // Row 0: x=0 blocked, x=1..5 free. Row 1: only x=0 free, and that single cell has zero free
  // neighbors (its only would-be neighbors are row0's x=0, which is blocked, and out-of-bounds)
  // -- genuinely unreachable from row0's own free region, not just via a blocked straight line.
  auto grid = makeGrid(6, 2, {100, 0, 0, 0, 0, 0, 0, 100, 100, 100, 100, 100});

  coverage_planner::BoustrophedonPlanner planner;
  auto params = defaultParams();
  params.add_strip_connectors = true;
  auto result = planner.generatePath(grid, params);

  ASSERT_TRUE(result.success);
  // No route exists at all (not even a detour) -- connector must still be rejected: row0 segment
  // (2) + row1's own single-cell segment (2) = 4, no connector waypoints appended.
  EXPECT_EQ(result.path.poses.size(), 4u);
}

TEST(BoustrophedonPlannerTest, VerticalSweepStopsAtCurrentFreeCellBeforeObstacle)
{
  auto grid = makeGrid(2, 6, {0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0});

  coverage_planner::BoustrophedonPlanner planner;
  auto params = defaultParams();
  params.sweep_direction = "vertical";
  params.start_corner = "top_left";
  auto result = planner.generatePath(grid, params);

  ASSERT_TRUE(result.success);
  ASSERT_GE(result.path.poses.size(), 2u);

  const double first_segment_end_y = result.path.poses[1].pose.position.y;
  EXPECT_NEAR(first_segment_end_y, 2.5, 1e-6);
}

TEST(BoustrophedonPlannerTest, ConnectorInterpolationHandlesNegativeXDirection)
{
  // First sweep goes x:3->0 at y=0. With step=2, next sweep is y=2 and starts from right side.
  // The first free target there is x=1, forcing a negative-x connector (3,0) -> (1,2)
  // whose intermediate cell (2,1) is free and should be appended.
  auto grid = makeGrid(4, 3, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 100});

  coverage_planner::BoustrophedonPlanner planner;
  auto params = defaultParams();
  params.start_corner = "top_left";
  params.step = 2.0;
  params.add_strip_connectors = true;
  auto result = planner.generatePath(grid, params);

  ASSERT_TRUE(result.success);
  ASSERT_GE(result.path.poses.size(), 5u);

  bool found_intermediate_connector = false;
  for (const auto & ps : result.path.poses) {
    if (std::abs(ps.pose.position.y - 1.5) < 1e-6) {
      found_intermediate_connector = true;
      break;
    }
  }
  EXPECT_TRUE(found_intermediate_connector);
}

TEST(BoustrophedonPlannerTest, ConnectorSkippedWhenNextSweepHasNoFreeCell)
{
  auto grid = makeGrid(4, 2, {0, 0, 0, 0, 100, 100, 100, 100});

  coverage_planner::BoustrophedonPlanner planner;
  auto params = defaultParams();
  params.add_strip_connectors = true;
  auto result = planner.generatePath(grid, params);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.path.poses.size(), 2u);
}

TEST(BoustrophedonPlannerTest, SameColumnDoesNotReconnectAcrossObstacleSplit)
{
  // Vertical sweep, column x=0 split in two by an obstacle at y=2; column x=1 is fully free, so
  // an obstacle-aware detour would technically be possible. Confirmed live against a real wall:
  // that detour, when it has to hug a real (larger, multi-column) obstacle's boundary, produces
  // a "frame" traced around the obstacle instead of a clean lane pairing -- not the wanted
  // behavior. Same-column/row reconnection is removed; the two pieces of an obstacle-split strip
  // stay disconnected, same as before add_strip_connectors existed. Only the ordinary
  // column-to-column/row-to-row transition connector remains.
  auto grid = makeGrid(2, 5, {0, 0, 0, 0, 100, 0, 0, 0, 0, 0});

  coverage_planner::BoustrophedonPlanner planner;
  auto params = defaultParams();
  params.sweep_direction = "vertical";
  params.add_strip_connectors = true;
  auto result = planner.generatePath(grid, params);

  ASSERT_TRUE(result.success);
  const int strip_count =
    std::count_if(result.segments.begin(), result.segments.end(), [](const auto & segment) {
      return std::find(segment.tags.begin(), segment.tags.end(), "strip") != segment.tags.end();
    });
  ASSERT_EQ(strip_count, 3);  // x=0's two obstacle-split pieces + x=1's own single strip.
}

TEST(BoustrophedonPlannerTest, SameRowDoesNotReconnectAcrossObstacleSplit)
{
  // Horizontal-sweep mirror of SameColumnDoesNotReconnectAcrossObstacleSplit.
  auto grid = makeGrid(5, 2, {0, 0, 100, 0, 0, 0, 0, 0, 0, 0});

  coverage_planner::BoustrophedonPlanner planner;
  auto params = defaultParams();
  params.sweep_direction = "horizontal";
  params.add_strip_connectors = true;
  auto result = planner.generatePath(grid, params);

  ASSERT_TRUE(result.success);
  const int strip_count =
    std::count_if(result.segments.begin(), result.segments.end(), [](const auto & segment) {
      return std::find(segment.tags.begin(), segment.tags.end(), "strip") != segment.tags.end();
    });
  ASSERT_EQ(strip_count, 3);
}

TEST(BoustrophedonPlannerTest, StripSegmentsCarryRequiresContactFromParams)
{
  auto grid = makeGrid(4, 2, {0, 0, 0, 0, 0, 0, 0, 0});

  coverage_planner::BoustrophedonPlanner planner;
  auto params = defaultParams();
  params.requires_contact = true;
  auto result = planner.generatePath(grid, params);

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.segments.empty());
  for (const auto & segment : result.segments) {
    EXPECT_EQ(segment.mode, "cartesian");
    EXPECT_EQ(segment.planner, "cartesian");
    EXPECT_TRUE(segment.requires_contact);
    EXPECT_FALSE(segment.requires_retraction);
  }
}

TEST(BoustrophedonPlannerTest, ConnectorSegmentsAreFspAndRequireRetraction)
{
  // Same grid/step as ConnectorInterpolationHandlesNegativeXDirection above -- known to produce
  // exactly one accepted (non-trivial, multi-cell) connector.
  auto grid = makeGrid(4, 3, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 100});

  coverage_planner::BoustrophedonPlanner planner;
  auto params = defaultParams();
  params.start_corner = "top_left";
  params.step = 2.0;
  params.add_strip_connectors = true;
  params.requires_contact = true;  // connectors must ignore this -- always false/fsp/retraction.
  auto result = planner.generatePath(grid, params);

  ASSERT_TRUE(result.success);
  bool found_connector = false;
  for (const auto & segment : result.segments) {
    if (segment.mode == "fsp") {
      found_connector = true;
      EXPECT_EQ(segment.planner, "ompl");
      EXPECT_FALSE(segment.requires_contact);
      EXPECT_TRUE(segment.requires_retraction);
    }
  }
  EXPECT_TRUE(found_connector);
}

TEST(BoustrophedonPlannerTest, SegmentIndicesReferenceCorrectPosesInFinalPath)
{
  // Every emitted segment's [start_index, end_index] must point at the exact waypoints it
  // describes in the final, concatenated result.path -- this is what a downstream per-segment
  // consumer (ManipulationManager) actually indexes into.
  auto grid = makeGrid(6, 2, {0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0});

  coverage_planner::BoustrophedonPlanner planner;
  auto params = defaultParams();
  auto result = planner.generatePath(grid, params);

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.segments.empty());
  for (const auto & segment : result.segments) {
    ASSERT_LT(segment.end_index, result.path.poses.size());
    ASSERT_LE(segment.start_index, segment.end_index);
  }
}

TEST(BoustrophedonPlannerTest, BoundaryPassEmitsOneSegmentSpanningItsWaypoints)
{
  // Fully free interior -> the whole perimeter is one unbroken run, so this must still emit
  // exactly one boundary segment (see BoundaryPassSplitsAtObstacleGapInsteadOfBridgingIt below
  // for the case where an obstacle actually breaks the perimeter into more than one run).
  auto grid = makeGrid(6, 6, std::vector<int8_t>(36, 0));

  coverage_planner::BoustrophedonPlanner planner;
  auto params = defaultParams();
  params.enable_boundary_pass = true;
  params.boundary_offset = 1.0;
  auto result = planner.generatePath(grid, params);

  ASSERT_TRUE(result.success);
  int boundary_segment_count = 0;
  for (const auto & segment : result.segments) {
    if (std::find(segment.tags.begin(), segment.tags.end(), "boundary") != segment.tags.end()) {
      ++boundary_segment_count;
      ASSERT_LT(segment.end_index, result.path.poses.size());
      ASSERT_LE(segment.start_index, segment.end_index);
    }
  }
  EXPECT_EQ(boundary_segment_count, 1);
}

TEST(BoustrophedonPlannerTest, BoundaryPassSplitsAtObstacleGapInsteadOfBridgingIt)
{
  // An obstacle column sits on the perimeter's bottom edge (offset_min_y=1 row, x=3), so the
  // bottom-edge scan must skip it -- the fix under test is that this produces two boundary
  // segments (one on each side of the gap) instead of one segment whose waypoints jump straight
  // across the obstacle.
  std::vector<int8_t> cells(64, 0);
  cells[1 * 8 + 3] = 100;  // row y=1 (the offset bottom edge for an 8x8 grid, offset=1), x=3
  auto grid = makeGrid(8, 8, cells);

  coverage_planner::BoustrophedonPlanner planner;
  auto params = defaultParams();
  params.enable_boundary_pass = true;
  params.boundary_offset = 1.0;
  auto result = planner.generatePath(grid, params);

  ASSERT_TRUE(result.success);
  std::vector<const coverage_planner_msgs::msg::MotionPlanSegment *> boundary_segments;
  for (const auto & segment : result.segments) {
    if (std::find(segment.tags.begin(), segment.tags.end(), "boundary") != segment.tags.end()) {
      boundary_segments.push_back(&segment);
    }
  }
  ASSERT_EQ(boundary_segments.size(), 2u);

  // Every pair of consecutive poses actually drawn within a boundary run must be adjacent grid
  // cells (Chebyshev distance <= 1) -- i.e. no straight line bridging the skipped obstacle cell.
  for (const auto * segment : boundary_segments) {
    for (uint32_t i = segment->start_index; i < segment->end_index; ++i) {
      const auto & a = result.path.poses[i].pose.position;
      const auto & b = result.path.poses[i + 1].pose.position;
      const double resolution = grid.info.resolution;
      EXPECT_LE(std::abs(a.x - b.x), resolution * 1.5);
      EXPECT_LE(std::abs(a.y - b.y), resolution * 1.5);
    }
  }
}

TEST(BoustrophedonPlannerTest, SegmentIndicesSurviveOptimizeStripOrderReversal)
{
  // Two disjoint free-space components, far apart, that optimize_strip_order will want to
  // connect via a reversed traversal of one of them -- exercises the reversed-index remap path
  // in appendComponentSegments (component_size - 1 - segment.end_index, etc).
  const int width = 10;
  const int height = 4;
  std::vector<int8_t> data(width * height, 100);
  // Component A: a 2x2 free block at top-left.
  for (int y = 0; y < 2; ++y) {
    for (int x = 0; x < 2; ++x) {
      data[y * width + x] = 0;
    }
  }
  // Component B: a 2x2 free block at bottom-right, far from A.
  for (int y = 2; y < 4; ++y) {
    for (int x = 8; x < 10; ++x) {
      data[y * width + x] = 0;
    }
  }
  auto grid = makeGrid(width, height, data);

  coverage_planner::BoustrophedonPlanner planner;
  auto params = defaultParams();
  params.plan_all_components = true;
  params.optimize_strip_order = true;
  params.min_component_cells = 1;
  auto result = planner.generatePath(grid, params);

  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.segments.empty());
  for (const auto & segment : result.segments) {
    ASSERT_LT(segment.end_index, result.path.poses.size());
    ASSERT_LE(segment.start_index, segment.end_index);
  }
}
