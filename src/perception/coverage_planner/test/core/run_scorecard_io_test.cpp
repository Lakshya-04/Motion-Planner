#include <gtest/gtest.h>

#include <sstream>
#include <stdexcept>

#include "coverage_planner/core/run_scorecard_io.hpp"

using namespace coverage_planner;  // NOLINT

TEST(RunScorecardIo, ParsesRowsSkippingCommentsAndBlankLines)
{
  std::istringstream in("# a recording\nt,x,y,z,force,active\n\n0.0,1,2,3,4.5,0\n0.1,1,2,3,5.5,1\r\n");
  const auto s = parseRunCsv(in);
  ASSERT_EQ(s.size(), 2U);
  EXPECT_DOUBLE_EQ(s[0].force_n, 4.5);
  EXPECT_FALSE(s[0].tool_active);
  EXPECT_TRUE(s[1].tool_active);
  EXPECT_DOUBLE_EQ(s[1].t_s, 0.1);
}

TEST(RunScorecardIo, RejectsBadCsv)
{
  std::istringstream wrong_header("time,x\n1,2\n");
  EXPECT_THROW(parseRunCsv(wrong_header), std::runtime_error);
  std::istringstream short_row("t,x,y,z,force,active\n1,2,3\n");
  EXPECT_THROW(parseRunCsv(short_row), std::runtime_error);
  std::istringstream not_number("t,x,y,z,force,active\n1,2,3,4,abc,0\n");
  EXPECT_THROW(parseRunCsv(not_number), std::runtime_error);
  std::istringstream empty("");
  EXPECT_THROW(parseRunCsv(empty), std::runtime_error);
}

TEST(RunScorecardIo, WallFromJsonRoundTripsTheBasisAndRanges)
{
  const auto j = nlohmann::json::parse(
    R"({"origin":[1,0,0],"u":[0,1,0],"v":[0,0,1],"u_range":[0,2],"v_range":[0,1.5]})");
  const auto wall = wallFromJson(j);
  EXPECT_DOUBLE_EQ(wall.u_max, 2.0);
  EXPECT_DOUBLE_EQ(wall.v_max, 1.5);
  // a point on the wall at u = 0.5, v = 0.5 sits at world (1, 0.5, 0.5): score a run along it
  std::vector<RunSample> run;
  for (double u = 0.0; u <= 2.0; u += 0.02) {run.push_back({u, 1.0, u, 0.5, 10.0, false});}
  ScorecardOptions o;
  o.swath_m = 0.2; o.resolution_m = 0.05;
  const auto card = scoreRun(wall, {}, run, o);
  EXPECT_NEAR(card.coverage.covered_fraction, 0.2 / 1.5, 0.05);   // one 0.2 m band across the 1.5 m height
}

TEST(RunScorecardIo, ObstaclesFromJson)
{
  const auto j = nlohmann::json::parse(
    R"({"obstacles":[{"id":4,"u":[0.1,0.2],"v":[0.3,0.5],"height_m":0.04},{"u":[1,2],"v":[1,2]}]})");
  const auto o = obstaclesFromJson(j, 7);
  ASSERT_EQ(o.size(), 2U);
  EXPECT_EQ(o[0].id, 4U);
  EXPECT_EQ(o[0].parent_wall, 7U);
  EXPECT_DOUBLE_EQ(o[0].height_m, 0.04);
  EXPECT_EQ(o[1].id, 5U);
  EXPECT_DOUBLE_EQ(o[1].height_m, 0.0);
  EXPECT_TRUE(obstaclesFromJson(nlohmann::json::parse("{}"), 1).empty());
  EXPECT_THROW(
    obstaclesFromJson(nlohmann::json::parse(R"({"obstacles":[{"u":[2,1],"v":[0,1]}]})"), 1), std::runtime_error);
  EXPECT_THROW(
    obstaclesFromJson(nlohmann::json::parse(R"({"obstacles":[{"v":[0,1]}]})"), 1), std::runtime_error);
}

TEST(RunScorecardIo, CardJsonCarriesObstacleContacts)
{
  RunScorecard c;
  c.obstacle_contacts = 2;
  c.obstacle_contact_samples = 9;
  const auto j = scorecardToJson(c);
  EXPECT_EQ(j["obstacle"]["contacts"], 2);
  EXPECT_EQ(j["obstacle"]["contact_samples"], 9);
}

TEST(RunScorecardIo, RejectsABadWall)
{
  EXPECT_THROW(wallFromJson(nlohmann::json::parse(R"({"origin":[0,0,0]})")), std::runtime_error);
  EXPECT_THROW(wallFromJson(nlohmann::json::parse(
    R"({"origin":[0,0,0],"u":[1,0,0],"v":[1,0,0],"u_range":[0,1],"v_range":[0,1]})")), std::runtime_error);
  EXPECT_THROW(wallFromJson(nlohmann::json::parse(
    R"({"origin":[0,0,0],"u":[1,0,0],"v":[0,1,0],"u_range":[1,0],"v_range":[0,1]})")), std::runtime_error);
}

TEST(RunScorecardIo, CardJsonCarriesTheNumbers)
{
  RunScorecard c;
  c.passed = false;
  c.failures = {"coverage 0.5 < 0.95"};
  c.coverage.covered_fraction = 0.5;
  const auto j = scorecardToJson(c);
  EXPECT_FALSE(j.at("passed").get<bool>());
  EXPECT_DOUBLE_EQ(j.at("coverage").at("covered_fraction").get<double>(), 0.5);
  EXPECT_EQ(j.at("failures").size(), 1U);
}
