#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "coverage_planner/core/run_scorecard.hpp"
#include "coverage_planner/core/run_scorecard_io.hpp"

using namespace coverage_planner;  // NOLINT

namespace
{
// A 2 m x 1 m wall at the origin with u = x, v = y (identity pose).
WallPlane wall()
{
  WallPlane w;
  w.id = 1;
  w.pose.pose.orientation.w = 1.0;
  w.u_min = 0.0; w.u_max = 2.0; w.v_min = 0.0; w.v_max = 1.0;
  return w;
}

// Vertical strokes every `spacing`, tool pressing at `force`, tool lifted (force 0) between them.
std::vector<RunSample> run(double spacing, double force, double swath_unused = 0.0)
{
  (void)swath_unused;
  std::vector<RunSample> out;
  double t = 0.0;
  for (double u = 0.1; u <= 1.9001; u += spacing) {
    for (double y = 0.0; y <= 1.0001; y += 0.05) {
      RunSample s;
      s.t_s = t; t += 0.1;
      s.x = u; s.y = y; s.z = 0.0;
      s.force_n = force;
      out.push_back(s);
    }
    RunSample lift;   // retract between strokes
    lift.t_s = t; t += 0.1;
    lift.x = u; lift.y = 1.0; lift.force_n = 0.0;
    out.push_back(lift);
  }
  return out;
}

ScorecardOptions contactOptions()
{
  ScorecardOptions o;
  o.contact = true; o.swath_m = 0.2; o.resolution_m = 0.01;
  o.engaged_force_n = 1.0; o.target_force_n = 10.0; o.force_band_n = 1.0;
  return o;
}
}  // namespace

TEST(RunScorecard, GoodRunPasses)
{
  ScorecardLimits limits;
  limits.min_covered_fraction = 0.95; limits.max_force_rms_error_n = 0.5; limits.max_force_peak_n = 15.0;
  limits.min_in_band_fraction = 0.9;
  const auto card = scoreRun(wall(), {}, run(0.16, 10.1), contactOptions(), limits);
  EXPECT_TRUE(card.passed) << (card.failures.empty() ? "" : card.failures.front());
  EXPECT_GT(card.coverage.covered_fraction, 0.95);
  EXPECT_NEAR(card.force_rms_error_n, 0.1, 1e-9);
  EXPECT_DOUBLE_EQ(card.in_band_fraction, 1.0);
  EXPECT_EQ(card.strokes, 12U);   // 0.1 .. 1.86: one stroke per vertical pass, split by the lifts
}

TEST(RunScorecard, GapsInTheWallFailCoverage)
{
  const auto card = scoreRun(wall(), {}, run(0.5, 10.0), contactOptions());   // spacing 2.5x the swath
  EXPECT_FALSE(card.passed);
  EXPECT_LT(card.coverage.covered_fraction, 0.6);
  ASSERT_FALSE(card.failures.empty());
  EXPECT_NE(card.failures.front().find("coverage"), std::string::npos);
}

TEST(RunScorecard, ForceOffTargetAndOverPeakFail)
{
  ScorecardLimits limits;
  limits.max_force_rms_error_n = 0.5; limits.max_force_peak_n = 12.0;
  const auto card = scoreRun(wall(), {}, run(0.16, 14.0), contactOptions(), limits);
  EXPECT_FALSE(card.passed);
  EXPECT_NEAR(card.force_rms_error_n, 4.0, 1e-9);
  EXPECT_DOUBLE_EQ(card.force_peak_n, 14.0);
  EXPECT_GE(card.failures.size(), 2U);
}

TEST(RunScorecard, ObstaclesAreExcludedFromTheTarget)
{
  ObstacleRegion hole;
  hole.parent_wall = 1; hole.u_min = 0.9; hole.u_max = 1.1; hole.v_min = 0.0; hole.v_max = 1.0;
  const auto without = scoreRun(wall(), {}, run(0.5, 10.0), contactOptions());
  const auto with = scoreRun(wall(), {hole}, run(0.5, 10.0), contactOptions());
  EXPECT_LT(with.coverage.target_cells, without.coverage.target_cells);
  EXPECT_NEAR(static_cast<double>(with.coverage.target_cells), 180.0 * 100.0, 1.0);
}

TEST(RunScorecard, NonContactCountsWhileTheValveIsOpen)
{
  auto samples = run(0.16, 0.0);   // no force at all: a spray tool does not press
  for (auto & s : samples) {s.tool_active = (s.y < 1.0);}   // valve closed during the lifts
  ScorecardOptions o = contactOptions();
  o.contact = false;
  const auto card = scoreRun(wall(), {}, samples, o);
  EXPECT_GT(card.coverage.covered_fraction, 0.95);
  EXPECT_DOUBLE_EQ(card.force_rms_error_n, 0.0);   // force statistics are for contact tools
  EXPECT_DOUBLE_EQ(card.in_band_fraction, 0.0);
}

TEST(RunScorecard, SprayRunCountsOnlyValveOpenSamplesAndReportsThickness)
{
  auto samples = run(0.12, 0.0);
  for (auto & s : samples) {s.tool_active = (s.y < 1.0);}
  ScorecardOptions o = contactOptions();
  o.contact = false;
  o.swath_m = 0.14;
  o.spray = SprayFootprint::fromCone(0.30, 20.0);
  ScorecardLimits limits;
  limits.min_covered_fraction = 0.9;
  const auto card = scoreRun(wall(), {}, samples, o, limits);
  EXPECT_TRUE(card.passed);
  ASSERT_TRUE(card.coverage.thickness.valid);
  EXPECT_GT(card.coverage.thickness.mean, 0.0);
  EXPECT_GE(card.coverage.thickness.cov, 0.0);
  EXPECT_TRUE(scorecardToJson(card).contains("thickness"));

  // the valve closed for the whole run: nothing counted, no coverage, fails
  auto closed = samples;
  for (auto & s : closed) {s.tool_active = false;}
  const auto none = scoreRun(wall(), {}, closed, o, limits);
  EXPECT_FALSE(none.passed);
  EXPECT_DOUBLE_EQ(none.coverage.covered_fraction, 0.0);

  // a thickness CoV limit is enforced
  limits.max_thickness_cov = 1e-6;
  EXPECT_FALSE(scoreRun(wall(), {}, samples, o, limits).passed);
}

TEST(RunScorecard, ContactRunHasNoThicknessSection)
{
  const auto card = scoreRun(wall(), {}, run(0.16, 10.0), contactOptions());
  EXPECT_FALSE(card.coverage.thickness.valid);
  EXPECT_FALSE(scorecardToJson(card).contains("thickness"));
}

TEST(RunScorecard, DurationAndEngagedTime)
{
  const auto samples = run(0.16, 10.0);
  const auto card = scoreRun(wall(), {}, samples, contactOptions());
  EXPECT_NEAR(card.duration_s, samples.back().t_s - samples.front().t_s, 1e-9);
  EXPECT_GT(card.engaged_s, 0.0);
  EXPECT_LT(card.engaged_s, card.duration_s);
}

TEST(RunScorecard, EmptyRunCoversNothingAndRejectsBadOptions)
{
  const auto card = scoreRun(wall(), {}, {}, contactOptions());
  EXPECT_FALSE(card.passed);
  EXPECT_DOUBLE_EQ(card.coverage.covered_fraction, 0.0);
  ScorecardOptions bad = contactOptions();
  bad.swath_m = 0.0;
  EXPECT_THROW(scoreRun(wall(), {}, {}, bad), std::invalid_argument);
}

namespace
{
ObstacleRegion box()
{
  ObstacleRegion o;
  o.id = 3; o.parent_wall = 1;
  o.u_min = 0.9; o.u_max = 1.1; o.v_min = 0.4; o.v_max = 0.6;
  o.height_m = 0.04;
  return o;
}

RunSample at(double u, double v, double standoff)
{
  RunSample s;
  s.x = u; s.y = v; s.z = -standoff;   // the wall's normal u x v is +z: standing off the wall is -z
  s.force_n = 0.0;
  return s;
}
}  // namespace

TEST(RunScorecard, ObstacleContactsCountEpisodesNotSamples)
{
  std::vector<RunSample> s;
  for (int i = 0; i < 5; ++i) {s.push_back(at(1.0, 0.5, 0.0));}   // pressing on the box's footprint
  s.push_back(at(0.5, 0.5, 0.0));                                 // away
  for (int i = 0; i < 3; ++i) {s.push_back(at(1.0, 0.5, 0.02));}  // again, still below its top
  const auto card = scoreRun(wall(), {box()}, s, contactOptions());
  EXPECT_EQ(card.obstacle_contact_samples, 8U);
  EXPECT_EQ(card.obstacle_contacts, 2U);
}

TEST(RunScorecard, PassingOverAnObstacleOrBesideItIsNotAContact)
{
  std::vector<RunSample> s = {at(1.0, 0.5, 0.06), at(1.0, 0.5, 0.2), at(0.85, 0.5, 0.0), at(1.0, 0.35, 0.0)};
  EXPECT_EQ(scoreRun(wall(), {box()}, s, contactOptions()).obstacle_contacts, 0U);
  // another wall's obstacle is ignored
  ObstacleRegion other = box();
  other.parent_wall = 2;
  EXPECT_EQ(scoreRun(wall(), {other}, {at(1.0, 0.5, 0.0)}, contactOptions()).obstacle_contacts, 0U);
}

TEST(RunScorecard, ObstacleMarginGrowsTheFootprintForTheContactCheck)
{
  ScorecardOptions o = contactOptions();
  const std::vector<RunSample> s = {at(0.88, 0.5, 0.0)};   // 2 cm left of the box
  EXPECT_EQ(scoreRun(wall(), {box()}, s, o).obstacle_contacts, 0U);
  o.obstacle_margin_m = 0.03;
  EXPECT_EQ(scoreRun(wall(), {box()}, s, o).obstacle_contacts, 1U);
}

TEST(RunScorecard, FlushOpeningCountsOnlyAtTheWallPlane)
{
  ObstacleRegion hole = box();
  hole.height_m = 0.0;
  EXPECT_EQ(scoreRun(wall(), {hole}, {at(1.0, 0.5, 0.0)}, contactOptions()).obstacle_contacts, 1U);
  EXPECT_EQ(scoreRun(wall(), {hole}, {at(1.0, 0.5, 0.03)}, contactOptions()).obstacle_contacts, 0U);
}

TEST(RunScorecard, ContactLimitFailsTheRun)
{
  ScorecardLimits l;
  l.min_covered_fraction = 0.0;
  l.max_obstacle_contacts = 0;
  const auto card = scoreRun(wall(), {box()}, {at(1.0, 0.5, 0.0)}, contactOptions(), l);
  EXPECT_FALSE(card.passed);
  ASSERT_FALSE(card.failures.empty());
  EXPECT_NE(card.failures.back().find("obstacle contacts 1"), std::string::npos);
  l.max_obstacle_contacts = -1;   // not checked
  EXPECT_TRUE(scoreRun(wall(), {box()}, {at(1.0, 0.5, 0.0)}, contactOptions(), l).passed);
}

TEST(RunScorecard, KeepoutMarginShrinksTheCoverageTarget)
{
  const auto plain = scoreRun(wall(), {box()}, {}, contactOptions());
  ScorecardOptions o = contactOptions();
  o.keepout_margin_m = 0.1;
  const auto keepout = scoreRun(wall(), {box()}, {}, o);
  const auto none = scoreRun(wall(), {}, {}, contactOptions());
  EXPECT_LT(plain.coverage.target_cells, none.coverage.target_cells);
  EXPECT_LT(keepout.coverage.target_cells, plain.coverage.target_cells);
}

namespace
{
// The good run, with every sample `standoff` in front of the wall and tilted by `tilt_deg`.
std::vector<RunSample> poseRun(double standoff, double tilt_deg)
{
  auto s = run(0.16, 10.1);
  for (auto & r : s) {r.z = -standoff; r.tilt_deg = tilt_deg;}
  return s;
}
}  // namespace

TEST(RunScorecard, PoseGateOffKeepsTheOldBehaviour)
{
  // far from the wall and tilted 40 degrees, but force reads 10 N: counted, as before the gate existed
  const auto card = scoreRun(wall(), {}, poseRun(0.2, 40.0), contactOptions());
  EXPECT_GT(card.coverage.covered_fraction, 0.95);
  EXPECT_EQ(card.bad_pose_samples, 0U);
}

TEST(RunScorecard, ForceWithTheToolOffTheWallIsNotCoverage)
{
  ScorecardOptions o = contactOptions();
  o.max_wall_gap_m = 0.01;
  ScorecardLimits l;
  l.max_bad_pose_fraction = 0.05;
  const auto card = scoreRun(wall(), {}, poseRun(0.05, 0.0), o, l);   // 5 cm off the plane: something else is pressing
  EXPECT_FALSE(card.passed);
  EXPECT_LT(card.coverage.covered_fraction, 0.01);
  EXPECT_GT(card.off_wall_samples, 0U);
  EXPECT_EQ(card.tilted_samples, 0U);
  EXPECT_NEAR(card.bad_pose_fraction, 1.0, 1e-9);
}

TEST(RunScorecard, TiltedToolIsNotCoverageAndTheSquareOneIs)
{
  ScorecardOptions o = contactOptions();
  o.max_wall_gap_m = 0.01;
  o.max_tilt_deg = 10.0;
  ScorecardLimits l;
  l.max_bad_pose_fraction = 0.05;
  const auto tilted = scoreRun(wall(), {}, poseRun(0.002, 25.0), o, l);
  EXPECT_FALSE(tilted.passed);
  EXPECT_GT(tilted.tilted_samples, 0U);
  EXPECT_EQ(tilted.off_wall_samples, 0U);

  const auto square = scoreRun(wall(), {}, poseRun(0.002, 3.0), o, l);
  EXPECT_TRUE(square.passed) << (square.failures.empty() ? "" : square.failures.front());
  EXPECT_NEAR(square.tilt_mean_deg, 3.0, 1e-9);
  EXPECT_NEAR(square.wall_gap_mean_m, 0.002, 1e-9);
  EXPECT_EQ(square.bad_pose_samples, 0U);
}

TEST(RunScorecard, UnrecordedTiltIsNotChecked)
{
  ScorecardOptions o = contactOptions();
  o.max_tilt_deg = 10.0;
  const auto card = scoreRun(wall(), {}, poseRun(0.0, -1.0), o);
  EXPECT_EQ(card.tilted_samples, 0U);
  EXPECT_GT(card.coverage.covered_fraction, 0.95);
}

TEST(RunScorecard, AngledToolIsScoredAgainstItsIntendedTilt)
{
  ScorecardOptions o = contactOptions();
  o.max_wall_gap_m = 0.01;
  o.max_tilt_deg = 5.0;
  o.nominal_tilt_deg = 30.0;
  ScorecardLimits l;
  l.max_bad_pose_fraction = 0.05;
  EXPECT_TRUE(scoreRun(wall(), {}, poseRun(0.002, 31.0), o, l).passed);    // at its intended 30 degrees
  const auto flat = scoreRun(wall(), {}, poseRun(0.002, 0.0), o, l);       // held flat: not what this tool should do
  EXPECT_FALSE(flat.passed);
  EXPECT_GT(flat.tilted_samples, 0U);
}

TEST(RunScorecard, GatedRunStillReportsItsTiltAndGap)
{
  ScorecardOptions o = contactOptions();
  o.max_wall_gap_m = 0.01;
  o.max_tilt_deg = 10.0;
  const auto card = scoreRun(wall(), {}, poseRun(0.002, 25.0), o);   // held at 25 degrees: every sample refused
  EXPECT_LT(card.coverage.covered_fraction, 0.01);
  EXPECT_GT(card.tilted_samples, 0U);
  EXPECT_NEAR(card.tilt_mean_deg, 25.0, 1e-9);
  EXPECT_NEAR(card.tilt_max_deg, 25.0, 1e-9);
  EXPECT_NEAR(card.wall_gap_mean_m, 0.002, 1e-9);
  EXPECT_NEAR(card.wall_gap_max_abs_m, 0.002, 1e-9);
}

TEST(RunScorecard, NonContactRunIgnoresTheWallGapGate)
{
  auto samples = poseRun(0.30, 0.0);   // a spray nozzle 0.30 m from the wall
  for (auto & s : samples) {s.force_n = 0.0; s.tool_active = (s.y < 1.0);}
  ScorecardOptions o = contactOptions();
  o.contact = false;
  o.max_wall_gap_m = 0.01;
  const auto card = scoreRun(wall(), {}, samples, o);
  EXPECT_EQ(card.bad_pose_samples, 0U);
  EXPECT_EQ(card.off_wall_samples, 0U);
  EXPECT_GT(card.coverage.covered_fraction, 0.95);
  EXPECT_NEAR(card.wall_gap_mean_m, 0.30, 1e-9);   // still reported
}

TEST(RunScorecard, GateOptionChecks)
{
  ScorecardOptions o = contactOptions();
  ScorecardLimits l;
  EXPECT_NO_THROW(validateGateOptions(o, l));
  l.max_bad_pose_fraction = 0.05;
  EXPECT_THROW(validateGateOptions(o, l), std::runtime_error);   // no gate to produce bad poses
  o.max_wall_gap_m = 0.01;
  EXPECT_NO_THROW(validateGateOptions(o, l));
  o.contact = false;                                              // the gap gate is ignored without contact
  EXPECT_THROW(validateGateOptions(o, l), std::runtime_error);
  o.max_tilt_deg = 10.0;
  EXPECT_NO_THROW(validateGateOptions(o, l));
  ScorecardOptions n = contactOptions();
  n.nominal_tilt_deg = 30.0;
  EXPECT_THROW(validateGateOptions(n, ScorecardLimits{}), std::runtime_error);
  n.max_tilt_deg = 5.0;
  EXPECT_NO_THROW(validateGateOptions(n, ScorecardLimits{}));
}
