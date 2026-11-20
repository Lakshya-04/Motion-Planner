#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "manipulation_manager/core/mission_fsm.hpp"
#include "manipulation_manager/core/mission_ledger.hpp"

using namespace manipulation_manager;

namespace
{
std::string tmpPath(const std::string & name)
{
  const auto p = std::filesystem::temp_directory_path() / ("ml_test_" + name + ".json");
  std::filesystem::remove(p);
  std::filesystem::remove(p.string() + ".corrupt");
  std::filesystem::remove(p.string() + ".tmp");
  return p.string();
}
}  // namespace

TEST(MissionLedger, RoundTrip)
{
  const auto path = tmpPath("rt");
  MissionLedger a(path);
  a.begin("m1", 4, false);
  a.markRunning(0, 0);
  a.markDone(0, 0);
  a.markSkipped(1, 0, SkipReason::kDockFailed);
  a.markFailed(2, 1);
  MissionLedger b(path);
  ASSERT_TRUE(b.load());
  EXPECT_EQ(b.mission(), "m1");
  EXPECT_EQ(b.size(), 4u);
  EXPECT_EQ(b.get(0).state, LedgerState::kDone);
  EXPECT_EQ(b.get(1).state, LedgerState::kSkipped);
  EXPECT_EQ(b.get(1).reason, SkipReason::kDockFailed);
  EXPECT_EQ(b.get(2).state, LedgerState::kFailed);
  EXPECT_EQ(b.get(2).stand, 1);
  EXPECT_EQ(b.get(3).state, LedgerState::kPending);
}

TEST(MissionLedger, AtomicWriteLeavesNoTempFile)
{
  const auto path = tmpPath("atomic");
  MissionLedger a(path);
  a.begin("m", 2, false);
  a.markDone(0, 0);
  EXPECT_TRUE(std::filesystem::exists(path));
  EXPECT_FALSE(std::filesystem::exists(path + ".tmp"));
  // A stale partial temp file from a crash does not disturb loading the last complete file.
  std::ofstream(path + ".tmp") << "{ \"version\": 1, \"miss";
  MissionLedger b(path);
  EXPECT_TRUE(b.load());
  EXPECT_TRUE(b.isDone(0));
}

TEST(MissionLedger, ResumeSkipsDoneAndRerunsRunning)
{
  const auto path = tmpPath("resume");
  {
    MissionLedger a(path);
    a.begin("m", 4, false);
    a.markDone(0, 0);
    a.markDone(1, 0);
    a.markRunning(2, 0);   // the process dies here
  }
  MissionLedger b(path);
  ASSERT_TRUE(b.load());
  EXPECT_TRUE(b.begin("m", 4, true));
  EXPECT_TRUE(b.resumeSkips(0));
  EXPECT_TRUE(b.resumeSkips(1));
  EXPECT_FALSE(b.resumeSkips(2));
  EXPECT_EQ(b.get(2).state, LedgerState::kPending);
  EXPECT_FALSE(b.resumeSkips(3));
}

TEST(MissionLedger, FreshStartOrOtherMissionClears)
{
  const auto path = tmpPath("fresh");
  MissionLedger a(path);
  a.begin("m", 2, false);
  a.markDone(0, 0);
  EXPECT_FALSE(a.begin("m", 2, false));
  EXPECT_FALSE(a.isDone(0));
  a.markDone(0, 0);
  EXPECT_FALSE(a.begin("other", 2, true));
  EXPECT_FALSE(a.isDone(0));
}

TEST(MissionLedger, OverloadIsNotRetriedOnResume)
{
  MissionLedger a;   // not persistent
  a.begin("m", 3, false);
  a.markSkipped(0, 0, SkipReason::kContactOverload);
  a.markSkipped(1, 0, SkipReason::kNavFailed);
  EXPECT_TRUE(a.resumeSkips(0));
  EXPECT_FALSE(a.resumeSkips(1));
}

TEST(MissionLedger, CorruptedFileIsMovedAsideAndTreatedAsEmpty)
{
  const auto path = tmpPath("corrupt");
  std::ofstream(path) << "{ not json";
  MissionLedger a(path);
  EXPECT_FALSE(a.load());
  EXPECT_EQ(a.size(), 0u);
  EXPECT_TRUE(std::filesystem::exists(path + ".corrupt"));
  EXPECT_FALSE(a.begin("m", 2, true));   // nothing to resume from
  EXPECT_FALSE(a.resumeSkips(0));
  std::ofstream(path)
    << R"({"version":1,"mission":"m","columns":[{"column":0,"stand":0,"state":"BOGUS","reason":"NONE"}]})";
  MissionLedger b(path);
  EXPECT_FALSE(b.load());
}

TEST(MissionLedger, MissingFileIsEmpty)
{
  MissionLedger a(tmpPath("missing"));
  EXPECT_TRUE(a.load());
  EXPECT_EQ(a.size(), 0u);
}

TEST(MissionLedger, MissionIdFollowsContent)
{
  EXPECT_EQ(missionIdFromBytes("abc"), missionIdFromBytes("abc"));
  EXPECT_NE(missionIdFromBytes("abc"), missionIdFromBytes("abd"));
  EXPECT_EQ(missionIdFromBytes("abc").size(), 16u);
}

// The acceptance scenario without the sim: kill mid-wall, restart, only the unfinished columns rerun.
TEST(MissionLedger, KillMidWallRestartRerunsOnlyUnfinished)
{
  const auto path = tmpPath("accept");
  {
    MissionLedger l(path);
    l.begin("wall", 6, false);
    for (size_t c = 0; c < 6; ++c) {
      l.markRunning(c, 0);
      if (c == 3) {break;}   // killed while column 3 runs
      l.markDone(c, 0);
    }
  }
  MissionLedger l(path);
  ASSERT_TRUE(l.load());
  ASSERT_TRUE(l.begin("wall", 6, true));
  std::vector<size_t> second_run;
  for (size_t c = 0; c < 6; ++c) {
    if (l.resumeSkips(c)) {continue;}
    l.markRunning(c, 0);
    second_run.push_back(c);
    l.markDone(c, 0);
  }
  EXPECT_EQ(second_run, (std::vector<size_t>{3, 4, 5}));
  EXPECT_EQ(l.columnsIn(LedgerState::kDone).size(), 6u);
}

TEST(MissionFsm, PauseTakesEffectAtBoundary)
{
  MissionFsm f;
  EXPECT_EQ(f.state(), MissionState::kIdle);
  EXPECT_FALSE(f.requestPause());
  ASSERT_TRUE(f.start());
  EXPECT_TRUE(f.requestPause());
  EXPECT_EQ(f.state(), MissionState::kActive);   // not yet: mid-segment
  EXPECT_TRUE(f.atBoundary());
  EXPECT_EQ(f.state(), MissionState::kPaused);
  EXPECT_TRUE(f.atBoundary());
  EXPECT_TRUE(f.resume());
  EXPECT_EQ(f.state(), MissionState::kActive);
  EXPECT_FALSE(f.atBoundary());
}

TEST(MissionFsm, ResumeWithdrawsPendingPause)
{
  MissionFsm f;
  f.start();
  f.requestPause();
  EXPECT_TRUE(f.resume());
  EXPECT_FALSE(f.atBoundary());
}

TEST(MissionFsm, StopFromAnyStateAndRecovery)
{
  MissionFsm f;
  EXPECT_FALSE(f.stop());
  f.start();
  EXPECT_TRUE(f.enterRecovery());
  EXPECT_EQ(f.state(), MissionState::kRecovery);
  EXPECT_FALSE(f.requestPause());
  EXPECT_TRUE(f.leaveRecovery());
  f.requestPause();
  f.atBoundary();
  EXPECT_TRUE(f.stop());   // cancel while paused
  EXPECT_EQ(f.state(), MissionState::kStopped);
  EXPECT_FALSE(f.resume());
  EXPECT_TRUE(f.start());
}

TEST(MissionCommand, PauseResumeStatusThroughTheTypedReply)
{
  MissionFsm f;
  auto r = applyMissionCommand(f, "status");
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(r.state, "IDLE");
  EXPECT_FALSE(r.pause_pending);

  EXPECT_FALSE(applyMissionCommand(f, "pause").accepted);   // idle: nothing to pause
  f.start();
  r = applyMissionCommand(f, "pause");
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(r.state, "ACTIVE");
  EXPECT_TRUE(r.pause_pending);                              // mid-column, not yet paused

  f.atBoundary();
  r = applyMissionCommand(f, "status");
  EXPECT_EQ(r.state, "PAUSED");

  r = applyMissionCommand(f, "resume");
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(r.state, "ACTIVE");
  EXPECT_FALSE(applyMissionCommand(f, "resume").accepted);   // nothing left to release
}

TEST(MissionCommand, UnknownCommandChangesNothing)
{
  MissionFsm f;
  f.start();
  const auto r = applyMissionCommand(f, "abort");
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(r.state, "ACTIVE");
  EXPECT_FALSE(f.pauseRequested());
  EXPECT_NE(r.message.find("unknown"), std::string::npos);
}

TEST(MissionCommand, PendingPauseIsWithdrawnByResume)
{
  MissionFsm f;
  f.start();
  applyMissionCommand(f, "pause");
  const auto r = applyMissionCommand(f, "resume");
  EXPECT_TRUE(r.accepted);
  EXPECT_FALSE(r.pause_pending);
  EXPECT_FALSE(f.atBoundary());
}
