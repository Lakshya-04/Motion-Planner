// Backend routing from a planning config: configs with `free_space: vamp` / `free_space: curobo`
// and `segments: {contact: cartesian, transition: free_space}` are parsed, built through
// PlanningPipeline, and segments are planned. The check is which backend receives which segment:
// transition segments go to the selected free-space backend, contact segments to MoveIt Cartesian.
//
// Every backend name is resolved through the pluginlib registry first. A real plugin that loads and
// initializes is used (VAMP in the full image; cuRobo with its node absent, where it answers
// "unavailable"); otherwise a fake takes its place. The MoveIt plugins are always faked here, they
// need a MoveIt stack. Each case prints whether the backend was real, so a run on the CPU image
// does not pass for a run on the full one.
#include <gtest/gtest.h>

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "motion_planner/core/planning_pipeline.hpp"
#include "motion_planner/ros/planning_config.hpp"

namespace mp = motion_planner;

namespace
{

struct Log
{
  std::map<std::string, int> calls;     // plugin name -> plan() calls
  std::map<std::string, bool> is_real;  // plugin name -> a real plugin answered
};

/// Counts plan() calls and delegates to the real plugin when it loaded and initialized.
class RoutingProbe : public mp::PlannerPlugin
{
public:
  RoutingProbe(
    std::string name, std::unique_ptr<mp::PlannerPlugin> real, std::shared_ptr<Log> log,
    bool cartesian_like)
  : name_(std::move(name)), real_(std::move(real)), log_(std::move(log)),
    cartesian_like_(cartesian_like) {}

  mp::PluginStatus initialize(mp::PluginContext & context, const mp::PluginParams & params) override
  {
    use_real_ = real_ && real_->initialize(context, params).ok;
    log_->is_real[name_] = use_real_;
    return mp::PluginStatus::success();
  }
  std::string name() const override {return name_;}
  mp::PlanningResult plan(const mp::PlanningRequest & request) override
  {
    ++log_->calls[name_];
    if (use_real_) {
      return real_->plan(request);
    }
    mp::PlanningResult r;
    // A fake Cartesian planner cannot do a joint-space move, so a transition falls through to
    // the free-space backend, as MoveIt Cartesian does on a joint goal.
    r.success = !cartesian_like_ || request.allow_target_contact;
    r.path_fraction = r.success ? 1.0 : 0.0;
    r.failure_reason_code = r.success ? "none" : "fake_cartesian_cannot_plan";
    return r;
  }
  mp::IkResult ik(const mp::IkRequest &) override {return mp::IkResult{};}
  mp::CollisionResult collision(const mp::CollisionRequest &) override
  {
    return mp::CollisionResult{};
  }

private:
  std::string name_;
  std::unique_ptr<mp::PlannerPlugin> real_;
  std::shared_ptr<Log> log_;
  bool cartesian_like_;
  bool use_real_{false};
};

class StubIk : public mp::IkPlugin
{
public:
  mp::PluginStatus initialize(mp::PluginContext &, const mp::PluginParams &) override
  {
    return mp::PluginStatus::success();
  }
  mp::IkResult ik(const mp::IkRequest &) override {return mp::IkResult{};}
};

class StubCollision : public mp::CollisionPlugin
{
public:
  mp::PluginStatus initialize(mp::PluginContext &, const mp::PluginParams &) override
  {
    return mp::PluginStatus::success();
  }
  mp::CollisionResult collision(const mp::CollisionRequest &) override
  {
    return mp::CollisionResult{};
  }
};

/// Registry that wraps the pluginlib registry: every planner name gets a probe, with the real
/// plugin inside when it loads. IK and collision are stubs (the pipeline only publishes them).
class ProbeRegistry : public mp::PluginRegistry
{
public:
  ProbeRegistry(std::shared_ptr<Log> log, std::vector<std::string> names)
  : real_(mp::makePluginlibRegistry()), log_(std::move(log)), names_(std::move(names)) {}

  std::unique_ptr<mp::PlannerPlugin> createPlanner(
    const std::string & name, mp::PluginStatus & status) override
  {
    bool known = false;
    for (const auto & n : names_) {known = known || n == name;}
    if (!known) {
      status = mp::PluginStatus::fail("unknown_plugin", "no planner '" + name + "'");
      return nullptr;
    }
    std::unique_ptr<mp::PlannerPlugin> real;
    if (name.rfind("moveit_", 0) != 0) {  // MoveIt needs a MoveIt stack: always a fake here
      mp::PluginStatus ignored;
      real = real_->createPlanner(name, ignored);
    }
    status = mp::PluginStatus::success();
    return std::make_unique<RoutingProbe>(name, std::move(real), log_, name == "moveit_cartesian");
  }
  std::unique_ptr<mp::IkPlugin> createIk(const std::string &, mp::PluginStatus & s) override
  {
    s = mp::PluginStatus::success();
    return std::make_unique<StubIk>();
  }
  std::unique_ptr<mp::CollisionPlugin> createCollision(
    const std::string &, mp::PluginStatus & s) override
  {
    s = mp::PluginStatus::success();
    return std::make_unique<StubCollision>();
  }
  std::unique_ptr<mp::ScenePlugin> createScene(const std::string &, mp::PluginStatus & s) override
  {
    s = mp::PluginStatus::fail("unknown_plugin", "no scene plugins in this test");
    return nullptr;
  }
  std::vector<std::string> names(const std::string & kind) const override
  {
    return kind == "planner" ? names_ : std::vector<std::string>{};
  }

private:
  std::unique_ptr<mp::PluginRegistry> real_;
  std::shared_ptr<Log> log_;
  std::vector<std::string> names_;
};

mp::PlanningRequest jointRequest(bool contact)
{
  mp::PlanningRequest r;
  r.waypoint_indices = {0};
  r.min_path_fraction = 0.5;
  r.allow_target_contact = contact;
  r.start_joint_state = {0, -1.5, 1.5, -1.5, -1.5, 0};
  r.target_joint_state = {0.3, -1.2, 1.2, -1.5, -1.5, 0.1};
  return r;
}

struct Outcome
{
  std::string mode;
  std::map<std::string, int> calls;
  std::map<std::string, bool> real;
};

/// Parses the config, builds the pipeline and plans one segment; returns what each backend saw.
Outcome route(const std::string & yaml, bool contact, const std::string & hint)
{
  mp::PlanningSelection selection;
  std::string error;
  EXPECT_TRUE(mp::parsePlanningSelection(yaml, selection, error)) << error;

  auto log = std::make_shared<Log>();
  ProbeRegistry registry(log, {"moveit_cartesian", "moveit_ompl", "vamp", "curobo"});
  mp::PluginContext context;
  context.setFactory<rclcpp::Node>(
    "node", [] {return std::make_shared<rclcpp::Node>("backend_routing_test");});
  mp::PlanningPipeline pipeline;
  mp::PipelineError pipeline_error;
  EXPECT_TRUE(pipeline.build(selection, registry, context, pipeline_error))
    << pipeline_error.message;
  auto planner = pipeline.makeMotionPlanner();
  EXPECT_TRUE(planner != nullptr);
  Outcome out;
  if (!planner) {return out;}
  mp::SegmentRoutingSpec routing;
  routing.mode_hint = hint;
  out.mode = planner->planSegment(jointRequest(contact), routing).mode_used;
  out.calls = log->calls;
  out.real = log->is_real;
  return out;
}

int count(const Outcome & o, const std::string & name)
{
  const auto it = o.calls.find(name);
  return it == o.calls.end() ? 0 : it->second;
}

std::string config(const std::string & free_space, const std::string & params)
{
  return "planning:\n  ik: moveit_ik\n  collision: moveit_collision\n"
         "  cartesian: moveit_cartesian\n  free_space: " + free_space +
         "\n  segments: {contact: cartesian, transition: free_space}\n"
         "  plugin_params:\n" + params + "\n";
}

class BackendRouting : public ::testing::TestWithParam<std::pair<std::string, std::string>> {};

}  // namespace

TEST_P(BackendRouting, TransitionGoesToTheSelectedBackendContactToMoveIt)
{
  const auto & backend = GetParam().first;
  const std::string yaml = config(backend, GetParam().second);

  // Transition: even hinted "cartesian", the override sends it to free space after the Cartesian
  // attempt (which cannot plan a joint goal) falls short.
  const auto transition = route(yaml, false, "cartesian");
  EXPECT_EQ(transition.mode, "fsp");
  EXPECT_EQ(count(transition, backend), 1);
  EXPECT_EQ(count(transition, "moveit_ompl"), 0);
  EXPECT_EQ(count(transition, "moveit_cartesian"), 1);

  // Contact: hinted "fsp", the override keeps it on MoveIt Cartesian; the backend is never asked.
  const auto contact = route(yaml, true, "fsp");
  EXPECT_EQ(contact.mode, "cartesian");
  EXPECT_EQ(count(contact, "moveit_cartesian"), 1);
  EXPECT_EQ(count(contact, backend), 0);

  const auto it = transition.real.find(backend);
  const bool real = it != transition.real.end() && it->second;
  std::cout << "[routing] " << backend << ": "
            << (real ? "real plugin" : "fake (real plugin not available here)") << "\n";
}

INSTANTIATE_TEST_SUITE_P(
  Backends, BackendRouting,
  ::testing::Values(
    std::make_pair(std::string("vamp"), std::string("    vamp: {robot: ur5}")),
    std::make_pair(
      std::string("curobo"),
      std::string("    curobo: {wait_for_service_sec: 0.1, timeout_sec: 0.2}"))));

TEST(BackendRoutingDefault, MoveItOmplIsTheFreeSpaceBackendWithoutAnOverride)
{
  const auto transition = route(config("moveit_ompl", "    {}"), false, "fsp");
  EXPECT_EQ(transition.mode, "fsp");
  EXPECT_EQ(count(transition, "moveit_ompl"), 1);
  EXPECT_EQ(count(transition, "vamp"), 0);
  EXPECT_EQ(count(transition, "curobo"), 0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
