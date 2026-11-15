// VAMP side of docker/tools/bench_spheres.py: plans the benchmark problems with one Cricket-generated
// robot header and writes one JSON object per line.
//
//   g++ -O3 -std=c++17 -march=x86-64-v3 -mavx2 -DROBOT_HEADER='"/hdr/UR10eFoam.hh"' -DROBOT_STRUCT=UR10eFoam \
//       -I/opt/vamp/include -I/usr/include/eigen3 vamp_bench.cpp -L/opt/vamp/lib -lsimdxorshift -lcdd -o vamp_bench
//   vamp_bench vamp_problems.txt out.jsonl <set-label>
//
// The planner call is the same as src/manipulation/vamp_planner (RRT-Connect, then shortcut/B-spline
// simplification with the default settings, Halton sampler reset per plan; max 100000 iterations,
// range 2.0), so the numbers describe what that adapter would do. Timing is wall clock
// (steady_clock) around solve + simplify only; the path copy-out is outside it.
//
// Input (text): "S id nboxes nproblems nprobes", then nboxes lines "B cx cy cz  xx xy xz  yx yy yz
// zx zy zz  hx hy hz" (centre, box axes, half extents), nproblems lines "P id q0..q5 g0..g5", nprobes
// lines "Q q0..q5". Probe states only get a validity bit (cross-check of the model frames against
// the numpy sphere model), plus micro timings of the collision kernel.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <vamp/collision/environment.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/halton.hh>
#include <vamp/vector.hh>

#include ROBOT_HEADER

using Robot = vamp::robots::ROBOT_STRUCT;
constexpr std::size_t kRake = vamp::FloatVectorWidth;
constexpr std::size_t kDim = Robot::dimension;
using Configuration = typename Robot::Configuration;
using ConfigurationArray = typename Robot::ConfigurationArray;
using EnvVector = vamp::collision::Environment<vamp::FloatVector<kRake>>;
using Clock = std::chrono::steady_clock;

static Configuration toConfig(const std::vector<float> & q)
{
  ConfigurationArray a{};
  for (std::size_t i = 0; i < kDim; ++i) {
    a[i] = q[i];
  }
  return Configuration(a);
}

struct Result
{
  bool ok{false};
  double us{0};
  std::vector<std::vector<float>> path;
};

static Result plan(const std::vector<float> & s, const std::vector<float> & g, const EnvVector & env)
{
  using Planner = vamp::planning::RRTC<Robot, kRake, Robot::resolution>;
  vamp::planning::RRTCSettings settings;
  settings.max_iterations = 100000;
  settings.max_samples = 100000;
  settings.range = 2.0F;
  auto rng = std::make_shared<vamp::rng::Halton<Robot>>();
  Result r;
  const auto t0 = Clock::now();
  auto result = Planner::solve(toConfig(s), toConfig(g), env, settings, rng);
  if (!result.path.empty()) {
    rng->reset();
    auto simplified = vamp::planning::simplify<Robot, kRake, Robot::resolution>(
      result.path, env, vamp::planning::SimplifySettings{}, rng);
    if (!simplified.path.empty()) {
      result.path = std::move(simplified.path);
    }
  }
  r.us = std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
  r.ok = !result.path.empty();
  for (const auto & c : result.path) {
    const auto arr = c.to_array();
    r.path.emplace_back(arr.begin(), arr.begin() + kDim);
  }
  return r;
}

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::fprintf(stderr, "usage: %s problems.txt out.jsonl label\n", argv[0]);
    return 2;
  }
  std::ifstream in(argv[1]);
  std::FILE * out = std::fopen(argv[2], "w");
  if (!in || !out) {
    std::fprintf(stderr, "cannot open files\n");
    return 2;
  }
  std::fprintf(
    out, "{\"type\":\"header\",\"backend\":\"vamp\",\"set\":\"%s\",\"robot\":\"%s\",\"n_spheres\":%zu,\"dim\":%zu}\n",
    argv[3], Robot::name, static_cast<std::size_t>(Robot::n_spheres), kDim);

  std::string line;
  bool first_call_done = false;
  std::mt19937 gen(0);
  std::uniform_real_distribution<float> uni(-3.14159265F, 3.14159265F);
  while (std::getline(in, line)) {
    if (line.empty() || line[0] != 'S') {
      continue;
    }
    std::istringstream hs(line.substr(1));
    int scene, nb, np, nq;
    hs >> scene >> nb >> np >> nq;
    vamp::collision::Environment<float> env;
    std::vector<std::pair<int, std::vector<float>>> problems;
    std::vector<std::vector<float>> probes;
    for (int i = 0; i < nb + np + nq; ++i) {
      std::getline(in, line);
      std::istringstream ls(line.substr(1));
      std::vector<float> v;
      int id = 0;
      if (line[0] == 'P') {
        ls >> id;
      }
      float x;
      while (ls >> x) {
        v.push_back(x);
      }
      if (line[0] == 'B') {
        env.cuboids.emplace_back(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10], v[11],
          v[12], v[13], v[14]);
      } else if (line[0] == 'P') {
        problems.emplace_back(id, v);
      } else {
        probes.push_back(v);
      }
    }
    env.sort();
    const EnvVector env_v(env);

    if (!first_call_done && !problems.empty()) {
      const std::vector<float> s(problems[0].second.begin(), problems[0].second.begin() + kDim);
      const std::vector<float> g(problems[0].second.begin() + kDim, problems[0].second.end());
      const auto r = plan(s, g, env_v);
      std::fprintf(out, "{\"type\":\"first_call\",\"us\":%.1f}\n", r.us);
      first_call_done = true;
    }
    for (const auto & p : problems) {
      const std::vector<float> s(p.second.begin(), p.second.begin() + kDim);
      const std::vector<float> g(p.second.begin() + kDim, p.second.end());
      const bool s_ok = vamp::planning::validate_motion<Robot, kRake, 1>(toConfig(s), toConfig(s), env_v);
      const bool g_ok = vamp::planning::validate_motion<Robot, kRake, 1>(toConfig(g), toConfig(g), env_v);
      const auto r = plan(s, g, env_v);
      std::fprintf(out, "{\"type\":\"plan\",\"scene\":%d,\"id\":%d,\"success\":%d,\"us\":%.1f,\"endpoints_valid\":[%d,%d],\"path\":[",
        scene, p.first, r.ok ? 1 : 0, r.us, s_ok ? 1 : 0, g_ok ? 1 : 0);
      for (std::size_t i = 0; i < r.path.size(); ++i) {
        std::fprintf(out, "%s[", i ? "," : "");
        for (std::size_t j = 0; j < kDim; ++j) {
          std::fprintf(out, "%s%.5f", j ? "," : "", r.path[i][j]);
        }
        std::fprintf(out, "]");
      }
      std::fprintf(out, "]}\n");
    }

    // probe validity (frame cross-check) and micro timings of the collision kernel
    std::fprintf(out, "{\"type\":\"probes\",\"scene\":%d,\"valid\":[", scene);
    for (std::size_t i = 0; i < probes.size(); ++i) {
      const auto c = toConfig(probes[i]);
      std::fprintf(out, "%s%d", i ? "," : "", vamp::planning::validate_motion<Robot, kRake, 1>(c, c, env_v) ? 1 : 0);
    }
    std::fprintf(out, "]}\n");

    const int kStates = 20000;
    std::vector<Configuration> qs;
    for (int i = 0; i < kStates; ++i) {
      std::vector<float> q(kDim);
      for (auto & x : q) {
        x = uni(gen);
      }
      qs.push_back(toConfig(q));
    }
    int valid = 0;
    auto t0 = Clock::now();
    for (const auto & c : qs) {
      valid += vamp::planning::validate_motion<Robot, kRake, 1>(c, c, env_v) ? 1 : 0;
    }
    const double state_ns = std::chrono::duration<double, std::nano>(Clock::now() - t0).count() / kStates;
    const int kMotions = 2000;
    int mvalid = 0;
    t0 = Clock::now();
    for (int i = 0; i + 1 < kMotions; ++i) {
      mvalid += vamp::planning::validate_motion<Robot, kRake, Robot::resolution>(qs[i], qs[i + 1], env_v) ? 1 : 0;
    }
    const double motion_us = std::chrono::duration<double, std::micro>(Clock::now() - t0).count() / (kMotions - 1);
    std::fprintf(out,
      "{\"type\":\"micro\",\"scene\":%d,\"state_ns\":%.1f,\"state_valid_fraction\":%.4f,\"motion_us\":%.2f,\"motion_valid_fraction\":%.4f}\n",
      scene, state_ns, valid / double(kStates), motion_us, mvalid / double(kMotions - 1));
    std::fflush(out);
  }
  std::fclose(out);
  return 0;
}
