// standoff_sweep: reachable fraction and pose quality of a wall as the base's distance to it varies, and the
// robust recommended distance. ROS-free.
//
//   standoff_sweep (--map file.rm4d | --chain chain.yaml [--samples N --resolution m --extent m --zmin m --zmax m
//                    --threads T --save-map out.rm4d])
//                  (--wall-scene scene.json [--wall-id 0] | --wall-point x y z --wall-normal nx ny nz)
//                  [--wall-patch u0 u1 v0 v1] [--wall-z z0 z1] [--patch-res 0.1] [--tool-standoff m]
//                  [--profile profile.yaml] [--min-clearance m] [--standoff-min m --standoff-max m --standoff-step m]
//                  [--lift-min m --lift-max m --lift-step m] [--mount-z m] [--lateral-step m]
//                  [--nav-tol m] [--quality-min q] [--count-ref n] [--fraction-tol f] [--at d1,d2,...]
//                  [--out-json f.json] [--out-csv f.csv] [--quiet 1]
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "reachability/core/arm_model.hpp"
#include "reachability/core/base_model.hpp"
#include "reachability/core/builder.hpp"
#include "reachability/core/io.hpp"
#include "standoff_advisor/standoff_sweep.hpp"

using namespace standoff_advisor;  // NOLINT

namespace
{
using Args = std::map<std::string, std::vector<std::string>>;

// --name v1 [v2 ...]: the values are the following arguments up to the next "--".
Args parse(int argc, char ** argv)
{
  Args a;
  std::string cur;
  for (int i = 1; i < argc; ++i) {
    if (std::strncmp(argv[i], "--", 2) == 0) {
      cur = argv[i] + 2;
      a[cur];
    } else if (!cur.empty()) {
      a[cur].push_back(argv[i]);
    } else {
      std::fprintf(stderr, "unexpected argument '%s'\n", argv[i]);
      std::exit(2);
    }
  }
  return a;
}

[[noreturn]] void die(const std::string & m)
{
  std::fprintf(stderr, "error: %s\n", m.c_str());
  std::exit(2);
}

double number(const std::string & s, const char * name)
{
  char * end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0' || !std::isfinite(v)) {die(std::string("--") + name + ": '" + s + "' is not a finite number");}
  return v;
}

std::vector<double> numbers(const Args & a, const char * k, size_t n)
{
  const auto it = a.find(k);
  if (it == a.end()) {return {};}
  if (it->second.size() != n) {die(std::string("--") + k + " needs " + std::to_string(n) + " value(s)");}
  std::vector<double> v;
  for (const auto & s : it->second) {v.push_back(number(s, k));}
  return v;
}

double get(const Args & a, const char * k, double dflt)
{
  const auto v = numbers(a, k, 1);
  return v.empty() ? dflt : v[0];
}

std::string str(const Args & a, const char * k, const std::string & dflt = "")
{
  const auto it = a.find(k);
  if (it == a.end()) {return dflt;}
  if (it->second.size() != 1) {die(std::string("--") + k + " needs one value");}
  return it->second[0];
}

Vec3 vec3(const YAML::Node & n)
{
  if (!n || !n.IsSequence() || n.size() != 3) {throw std::runtime_error("wall scene: expected a 3-vector");}
  return Vec3(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
}

std::string js(double v)
{
  if (!std::isfinite(v)) {return "null";}
  char b[40];
  std::snprintf(b, sizeof(b), "%.6g", v);
  return b;
}

std::string quoted(const std::string & s)
{
  std::string o = "\"";
  for (const char c : s) {
    if (c == '"' || c == '\\') {o += '\\';}
    o += (c == '\n') ? ' ' : c;
  }
  return o + "\"";
}

std::string bar(double f, int width, char fill)
{
  const int n = static_cast<int>(std::lround(std::max(0.0, std::min(1.0, f)) * width));
  return std::string(static_cast<size_t>(n), fill) + std::string(static_cast<size_t>(width - n), ' ');
}
}  // namespace

int main(int argc, char ** argv)
{
  const Args a = parse(argc, argv);
  if (a.empty() || a.count("help")) {
    std::fprintf(stderr, "see the header of apps/standoff_sweep.cpp and docs/standoff.md\n");
    return 2;
  }
  try {
    // 1. The map.
    reachability::Map4D map;
    std::string map_source;
    if (a.count("map") && a.count("chain")) {die("give --map or --chain, not both");}
    if (a.count("map")) {
      map = reachability::loadMap(str(a, "map"));
      map_source = str(a, "map");
    } else if (a.count("chain")) {
      const auto arm = reachability::loadChainYaml(str(a, "chain"));
      reachability::MapParams p;
      p.resolution = get(a, "resolution", p.resolution);
      p.xy_extent = get(a, "extent", 2.0);
      p.z_min = get(a, "zmin", -1.8);
      p.z_max = get(a, "zmax", 2.0);
      reachability::BuildOptions b;
      b.samples = static_cast<size_t>(get(a, "samples", 3e6));
      b.threads = static_cast<int>(get(a, "threads", 0));
      b.seed = static_cast<uint32_t>(get(a, "seed", 1));
      reachability::BuildStats st;
      map = reachability::buildMap(arm, p, b, &st);
      map_source = str(a, "chain") + " (built in place, " + std::to_string(b.samples) + " samples)";
      if (a.count("save-map")) {
        reachability::MapInfo info;
        info.arm_dof = static_cast<uint32_t>(arm.dof());
        reachability::saveMap(str(a, "save-map"), map, arm.fingerprint(), info);
      }
    } else {
      die("give --map file.rm4d or --chain chain.yaml");
    }

    // 2. The wall and the work on it.
    StandoffSweepRequest r;
    const double res = get(a, "patch-res", 0.1);
    if (a.count("wall-scene")) {
      const YAML::Node scene = YAML::LoadFile(str(a, "wall-scene"));
      const auto id = static_cast<size_t>(get(a, "wall-id", 0));
      if (!scene["walls"] || id >= scene["walls"].size()) {die("wall scene has no wall " + std::to_string(id));}
      const YAML::Node w = scene["walls"][id];
      WallPatch p;
      p.origin = vec3(w["origin_xyz"]);
      p.u_axis = vec3(w["basis_u"]);
      p.v_axis = vec3(w["basis_v"]);
      r.wall_normal = vec3(w["normal_outward"]);
      r.wall_point = p.origin;
      p.u0 = w["u_range_m"][0].as<double>();
      p.u1 = w["u_range_m"][1].as<double>();
      p.v0 = w["v_range_m"][0].as<double>();
      p.v1 = w["v_range_m"][1].as<double>();
      p.resolution = res;
      if (const auto v = numbers(a, "wall-patch", 4); !v.empty()) {
        p.u0 = v[0]; p.u1 = v[1]; p.v0 = v[2]; p.v1 = v[3];
      }
      if (const auto z = numbers(a, "wall-z", 2); !z.empty()) {
        const double vz = p.v_axis.normalized().z();
        if (std::fabs(vz) < 0.5) {die("--wall-z needs a wall whose v axis is vertical");}
        const double v0 = (z[0] - p.origin.z()) / vz, v1 = (z[1] - p.origin.z()) / vz;
        p.v0 = std::min(v0, v1);
        p.v1 = std::max(v0, v1);
      }
      r.patch = p;
    } else {
      if (const auto pt = numbers(a, "wall-point", 3); !pt.empty()) {r.wall_point = Vec3(pt[0], pt[1], pt[2]);}
      if (const auto nm = numbers(a, "wall-normal", 3); !nm.empty()) {r.wall_normal = Vec3(nm[0], nm[1], nm[2]);}
      auto v = numbers(a, "wall-patch", 4);
      if (v.empty()) {die("give --wall-scene scene.json or --wall-point/--wall-normal with --wall-patch u0 u1 v0 v1");}
      r.patch = patchFromPlane(r.wall_point, r.wall_normal, v[0], v[1], v[2], v[3], res);
    }
    const double tool_standoff = get(a, "tool-standoff", 0.0);
    if (tool_standoff != 0.0 && r.patch) {
      if (!std::isfinite(r.wall_normal.norm()) || r.wall_normal.norm() < 1e-9) {die("the wall normal is zero");}
      r.patch->origin += r.wall_normal.normalized() * tool_standoff;   // hover poses, off the wall
    }

    // 3. The assembled robot: base, lift, arm mount.
    double lift_stroke = 1.5;
    std::string profile_name;
    if (a.count("profile")) {
      const std::string path = str(a, "profile");
      r.base = reachability::loadBaseProfile(path);
      profile_name = path;
      const YAML::Node prof = YAML::LoadFile(path);
      if (prof["lift"] && prof["lift"]["stroke_m"]) {lift_stroke = prof["lift"]["stroke_m"].as<double>();}
      if (const auto z = reachability::armBaseHeightAtLiftZero(*r.base)) {r.arm_mount_offset_z = *z;}
    }
    r.arm_mount_offset_z = get(a, "mount-z", r.arm_mount_offset_z);
    r.lift_min = get(a, "lift-min", 0.0);
    r.lift_max = get(a, "lift-max", lift_stroke);
    r.lift_step = get(a, "lift-step", r.lift_step);
    r.lateral_step = get(a, "lateral-step", r.lateral_step);
    r.min_clearance = get(a, "min-clearance", r.min_clearance);
    r.standoff_min = get(a, "standoff-min", r.standoff_min);
    r.standoff_max = get(a, "standoff-max", r.standoff_max);
    r.standoff_step = get(a, "standoff-step", r.standoff_step);
    r.nav_tolerance_m = get(a, "nav-tol", r.nav_tolerance_m);
    r.quality_min = get(a, "quality-min", r.quality_min);
    r.margin.count_ref = get(a, "count-ref", r.margin.count_ref);
    r.fraction_tolerance = get(a, "fraction-tol", r.fraction_tolerance);

    const auto result = sweepStandoff(map, r);
    if (result.status == SweepStatus::kInvalidRequest) {die(result.problem);}

    std::vector<StandoffRow> at_rows;
    if (a.count("at")) {
      std::stringstream ss(str(a, "at"));
      for (std::string tok; std::getline(ss, tok, ',');) {at_rows.push_back(evaluateStandoff(map, r, number(tok, "at")));}
    }

    // 4. Output.
    const auto & rec = result.recommended;
    const double mount_x = r.base ? r.base->arm_mount.x() : 0.0;
    if (a.count("out-csv")) {
      std::ofstream f(str(a, "out-csv"));
      f << "distance_m,reachable_fraction,robust_fraction,reachable_count,mean_quality,min_quality,lateral_m,lift_bands,"
           "lift_low_m,lift_high_m,footprint_fits,footprint_clearance_m,base_origin_distance_m,base_yaw_rad,in_plateau\n";
      for (const auto & w : result.rows) {
        f << js(w.distance) << ',' << js(w.reachable_fraction) << ',' << js(w.robust_fraction) << ',' << w.reachable_count
          << ',' << js(w.mean_quality) << ',' << js(w.min_quality) << ',' << js(w.lateral_m) << ',' << w.lift_bands << ','
          << js(w.lift_low) << ',' << js(w.lift_high) << ',' << (w.footprint_fits ? 1 : 0) << ','
          << js(w.footprint_clearance) << ',' << js(w.base_origin_distance) << ',' << js(w.base_yaw) << ','
          << (w.in_plateau ? 1 : 0) << '\n';
      }
    }
    const auto row_json = [&](const StandoffRow & w) {
        std::ostringstream o;
        o << "{\"distance_m\":" << js(w.distance) << ",\"reachable_fraction\":" << js(w.reachable_fraction)
          << ",\"robust_fraction\":" << js(w.robust_fraction) << ",\"reachable_count\":" << w.reachable_count
          << ",\"mean_quality\":" << js(w.mean_quality) << ",\"min_quality\":" << js(w.min_quality)
          << ",\"lateral_m\":" << js(w.lateral_m) << ",\"lift_bands\":" << w.lift_bands
          << ",\"lift_low_m\":" << js(w.lift_low) << ",\"lift_high_m\":" << js(w.lift_high)
          << ",\"footprint_fits\":" << (w.footprint_fits ? "true" : "false")
          << ",\"footprint_clearance_m\":" << js(w.footprint_clearance)
          << ",\"base_origin_distance_m\":" << js(w.base_origin_distance)
          << ",\"base_yaw_rad\":" << js(w.base_yaw) << ",\"in_plateau\":" << (w.in_plateau ? "true" : "false") << "}";
        return o.str();
      };
    if (a.count("out-json")) {
      std::ofstream f(str(a, "out-json"));
      f << "{\n \"status\":" << quoted(toString(result.status)) << ",\n \"map\":" << quoted(map_source)
        << ",\n \"profile\":" << quoted(profile_name) << ",\n \"targets\":" << result.target_count
        << ",\n \"tool_standoff_m\":" << js(tool_standoff) << ",\n \"arm_mount_offset_z_m\":" << js(r.arm_mount_offset_z)
        << ",\n \"lift_range_m\":[" << js(r.lift_min) << "," << js(r.lift_max) << "]"
        << ",\n \"min_clearance_m\":" << js(r.min_clearance) << ",\n \"max_fraction\":" << js(result.max_fraction)
        << ",\n \"recommended\":{\"found\":" << (rec.found ? "true" : "false")
        << ",\"recommended_standoff_m\":" << js(rec.standoff_m)
        << ",\"arm_base_to_wall_m\":" << js(rec.standoff_m)
        << ",\"base_origin_to_wall_m\":" << js(rec.base_origin_distance)
        << ",\"arm_mount_x_in_base_m\":" << js(mount_x)
        << ",\"base_yaw_rad\":" << js(rec.base_yaw)
        << ",\"min_distance_m\":" << js(rec.min_distance) << ",\"max_distance_m\":" << js(rec.max_distance)
        << ",\"robust_min_m\":" << js(rec.robust_min) << ",\"robust_max_m\":" << js(rec.robust_max)
        << ",\"robust\":" << (rec.robust ? "true" : "false") << ",\"quality_ok\":" << (rec.quality_ok ? "true" : "false")
        << ",\"touches_range\":" << (rec.touches_range ? "true" : "false")
        << ",\"nav_tolerance_m\":" << js(rec.nav_tolerance_m)
        << ",\"reachable_fraction\":" << js(rec.reachable_fraction) << ",\"robust_fraction\":" << js(rec.robust_fraction)
        << ",\"mean_quality\":" << js(rec.mean_quality) << ",\"lift_low_m\":" << js(rec.lift_low)
        << ",\"lift_high_m\":" << js(rec.lift_high) << ",\"lateral_m\":" << js(rec.lateral_m)
        << ",\"reason\":" << quoted(rec.reason) << "}";
      f << ",\n \"evaluated_at\":[";
      for (size_t i = 0; i < at_rows.size(); ++i) {f << (i ? "," : "") << row_json(at_rows[i]);}
      f << "],\n \"sweep\":[\n";
      for (size_t i = 0; i < result.rows.size(); ++i) {
        f << "  " << row_json(result.rows[i]) << (i + 1 < result.rows.size() ? ",\n" : "\n");
      }
      f << " ]\n}\n";
    }

    std::printf("standoff sweep: %zu targets, map %s%s%s\n", result.target_count, map_source.c_str(),
      profile_name.empty() ? "" : ", profile ", profile_name.c_str());
    std::printf("arm base %.3f m above the ground at lift zero, lift %.2f..%.2f m, clearance %.2f m, tool standoff %.3f m\n",
      r.arm_mount_offset_z, r.lift_min, r.lift_max, r.min_clearance, tool_standoff);
    if (a.count("quiet") == 0) {
      std::printf("\n dist  reach  robust quality  lift(m)       bands fit  clear  base-origin\n");
      for (const auto & w : result.rows) {
        std::printf(" %.2f  %5.1f%% %5.1f%%  %.2f   %.2f..%.2f     %zu    %s   %.2f    %.2f %s\n", w.distance,
          100.0 * w.reachable_fraction, 100.0 * w.robust_fraction, w.mean_quality, w.lift_low, w.lift_high,
          w.lift_bands, w.footprint_fits ? "y" : "n", w.footprint_clearance, w.base_origin_distance,
          w.in_plateau ? "*" : "");
      }
      std::printf("\nreachable fraction vs arm-base distance to the wall ('=' plateau, '#' other, '<' recommended)\n");
      int rec_row = 0;
      for (size_t i = 0; i < result.rows.size(); ++i) {
        if (std::fabs(result.rows[i].distance - rec.standoff_m) < std::fabs(result.rows[rec_row].distance - rec.standoff_m)) {
          rec_row = static_cast<int>(i);
        }
      }
      for (size_t i = 0; i < result.rows.size(); ++i) {
        const auto & w = result.rows[i];
        std::printf(" %.2f |%s| %5.1f%%%s\n", w.distance, bar(w.reachable_fraction, 50, w.in_plateau ? '=' : '#').c_str(),
          100.0 * w.reachable_fraction, (rec.found && static_cast<int>(i) == rec_row) ? " <" : "");
      }
    }
    for (const auto & w : at_rows) {
      std::printf("at %.3f m: reachable %.1f%% (%zu of %zu), mean quality %.2f, lift %.2f..%.2f m, footprint %s (clearance %.2f m)\n",
        w.distance, 100.0 * w.reachable_fraction, w.reachable_count, result.target_count, w.mean_quality, w.lift_low,
        w.lift_high, w.footprint_fits ? "fits" : "does NOT fit", w.footprint_clearance);
    }
    std::printf("\nstatus: %s\n", toString(result.status));
    if (rec.found) {
      std::printf("recommended_standoff_m: %.3f (arm base to wall); base origin to wall: %.3f m; plateau %.2f..%.2f m, "
        "robust %.2f..%.2f m (nav tolerance %.0f mm); reachable %.1f%%, quality %.2f, lift %.2f..%.2f m\n",
        rec.standoff_m, rec.base_origin_distance, rec.min_distance, rec.max_distance, rec.robust_min, rec.robust_max,
        1000.0 * rec.nav_tolerance_m, 100.0 * rec.reachable_fraction, rec.mean_quality, rec.lift_low, rec.lift_high);
    }
    std::printf("reason: %s\n", rec.reason.c_str());
    return rec.found ? 0 : 1;
  } catch (const std::exception & e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
