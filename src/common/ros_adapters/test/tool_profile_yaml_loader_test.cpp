#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <unistd.h>

#include <gtest/gtest.h>

#include "ros_adapters/tool_profile_yaml_loader.hpp"

namespace ros_adapters
{
namespace
{

class TempYamlFile
{
public:
  explicit TempYamlFile(const std::string & contents)
  {
    std::vector<char> name_template(
      {'/', 't', 'm', 'p', '/', 't', 'p', 'y', 'l', '_', 't', 'e', 's', 't',
        'X', 'X', 'X', 'X', 'X', 'X', '\0'});
    const int fd = mkstemp(name_template.data());
    path_ = name_template.data();
    if (fd >= 0) {
      close(fd);
    }
    std::ofstream out(path_);
    out << contents;
  }

  ~TempYamlFile()
  {
    std::remove(path_.c_str());
  }

  const std::string & path() const {return path_;}

private:
  std::string path_;
};

constexpr char kSprayerYaml[] =
  "schema_version: 1\n"
  "tools:\n"
  "  sprayer:\n"
  "    geometry:\n"
  "      mesh: \"spray_nozzle.stl\"\n"
  "      tcp_xyz: [0.0, 0.0, 0.150]\n"
  "      contact_allowed_links: []\n"
  "    task:\n"
  "      contact: false\n"
  "      standoff_m: 0.15\n"
  "      tool_normal_policy: \"tilt_at_edges\"\n"
  "      max_edge_tilt_deg: 25.0\n"
  "      reachability_tilt_deg_max: 15.0\n"
  "      tool_command: \"pump\"\n"
  "    coverage:\n"
  "      swath_m: 0.15\n"
  "      overlap_pct: 50.0\n"
  "      sweep_direction: \"horizontal\"\n"
  "      obstacle_inflation: \"primary_axis\"\n"
  "    control:\n"
  "      controller: \"jtc_only\"\n";

constexpr char kMopYaml[] =
  "schema_version: 1\n"
  "tools:\n"
  "  mop:\n"
  "    geometry:\n"
  "      mesh: \"cam_mop_ft.stl\"\n"
  "      contact_allowed_links: [\"link_tcp\"]\n"
  "    task:\n"
  "      contact: true\n"
  "      target_force_n: 15.0\n"
  "      standoff_m: 0.0\n"
  "    coverage:\n"
  "      swath_m: 0.25\n"
  "      overlap_pct: 20.0\n"
  "      passes:\n"
  "        - {}\n"
  "        - { reverse_direction: true }\n"
  "    control:\n"
  "      controller: \"admittance_chain\"\n"
  "      admittance:\n"
  "        wrench_reference_z_n: 15.0\n";

TEST(ToolProfileYamlLoader, LoadsSprayerProfileFields)
{
  TempYamlFile file(kSprayerYaml);
  const auto profile = loadToolProfile(file.path(), "sprayer");

  EXPECT_EQ(profile.name, "sprayer");
  EXPECT_FALSE(profile.task.contact);
  EXPECT_DOUBLE_EQ(profile.task.standoff_m, 0.15);
  EXPECT_EQ(profile.task.tool_normal_policy, "tilt_at_edges");
  EXPECT_DOUBLE_EQ(profile.task.reachability_tilt_deg_max, 15.0);
  EXPECT_DOUBLE_EQ(profile.coverage.swath_m, 0.15);
  EXPECT_EQ(profile.coverage.obstacle_inflation, "primary_axis");
  EXPECT_EQ(profile.control.controller, "jtc_only");
}

TEST(ToolProfileYamlLoader, LoadsMopProfileWithTwoPassPassList)
{
  TempYamlFile file(kMopYaml);
  const auto profile = loadToolProfile(file.path(), "mop");

  EXPECT_TRUE(profile.task.contact);
  ASSERT_EQ(profile.coverage.passes.size(), 2u);
  EXPECT_FALSE(profile.coverage.passes[0].reverse_direction);
  EXPECT_TRUE(profile.coverage.passes[1].reverse_direction);
  EXPECT_EQ(profile.control.controller, "admittance_chain");
}

TEST(ToolProfileYamlLoader, ThrowsWhenToolMissing)
{
  TempYamlFile file(kSprayerYaml);
  EXPECT_THROW(loadToolProfile(file.path(), "nonexistent_tool"), std::runtime_error);
}

TEST(ToolProfileYamlLoader, ThrowsWhenContactWithoutAdmittanceChain)
{
  TempYamlFile file(
    "schema_version: 1\n"
    "tools:\n"
    "  bad:\n"
    "    task:\n"
    "      contact: true\n"
    "      target_force_n: 5.0\n"
    "    control:\n"
    "      controller: \"jtc_only\"\n");
  EXPECT_THROW(loadToolProfile(file.path(), "bad"), std::runtime_error);
}

TEST(ToolProfileYamlLoader, ThrowsWhenWrenchReferenceMismatchesTargetForce)
{
  TempYamlFile file(
    "schema_version: 1\n"
    "tools:\n"
    "  bad:\n"
    "    task:\n"
    "      contact: true\n"
    "      target_force_n: 5.0\n"
    "    control:\n"
    "      controller: \"admittance_chain\"\n"
    "      admittance:\n"
    "        wrench_reference_z_n: 15.0\n");
  EXPECT_THROW(loadToolProfile(file.path(), "bad"), std::runtime_error);
}

TEST(ToolProfileYamlLoader, ThrowsWhenNonContactHasZeroStandoff)
{
  TempYamlFile file(
    "schema_version: 1\n"
    "tools:\n"
    "  bad:\n"
    "    task:\n"
    "      contact: false\n"
    "      standoff_m: 0.0\n"
    "    control:\n"
    "      controller: \"jtc_only\"\n");
  EXPECT_THROW(loadToolProfile(file.path(), "bad"), std::runtime_error);
}

TEST(ToolProfileYamlLoader, ThrowsWhenReachabilityTiltAllowedOnContactTool)
{
  TempYamlFile file(
    "schema_version: 1\n"
    "tools:\n"
    "  bad:\n"
    "    task:\n"
    "      contact: true\n"
    "      target_force_n: 5.0\n"
    "      reachability_tilt_deg_max: 10.0\n"
    "    control:\n"
    "      controller: \"admittance_chain\"\n"
    "      admittance:\n"
    "        wrench_reference_z_n: 5.0\n");
  EXPECT_THROW(loadToolProfile(file.path(), "bad"), std::runtime_error);
}

TEST(ToolProfileYamlLoader, LoadsSprayConeFields)
{
  TempYamlFile file(
    "schema_version: 1\n"
    "tools:\n"
    "  gun:\n"
    "    task: {contact: false, standoff_m: 0.3}\n"
    "    coverage: {spray_half_angle_deg: 20.0, effective_width_fraction: 0.8}\n");
  const auto profile = loadToolProfile(file.path(), "gun");
  EXPECT_DOUBLE_EQ(profile.coverage.spray_half_angle_deg, 20.0);
  EXPECT_DOUBLE_EQ(profile.coverage.effective_width_fraction, 0.8);
}

TEST(ToolProfileYamlLoader, ThrowsOnBadSprayCone)
{
  TempYamlFile contact(
    "schema_version: 1\n"
    "tools:\n"
    "  bad:\n"
    "    task: {contact: true, target_force_n: 5.0}\n"
    "    coverage: {spray_half_angle_deg: 20.0}\n"
    "    control: {controller: admittance_chain, admittance: {wrench_reference_z_n: 5.0}}\n");
  EXPECT_THROW(loadToolProfile(contact.path(), "bad"), std::runtime_error);
  TempYamlFile fraction(
    "schema_version: 1\n"
    "tools:\n"
    "  bad:\n"
    "    task: {contact: false, standoff_m: 0.3}\n"
    "    coverage: {spray_half_angle_deg: 20.0, effective_width_fraction: 1.4}\n");
  EXPECT_THROW(loadToolProfile(fraction.path(), "bad"), std::runtime_error);
}

TEST(ToolProfileYamlLoader, LoadsContactAreaAndEdgeSpacing)
{
  TempYamlFile file(
    "schema_version: 1\n"
    "tools:\n"
    "  pad:\n"
    "    task: {contact: true, target_force_n: 20.0}\n"
    "    coverage:\n"
    "      contact_area: {pad_width_m: 0.2, pad_length_m: 0.1, reference_force_n: 10.0,\n"
    "                     width_gain_per_n: 0.01, max_width_factor: 1.3, nominal_tilt_deg: 10.0}\n"
    "      edge_zone_m: 0.15\n"
    "      edge_spacing_scale: 0.6\n"
    "    control: {controller: admittance_chain, admittance: {wrench_reference_z_n: 20.0}}\n");
  const auto profile = loadToolProfile(file.path(), "pad");
  const auto & a = profile.coverage.contact_area;
  EXPECT_TRUE(a.enabled());
  EXPECT_DOUBLE_EQ(a.pad_width_m, 0.2);
  EXPECT_DOUBLE_EQ(a.reference_force_n, 10.0);
  EXPECT_DOUBLE_EQ(a.width_gain_per_n, 0.01);
  EXPECT_DOUBLE_EQ(a.max_width_factor, 1.3);
  EXPECT_DOUBLE_EQ(a.min_width_factor, 0.5);  // default
  EXPECT_DOUBLE_EQ(a.nominal_tilt_deg, 10.0);
  EXPECT_DOUBLE_EQ(profile.coverage.edge_zone_m, 0.15);
  EXPECT_DOUBLE_EQ(profile.coverage.edge_spacing_scale, 0.6);
}

TEST(ToolProfileYamlLoader, ContactAreaDefaultsAreOff)
{
  TempYamlFile file(
    "schema_version: 1\n"
    "tools:\n"
    "  gun:\n"
    "    task: {contact: false, standoff_m: 0.3}\n");
  const auto profile = loadToolProfile(file.path(), "gun");
  EXPECT_FALSE(profile.coverage.contact_area.enabled());
  EXPECT_DOUBLE_EQ(profile.coverage.edge_zone_m, 0.0);
  EXPECT_DOUBLE_EQ(profile.coverage.edge_spacing_scale, 1.0);
}

TEST(ToolProfileYamlLoader, ThrowsOnBadContactAreaOrEdgeSpacing)
{
  const char * contact_head =
    "schema_version: 1\ntools:\n  bad:\n    task: {contact: true, target_force_n: 5.0}\n"
    "    control: {controller: admittance_chain, admittance: {wrench_reference_z_n: 5.0}}\n";
  for (const char * coverage : {
      "    coverage: {contact_area: {pad_width_m: 0.2, min_width_factor: 1.5}}\n",
      "    coverage: {contact_area: {pad_width_m: 0.2, nominal_tilt_deg: 95.0}}\n",
      "    coverage: {contact_area: {pad_width_m: 0.2, pad_stiffness_n_per_m: -1.0}}\n",
      "    coverage: {edge_zone_m: 0.1, edge_spacing_scale: 1.5}\n",
      "    coverage: {edge_zone_m: -0.1}\n"})
  {
    TempYamlFile f(std::string(contact_head) + coverage);
    EXPECT_THROW(loadToolProfile(f.path(), "bad"), std::runtime_error) << coverage;
  }
  TempYamlFile non_contact(
    "schema_version: 1\ntools:\n  bad:\n    task: {contact: false, standoff_m: 0.3}\n"
    "    coverage: {contact_area: {pad_width_m: 0.2}}\n");
  EXPECT_THROW(loadToolProfile(non_contact.path(), "bad"), std::runtime_error);
  TempYamlFile non_contact_edge(
    "schema_version: 1\ntools:\n  bad:\n    task: {contact: false, standoff_m: 0.3}\n"
    "    coverage: {edge_zone_m: 0.1, edge_spacing_scale: 0.5}\n");
  EXPECT_THROW(loadToolProfile(non_contact_edge.path(), "bad"), std::runtime_error);
}

}  // namespace
}  // namespace ros_adapters
