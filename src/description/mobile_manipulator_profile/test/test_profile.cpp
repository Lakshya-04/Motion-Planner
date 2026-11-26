#include <cstdlib>
#include <filesystem>

#include "gtest/gtest.h"
#include "mobile_manipulator_profile/profile.hpp"

using mobile_manipulator_profile::loadProfile;

class ProfileTest : public ::testing::Test
{
protected:
  void SetUp() override {setenv("MOBILE_MANIPULATOR_PROFILE_DIR", PROFILE_DIR, 1);}
};

TEST_F(ProfileTest, Ur10eHasSixArmJoints)
{
  const auto p = loadProfile("ur10e_telescopic");
  EXPECT_EQ(p.arm, "ur10e");
  EXPECT_EQ(p.armDof(), 6u);
  EXPECT_EQ(p.dof(), 7u);
  EXPECT_EQ(p.joints().front(), "lift_joint");
  EXPECT_EQ(p.joints()[1], "arm_shoulder_pan_joint");
  EXPECT_EQ(p.whole_body_group, "lift_arm");
  EXPECT_EQ(p.arm_group, "arm");
  EXPECT_EQ(p.tcpLink(), "tcp_link");
  EXPECT_EQ(p.homePositions().size(), 7u);
}

TEST_F(ProfileTest, PandaHasSevenArmJoints)
{
  const auto p = loadProfile("panda_telescopic");
  EXPECT_EQ(p.armDof(), 7u);
  EXPECT_EQ(p.dof(), 8u);
  EXPECT_EQ(p.joints().back(), "arm_joint7");
  EXPECT_EQ(p.controller_family, "panda");
}

TEST_F(ProfileTest, GenericArmLoadsByPath)
{
  const auto p = loadProfile(std::string(PROFILE_DIR) + "/generic_6dof_telescopic.yaml");
  EXPECT_EQ(p.name, "generic_6dof_telescopic");
  EXPECT_EQ(p.armDof(), 6u);
}

TEST_F(ProfileTest, MissingProfileThrows)
{
  EXPECT_THROW(loadProfile("no_such_profile"), std::runtime_error);
}

TEST_F(ProfileTest, DofMismatchThrows)
{
  auto root = YAML::LoadFile(std::string(PROFILE_DIR) + "/panda_telescopic.yaml");
  root["arm"]["dof"] = 6;
  EXPECT_THROW(mobile_manipulator_profile::parseProfile(root, "bad"), std::runtime_error);
}

TEST_F(ProfileTest, HomeMustCoverEveryJoint)
{
  auto root = YAML::LoadFile(std::string(PROFILE_DIR) + "/panda_telescopic.yaml");
  root["home"].remove("arm_joint7");
  EXPECT_THROW(mobile_manipulator_profile::parseProfile(root, "bad"), std::runtime_error);
}

TEST_F(ProfileTest, NoLiftProfileHasNoLiftJoint)
{
  const auto p = loadProfile("ur10e_nolift");
  EXPECT_TRUE(p.lift_joint.empty());
  EXPECT_EQ(p.dof(), p.armDof());
  EXPECT_EQ(p.homePositions().size(), p.dof());
}

TEST_F(ProfileTest, NullMimicIsEmpty)
{
  auto root = YAML::LoadFile(std::string(PROFILE_DIR) + "/panda_telescopic.yaml");
  root["joints"]["mimic"] = YAML::Node(YAML::NodeType::Null);
  EXPECT_TRUE(mobile_manipulator_profile::parseProfile(root, "p").mimic_joints.empty());
}

TEST_F(ProfileTest, EveryShippedProfileLoads)
{
  for (const auto & e : std::filesystem::directory_iterator(PROFILE_DIR)) {
    if (e.path().extension() != ".yaml") {continue;}
    EXPECT_NO_THROW(loadProfile(e.path().string())) << e.path();
  }
}

namespace
{
YAML::Node withSensors(const std::string & sensors_yaml)
{
  auto root = YAML::LoadFile(std::string(PROFILE_DIR) + "/ur10e_telescopic.yaml");
  root["sensors"] = YAML::Load(sensors_yaml);
  return root;
}

const char kLidar[] = R"(
- type: lidar2d
  name: lidar
  frame: lidar_link
  mount: {xyz: [0.3, 0.0, 0.2], rpy: [0, 0, 0]}
  factory: false
  source: "test"
  spec: {angle_min_rad: -2.0, angle_max_rad: 2.0, angle_increment_rad: 0.01,
         range_min_m: 0.1, range_max_m: 10.0, rate_hz: 40.0}
)";

const char kCamera[] = R"(
- type: rgbd
  name: cam
  frame: cam_link
  mount: {xyz: [0.3, 0.0, 0.3], rpy: [0, 0, 0]}
  factory: true
  source: "test"
  spec: {width: 640, height: 480, hfov_rad: 1.5, range_min_m: 0.3, range_max_m: 3.0,
         rate_hz: 30.0, depth_noise_note: "none"}
)";

// Parse the lidar block after one text substitution and expect a failure naming profile and entry.
void expectLidarFails(const std::string & from, const std::string & to, const std::string & msg)
{
  std::string y = kLidar;
  const auto pos = y.find(from);
  ASSERT_NE(pos, std::string::npos) << from;
  y.replace(pos, from.size(), to);
  try {
    mobile_manipulator_profile::parseProfile(withSensors(y), "badsens");
    FAIL() << "no throw for " << from << " -> " << to;
  } catch (const std::runtime_error & e) {
    const std::string w = e.what();
    EXPECT_NE(w.find("profile badsens"), std::string::npos) << w;
    EXPECT_NE(w.find("(lidar)"), std::string::npos) << w;
    EXPECT_NE(w.find(msg), std::string::npos) << w;
  }
}
}  // namespace

TEST_F(ProfileTest, SensorsAbsentIsEmpty)
{
  EXPECT_TRUE(loadProfile("ur10e_telescopic").sensors.empty());
}

TEST_F(ProfileTest, SensorsBlockParses)
{
  const auto p = mobile_manipulator_profile::parseProfile(
    withSensors(std::string(kLidar) + kCamera), "ok");
  ASSERT_EQ(p.sensors.size(), 2u);
  EXPECT_EQ(p.sensors[0].type, "lidar2d");
  EXPECT_EQ(p.sensors[0].frame, "lidar_link");
  EXPECT_DOUBLE_EQ(p.sensors[0].xyz[0], 0.3);
  EXPECT_FALSE(p.sensors[0].factory);
  EXPECT_DOUBLE_EQ(p.sensors[0].spec.at("range_max_m"), 10.0);
  EXPECT_EQ(p.sensors[0].spec.count("range_noise_std_m"), 0u);
  EXPECT_TRUE(p.sensors[1].factory);
  EXPECT_EQ(p.sensors[1].depth_noise_note, "none");
}

TEST_F(ProfileTest, SensorsShippedPlatformProfileHasSources)
{
  const auto p = loadProfile("ur10e_husky_telescopic");
  ASSERT_FALSE(p.sensors.empty());
  for (const auto & s : p.sensors) {EXPECT_FALSE(s.source.empty()) << s.name;}
}

TEST_F(ProfileTest, SensorsUnknownTypeThrows)
{
  expectLidarFails("type: lidar2d", "type: sonar", "unknown type");
}

TEST_F(ProfileTest, SensorsAngleOrderThrows)
{
  expectLidarFails("angle_min_rad: -2.0", "angle_min_rad: 2.5", "angle_min_rad must be below");
}

TEST_F(ProfileTest, SensorsNonPositiveRateThrows)
{
  expectLidarFails("rate_hz: 40.0", "rate_hz: 0.0", "rate_hz must be positive");
}

TEST_F(ProfileTest, SensorsNonFiniteRangeThrows)
{
  expectLidarFails("range_max_m: 10.0", "range_max_m: .inf", "not finite");
}

TEST_F(ProfileTest, SensorsRangeOrderThrows)
{
  expectLidarFails("range_max_m: 10.0", "range_max_m: 0.05", "must exceed range_min_m");
}

TEST_F(ProfileTest, SensorsNegativeRangeThrows)
{
  expectLidarFails("range_min_m: 0.1", "range_min_m: -0.1", "must be positive");
}

TEST_F(ProfileTest, SensorsMountNeedsThreeNumbers)
{
  expectLidarFails("xyz: [0.3, 0.0, 0.2]", "xyz: [0.3, 0.0]", "mount.xyz needs three numbers");
  expectLidarFails("rpy: [0, 0, 0]", "rpy: [0, 0, x]", "mount.rpy is not a number");
}

TEST_F(ProfileTest, SensorsMissingSourceThrows)
{
  expectLidarFails("source: \"test\"", "source: \"\"", "missing source");
}

TEST_F(ProfileTest, SensorsDuplicateNameOrFrameThrows)
{
  const std::string two = std::string(kLidar) + kLidar;
  EXPECT_THROW(mobile_manipulator_profile::parseProfile(withSensors(two), "dup"),
    std::runtime_error);
  std::string cam = kCamera;
  cam.replace(cam.find("name: cam"), 9, "name: other");
  cam.replace(cam.find("frame: cam_link"), 15, "frame: lidar_link");
  EXPECT_THROW(
    mobile_manipulator_profile::parseProfile(withSensors(std::string(kLidar) + cam), "dup"),
    std::runtime_error);
  cam = kCamera;
  cam.replace(cam.find("frame: cam_link"), 15, "frame: base_link");
  EXPECT_THROW(mobile_manipulator_profile::parseProfile(withSensors(cam), "dup"),
    std::runtime_error);
}

TEST_F(ProfileTest, SensorsRgbdNeedsNoiseNote)
{
  std::string cam = kCamera;
  cam.replace(cam.find(", depth_noise_note: \"none\""), 26, "");
  EXPECT_THROW(mobile_manipulator_profile::parseProfile(withSensors(cam), "cam"),
    std::runtime_error);
}
