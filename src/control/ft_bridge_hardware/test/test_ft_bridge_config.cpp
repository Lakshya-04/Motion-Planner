#include <gtest/gtest.h>

#include "ft_bridge_hardware/ft_bridge_config.hpp"

namespace ft_bridge_hardware
{
namespace
{

TEST(FtBridgeConfig, EmptyParametersResolveToDocumentedDefaults)
{
  const auto config = parseFtBridgeConfig({});

  EXPECT_EQ(config.source_topic, "/ufactory/uf_ftsensor_ext_states");
  EXPECT_EQ(config.injection_topic, "~/injected_wrench");
  EXPECT_EQ(config.setpoint_topic, "~/target_wrench");
  EXPECT_FALSE(config.injection_mode);
}

TEST(FtBridgeConfig, OverridesEachFieldIndependently)
{
  const std::unordered_map<std::string, std::string> params = {
    {"source_topic", "/custom/source"},
    {"injection_topic", "/custom/injected"},
    {"setpoint_topic", "/custom/setpoint"},
  };
  const auto config = parseFtBridgeConfig(params);

  EXPECT_EQ(config.source_topic, "/custom/source");
  EXPECT_EQ(config.injection_topic, "/custom/injected");
  EXPECT_EQ(config.setpoint_topic, "/custom/setpoint");
  EXPECT_FALSE(config.injection_mode) << "unset injection_mode must default false even when "
    "other keys are present";
}

TEST(FtBridgeConfig, InjectionModeParsesTrueLiterally)
{
  const auto config = parseFtBridgeConfig({{"injection_mode", "true"}});
  EXPECT_TRUE(config.injection_mode);
}

TEST(FtBridgeConfig, InjectionModeIsFalseForAnyNonTrueValueNeverThrows)
{
  EXPECT_FALSE(parseFtBridgeConfig({{"injection_mode", "false"}}).injection_mode);
  EXPECT_FALSE(parseFtBridgeConfig({{"injection_mode", "1"}}).injection_mode);
  EXPECT_FALSE(parseFtBridgeConfig({{"injection_mode", "TRUE"}}).injection_mode);
  EXPECT_FALSE(parseFtBridgeConfig({{"injection_mode", ""}}).injection_mode);
}

TEST(FtBridgeConfig, UnrecognizedKeysAreIgnoredNotFatal)
{
  const auto config = parseFtBridgeConfig({{"some_future_param", "whatever"}});
  EXPECT_EQ(config.source_topic, "/ufactory/uf_ftsensor_ext_states");
}

}  // namespace
}  // namespace ft_bridge_hardware
