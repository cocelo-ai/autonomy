#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

#include "ExternalContracts.h"
#include "middleware/external_contract.hpp"

#define EXPECT(condition)                                                        \
  do {                                                                           \
    if (!(condition)) {                                                         \
      std::cerr << __FILE__ << ':' << __LINE__ << ": expectation failed: "    \
                << #condition << '\n';                                         \
      return EXIT_FAILURE;                                                       \
    }                                                                            \
  } while (false)

int main() {
  using autonomy_light::contract::isValidAutopilotCommand;
  using autonomy_light::contract::isValidHeightMap;
  using autonomy_light::contract::rosTopicToDdsTopic;

  const float height_map[]{0.1F, 0.2F, 0.3F, 0.4F};
  EXPECT(std::string(autonomy_light::contract::kHeightMapDdsTopic) == "height_map");
  EXPECT(std::string(autonomy_light::contract::kAutopilotCommandDdsTopic) ==
         "rt/control_command/autopilot");
  EXPECT(rosTopicToDdsTopic("/height_map") == "rt/height_map");
  EXPECT(rosTopicToDdsTopic("/control_command/autopilot") ==
         autonomy_light::contract::kAutopilotCommandDdsTopic);
  EXPECT(rosTopicToDdsTopic("height_map").empty());
  EXPECT(isValidHeightMap(height_map, 4U, 0.1F, 0.2F, 0.2F));
  EXPECT(!isValidHeightMap(height_map, 3U, 0.1F, 0.2F, 0.2F));
  EXPECT(!isValidHeightMap(height_map, 4U, 0.1F, 0.21F, 0.2F));
  const float invalid[]{0.1F, std::numeric_limits<float>::quiet_NaN(), 0.3F, 0.4F};
  EXPECT(!isValidHeightMap(invalid, 4U, 0.1F, 0.2F, 0.2F));
  EXPECT(isValidAutopilotCommand(0.1, -0.2, 0.3));
  EXPECT(!isValidAutopilotCommand(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
  EXPECT(!isValidAutopilotCommand(std::numeric_limits<double>::max(), 0.0, 0.0));
  EXPECT(std::string(core_dds_HeightMap_desc.m_typename) == "core_dds::HeightMap");
  EXPECT(std::string(core_msg_dds__CommandCore__desc.m_typename) ==
         "core::msg::dds_::CommandCore_");
  return 0;
}

#undef EXPECT
