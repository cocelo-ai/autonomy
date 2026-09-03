#include "middleware/external_dds_bridge.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>

#include <dds/dds.h>
#include <geometry_msgs/msg/twist.hpp>

#include "ExternalContracts.h"
#include "autonomy_light/msg/height_map.hpp"
#include "middleware/external_contract.hpp"

namespace autonomy_light::middleware {
namespace {

constexpr int kMinimumDdsDomain = 0;
constexpr int kMaximumDdsDomain = 232;

struct BridgeOptions {
  std::size_t internal_domain{42U};
  // The command transport remains on its existing ROS-compatible domain 0,
  // while the control SBC consumes raw core_dds::HeightMap on domain 1.
  std::size_t command_external_domain{0U};
  std::size_t height_map_external_domain{1U};
  std::string height_map_input_topic{"/autonomy_light/elevation_map"};
  std::string command_input_topic{contract::kAutopilotCommandRosTopic};
};

std::size_t validateDomainId(const int value, const char *const parameter_name) {
  if (value < kMinimumDdsDomain || value > kMaximumDdsDomain) {
    throw std::invalid_argument(std::string(parameter_name) + " must be in [0, 232]");
  }
  return static_cast<std::size_t>(value);
}

BridgeOptions loadOptions(rclcpp::Node &node) {
  BridgeOptions options;
  options.internal_domain = validateDomainId(
      node.declare_parameter<int>("internal_domain_id", static_cast<int>(options.internal_domain)),
      "internal_domain_id");
  options.command_external_domain = validateDomainId(
      node.declare_parameter<int>("external_domain_id",
                                  static_cast<int>(options.command_external_domain)),
      "external_domain_id");
  options.height_map_external_domain = validateDomainId(
      node.declare_parameter<int>("height_map_external_domain_id",
                                  static_cast<int>(options.height_map_external_domain)),
      "height_map_external_domain_id");
  options.height_map_input_topic = node.declare_parameter<std::string>(
      "height_map_input_topic", options.height_map_input_topic);
  options.command_input_topic = node.declare_parameter<std::string>(
      "command_input_topic", options.command_input_topic);
  if (options.internal_domain == options.command_external_domain ||
      options.internal_domain == options.height_map_external_domain ||
      options.height_map_input_topic.empty() || options.command_input_topic.empty()) {
    throw std::invalid_argument(
        "DDS bridge output domains must differ from the internal domain and input topic names must be non-empty");
  }
  return options;
}

std::string ddsError(const char *const operation, const dds_return_t result) {
  return std::string(operation) + ": " + dds_strretcode(-result);
}

dds_qos_t *makeOutputQos(const std::size_t history_depth, const bool reliable) {
  dds_qos_t *const qos = dds_create_qos();
  if (qos == nullptr) {
    throw std::runtime_error("DDS QoS allocation failed");
  }
  dds_qset_reliability(qos, reliable ? DDS_RELIABILITY_RELIABLE : DDS_RELIABILITY_BEST_EFFORT,
                       reliable ? DDS_MSECS(500) : DDS_SECS(0));
  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, static_cast<int32_t>(history_depth));
  return qos;
}

dds_entity_t makeWriter(const dds_entity_t participant,
                        const dds_topic_descriptor_t *const descriptor,
                        const char *const topic_name,
                        const std::size_t history_depth,
                        const bool reliable) {
  const dds_entity_t topic = dds_create_topic(participant, descriptor, topic_name, nullptr, nullptr);
  if (topic < 0) {
    throw std::runtime_error(ddsError("DDS topic creation failed", topic));
  }
  dds_qos_t *const qos = makeOutputQos(history_depth, reliable);
  const dds_entity_t writer = dds_create_writer(participant, topic, qos, nullptr);
  dds_delete_qos(qos);
  if (writer < 0) {
    throw std::runtime_error(ddsError("DDS writer creation failed", writer));
  }
  return writer;
}

class ExternalDdsWriter final {
 public:
  ExternalDdsWriter(const std::size_t height_map_domain_id,
                    const std::size_t command_domain_id) {
    height_map_participant_ =
        dds_create_participant(static_cast<dds_domainid_t>(height_map_domain_id), nullptr, nullptr);
    if (height_map_participant_ < 0) {
      throw std::runtime_error(
          ddsError("HeightMap DDS participant creation failed", height_map_participant_));
    }
    command_participant_ =
        dds_create_participant(static_cast<dds_domainid_t>(command_domain_id), nullptr, nullptr);
    if (command_participant_ < 0) {
      (void)dds_delete(height_map_participant_);
      height_map_participant_ = 0;
      throw std::runtime_error(
          ddsError("Autopilot command DDS participant creation failed", command_participant_));
    }
    try {
      height_map_writer_ = makeWriter(height_map_participant_, &core_dds_HeightMap_desc,
                                      contract::kHeightMapDdsTopic, contract::kHeightMapHistoryDepth,
                                      false);
      command_writer_ = makeWriter(command_participant_, &core_msg_dds__CommandCore__desc,
                                   contract::kAutopilotCommandDdsTopic,
                                   contract::kAutopilotCommandHistoryDepth,
                                   true);
    } catch (...) {
      deleteParticipants();
      throw;
    }
  }

  ~ExternalDdsWriter() {
    deleteParticipants();
  }

  ExternalDdsWriter(const ExternalDdsWriter &) = delete;
  ExternalDdsWriter &operator=(const ExternalDdsWriter &) = delete;

  void writeHeightMap(const msg::HeightMap &message) const {
    core_dds_HeightMap sample{};
    sample.data._maximum = static_cast<uint32_t>(message.data.size());
    sample.data._length = static_cast<uint32_t>(message.data.size());
    sample.data._buffer = const_cast<float *>(message.data.data());
    sample.data._release = false;
    const dds_return_t result = dds_write(height_map_writer_, &sample);
    if (result < 0) {
      throw std::runtime_error(ddsError("HeightMap DDS write failed", result));
    }
  }

  void writeAutopilotCommand(const geometry_msgs::msg::Twist &message) const {
    core_msg_dds__CommandCore_ sample{};
    sample.motion_command[0] = static_cast<float>(message.linear.x);
    sample.motion_command[1] = static_cast<float>(message.linear.y);
    sample.motion_command[2] = static_cast<float>(message.angular.z);
    const dds_return_t result = dds_write(command_writer_, &sample);
    if (result < 0) {
      throw std::runtime_error(ddsError("Autopilot command DDS write failed", result));
    }
  }

 private:
  void deleteParticipants() {
    if (command_participant_ > 0) {
      (void)dds_delete(command_participant_);
      command_participant_ = 0;
    }
    if (height_map_participant_ > 0) {
      (void)dds_delete(height_map_participant_);
      height_map_participant_ = 0;
    }
  }

  dds_entity_t height_map_participant_{0};
  dds_entity_t command_participant_{0};
  dds_entity_t height_map_writer_{0};
  dds_entity_t command_writer_{0};
};

}  // namespace

struct ExternalDdsBridge::Impl {
  explicit Impl(ExternalDdsBridge &node)
      : node(node), options(loadOptions(node)),
        writer(options.height_map_external_domain, options.command_external_domain) {
    const auto map_qos = rclcpp::QoS(1).reliable();
    height_map_subscription = node.create_subscription<msg::HeightMap>(
        options.height_map_input_topic, map_qos,
        [this](msg::HeightMap::ConstSharedPtr message) { forwardHeightMap(*message); });
    command_subscription = node.create_subscription<geometry_msgs::msg::Twist>(
        options.command_input_topic,
        rclcpp::QoS(contract::kAutopilotCommandHistoryDepth).reliable(),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
          forwardAutopilotCommand(*message);
        });

    RCLCPP_INFO(node.get_logger(),
                "DDS egress: %s -> domain %zu/%s (core_dds::HeightMap, best-effort, depth %zu), "
                "%s -> domain %zu/%s (core::msg::dds_::CommandCore_, reliable, depth %zu)",
                options.height_map_input_topic.c_str(), options.height_map_external_domain,
                contract::kHeightMapDdsTopic, contract::kHeightMapHistoryDepth,
                options.command_input_topic.c_str(), options.command_external_domain,
                contract::kAutopilotCommandDdsTopic, contract::kAutopilotCommandHistoryDepth);
  }

 private:
  void forwardHeightMap(const msg::HeightMap &message) {
    if (!contract::isValidHeightMap(message.data.data(), message.data.size(), message.resolution,
                                    message.x_length, message.y_length)) {
      RCLCPP_ERROR_THROTTLE(node.get_logger(), *node.get_clock(), 5000,
                            "Dropped malformed HeightMap before DDS egress");
      return;
    }
    try {
      writer.writeHeightMap(message);
    } catch (const std::exception &error) {
      RCLCPP_ERROR_THROTTLE(node.get_logger(), *node.get_clock(), 5000,
                            "HeightMap DDS egress failed: %s", error.what());
    }
  }

  void forwardAutopilotCommand(const geometry_msgs::msg::Twist &message) {
    if (!contract::isValidAutopilotCommand(message.linear.x, message.linear.y,
                                           message.angular.z)) {
      RCLCPP_ERROR_THROTTLE(node.get_logger(), *node.get_clock(), 5000,
                            "Dropped malformed autopilot command before DDS egress");
      return;
    }
    try {
      writer.writeAutopilotCommand(message);
    } catch (const std::exception &error) {
      RCLCPP_ERROR_THROTTLE(node.get_logger(), *node.get_clock(), 5000,
                            "Autopilot command DDS egress failed: %s", error.what());
    }
  }

  ExternalDdsBridge &node;
  BridgeOptions options;
  ExternalDdsWriter writer;
  rclcpp::Subscription<msg::HeightMap>::SharedPtr height_map_subscription;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription;
};

ExternalDdsBridge::ExternalDdsBridge(const rclcpp::NodeOptions &options)
    : Node("external_dds_bridge", options), impl_(std::make_unique<Impl>(*this)) {}

ExternalDdsBridge::~ExternalDdsBridge() = default;

}  // namespace autonomy_light::middleware
