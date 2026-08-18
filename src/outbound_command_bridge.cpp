#include <cstddef>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <core/msg/command_core.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

struct BridgeOptions {
  std::size_t internal_domain{10};
  std::size_t external_domain{0};
  std::string input_topic{"/autonomy_light/command_out"};
  std::string output_topic{"/control_command/autopilot"};
};

std::size_t parseDomainId(const std::string &value, const char *option) {
  std::size_t consumed = 0;
  unsigned long parsed = 0;
  try {
    parsed = std::stoul(value, &consumed);
  } catch (const std::exception &) {
    throw std::invalid_argument(std::string(option) + " must be an integer in [0, 232]");
  }
  if (consumed != value.size() || parsed > 232U) {
    throw std::invalid_argument(std::string(option) + " must be an integer in [0, 232]");
  }
  return static_cast<std::size_t>(parsed);
}

BridgeOptions parseOptions(int argc, char **argv) {
  BridgeOptions options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--internal-domain" || argument == "--external-domain" ||
        argument == "--input-topic" || argument == "--output-topic") {
      if (++index == argc) {
        throw std::invalid_argument(argument + " requires a value");
      }
      const std::string value(argv[index]);
      if (argument == "--internal-domain") {
        options.internal_domain = parseDomainId(value, "--internal-domain");
      } else if (argument == "--external-domain") {
        options.external_domain = parseDomainId(value, "--external-domain");
      } else if (argument == "--input-topic") {
        options.input_topic = value;
      } else {
        options.output_topic = value;
      }
    } else if (argument == "--help" || argument == "-h") {
      std::cout << "Usage: outbound_command_bridge [--internal-domain ID] [--external-domain ID] "
                << "[--input-topic TOPIC] [--output-topic TOPIC]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unsupported option: " + argument);
    }
  }
  if (options.internal_domain == options.external_domain || options.input_topic.empty() ||
      options.output_topic.empty()) {
    throw std::invalid_argument("bridge domains and topic names must differ and be non-empty");
  }
  return options;
}

rclcpp::Context::SharedPtr makeContext(std::size_t domain_id) {
  rclcpp::InitOptions init_options;
  init_options.auto_initialize_logging(false);
  init_options.set_domain_id(domain_id);
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr, init_options);
  return context;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const BridgeOptions options = parseOptions(argc, argv);
    auto internal_context = makeContext(options.internal_domain);
    auto external_context = makeContext(options.external_domain);
    rclcpp::install_signal_handlers();

    rclcpp::NodeOptions internal_node_options;
    internal_node_options.context(internal_context);
    auto internal_node = std::make_shared<rclcpp::Node>(
      "outbound_command_bridge_internal", internal_node_options);

    rclcpp::NodeOptions external_node_options;
    external_node_options.context(external_context);
    auto external_node = std::make_shared<rclcpp::Node>(
      "outbound_command_bridge_external", external_node_options);

    auto publisher = external_node->create_publisher<core::msg::CommandCore>(
      options.output_topic, rclcpp::QoS(1).reliable());
    auto command_subscription = internal_node->create_subscription<core::msg::CommandCore>(
      options.input_topic, rclcpp::QoS(1).reliable(),
      [publisher](core::msg::CommandCore::ConstSharedPtr command) {
        publisher->publish(*command);
      });
    std::cout << "outbound_command_bridge: " << options.input_topic << " (domain "
              << options.internal_domain << ") -> " << options.output_topic << " (domain "
              << options.external_domain << '\n';

    rclcpp::ExecutorOptions internal_executor_options;
    internal_executor_options.context = internal_context;
    rclcpp::executors::SingleThreadedExecutor internal_executor(internal_executor_options);
    internal_executor.add_node(internal_node);

    rclcpp::ExecutorOptions external_executor_options;
    external_executor_options.context = external_context;
    rclcpp::executors::SingleThreadedExecutor external_executor(external_executor_options);
    external_executor.add_node(external_node);
    std::thread external_spin([&external_executor]() { external_executor.spin(); });
    internal_executor.spin();
    external_executor.cancel();
    external_spin.join();

    external_context->shutdown("outbound command bridge stopped");
    internal_context->shutdown("outbound command bridge stopped");
  } catch (const std::exception &error) {
    std::cerr << "outbound_command_bridge: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
