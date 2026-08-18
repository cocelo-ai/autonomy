#include <chrono>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <core/msg/command_core.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace autonomy_light {

class CommandUserBridge final : public rclcpp::Node {
 public:
  CommandUserBridge() : Node("command_user_bridge") {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/nav2/cmd_vel");
    const auto output_topic = declare_parameter<std::string>(
        "output_topic", "/control_command/autopilot");
    const auto publish_rate_hz = declare_parameter<double>("publish_rate_hz", 20.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.5);
    const auto custom_drive_id = declare_parameter<int>("custom_drive_id", 2);

    if (input_topic.empty() || output_topic.empty() || publish_rate_hz <= 0.0 ||
        command_timeout_sec_ < 0.0 || custom_drive_id < 0 || custom_drive_id > 255) {
      throw std::invalid_argument("command_user_bridge parameters are invalid");
    }
    custom_drive_id_ = static_cast<std::uint8_t>(custom_drive_id);

    publisher_ = create_publisher<core::msg::CommandCore>(output_topic, rclcpp::QoS(10));
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        input_topic, rclcpp::QoS(10),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr command) {
          std::lock_guard<std::mutex> lock(mutex_);
          command_ = *command;
          last_command_time_ = std::chrono::steady_clock::now();
          have_command_ = true;
        });
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_hz)),
        [this]() { publish(); });

    RCLCPP_INFO(get_logger(), "Autopilot bridge: %s -> %s (custom_drive_id: %u)",
                input_topic.c_str(), output_topic.c_str(),
                static_cast<unsigned int>(custom_drive_id_));
  }

 private:
  void publish() {
    core::msg::CommandCore message;
    message.motion_command.fill(0.0F);
    message.event.estop = false;
    message.event.wake = false;
    message.event.sleep = false;
    message.event.rough_drive_toggle = false;
    message.event.custom_drive_toggle = false;
    message.custom_drive_id = custom_drive_id_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (have_command_ &&
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        last_command_time_)
                  .count() <= command_timeout_sec_) {
        message.motion_command[0] = static_cast<float>(command_.linear.x);
        message.motion_command[1] = static_cast<float>(command_.linear.y);
        message.motion_command[2] = static_cast<float>(command_.angular.z);
      }
    }
    publisher_->publish(message);
  }

  double command_timeout_sec_{0.5};
  std::uint8_t custom_drive_id_{2};
  std::mutex mutex_;
  geometry_msgs::msg::Twist command_;
  std::chrono::steady_clock::time_point last_command_time_{};
  bool have_command_{false};
  rclcpp::Publisher<core::msg::CommandCore>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autonomy_light

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomy_light::CommandUserBridge>());
  rclcpp::shutdown();
  return 0;
}
