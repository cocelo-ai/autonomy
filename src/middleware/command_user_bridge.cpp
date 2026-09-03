#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include "middleware/command_user_bridge.hpp"
#include "middleware/external_contract.hpp"

namespace autonomy_light::middleware {

class CommandUserBridge final : public rclcpp::Node {
 public:
  CommandUserBridge() : Node("command_user_bridge") {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/nav2/cmd_vel");
    const auto output_topic = declare_parameter<std::string>(
        "output_topic", contract::kAutopilotCommandRosTopic);
    const auto goal_topic = declare_parameter<std::string>("goal_topic", "/goal_pose");
    const auto goal_status_topic = declare_parameter<std::string>(
        "goal_status_topic", "/navigate_to_pose/_action/status");
    require_goal_before_publish_ =
        declare_parameter<bool>("require_goal_before_publish", true);
    const auto publish_rate_hz = declare_parameter<double>("publish_rate_hz", 20.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.5);

    if (input_topic.empty() || output_topic.empty() || publish_rate_hz <= 0.0 ||
        command_timeout_sec_ < 0.0 ||
        (require_goal_before_publish_ && goal_topic.empty() && goal_status_topic.empty())) {
      throw std::invalid_argument("command_user_bridge parameters are invalid");
    }

    goal_gate_open_ = !require_goal_before_publish_;

    publisher_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, rclcpp::QoS(10));
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        input_topic, rclcpp::QoS(10),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr command) {
          std::lock_guard<std::mutex> lock(mutex_);
          command_ = *command;
          last_command_time_ = std::chrono::steady_clock::now();
          have_command_ = true;
        });
    if (require_goal_before_publish_ && !goal_topic.empty()) {
      goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          goal_topic, rclcpp::QoS(10),
          [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr) {
            open_goal_gate("goal pose");
          });
    }
    if (require_goal_before_publish_ && !goal_status_topic.empty()) {
      goal_status_subscription_ = create_subscription<action_msgs::msg::GoalStatusArray>(
          goal_status_topic, rclcpp::QoS(10),
          [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr statuses) {
            for (const auto &goal : statuses->status_list) {
              if (goal.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
                  goal.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
                  goal.status == action_msgs::msg::GoalStatus::STATUS_CANCELING) {
                open_goal_gate("NavigateToPose action");
                return;
              }
            }
          });
    }
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_hz)),
        [this]() { publish(); });

    RCLCPP_INFO(get_logger(),
                "Autopilot command bridge: %s -> %s (goal gate: %s)", input_topic.c_str(),
                output_topic.c_str(), require_goal_before_publish_ ? "enabled" : "disabled");
  }

 private:
  void open_goal_gate(const char *source) {
    bool opened = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!goal_gate_open_) {
        goal_gate_open_ = true;
        opened = true;
      }
    }
    if (opened) {
      RCLCPP_INFO(get_logger(), "Goal received from %s; command publishing enabled", source);
    }
  }

  void publish() {
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!goal_gate_open_ || !have_command_) {
        return;
      }
      if (have_command_ &&
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        last_command_time_)
                  .count() <= command_timeout_sec_) {
        vx = command_.linear.x;
        vy = command_.linear.y;
        wz = command_.angular.z;
      }
    }
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Ignoring non-finite Nav2 command");
      vx = 0.0;
      vy = 0.0;
      wz = 0.0;
    }
    geometry_msgs::msg::Twist message;
    message.linear.x = vx;
    message.linear.y = vy;
    message.angular.z = wz;
    publisher_->publish(message);
  }

  double command_timeout_sec_{0.5};
  std::mutex mutex_;
  geometry_msgs::msg::Twist command_;
  std::chrono::steady_clock::time_point last_command_time_{};
  bool have_command_{false};
  bool require_goal_before_publish_{true};
  bool goal_gate_open_{false};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr
      goal_status_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

std::shared_ptr<rclcpp::Node> createCommandUserBridge() {
  return std::make_shared<CommandUserBridge>();
}

}  // namespace autonomy_light::middleware
