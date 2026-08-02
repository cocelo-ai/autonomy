#pragma once

#include "autonomy_light/types.hpp"

namespace autonomy_light {

class ChildProcesses {
public:
  ~ChildProcesses();

  void start(rclcpp::Logger logger, const std::string &name,
             const std::vector<std::string> &command);
  void stopAll(double grace_seconds = 0.8);

private:
  struct Child {
    std::string name;
    pid_t pid{-1};
  };

  static void forceStop(const Child &child);
  std::vector<Child> children_;
};

} // namespace autonomy_light
