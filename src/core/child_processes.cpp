#include "autonomy_light/child_processes.hpp"

namespace autonomy_light {

ChildProcesses::~ChildProcesses() { stopAll(); }

void ChildProcesses::start(rclcpp::Logger logger, const std::string &name,
                           const std::vector<std::string> &command) {
  if (command.empty()) {
    RCLCPP_INFO(logger, "%s launch disabled: command is empty", name.c_str());
    return;
  }
  const pid_t pid = fork();
  if (pid < 0) {
    RCLCPP_ERROR(logger, "Failed to fork %s: %s", name.c_str(),
                 std::strerror(errno));
    return;
  }
  if (pid == 0) {
    std::vector<char *> argv;
    argv.reserve(command.size() + 1);
    for (const auto &part : command) {
      argv.push_back(const_cast<char *>(part.c_str()));
    }
    argv.push_back(nullptr);
    execvp(argv.front(), argv.data());
    std::fprintf(stderr, "Failed to exec %s: %s\n", command.front().c_str(),
                 std::strerror(errno));
    _exit(127);
  }
  children_.push_back({name, pid});
  RCLCPP_INFO(logger, "Started %s pid=%d", name.c_str(), static_cast<int>(pid));
}

void ChildProcesses::stopAll(const double grace_seconds) {
  for (const auto &child : children_) {
    kill(child.pid, SIGINT);
  }
  const bool wait_forever = grace_seconds < 0.0;
  const auto grace = std::chrono::duration<double>(
      wait_forever ? 5.0 : std::max(0.0, grace_seconds));
  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(grace);
  while (!children_.empty() && std::chrono::steady_clock::now() < deadline) {
    for (auto it = children_.begin(); it != children_.end();) {
      int status = 0;
      const pid_t ret = waitpid(it->pid, &status, WNOHANG);
      it = (ret == it->pid || ret < 0) ? children_.erase(it) : std::next(it);
    }
    if (!children_.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  if (wait_forever) {
    for (auto it = children_.begin(); it != children_.end();) {
      if (it->name == "Super-LIO") {
        ++it;
      } else {
        forceStop(*it);
        it = children_.erase(it);
      }
    }
    while (!children_.empty()) {
      for (auto it = children_.begin(); it != children_.end();) {
        int status = 0;
        const pid_t ret = waitpid(it->pid, &status, WNOHANG);
        it = (ret == it->pid || ret < 0) ? children_.erase(it) : std::next(it);
      }
      if (!children_.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    return;
  }
  for (const auto &child : children_) {
    forceStop(child);
  }
  children_.clear();
}

void ChildProcesses::forceStop(const Child &child) {
  int status = 0;
  if (waitpid(child.pid, &status, WNOHANG) != 0) {
    return;
  }
  kill(child.pid, SIGTERM);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < deadline) {
    const pid_t ret = waitpid(child.pid, &status, WNOHANG);
    if (ret == child.pid || ret < 0) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (waitpid(child.pid, &status, WNOHANG) == 0) {
    kill(child.pid, SIGKILL);
    waitpid(child.pid, &status, 0);
  }
}

} // namespace autonomy_light
