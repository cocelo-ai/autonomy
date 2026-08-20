#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace autonomy_light {

class DdsHeightMapPublisher final {
 public:
  DdsHeightMapPublisher(std::uint32_t domain_id, std::string topic_name,
                        std::uint32_t history_depth,
                        std::string network_interface,
                        std::string peer_address);
  ~DdsHeightMapPublisher();

  DdsHeightMapPublisher(const DdsHeightMapPublisher &) = delete;
  DdsHeightMapPublisher &operator=(const DdsHeightMapPublisher &) = delete;

  [[nodiscard]] bool ready() const;
  [[nodiscard]] const std::string &error() const;
  [[nodiscard]] bool publish(const std::vector<float> &data);

 private:
  void initialize();
  void cleanup();

  std::uint32_t domain_id_;
  std::string topic_name_;
  std::uint32_t history_depth_;
  std::string network_interface_;
  std::string peer_address_;
  std::string error_;
  int domain_{0};
  int participant_{0};
  int topic_{0};
  int writer_{0};
};

}  // namespace autonomy_light
