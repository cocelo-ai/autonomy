#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace autonomy_light {

class DdsHeightMapPublisher final {
public:
  DdsHeightMapPublisher(std::uint32_t domain_id, std::string topic_name,
                        std::string type_name, std::uint32_t history_depth);
  ~DdsHeightMapPublisher();

  DdsHeightMapPublisher(const DdsHeightMapPublisher &) = delete;
  DdsHeightMapPublisher &operator=(const DdsHeightMapPublisher &) = delete;

  [[nodiscard]] bool isReady() const;
  [[nodiscard]] const std::string &error() const;
  [[nodiscard]] bool publish(const std::vector<float> &data);

private:
  void initialize();
  void cleanup();

  std::uint32_t domain_id_;
  std::string topic_name_;
  std::string type_name_;
  std::uint32_t history_depth_;
  std::string error_;
  int participant_{0};
  int topic_{0};
  int writer_{0};
};

} // namespace autonomy_light
