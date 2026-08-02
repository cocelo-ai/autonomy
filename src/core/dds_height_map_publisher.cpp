#include "autonomy_light/dds_height_map_publisher.hpp"

#include <algorithm>
#include <utility>

#include <dds/dds.h>

#include "HeightMap.h"

namespace autonomy_light {
namespace {

std::string ddsError(const char *action, const int ret) {
  return std::string(action) + ": " + dds_strretcode(-ret);
}

} // namespace

DdsHeightMapPublisher::DdsHeightMapPublisher(const std::uint32_t domain_id,
                                             std::string topic_name,
                                             std::string type_name,
                                             const std::uint32_t history_depth)
    : domain_id_(domain_id), topic_name_(std::move(topic_name)),
      type_name_(std::move(type_name)), history_depth_(history_depth) {
  initialize();
}

DdsHeightMapPublisher::~DdsHeightMapPublisher() { cleanup(); }

bool DdsHeightMapPublisher::isReady() const { return writer_ > 0; }

const std::string &DdsHeightMapPublisher::error() const { return error_; }

bool DdsHeightMapPublisher::publish(const std::vector<float> &data) {
  if (!isReady()) {
    return false;
  }
  core_dds_HeightMap sample{};
  sample.data._maximum = static_cast<std::uint32_t>(data.size());
  sample.data._length = static_cast<std::uint32_t>(data.size());
  sample.data._buffer = const_cast<float *>(data.data());
  sample.data._release = false;
  const int ret = dds_write(writer_, &sample);
  if (ret >= 0) {
    return true;
  }
  error_ = ddsError("failed to write DDS height map", ret);
  return false;
}

void DdsHeightMapPublisher::initialize() {
  if (topic_name_.empty()) {
    error_ = "DDS height map topic must not be empty";
    return;
  }
  if (type_name_ != core_dds_HeightMap_desc.m_typename) {
    error_ = "DDS height map type must be " +
             std::string(core_dds_HeightMap_desc.m_typename) + ", got " +
             type_name_;
    return;
  }
  participant_ = dds_create_participant(static_cast<dds_domainid_t>(domain_id_),
                                        nullptr, nullptr);
  if (participant_ < 0) {
    error_ = ddsError("failed to create DDS participant", participant_);
    participant_ = 0;
    return;
  }
  topic_ = dds_create_topic(participant_, &core_dds_HeightMap_desc,
                            topic_name_.c_str(), nullptr, nullptr);
  if (topic_ < 0) {
    error_ = ddsError("failed to create DDS height map topic", topic_);
    topic_ = 0;
    cleanup();
    return;
  }
  dds_qos_t *qos = dds_create_qos();
  if (qos == nullptr) {
    error_ = "failed to allocate DDS writer QoS";
    cleanup();
    return;
  }
  dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(0));
  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST,
                   static_cast<int>(std::max(1U, history_depth_)));
  writer_ = dds_create_writer(participant_, topic_, qos, nullptr);
  dds_delete_qos(qos);
  if (writer_ < 0) {
    error_ = ddsError("failed to create DDS height map writer", writer_);
    writer_ = 0;
    cleanup();
  }
}

void DdsHeightMapPublisher::cleanup() {
  if (writer_ > 0) {
    dds_delete(writer_);
    writer_ = 0;
  }
  if (topic_ > 0) {
    dds_delete(topic_);
    topic_ = 0;
  }
  if (participant_ > 0) {
    dds_delete(participant_);
    participant_ = 0;
  }
}

} // namespace autonomy_light
