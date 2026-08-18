// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "topic_stats_shm/shm_sink.hpp"

#include <unistd.h>

#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace topic_stats_shm
{

SharedMemorySink::SharedMemorySink(std::unique_ptr<Segment> segment)
: segment_(std::move(segment))
{}

std::unique_ptr<SharedMemorySink> SharedMemorySink::create(const Options & options, std::string & error)
{
  const std::string name =
    options.segment_name.empty() ? segment_name_for(current_process_identity()) : options.segment_name;
  auto segment = Segment::create(name, options.capacity, error);
  if (segment == nullptr) {
    return nullptr;
  }
  if (!options.unlink_on_destroy) {
    segment->release_ownership();
  }
  return std::unique_ptr<SharedMemorySink>(new SharedMemorySink(std::move(segment)));
}

const std::string & SharedMemorySink::segment_name() const
{
  return segment_->name();
}

void SharedMemorySink::write_sample(
  const topic_stats_core::NodeReport & node_report,
  const topic_stats_core::StatSample & sample,
  uint64_t snapshot_id,
  int64_t snapshot_ns)
{
  uint8_t wire_stat = 0;
  if (!to_wire_stat(sample.stat, wire_stat)) {
    unrepresentable_samples_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  Header & header = segment_->header();
  // Single writer, so nobody else advances this and a relaxed load is enough.
  const uint64_t index = header.write_count.load(std::memory_order_relaxed);
  Sample & slot = segment_->sample(index);

  // Mark the slot invalid before touching the payload, so a reader mid-copy can tell.
  slot.sequence.store(0, std::memory_order_release);

  slot.mean_ns = sample.mean.count();
  slot.min_ns = sample.min.count();
  slot.max_ns = sample.max.count();
  slot.snapshot_id = snapshot_id;
  slot.snapshot_ns = snapshot_ns;
  slot.window_count = sample.window_count;
  slot.stat = wire_stat;

  uint8_t flags = kFlagNone;
  if (copy_bounded(node_report.node_name, slot.node_name, kMaxNodeNameSize)) {
    flags |= kFlagNodeNameTruncated;
  }
  if (copy_bounded(sample.topic_name, slot.topic_name, kMaxTopicNameSize)) {
    flags |= kFlagTopicNameTruncated;
  }
  slot.flags = flags;
  slot.reserved[0] = 0;
  slot.reserved[1] = 0;
  if (flags != kFlagNone) {
    truncated_names_.fetch_add(1, std::memory_order_relaxed);
  }

  // Publishes the payload to any reader that subsequently sees this sequence value.
  slot.sequence.store(index + 1, std::memory_order_release);
  header.write_count.store(index + 1, std::memory_order_release);
}

void SharedMemorySink::publish(const topic_stats_core::StatsReport & report)
{
  if (report.empty()) {
    return;
  }

  Header & header = segment_->header();
  const uint64_t snapshot_id = header.snapshot_count.load(std::memory_order_relaxed) + 1;
  const auto snapshot_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(report.timestamp.time_since_epoch()).count();

  for (const auto & node_report : report.nodes) {
    for (const auto & sample : node_report.samples) {
      write_sample(node_report, sample, snapshot_id, snapshot_ns);
    }
  }

  // Published last, so a reader that sees this snapshot id knows every sample of it is already in
  // the ring.
  header.last_snapshot_ns.store(snapshot_ns, std::memory_order_release);
  header.snapshot_count.store(snapshot_id, std::memory_order_release);
}

}  // namespace topic_stats_shm
