// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "topic_stats_shm/shm_reader.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace topic_stats_shm
{

namespace
{

/// Reads a NUL terminated field without trusting it to be terminated.
std::string bounded_string(const char * field, size_t size)
{
  const char * end = static_cast<const char *>(std::memchr(field, '\0', size));
  return std::string(field, end == nullptr ? size : static_cast<size_t>(end - field));
}

}  // namespace

SegmentReader::SegmentReader(std::unique_ptr<Segment> segment)
: segment_(std::move(segment))
{}

std::unique_ptr<SegmentReader> SegmentReader::open(const std::string & name, std::string & error)
{
  auto segment = Segment::open_readonly(name, error);
  if (segment == nullptr) {
    return nullptr;
  }
  return std::unique_ptr<SegmentReader>(new SegmentReader(std::move(segment)));
}

ProcessIdentity SegmentReader::writer_identity() const
{
  ProcessIdentity identity;
  identity.pid = segment_->header().creator_pid;
  identity.pid_namespace = segment_->header().creator_pid_namespace;
  return identity;
}

bool SegmentReader::writer_alive() const
{
  return segment_->writer_alive();
}

const std::string & SegmentReader::name() const
{
  return segment_->name();
}

void SegmentReader::skip_to_head()
{
  read_position_ = segment_->header().write_count.load(std::memory_order_acquire);
}

DrainResult SegmentReader::drain()
{
  DrainResult result;
  const Header & header = segment_->header();
  const uint32_t capacity = header.capacity;
  const uint64_t write_count = header.write_count.load(std::memory_order_acquire);

  if (write_count < read_position_) {
    // The writer's counter went backwards, which means the segment was recreated under the same
    // name by a new process reusing the pid. Start over rather than reporting a huge backlog.
    read_position_ = 0;
  }

  if (write_count - read_position_ > capacity) {
    result.lapped = write_count - read_position_ - capacity;
    read_position_ = write_count - capacity;
  }

  // Samples are grouped back into the snapshot they were written as. Insertion order is preserved
  // so that reports come out oldest first.
  std::vector<uint64_t> snapshot_order;
  std::unordered_map<uint64_t, topic_stats_core::StatsReport> by_snapshot;

  for (uint64_t index = read_position_; index < write_count; ++index) {
    const Sample & slot = segment_->sample(index);
    const uint64_t expected = index + 1;

    if (slot.sequence.load(std::memory_order_acquire) != expected) {
      // Either not written yet, or already overwritten by a later lap.
      result.torn++;
      continue;
    }

    Sample copy;
    std::memcpy(static_cast<void *>(&copy), static_cast<const void *>(&slot), sizeof(Sample));

    // Re-check after the copy: if the writer lapped us partway through, the payload just read is a
    // mixture of two samples and must be thrown away.
    if (slot.sequence.load(std::memory_order_acquire) != expected) {
      result.torn++;
      continue;
    }

    topic_stats_core::StatKind kind{};
    if (!from_wire_stat(copy.stat, kind)) {
      result.unknown_statistics++;
      continue;
    }

    topic_stats_core::StatSample sample;
    sample.stat = kind;
    sample.node_name = bounded_string(copy.node_name, kMaxNodeNameSize);
    sample.topic_name = bounded_string(copy.topic_name, kMaxTopicNameSize);
    sample.window_count = copy.window_count;
    sample.mean = topic_stats_core::Duration{copy.mean_ns};
    sample.min = topic_stats_core::Duration{copy.min_ns};
    sample.max = topic_stats_core::Duration{copy.max_ns};

    auto it = by_snapshot.find(copy.snapshot_id);
    if (it == by_snapshot.end()) {
      topic_stats_core::StatsReport report;
      report.timestamp = topic_stats_core::SysTime{} + std::chrono::duration_cast<topic_stats_core::SysClock::duration>(
                                                         topic_stats_core::Duration{copy.snapshot_ns});
      it = by_snapshot.emplace(copy.snapshot_id, std::move(report)).first;
      snapshot_order.push_back(copy.snapshot_id);
    }

    // Node identity across processes is the name; the collector has no meaningful NodeId to assign.
    auto & report = it->second;
    auto node_it = std::find_if(
      report.nodes.begin(), report.nodes.end(), [&](const auto & node) { return node.node_name == sample.node_name; });
    if (node_it == report.nodes.end()) {
      topic_stats_core::NodeReport node_report;
      node_report.node_name = sample.node_name;
      report.nodes.push_back(std::move(node_report));
      node_it = std::prev(report.nodes.end());
    }
    node_it->samples.push_back(std::move(sample));
  }

  read_position_ = write_count;

  std::sort(snapshot_order.begin(), snapshot_order.end());
  result.reports.reserve(snapshot_order.size());
  for (const uint64_t id : snapshot_order) {
    result.reports.push_back(std::move(by_snapshot[id]));
  }
  return result;
}

}  // namespace topic_stats_shm
