// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPIC_STATS_SHM__SHM_SINK_HPP_
#define TOPIC_STATS_SHM__SHM_SINK_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "topic_stats_core/sink.hpp"
#include "topic_stats_core/types.hpp"
#include "topic_stats_shm/segment.hpp"

namespace topic_stats_shm
{

/// Egress adapter that writes snapshots into a shared memory segment for a separate collector
/// process to publish.
///
/// The point of this arrangement is that an instrumented process never creates a ROS entity, never
/// links the middleware, and never appears in the graph it is measuring. Nothing here can re-enter
/// the middleware, so unlike the publishing sink it is safe to call from inside a middleware call
/// stack.
///
/// Single writer. The report scheduler is the only thread that calls publish().
class SharedMemorySink : public topic_stats_core::StatsSink
{
public:
  struct Options
  {
    /// Ring slots. Bounds how far a collector may fall behind before samples are overwritten.
    uint32_t capacity = kDefaultCapacity;
    /// Defaults to this process's own segment name.
    std::string segment_name;

    /// Remove the segment on clean shutdown. Turning this off leaves it for the collector to
    /// reclaim, which is what happens anyway when a process dies rather than exits, since a dying
    /// process runs no destructor. Mainly useful for exercising that path deliberately.
    bool unlink_on_destroy = true;
  };

  /// Returns nullptr with a reason in `error` if the segment could not be created. Callers are
  /// expected to carry on without statistics rather than fail.
  static std::unique_ptr<SharedMemorySink> create(const Options & options, std::string & error);

  void publish(const topic_stats_core::StatsReport & report) override;

  /// Samples the core produced that could not be written, because the statistic has no wire value.
  uint64_t unrepresentable_samples() const
  {
    return unrepresentable_samples_.load(std::memory_order_relaxed);
  }

  /// Samples whose node or topic name did not fit and was truncated. The sample is still written.
  uint64_t truncated_names() const
  {
    return truncated_names_.load(std::memory_order_relaxed);
  }

  const std::string & segment_name() const;

private:
  explicit SharedMemorySink(std::unique_ptr<Segment> segment);

  void write_sample(
    const topic_stats_core::NodeReport & node_report,
    const topic_stats_core::StatSample & sample,
    uint64_t snapshot_id,
    int64_t snapshot_ns);

  std::unique_ptr<Segment> segment_;
  std::atomic<uint64_t> unrepresentable_samples_{0};
  std::atomic<uint64_t> truncated_names_{0};
};

}  // namespace topic_stats_shm

#endif  // TOPIC_STATS_SHM__SHM_SINK_HPP_
