// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "topic_stats_tracetools/trace_collector.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "topic_stats_shm/segment.hpp"
#include "topic_stats_shm/shm_sink.hpp"

namespace topic_stats_tracetools
{

namespace
{

std::string get_env(const char * name, const char * fallback)
{
  const char * value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') {
    return fallback;
  }
  return value;
}

/// Name of the segment to remove at exit, empty when there is nothing to remove. A function-local
/// static so that it is alive before main() and still alive during atexit.
std::string & segment_to_unlink()
{
  static std::string name;
  return name;
}

std::string fully_qualified_node_name(const char * name, const char * name_space)
{
  const std::string ns = (name_space == nullptr) ? "/" : name_space;
  const std::string leaf = (name == nullptr) ? "" : name;
  if (!ns.empty() && ns.back() == '/') {
    return ns + leaf;
  }
  return ns + "/" + leaf;
}

}  // namespace

TraceCollector::TraceCollector(
  const Config & config,
  std::shared_ptr<topic_stats_core::Collector> collector,
  std::unique_ptr<topic_stats_core::StatsSink> sink)
: config_(config)
, collector_(std::move(collector))
, sink_(std::move(sink))
{}

TraceCollector::~TraceCollector()
{
  // Stopped first so the timer thread cannot be mid-report while the sink is destroyed.
  if (timer_) {
    timer_->stop();
  }
}

void TraceCollector::start()
{
  timer_.emplace(std::bind(&TraceCollector::report, this), config_.report_period);
  timer_->start();
}

TraceCollector & TraceCollector::instance()
{
  static TraceCollector * instance_ = [] {
    Config config;
    config.window_size = std::stoul(get_env("ROS_TOPIC_STATISTICS_WINDOW_SIZE", "50"));
    config.report_period = std::chrono::milliseconds(
      static_cast<size_t>(std::stof(get_env("ROS_TOPIC_STATISTICS_PUBLISH_PERIOD", "1.0")) * 1000));
    config.idle_eviction_reports =
      static_cast<uint32_t>(std::stoul(get_env("ROS_TOPIC_STATISTICS_IDLE_EVICTION_REPORTS", "600")));

    topic_stats_core::Collector::Options options;
    options.window_size = config.window_size;
    auto collector =
      std::make_shared<topic_stats_core::Collector>(options, std::make_shared<topic_stats_core::SystemClock>());

    // Shared memory is the only egress offered here. Publishing would mean creating an RMW
    // publisher from inside a tracepoint that the middleware calls during rmw_publish, which is
    // the reentrancy problem at its sharpest; nothing in the shared memory path can call back into
    // the middleware at all.
    std::unique_ptr<topic_stats_core::StatsSink> sink;
    const std::string egress = get_env("ROS_TOPIC_STATISTICS_EGRESS", "shared_memory");
    if (egress == "shared_memory") {
      topic_stats_shm::SharedMemorySink::Options shm_options;
      shm_options.capacity = static_cast<uint32_t>(std::stoul(
        get_env("ROS_TOPIC_STATISTICS_SHM_CAPACITY", std::to_string(topic_stats_shm::kDefaultCapacity).c_str())));
      std::string error;
      auto shm_sink = topic_stats_shm::SharedMemorySink::create(shm_options, error);
      if (shm_sink != nullptr) {
        segment_to_unlink() = shm_sink->segment_name();
        sink = std::move(shm_sink);
      }
      if (sink == nullptr) {
        // Instrumentation must never stop a process from starting.
        fprintf(stderr, "[topic_stats_tracetools] shared memory unavailable (%s), reporting disabled\n", error.c_str());
      }
    } else if (egress != "none") {
      fprintf(
        stderr, "[topic_stats_tracetools] egress '%s' is not supported here, reporting disabled\n", egress.c_str());
    }
    if (sink == nullptr) {
      sink = std::make_unique<topic_stats_core::NullSink>();
    }

    // Deliberately leaked. The first tracepoint can fire before main() and the last can fire
    // during static destruction, so an object destroyed at exit would be a use-after-free waiting
    // to happen. The process is going away anyway.
    //
    // The consequence is that the sink is never destroyed either, so nothing unlinks the shared
    // memory segment on a clean exit. A collector reclaims segments whose writer has died, but on
    // a machine with no collector running they would otherwise accumulate in /dev/shm until
    // reboot. Unlinking at exit costs nothing and does not disturb the mapping: an unlinked
    // segment stays valid for whoever already has it open, so tracepoints firing during static
    // destruction still write somewhere harmless.
    std::atexit([] {
      if (!segment_to_unlink().empty()) {
        std::string error;
        topic_stats_shm::Segment::unlink(segment_to_unlink(), error);
      }
    });

    auto * built = new TraceCollector(config, std::move(collector), std::move(sink));
    built->start();
    return built;
  }();
  return *instance_;
}

void TraceCollector::on_node_init(const void * node_handle, const char * node_name, const char * node_namespace)
{
  if (node_handle == nullptr) {
    return;
  }
  const auto id = collector_->register_node({fully_qualified_node_name(node_name, node_namespace)});
  // A repeated handle means the address was reused by a new node, since nothing ever told us the
  // old one went away. The newest mapping is the correct one.
  if (!nodes_.add(node_handle, id)) {
    nodes_.update(node_handle, [&](topic_stats_core::NodeId & existing) { existing = id; });
  }
}

void TraceCollector::add_endpoint(
  const void * node_handle, const void * rmw_handle, topic_stats_core::EndpointKind kind, const char * topic_name)
{
  if (rmw_handle == nullptr || topic_name == nullptr) {
    return;
  }
  const auto node = nodes_.find(node_handle);
  if (!node) {
    // The node's init tracepoint was missed, which should not happen since rcl emits it before any
    // endpoint on that node.
    return;
  }

  topic_stats_core::EndpointDescriptor descriptor;
  descriptor.kind = kind;
  descriptor.topic_name = topic_name;

  const auto id = collector_->register_endpoint(*node, descriptor);
  if (!id.valid()) {
    return;
  }
  const EndpointRecord record{*node, id, descriptor};
  if (!endpoints_.add(rmw_handle, record)) {
    // Address reuse again: the middleware freed an endpoint and handed the same address out for a
    // new one. Replacing the record is what keeps measurements attributed to the right topic.
    endpoints_.update(rmw_handle, [&](EndpointRecord & existing) {
      collector_->unregister_endpoint(existing.endpoint);
      existing = record;
    });
  }
}

void TraceCollector::on_publisher_init(
  const void * node_handle, const void * rmw_publisher_handle, const char * topic_name)
{
  add_endpoint(node_handle, rmw_publisher_handle, topic_stats_core::EndpointKind::Publisher, topic_name);
}

void TraceCollector::on_subscription_init(
  const void * node_handle, const void * rmw_subscription_handle, const char * topic_name)
{
  add_endpoint(node_handle, rmw_subscription_handle, topic_stats_core::EndpointKind::Subscription, topic_name);
}

topic_stats_core::EndpointId TraceCollector::resolve(const void * rmw_handle)
{
  const auto record = endpoints_.find(rmw_handle);
  if (!record) {
    return {};
  }
  if (record->endpoint.valid()) {
    return record->endpoint;
  }

  // Evicted for going quiet, and now speaking again. Registering it afresh under the map's write
  // lock keeps two concurrent publishers from registering it twice and double counting.
  topic_stats_core::EndpointId revived;
  endpoints_.update(rmw_handle, [&](EndpointRecord & existing) {
    if (!existing.endpoint.valid()) {
      existing.endpoint = collector_->register_endpoint(existing.node, existing.descriptor);
      revived_endpoints_.fetch_add(1, std::memory_order_relaxed);
    }
    revived = existing.endpoint;
  });
  return revived;
}

void TraceCollector::on_publish(const void * rmw_publisher_handle)
{
  // Unmapped handles are expected rather than exceptional. The middleware has publishers of its
  // own for discovery which rcl never initialises, and their tracepoints fire before any node
  // exists.
  const auto id = resolve(rmw_publisher_handle);
  if (id.valid()) {
    collector_->record_publish(id);
  }
}

void TraceCollector::on_take(const void * rmw_subscription_handle, int64_t source_timestamp, bool taken)
{
  if (!taken) {
    // The middleware polls far more often than messages arrive, so most calls land here.
    return;
  }
  const auto id = resolve(rmw_subscription_handle);
  if (!id.valid()) {
    return;
  }
  collector_->record_take(id, source_timestamp);
}

void TraceCollector::report()
{
  // A report with nothing in it is not worth handing to a sink. Both shipping sinks drop empties
  // internally anyway, but a sink should not have to.
  const auto report = collector_->snapshot();
  if (!report.empty()) {
    sink_->publish(report);
  }

  if (config_.idle_eviction_reports == 0) {
    return;
  }
  const auto evicted = collector_->evict_idle(config_.idle_eviction_reports);
  if (evicted.empty()) {
    return;
  }
  // The handle stays mapped, with its descriptor, so traffic arriving later brings the endpoint
  // back rather than being dropped for the life of the process. One pass over the map rather than
  // one per evicted id, since both sets can be large on a process with many endpoints.
  const std::unordered_set<topic_stats_core::EndpointId> dropped(evicted.begin(), evicted.end());
  endpoints_.for_each_value([&](EndpointRecord & record) {
    if (dropped.count(record.endpoint) > 0) {
      record.endpoint = {};
    }
  });
}

}  // namespace topic_stats_tracetools
