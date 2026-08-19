// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

/// Definitions of the `ros_trace_*` symbols this library interposes.
///
/// Deliberately thin: every function translates its arguments and hands them to TraceCollector,
/// which is where anything worth testing lives. Nothing here needs a middleware to exercise.
///
/// How the interposition works. ELF symbol resolution takes the first definition in the global
/// lookup scope, and an LD_PRELOADed library comes before the DT_NEEDED libraries of everything
/// else, so these definitions win over libtracetools' for the symbols named here. Symbols not named
/// here are untouched and still resolve to the real library, which is why preloading this does not
/// disturb any other instrumentation.
///
/// Each function then chains to the real implementation through RTLD_NEXT, so a live LTTng session
/// records exactly what it would have without this library. Resolution is lazy and cached in a
/// function-local static: doing it at load time would mean calling dlsym before the dynamic linker
/// has finished, and these functions can fire before main().
///
/// Three variants exist per tracepoint because callers use two different forms. rcl and most RMW
/// implementations call `ros_trace_X` directly, but rmw_zenoh_cpp uses the guarded pair
/// `ros_trace_enabled_X` followed by `ros_trace_do_X` to avoid computing arguments when tracing is
/// off. Interposing only the plain form would silently miss every zenoh event.

#include <dlfcn.h>

#include <cstddef>
#include <cstdint>

#include "topic_stats_tracetools/trace_collector.hpp"

namespace
{

/// Resolves the next definition of a symbol along the search order, i.e. the real tracetools one.
/// Cached per call site; nullptr if the real library is absent, which is survivable.
template <typename Signature>
Signature next_symbol(const char * name)
{
  return reinterpret_cast<Signature>(dlsym(RTLD_NEXT, name));
}

}  // namespace

using topic_stats_tracetools::TraceCollector;

extern "C"
{
// -- Node ---------------------------------------------------------------------------------------

void ros_trace_rcl_node_init(
  const void * node_handle, const void * rmw_handle, const char * node_name, const char * node_namespace)
{
  TraceCollector::instance().on_node_init(node_handle, node_name, node_namespace);
  static const auto real = next_symbol<decltype(&ros_trace_rcl_node_init)>("ros_trace_rcl_node_init");
  if (real != nullptr) {
    real(node_handle, rmw_handle, node_name, node_namespace);
  }
}

bool ros_trace_enabled_rcl_node_init(void)
{
  return true;
}

void ros_trace_do_rcl_node_init(
  const void * node_handle, const void * rmw_handle, const char * node_name, const char * node_namespace)
{
  TraceCollector::instance().on_node_init(node_handle, node_name, node_namespace);
  static const auto enabled = next_symbol<bool (*)(void)>("ros_trace_enabled_rcl_node_init");
  static const auto real = next_symbol<decltype(&ros_trace_do_rcl_node_init)>("ros_trace_do_rcl_node_init");
  // Only forwarded when the real tracepoint is actually enabled, since the guarded form exists
  // precisely so that a disabled tracepoint costs nothing.
  if (real != nullptr && enabled != nullptr && enabled()) {
    real(node_handle, rmw_handle, node_name, node_namespace);
  }
}

// -- Publisher ----------------------------------------------------------------------------------

void ros_trace_rcl_publisher_init(
  const void * publisher_handle,
  const void * node_handle,
  const void * rmw_publisher_handle,
  const char * topic_name,
  const size_t queue_depth)
{
  TraceCollector::instance().on_publisher_init(node_handle, rmw_publisher_handle, topic_name);
  static const auto real = next_symbol<decltype(&ros_trace_rcl_publisher_init)>("ros_trace_rcl_publisher_init");
  if (real != nullptr) {
    real(publisher_handle, node_handle, rmw_publisher_handle, topic_name, queue_depth);
  }
}

bool ros_trace_enabled_rcl_publisher_init(void)
{
  return true;
}

void ros_trace_do_rcl_publisher_init(
  const void * publisher_handle,
  const void * node_handle,
  const void * rmw_publisher_handle,
  const char * topic_name,
  const size_t queue_depth)
{
  TraceCollector::instance().on_publisher_init(node_handle, rmw_publisher_handle, topic_name);
  static const auto enabled = next_symbol<bool (*)(void)>("ros_trace_enabled_rcl_publisher_init");
  static const auto real = next_symbol<decltype(&ros_trace_do_rcl_publisher_init)>("ros_trace_do_rcl_publisher_init");
  if (real != nullptr && enabled != nullptr && enabled()) {
    real(publisher_handle, node_handle, rmw_publisher_handle, topic_name, queue_depth);
  }
}

void ros_trace_rmw_publish(const void * rmw_publisher_handle, const void * message, int64_t timestamp)
{
  TraceCollector::instance().on_publish(rmw_publisher_handle);
  static const auto real = next_symbol<decltype(&ros_trace_rmw_publish)>("ros_trace_rmw_publish");
  if (real != nullptr) {
    real(rmw_publisher_handle, message, timestamp);
  }
}

bool ros_trace_enabled_rmw_publish(void)
{
  return true;
}

void ros_trace_do_rmw_publish(const void * rmw_publisher_handle, const void * message, int64_t timestamp)
{
  TraceCollector::instance().on_publish(rmw_publisher_handle);
  static const auto enabled = next_symbol<bool (*)(void)>("ros_trace_enabled_rmw_publish");
  static const auto real = next_symbol<decltype(&ros_trace_do_rmw_publish)>("ros_trace_do_rmw_publish");
  if (real != nullptr && enabled != nullptr && enabled()) {
    real(rmw_publisher_handle, message, timestamp);
  }
}

// -- Subscription -------------------------------------------------------------------------------

void ros_trace_rcl_subscription_init(
  const void * subscription_handle,
  const void * node_handle,
  const void * rmw_subscription_handle,
  const char * topic_name,
  const size_t queue_depth)
{
  TraceCollector::instance().on_subscription_init(node_handle, rmw_subscription_handle, topic_name);
  static const auto real = next_symbol<decltype(&ros_trace_rcl_subscription_init)>("ros_trace_rcl_subscription_init");
  if (real != nullptr) {
    real(subscription_handle, node_handle, rmw_subscription_handle, topic_name, queue_depth);
  }
}

bool ros_trace_enabled_rcl_subscription_init(void)
{
  return true;
}

void ros_trace_do_rcl_subscription_init(
  const void * subscription_handle,
  const void * node_handle,
  const void * rmw_subscription_handle,
  const char * topic_name,
  const size_t queue_depth)
{
  TraceCollector::instance().on_subscription_init(node_handle, rmw_subscription_handle, topic_name);
  static const auto enabled = next_symbol<bool (*)(void)>("ros_trace_enabled_rcl_subscription_init");
  static const auto real =
    next_symbol<decltype(&ros_trace_do_rcl_subscription_init)>("ros_trace_do_rcl_subscription_init");
  if (real != nullptr && enabled != nullptr && enabled()) {
    real(subscription_handle, node_handle, rmw_subscription_handle, topic_name, queue_depth);
  }
}

void ros_trace_rmw_take(
  const void * rmw_subscription_handle, const void * message, int64_t source_timestamp, const bool taken)
{
  TraceCollector::instance().on_take(rmw_subscription_handle, source_timestamp, taken);
  static const auto real = next_symbol<decltype(&ros_trace_rmw_take)>("ros_trace_rmw_take");
  if (real != nullptr) {
    real(rmw_subscription_handle, message, source_timestamp, taken);
  }
}

bool ros_trace_enabled_rmw_take(void)
{
  return true;
}

void ros_trace_do_rmw_take(
  const void * rmw_subscription_handle, const void * message, int64_t source_timestamp, const bool taken)
{
  TraceCollector::instance().on_take(rmw_subscription_handle, source_timestamp, taken);
  static const auto enabled = next_symbol<bool (*)(void)>("ros_trace_enabled_rmw_take");
  static const auto real = next_symbol<decltype(&ros_trace_do_rmw_take)>("ros_trace_do_rmw_take");
  if (real != nullptr && enabled != nullptr && enabled()) {
    real(rmw_subscription_handle, message, source_timestamp, taken);
  }
}
}  // extern "C"
