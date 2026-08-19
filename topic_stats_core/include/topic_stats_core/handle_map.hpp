// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPIC_STATS_CORE__HANDLE_MAP_HPP_
#define TOPIC_STATS_CORE__HANDLE_MAP_HPP_

#include <mutex>
#include <optional>
#include <shared_mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace topic_stats_core
{

/// Thread-safe association from an opaque middleware handle to whatever the statistics core handed
/// back for it.
///
/// This is the only state an ingest adapter needs of its own, and every adapter needs it: the RMW
/// wrapper keys on rmw_publisher_t pointers, the tracepoint adapter keys on the rmw handles the
/// tracepoints carry. It is deliberately its own type for two reasons. It is the one part of an
/// adapter that can be tested without a middleware, and getting its synchronization wrong is
/// exactly how the first implementation raced: entities are created and destroyed on any thread
/// while messages flow on others.
///
/// Handles are compared by pointer identity. Nothing is dereferenced, so a test can use fabricated
/// addresses.
template <typename Value>
class HandleMap
{
public:
  /// Returns false if the handle was already mapped, leaving the existing value in place.
  bool add(const void * handle, Value value)
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    return map_.emplace(handle, std::move(value)).second;
  }

  std::optional<Value> find(const void * handle) const
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    const auto it = map_.find(handle);
    if (it == map_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  /// Removes the handle, returning what it mapped to if anything.
  std::optional<Value> remove(const void * handle)
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    const auto it = map_.find(handle);
    if (it == map_.end()) {
      return std::nullopt;
    }
    Value value = std::move(it->second);
    map_.erase(it);
    return value;
  }

  /// Applies a mutation to the mapped value under this map's own write lock, so that a
  /// read-modify-write cannot interleave with another thread doing the same. Returns false if the
  /// handle is unknown.
  ///
  /// Needed by adapters that have to re-register an endpoint the core evicted: without it, two
  /// threads publishing at once would each register the endpoint again and it would be counted
  /// twice.
  template <typename Mutation>
  bool update(const void * handle, Mutation mutation)
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    const auto it = map_.find(handle);
    if (it == map_.end()) {
      return false;
    }
    mutation(it->second);
    return true;
  }

  /// Applies a mutation to every mapped value under this map's write lock.
  template <typename Mutation>
  void for_each_value(Mutation mutation)
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    for (auto & entry : map_) {
      mutation(entry.second);
    }
  }

  /// Removes every entry whose value matches, returning what was removed.
  ///
  /// Needed because a node can be destroyed without its endpoints being destroyed first, which
  /// would otherwise leave this map growing for the life of the process.
  template <typename Predicate>
  std::vector<Value> remove_if(Predicate predicate)
  {
    std::vector<Value> removed;
    std::unique_lock<std::shared_mutex> lock(mutex_);
    for (auto it = map_.begin(); it != map_.end();) {
      if (predicate(it->second)) {
        removed.push_back(std::move(it->second));
        it = map_.erase(it);
      } else {
        ++it;
      }
    }
    return removed;
  }

  size_t size() const
  {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return map_.size();
  }

  void clear()
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    map_.clear();
  }

private:
  mutable std::shared_mutex mutex_;
  std::unordered_map<const void *, Value> map_;
};

}  // namespace topic_stats_core

#endif  // TOPIC_STATS_CORE__HANDLE_MAP_HPP_
