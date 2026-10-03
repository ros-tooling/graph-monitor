// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef RMW_STATS_SHIM__HANDLE_MAP_HPP_
#define RMW_STATS_SHIM__HANDLE_MAP_HPP_

#include <mutex>
#include <optional>
#include <shared_mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rmw_stats_shim
{

/// Thread-safe association from an opaque middleware handle to whatever the statistics core handed
/// back for it.
///
/// This is the only state an ingest adapter needs of its own, and it is deliberately its own type
/// for two reasons. It is the one part of the adapter that can be tested without a middleware, and
/// getting its synchronization wrong is exactly how the previous implementation raced: RMW entities
/// can be created and destroyed on any thread while messages flow on others.
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

}  // namespace rmw_stats_shim

#endif  // RMW_STATS_SHIM__HANDLE_MAP_HPP_
