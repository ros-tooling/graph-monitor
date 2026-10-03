// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPIC_STATS_CORE__SLOT_TABLE_HPP_
#define TOPIC_STATS_CORE__SLOT_TABLE_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "topic_stats_core/types.hpp"

namespace topic_stats_core
{

/// Generation-checked slot storage with stable element addresses.
///
/// Endpoints come and go for the life of a process, so indices have to be recycled or the table
/// grows without bound in any process that churns subscriptions. Recycling means a stale handle
/// could otherwise silently address whatever took the slot next, hence the generation counter.
///
/// Elements are heap allocated individually so that a reference obtained under a shared lock stays
/// valid while another thread inserts. That also lets T contain a mutex, which the endpoint slots
/// do.
///
/// Not thread safe on its own. The collector serializes structural changes.
template <typename T, typename IdType>
class SlotTable
{
public:
  template <typename... Args>
  std::pair<IdType, T *> emplace(Args &&... args)
  {
    uint32_t index;
    if (!free_list_.empty()) {
      index = free_list_.back();
      free_list_.pop_back();
    } else {
      index = static_cast<uint32_t>(slots_.size());
      slots_.push_back(std::make_unique<Slot>());
    }
    Slot & slot = *slots_[index];
    slot.value.emplace(std::forward<Args>(args)...);
    return {IdType{index, slot.generation}, &(*slot.value)};
  }

  /// Returns nullptr for an out of range, freed, or stale handle.
  T * find(IdType id)
  {
    if (!id.valid() || id.index >= slots_.size()) {
      return nullptr;
    }
    Slot & slot = *slots_[id.index];
    if (!slot.value.has_value() || slot.generation != id.generation) {
      return nullptr;
    }
    return &(*slot.value);
  }

  const T * find(IdType id) const
  {
    return const_cast<SlotTable *>(this)->find(id);
  }

  /// Returns whether the handle referred to a live slot.
  bool erase(IdType id)
  {
    if (find(id) == nullptr) {
      return false;
    }
    Slot & slot = *slots_[id.index];
    slot.value.reset();
    // Wrapping the generation is not a practical concern: it would take four billion
    // create/destroy cycles on the same slot to alias, and the consequence is one bad sample.
    ++slot.generation;
    free_list_.push_back(id.index);
    return true;
  }

  size_t live_count() const
  {
    return slots_.size() - free_list_.size();
  }

  /// Total slots ever allocated, including freed ones. Exposed so tests can assert that indices
  /// are actually being recycled.
  size_t capacity() const
  {
    return slots_.size();
  }

  /// Calls fn(IdType, T &) for every live slot, in index order.
  template <typename Fn>
  void for_each(Fn && fn)
  {
    for (size_t i = 0; i < slots_.size(); ++i) {
      Slot & slot = *slots_[i];
      if (slot.value.has_value()) {
        fn(IdType{static_cast<uint32_t>(i), slot.generation}, *slot.value);
      }
    }
  }

private:
  struct Slot
  {
    uint32_t generation = 0;
    std::optional<T> value;
  };

  std::deque<std::unique_ptr<Slot>> slots_;
  std::vector<uint32_t> free_list_;
};

}  // namespace topic_stats_core

#endif  // TOPIC_STATS_CORE__SLOT_TABLE_HPP_
