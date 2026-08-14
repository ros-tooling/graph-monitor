// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "rosgraph_monitor/event.hpp"

namespace rosgraph_monitor
{

bool Event::is_set() const
{
  return state_.load();
}

void Event::set()
{
  state_.exchange(true);
  cv_.notify_all();
}

void Event::clear()
{
  state_.exchange(false);
}

bool Event::check_and_clear()
{
  return state_.exchange(false);
}

bool Event::wait_for(const std::chrono::milliseconds & timeout)
{
  if (is_set()) {
    return true;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  return cv_.wait_for(lock, timeout, [this]() { return state_.load(); });
}

}  // namespace rosgraph_monitor
