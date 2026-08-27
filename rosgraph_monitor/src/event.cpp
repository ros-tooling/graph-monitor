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
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_.store(true);
  }
  cv_.notify_all();
}

void Event::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_.store(false);
}

bool Event::check_and_clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_.exchange(false);
}

bool Event::wait_for(const std::chrono::milliseconds & timeout)
{
  return wait_until(std::chrono::steady_clock::now() + timeout);
}

bool Event::wait_until(std::chrono::steady_clock::time_point deadline)
{
  if (is_set()) {
    return true;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  return cv_.wait_until(lock, deadline, [this]() { return state_.load(); });
}

void Event::wait()
{
  if (is_set()) {
    return;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait(lock, [this]() { return state_.load(); });
}

}  // namespace rosgraph_monitor
