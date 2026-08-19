// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "topic_stats_core/timer.hpp"

#include <utility>

namespace topic_stats_core
{

Timer::Timer(std::function<void(void)> func, std::chrono::milliseconds interval)
: func_(std::move(func))
, interval_(interval)
{}

Timer::~Timer()
{
  stop();
}

void Timer::start()
{
  if (running_.exchange(true)) {
    return;
  }
  thread_ = std::thread(&Timer::runThread, this);
}

void Timer::runThread()
{
  std::unique_lock<std::mutex> lock(mu_);
  auto end_time = std::chrono::steady_clock::now() + interval_;
  while (running_) {
    if (cv_.wait_until(lock, end_time) != std::cv_status::timeout) {
      // Either a stop request or a spurious wakeup. Re-check rather than firing, so that stopping
      // never triggers one last callback into state the caller is already tearing down.
      continue;
    }

    // Released around the callback so that stop() is never made to wait on it.
    lock.unlock();
    func_();
    lock.lock();

    end_time += interval_;
    const auto now = std::chrono::steady_clock::now();
    if (end_time < now) {
      // The callback overran the interval. Skip the missed ticks rather than firing back to back
      // to catch up.
      end_time = now + interval_;
    }
  }
}

void Timer::stop()
{
  {
    // Taken so that the flag cannot be cleared between the worker checking it and waiting, which
    // would lose the wakeup and stall shutdown for a whole interval.
    std::lock_guard<std::mutex> lock(mu_);
    running_ = false;
  }
  cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
  thread_ = std::thread{};
}

bool Timer::isRunning()
{
  return running_;
}

}  // namespace topic_stats_core
