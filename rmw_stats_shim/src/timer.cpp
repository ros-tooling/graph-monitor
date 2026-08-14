// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "rmw_stats_shim/timer.hpp"

namespace rmw_stats_shim
{

Timer::Timer(std::function<void(void)> func, std::chrono::milliseconds interval)
: func_(func)
, interval_(interval)
{}

Timer::~Timer()
{
  stop();
}

void Timer::start()
{
  running_ = true;
  thread_ = std::thread(std::bind(&Timer::runThread, this));
}

void Timer::runThread()
{
  std::unique_lock<std::mutex> lock(mu_);
  auto now = std::chrono::steady_clock::now();
  auto end_time = now + interval_;
  while (running_) {
    now = end_time;
    end_time = now + interval_;
    while (running_ && cv_.wait_until(lock, end_time) != std::cv_status::timeout) {
    }
    func_();
  }
}

void Timer::stop()
{
  running_ = false;
  cv_.notify_all();
  thread_.join();
  thread_ = std::thread{};
}

bool Timer::isRunning()
{
  return running_;
}

}  // namespace rmw_stats_shim
