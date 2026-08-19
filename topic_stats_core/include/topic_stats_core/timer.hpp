// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPIC_STATS_CORE__TIMER_HPP_
#define TOPIC_STATS_CORE__TIMER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

namespace topic_stats_core
{

/// @brief Simple periodic timer that spawns a thread to call a function at specified interval.
///
/// Lives here rather than in an ingest adapter because reporting cadence is the same problem for
/// every adapter, and because the callback runs on its own thread, which is what keeps reporting
/// off the hot path.
class Timer
{
public:
  Timer() = delete;

  /// @brief Does not start the timer
  /// @param func Callback to call periodically
  /// @param interval How often to call func
  Timer(std::function<void(void)> func, std::chrono::milliseconds interval);

  virtual ~Timer();

  /// @brief Create background thread and start timer
  /// No-op if already running
  void start();

  /// @brief Interrupt and destroy background thread. Can be restarted with start()
  /// No-op if not running
  void stop();

  /// @brief If timer has been started
  bool isRunning();

private:
  void runThread();

  std::mutex mu_;
  std::condition_variable cv_;

  std::function<void(void)> func_;
  std::chrono::milliseconds interval_;
  std::thread thread_;
  std::atomic<bool> running_ = false;
};

}  // namespace topic_stats_core

#endif  // TOPIC_STATS_CORE__TIMER_HPP_
