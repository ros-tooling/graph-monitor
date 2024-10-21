// Copyright 2024, Bonsai Robotics, Inc - All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RMW_STATS_SHIM__TIMER_HPP_
#define RMW_STATS_SHIM__TIMER_HPP_

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace rmw_stats_shim
{

/// @brief Simple periodic timer that spawns a thread to call a function at specified interval.
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
  bool running_ = false;
};

}  // namespace rmw_stats_shim

#endif  // RMW_STATS_SHIM__TIMER_HPP_
