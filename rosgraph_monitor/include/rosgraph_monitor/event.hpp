// SPDX-FileCopyrightText: 2024 Bonsai Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef ROSGRAPH_MONITOR__EVENT_HPP_
#define ROSGRAPH_MONITOR__EVENT_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>

namespace rosgraph_monitor
{

/**
 * @brief A thread-safe event object, mimicking python's threading.Event
 */
class Event
{
public:
  /// @brief Check whether internal status has been set.
  /// @return True if the event has been set.
  bool is_set() const;

  /// @brief Set the internal status, notifies all waiting threads.
  void set();

  /// @brief Clear the internal status so the event can be reused.
  void clear();

  /// @brief Check whether internal status has been set, and clear the status.
  /// @return True if event has been set.
  bool check_and_clear();

  /// @brief Wait for the event to be set, with a timeout. Handles spurious wakeups.
  /// @tparam Rep (from std::chrono) Numerical type that represents number of ticks.
  /// @tparam Period (from std::chrono) std::ratio representing seconds per tick.
  /// @param timeout
  /// @return True if the event was already set, or became set within the timeout.
  /// @return False if the event was not set within the timeout
  bool wait_for(const std::chrono::milliseconds & timeout);

  template <class Rep, class Period>
  bool wait_for(const std::chrono::duration<Rep, Period> & timeout)
  {
    return wait_for(std::chrono::duration_cast<std::chrono::milliseconds>(timeout));
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  std::atomic_bool state_{false};
};

}  // namespace rosgraph_monitor

#endif  // ROSGRAPH_MONITOR__EVENT_HPP_
