// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef ROSGRAPH_MONITOR__MUTEX_PROTECTED_HPP_
#define ROSGRAPH_MONITOR__MUTEX_PROTECTED_HPP_

#pragma once

#include <mutex>
#include <utility>

namespace rosgraph_monitor
{

/// @brief Utility class to enforce mutex-synchronized access to a data member.
/// @tparam T Type of the underlying data to hold
template <typename T, typename Mutex = std::mutex>
class MutexProtected
{
public:
  /// @brief RAII guard for accessing the protected data.
  /// @tparam Value Either T or const T, so a const MutexProtected hands out const access.
  template <typename Value>
  class GuardT
  {
  public:
    GuardT(Value & value, Mutex & mtx)
    : lock_(mtx)
    , value_(value)
    {}

    // Access via pointer-style dereference
    Value * operator->() const
    {
      return &value_;
    }

    Value & operator*() const
    {
      return value_;
    }

    // Non-copyable
    GuardT(const GuardT &) = delete;
    GuardT & operator=(const GuardT &) = delete;

    // Movable
    GuardT(GuardT &&) = default;
    GuardT & operator=(GuardT &&) = default;

  private:
    std::unique_lock<Mutex> lock_;
    Value & value_;
  };

  using Guard = GuardT<T>;
  using ConstGuard = GuardT<const T>;

  /// @brief No default constructor, always uses the variadic template
  MutexProtected() = delete;

  /// @brief Copy constructor, passing a full T type
  /// @param initial_value
  explicit MutexProtected(const T & initial_value)
  : value_(initial_value)
  {}

  /// @brief Pass-through constructor, passes variadic arguments to T constructor
  /// @tparam ...Args
  /// @param ...args
  template <typename... Args>
  explicit MutexProtected(Args &&... args)
  : value_(std::forward<Args>(args)...)
  {}

  // Get RAII access to the protected data
  Guard lock()
  {
    return Guard(value_, mutex_);
  }

  /// @brief Read-only access, so const methods can still take the lock.
  /// @details The mutex is mutable, so locking does not itself mutate the object.
  ConstGuard lock() const
  {
    return ConstGuard(value_, mutex_);
  }

private:
  T value_;
  mutable Mutex mutex_;
};

}  // namespace rosgraph_monitor

#endif  // ROSGRAPH_MONITOR__MUTEX_PROTECTED_HPP_
