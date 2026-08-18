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
  // RAII guard for accessing the protected data
  class Guard
  {
  public:
    Guard(T & value, Mutex & mtx)
    : lock_(mtx)
    , value_(value)
    {}

    // Access via pointer-style dereference
    T * operator->()
    {
      return &value_;
    }

    T & operator*()
    {
      return value_;
    }

    // Non-copyable
    Guard(const Guard &) = delete;
    Guard & operator=(const Guard &) = delete;

    // Movable
    Guard(Guard &&) = default;
    Guard & operator=(Guard &&) = default;

  private:
    std::unique_lock<Mutex> lock_;
    T & value_;
  };

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

private:
  T value_;
  mutable Mutex mutex_;
};

}  // namespace rosgraph_monitor

#endif  // ROSGRAPH_MONITOR__MUTEX_PROTECTED_HPP_
