// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rosgraph_monitor
{

/// @brief A bounded, deduplicating, retrying queue of keyed asynchronous queries.
/// @details Runs at most `max_concurrent` attempts at once, retries a failed or timed out attempt
/// after `retry_delay` until it succeeds or is cancelled, and reports only successful results.
/// Results arrive on the queue's own thread.
/// After the destructor returns, the result callback never runs again.
/// @tparam Result Type of a successful query result.
template <typename Result>
class QueryQueue
{
public:
  /// Reports the outcome of one attempt. nullopt means failure; the queue will retry.
  using Done = std::function<void(std::optional<Result>)>;
  /// Starts one asynchronous attempt at a query.
  /// Must call done exactly once, from any thread, including synchronously.
  using StartCall = std::function<void(const std::string & key, Done done)>;
  /// Receives each successful result, on the queue's own thread.
  using OnResult = std::function<void(const std::string & key, Result result)>;

  struct Options
  {
    size_t max_concurrent = 4;
    /// An attempt with no response by this point has failed.
    std::chrono::milliseconds timeout{10000};
    /// Wait between attempts at one key.
    std::chrono::milliseconds retry_delay{5000};
  };

  QueryQueue(StartCall start_call, OnResult on_result, Options options = {})
  : start_call_(std::move(start_call))
  , on_result_(std::move(on_result))
  , options_(options)
  , mailbox_(std::make_shared<Mailbox>())
  , worker_([this] { run(); })
  {}

  /// Stops and joins the worker thread.
  ~QueryQueue()
  {
    {
      std::lock_guard<std::mutex> lock(mailbox_->mutex);
      mailbox_->stop = true;
    }
    wake();
    worker_.join();
  }

  QueryQueue(const QueryQueue &) = delete;
  QueryQueue & operator=(const QueryQueue &) = delete;

  /// @brief Queues an attempt at `key`.
  /// @details Does nothing if `key` is already queued, in flight, or awaiting retry.
  void request(const std::string & key)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (is_pending(key)) {
        return;
      }
      waiting_.push_back(key);
    }
    wake();
  }

  /// @brief Drops `key` from the queue and abandons any attempt in flight for it.
  /// @details A response to an abandoned attempt is never delivered.
  void cancel(const std::string & key)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      waiting_.erase(std::remove(waiting_.begin(), waiting_.end(), key), waiting_.end());
      in_flight_.erase(key);
      retry_at_.erase(key);
    }
    wake();
  }

  /// @return Number of keys not in flight: queued, or awaiting retry.
  size_t waiting_count() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return waiting_.size() + retry_at_.size();
  }

  /// @return Number of attempts in flight.
  size_t in_flight_count() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return in_flight_.size();
  }

private:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  /// One attempt in flight.
  struct Attempt
  {
    uint64_t generation;
    TimePoint timeout_at;
  };

  /// The outcome of one attempt, as reported by its done callback.
  struct Completion
  {
    std::string key;
    uint64_t generation;
    std::optional<Result> result;
  };

  /// Completions and wakeups. Kept alive by every done callback.
  struct Mailbox
  {
    std::mutex mutex;
    std::condition_variable signal;
    std::vector<Completion> completions;
    /// Set by any producer, cleared by the worker each pass.
    bool wake = false;
    bool stop = false;
  };

  /// Wakes the worker for a fresh pass.
  void wake()
  {
    {
      std::lock_guard<std::mutex> lock(mailbox_->mutex);
      mailbox_->wake = true;
    }
    mailbox_->signal.notify_all();
  }

  /// Dispatches attempts, applies completions, and delivers results, until stopped.
  void run()
  {
    while (true) {
      std::vector<Completion> completions;
      {
        std::lock_guard<std::mutex> lock(mailbox_->mutex);
        if (mailbox_->stop) {
          return;
        }
        mailbox_->wake = false;
        completions.swap(mailbox_->completions);
      }

      std::vector<std::pair<std::string, Result>> deliveries;
      std::vector<std::pair<std::string, Done>> starts;
      std::optional<TimePoint> deadline;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const TimePoint now = Clock::now();
        deliveries = settle(std::move(completions), now);
        retry_timed_out(now);
        promote_due_retries(now);
        starts = fill_slots(now);
        deadline = next_deadline();
      }

      for (auto & delivery : deliveries) {
        on_result_(delivery.first, std::move(delivery.second));
      }
      for (auto & start : starts) {
        start_call_(start.first, std::move(start.second));
      }

      std::unique_lock<std::mutex> lock(mailbox_->mutex);
      const auto woken = [this] { return mailbox_->wake; };
      if (deadline) {
        mailbox_->signal.wait_until(lock, *deadline, woken);
      } else {
        mailbox_->signal.wait(lock, woken);
      }
    }
  }

  /// @return Whether `key` is queued, in flight, or awaiting retry. Call with `state_mutex_` held.
  bool is_pending(const std::string & key) const
  {
    return in_flight_.count(key) != 0 || retry_at_.count(key) != 0 ||
           std::find(waiting_.begin(), waiting_.end(), key) != waiting_.end();
  }

  /// @brief Applies completions, scheduling a retry for each failure. Call with `state_mutex_` held.
  /// @details A completion that does not match the current attempt for its key is dropped.
  /// @return The successful results, to deliver with no lock held.
  std::vector<std::pair<std::string, Result>> settle(std::vector<Completion> completions, TimePoint now)
  {
    std::vector<std::pair<std::string, Result>> deliveries;
    for (auto & completion : completions) {
      const auto attempt = in_flight_.find(completion.key);
      if (attempt == in_flight_.end() || attempt->second.generation != completion.generation) {
        continue;
      }
      in_flight_.erase(attempt);
      if (completion.result) {
        deliveries.emplace_back(completion.key, std::move(*completion.result));
      } else {
        retry_at_[completion.key] = now + options_.retry_delay;
      }
    }
    return deliveries;
  }

  /// Abandons every attempt past its timeout and schedules a retry for it. Call with `state_mutex_` held.
  void retry_timed_out(TimePoint now)
  {
    for (auto attempt = in_flight_.begin(); attempt != in_flight_.end();) {
      if (attempt->second.timeout_at <= now) {
        retry_at_[attempt->first] = now + options_.retry_delay;
        attempt = in_flight_.erase(attempt);
      } else {
        ++attempt;
      }
    }
  }

  /// Queues every key whose retry time has arrived. Call with `state_mutex_` held.
  void promote_due_retries(TimePoint now)
  {
    for (auto retry = retry_at_.begin(); retry != retry_at_.end();) {
      if (retry->second <= now) {
        waiting_.push_back(retry->first);
        retry = retry_at_.erase(retry);
      } else {
        ++retry;
      }
    }
  }

  /// @brief Takes keys off the queue up to the concurrency bound. Call with `state_mutex_` held.
  /// @return Each started key with the done callback of its attempt, to call with no lock held.
  std::vector<std::pair<std::string, Done>> fill_slots(TimePoint now)
  {
    std::vector<std::pair<std::string, Done>> starts;
    while (in_flight_.size() < options_.max_concurrent && !waiting_.empty()) {
      const std::string key = std::move(waiting_.front());
      waiting_.pop_front();
      const uint64_t generation = ++last_generation_;
      in_flight_.insert_or_assign(key, Attempt{generation, now + options_.timeout});
      starts.emplace_back(key, make_done(key, generation));
    }
    return starts;
  }

  /// @return A done callback that records one attempt's outcome. It touches only the mailbox.
  Done make_done(const std::string & key, uint64_t generation)
  {
    return [mailbox = mailbox_, key, generation](std::optional<Result> result) {
      {
        std::lock_guard<std::mutex> lock(mailbox->mutex);
        mailbox->completions.push_back(Completion{key, generation, std::move(result)});
        mailbox->wake = true;
      }
      mailbox->signal.notify_all();
    };
  }

  /// @return The earliest in-flight timeout or scheduled retry, if there is one. Call with `state_mutex_` held.
  std::optional<TimePoint> next_deadline() const
  {
    std::optional<TimePoint> deadline;
    const auto keep_earliest = [&deadline](TimePoint candidate) {
      if (!deadline || candidate < *deadline) {
        deadline = candidate;
      }
    };
    for (const auto & attempt : in_flight_) {
      keep_earliest(attempt.second.timeout_at);
    }
    for (const auto & retry : retry_at_) {
      keep_earliest(retry.second);
    }
    return deadline;
  }

  const StartCall start_call_;
  const OnResult on_result_;
  const Options options_;

  mutable std::mutex state_mutex_;
  std::deque<std::string> waiting_;
  std::unordered_map<std::string, Attempt> in_flight_;
  std::unordered_map<std::string, TimePoint> retry_at_;
  uint64_t last_generation_ = 0;

  const std::shared_ptr<Mailbox> mailbox_;
  std::thread worker_;
};

}  // namespace rosgraph_monitor
