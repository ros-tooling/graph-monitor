// SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rosgraph_monitor/event.hpp"
#include "rosgraph_monitor/mutex_protected.hpp"

namespace rosgraph_monitor
{

/// @brief A bounded, deduplicating, retrying queue of keyed asynchronous queries.
/// @details Runs at most `max_concurrent` attempts at once, retries a failed or timed out attempt
/// after `first_retry_delay` doubled once per consecutive failure and capped at `retry_delay`,
/// until it succeeds or is cancelled, and reports only successful results.
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
    /// Longest wait between attempts at one key.
    std::chrono::milliseconds retry_delay{5000};
    /// Wait before a key's first retry; each consecutive failure doubles the wait, capped at `retry_delay`.
    std::chrono::milliseconds first_retry_delay{5000};
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
    stop_.store(true);
    mailbox_->arrived.set();
    worker_.join();
  }

  QueryQueue(const QueryQueue &) = delete;
  QueryQueue & operator=(const QueryQueue &) = delete;

  /// @brief Queues an attempt at `key`.
  /// @details Does nothing if `key` is already queued, in flight, or awaiting retry.
  void request(const std::string & key)
  {
    {
      auto state = state_.lock();
      if (is_pending(*state, key)) {
        return;
      }
      state->waiting.push_back(key);
    }
    mailbox_->arrived.set();
  }

  /// @brief Drops `key` from the queue and abandons any attempt in flight for it.
  /// @details A response to an abandoned attempt is never delivered.
  void cancel(const std::string & key)
  {
    {
      auto state = state_.lock();
      auto & waiting = state->waiting;
      waiting.erase(std::remove(waiting.begin(), waiting.end(), key), waiting.end());
      state->in_flight.erase(key);
      state->retry_at.erase(key);
      state->failures.erase(key);
    }
    mailbox_->arrived.set();
  }

  /// @return Number of keys not in flight: queued, or awaiting retry.
  size_t waiting_count() const
  {
    const auto state = state_.lock();
    return state->waiting.size() + state->retry_at.size();
  }

  /// @return Number of attempts in flight.
  size_t in_flight_count() const
  {
    const auto state = state_.lock();
    return state->in_flight.size();
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

  /// Every key the queue is working on, and the generation counter that stamps new attempts.
  struct State
  {
    std::deque<std::string> waiting;
    std::unordered_map<std::string, Attempt> in_flight;
    std::unordered_map<std::string, TimePoint> retry_at;
    /// Consecutive failed attempts at a key, dropped when it succeeds or is cancelled.
    std::unordered_map<std::string, uint32_t> failures;
    uint64_t last_generation = 0;
  };

  /// Completions and the signal that one arrived. Kept alive by every done callback.
  struct Mailbox
  {
    MutexProtected<std::vector<Completion>> completions{std::vector<Completion>{}};
    Event arrived;
  };

  /// Dispatches attempts, applies completions, and delivers results, until stopped.
  void run()
  {
    while (true) {
      // Clearing before the stop check keeps a stop signalled mid-pass from being swallowed.
      mailbox_->arrived.check_and_clear();
      if (stop_.load()) {
        return;
      }

      std::vector<Completion> completions;
      {
        auto arrivals = mailbox_->completions.lock();
        completions.swap(*arrivals);
      }

      std::vector<std::pair<std::string, Result>> deliveries;
      std::vector<std::pair<std::string, Done>> starts;
      std::optional<TimePoint> deadline;
      {
        auto state = state_.lock();
        const TimePoint now = Clock::now();
        deliveries = settle(*state, std::move(completions), now);
        retry_timed_out(*state, now);
        promote_due_retries(*state, now);
        starts = fill_slots(*state, now);
        deadline = next_deadline(*state);
      }

      for (auto & delivery : deliveries) {
        on_result_(delivery.first, std::move(delivery.second));
      }
      for (auto & start : starts) {
        start_call_(start.first, std::move(start.second));
      }

      if (deadline) {
        mailbox_->arrived.wait_until(*deadline);
      } else {
        mailbox_->arrived.wait();
      }
    }
  }

  /// @return Whether `key` is queued, in flight, or awaiting retry.
  static bool is_pending(const State & state, const std::string & key)
  {
    return state.in_flight.count(key) != 0 || state.retry_at.count(key) != 0 ||
           std::find(state.waiting.begin(), state.waiting.end(), key) != state.waiting.end();
  }

  /// @brief Applies completions, scheduling a retry for each failure.
  /// @details A completion that does not match the current attempt for its key is dropped.
  /// @return The successful results, to deliver with no lock held.
  std::vector<std::pair<std::string, Result>> settle(
    State & state, std::vector<Completion> completions, TimePoint now) const
  {
    std::vector<std::pair<std::string, Result>> deliveries;
    for (auto & completion : completions) {
      const auto attempt = state.in_flight.find(completion.key);
      if (attempt == state.in_flight.end() || attempt->second.generation != completion.generation) {
        continue;
      }
      state.in_flight.erase(attempt);
      if (completion.result) {
        state.failures.erase(completion.key);
        deliveries.emplace_back(completion.key, std::move(*completion.result));
      } else {
        schedule_retry(state, completion.key, now);
      }
    }
    return deliveries;
  }

  /// Abandons every attempt past its timeout and schedules a retry for it.
  void retry_timed_out(State & state, TimePoint now) const
  {
    for (auto attempt = state.in_flight.begin(); attempt != state.in_flight.end();) {
      if (attempt->second.timeout_at <= now) {
        schedule_retry(state, attempt->first, now);
        attempt = state.in_flight.erase(attempt);
      } else {
        ++attempt;
      }
    }
  }

  /// Counts one more consecutive failure at `key` and schedules its next attempt.
  void schedule_retry(State & state, const std::string & key, TimePoint now) const
  {
    const uint32_t failures = ++state.failures[key];
    state.retry_at[key] = now + retry_delay_for(failures);
  }

  /// @return The wait after `failures` consecutive failures: `first_retry_delay` doubled once per
  /// earlier failure, capped at `retry_delay`.
  std::chrono::milliseconds retry_delay_for(uint32_t failures) const
  {
    // Larger failure counts wait the cap, keeping the shift in range.
    constexpr uint32_t kMaxDoublings = 30;
    if (failures == 0 || failures > kMaxDoublings) {
      return options_.retry_delay;
    }
    const std::chrono::milliseconds delay = options_.first_retry_delay * (int64_t{1} << (failures - 1));
    return std::min(options_.retry_delay, delay);
  }

  /// Queues every key whose retry time has arrived.
  static void promote_due_retries(State & state, TimePoint now)
  {
    for (auto retry = state.retry_at.begin(); retry != state.retry_at.end();) {
      if (retry->second <= now) {
        state.waiting.push_back(retry->first);
        retry = state.retry_at.erase(retry);
      } else {
        ++retry;
      }
    }
  }

  /// @brief Takes keys off the queue up to the concurrency bound.
  /// @return Each started key with the done callback of its attempt, to call with no lock held.
  std::vector<std::pair<std::string, Done>> fill_slots(State & state, TimePoint now)
  {
    std::vector<std::pair<std::string, Done>> starts;
    while (state.in_flight.size() < options_.max_concurrent && !state.waiting.empty()) {
      const std::string key = std::move(state.waiting.front());
      state.waiting.pop_front();
      const uint64_t generation = ++state.last_generation;
      state.in_flight.insert_or_assign(key, Attempt{generation, now + options_.timeout});
      starts.emplace_back(key, make_done(key, generation));
    }
    return starts;
  }

  /// @return A done callback that records one attempt's outcome. It touches only the mailbox.
  Done make_done(const std::string & key, uint64_t generation)
  {
    return [mailbox = mailbox_, key, generation](std::optional<Result> result) {
      {
        auto completions = mailbox->completions.lock();
        completions->push_back(Completion{key, generation, std::move(result)});
      }
      mailbox->arrived.set();
    };
  }

  /// @return The earliest in-flight timeout or scheduled retry, if there is one.
  static std::optional<TimePoint> next_deadline(const State & state)
  {
    std::optional<TimePoint> deadline;
    const auto keep_earliest = [&deadline](TimePoint candidate) {
      if (!deadline || candidate < *deadline) {
        deadline = candidate;
      }
    };
    for (const auto & attempt : state.in_flight) {
      keep_earliest(attempt.second.timeout_at);
    }
    for (const auto & retry : state.retry_at) {
      keep_earliest(retry.second);
    }
    return deadline;
  }

  const StartCall start_call_;
  const OnResult on_result_;
  const Options options_;

  MutexProtected<State> state_{State{}};
  const std::shared_ptr<Mailbox> mailbox_;
  /// Written only by the destructor, read only by the worker.
  std::atomic<bool> stop_{false};
  std::thread worker_;
};

}  // namespace rosgraph_monitor
