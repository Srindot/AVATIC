// Copyright 2026 AVATIC contributors.
//
// Pure time delay on simulation time: Sample(t) returns the most recent value
// pushed at or before t - delay (or the initial value if none). Values must be
// pushed with non-decreasing timestamps. Entries older than needed are
// discarded, so memory is bounded by delay / push interval.

#ifndef PLUTO_X_COMMON_DELAY_LINE_HPP_
#define PLUTO_X_COMMON_DELAY_LINE_HPP_

#include <cstdint>
#include <deque>
#include <stdexcept>
#include <utility>

namespace pluto_x {

template <typename T>
class DelayLine {
 public:
  DelayLine(std::int64_t delay_ns, T initial)
      : delay_ns_(delay_ns), current_(std::move(initial)) {
    if (delay_ns < 0) {
      throw std::invalid_argument("DelayLine delay must be >= 0");
    }
  }

  /// Precondition (checked): time_ns >= the previous push time.
  void Push(std::int64_t time_ns, const T& value) {
    if (!pending_.empty() && time_ns < pending_.back().first) {
      throw std::invalid_argument("DelayLine timestamps must not decrease");
    }
    pending_.emplace_back(time_ns, value);
  }

  const T& Sample(std::int64_t now_ns) {
    const std::int64_t release_ns = now_ns - delay_ns_;
    while (!pending_.empty() && pending_.front().first <= release_ns) {
      current_ = pending_.front().second;
      pending_.pop_front();
    }
    return current_;
  }

 private:
  std::int64_t delay_ns_;
  T current_;
  std::deque<std::pair<std::int64_t, T>> pending_;
};

}  // namespace pluto_x

#endif  // PLUTO_X_COMMON_DELAY_LINE_HPP_
