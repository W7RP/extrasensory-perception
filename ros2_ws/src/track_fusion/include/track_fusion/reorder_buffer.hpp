// Holds detection batches briefly so the tracker sees them in time order.
//
// Agents publish independently, so batch k+1 from agent 2 can arrive before
// batch k from agent 1. The buffer releases a batch once no earlier one can
// still arrive: at the "watermark", the oldest newest-stamp among the agents
// heard from so far. An agent that goes quiet must not stall everyone, so the
// watermark never lags the newest stamp by more than `max_hold_s`; anything
// that arrives later than that is late, and the tracker refuses it (counted).
//
// Fixed capacity, sorted insertion: O(N) per push with N = kCapacity, no
// allocation. When full, the oldest batch is released early (counted).
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "track_fusion/types.hpp"

namespace track_fusion
{

class ReorderBuffer
{
public:
  static constexpr std::size_t kCapacity = 32;

  explicit ReorderBuffer(double max_hold_s) noexcept
  : max_hold_s_(max_hold_s)
  {
    agent_newest_.fill(-std::numeric_limits<double>::infinity());
  }

  // Insert a batch. If the buffer is full, the oldest batch is written to
  // `evicted` and true is returned: the caller must process it now.
  bool push(const DetectionBatch & b, DetectionBatch & evicted) noexcept
  {
    bool out = false;
    if (count_ == kCapacity) {
      evicted = buf_[0];
      shift_left();
      ++forced_;
      out = true;
    }
    std::size_t i = count_;
    while (i > 0 && buf_[i - 1].t > b.t) {
      buf_[i] = buf_[i - 1];
      --i;
    }
    buf_[i] = b;
    ++count_;
    max_depth_ = std::max(max_depth_, count_);
    if (b.agent >= 1 && b.agent <= kMaxAgents) {
      double & newest = agent_newest_[b.agent - 1U];
      newest = std::max(newest, b.t);
    }
    newest_ = std::max(newest_, b.t);
    return out;
  }

  // Time up to which batches can be released.
  [[nodiscard]] double watermark() const noexcept
  {
    double w = std::numeric_limits<double>::infinity();
    for (double t : agent_newest_) {
      if (t > -std::numeric_limits<double>::infinity()) {
        w = std::min(w, t);
      }
    }
    return std::max(w, newest_ - max_hold_s_);
  }

  // Pop the oldest batch if it is at or before the watermark.
  bool pop_ready(DetectionBatch & out) noexcept
  {
    if (count_ == 0 || buf_[0].t > watermark()) {
      return false;
    }
    out = buf_[0];
    shift_left();
    return true;
  }

  [[nodiscard]] std::size_t size() const noexcept {return count_;}
  [[nodiscard]] std::size_t max_depth() const noexcept {return max_depth_;}
  [[nodiscard]] std::uint64_t forced_releases() const noexcept {return forced_;}

private:
  void shift_left() noexcept
  {
    for (std::size_t i = 1; i < count_; ++i) {
      buf_[i - 1] = buf_[i];
    }
    --count_;
  }

  double max_hold_s_;
  std::array<DetectionBatch, kCapacity> buf_{};
  std::size_t count_{0};
  std::size_t max_depth_{0};
  std::uint64_t forced_{0};
  std::array<double, kMaxAgents> agent_newest_{};
  double newest_{-std::numeric_limits<double>::infinity()};
};

}  // namespace track_fusion
