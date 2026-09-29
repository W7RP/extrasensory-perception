#include "overwatch/zoom.hpp"

#include <algorithm>

namespace overwatch
{

std::optional<std::uint32_t> ZoomScheduler::choose(
  double t, std::span<const ZoomCandidate> candidates)
{
  const auto present = [&](std::uint32_t id) {
      return std::any_of(candidates.begin(), candidates.end(),
               [id](const auto & c) {return c.track_id == id;});
    };
  if (current_ && present(*current_)) {
    last_viewed_[*current_] = t;
    if (t - since_ < cfg_.dwell_s) {
      return current_;
    }
  }
  std::optional<std::uint32_t> best;
  double best_priority = -1e18;
  for (const auto & c : candidates) {
    const auto it = last_viewed_.find(c.track_id);
    const double age = it == last_viewed_.end() ? cfg_.max_age_credit_s :
      std::min(t - it->second, cfg_.max_age_credit_s);
    const double priority = age + (c.hidden ? cfg_.hidden_bonus_s : 0.0) +
      (c.coasting ? cfg_.coasting_bonus_s : 0.0);
    if (priority > best_priority) {
      best_priority = priority;
      best = c.track_id;
    }
  }
  // A new dwell starts, on another track or (the only one) the same again.
  current_ = best;
  if (best) {
    since_ = t;
    last_viewed_[*best] = t;
  }
  // Forget tracks long gone, so the map does not grow over a session.
  for (auto it = last_viewed_.begin(); it != last_viewed_.end(); ) {
    it = t - it->second > 10.0 * cfg_.max_age_credit_s && !present(it->first) ?
      last_viewed_.erase(it) : std::next(it);
  }
  return current_;
}

}  // namespace overwatch
