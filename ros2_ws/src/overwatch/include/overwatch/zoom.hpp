// Where the high agent's zoom camera looks next. ROS-free and unit-tested.
//
// The wide camera finds people; the zoom camera, with ten times the
// resolution over a 6 deg field, is for a closer look, one person at a time.
// Each cycle the scheduler keeps its current track for at least `dwell_s`
// (long enough to settle and take a good look), then moves to the candidate
// most overdue for one:
//   priority = min(time since last looked at, max_age_credit_s)
//              + hidden_bonus_s   if the device cannot see that spot itself
//              + coasting_bonus_s if the track has lost its detections
// so it cycles through everyone, returns first to the people the device
// needs help with, and goes looking for tracks that have gone quiet.
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <span>

#include <Eigen/Core>

namespace overwatch
{

struct ZoomConfig
{
  double dwell_s{3.0};
  double hidden_bonus_s{6.0};
  double coasting_bonus_s{4.0};
  double max_age_credit_s{20.0};
};

struct ZoomCandidate
{
  std::uint32_t track_id{0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  bool hidden{false};    // no line of sight from the device
  bool coasting{false};
};

class ZoomScheduler
{
public:
  explicit ZoomScheduler(ZoomConfig cfg) : cfg_(cfg) {}

  // The track to look at at time t [s], or nullopt when there is none.
  [[nodiscard]] std::optional<std::uint32_t> choose(
    double t, std::span<const ZoomCandidate> candidates);

  [[nodiscard]] std::optional<std::uint32_t> current() const noexcept {return current_;}

private:
  ZoomConfig cfg_;
  std::map<std::uint32_t, double> last_viewed_;
  std::optional<std::uint32_t> current_;
  double since_{0.0};
};

}  // namespace overwatch
